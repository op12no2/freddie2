/* Freddie 2 — motor bench-test console.
 *
 * Prints chip info at boot, then reads single-letter commands over the
 * USB-C console (type `?` for the list). Motor commands drive the Romeo
 * board's four DRV8876 channels in PH/EN mode (EN = PWM speed, PH =
 * direction). Motors auto-stop a few seconds after each command so a
 * forgotten command can't walk the robot off the bench.
 */

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "freddie2";

/* ---------------------------------------------------------------- motors */

#define MOTOR_N       4
#define MOTOR_PWM_HZ  20000                 /* above audible, fine for DRV8876 */
#define MOTOR_PWM_RES LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX ((1 << 10) - 1)

static const struct { int en, ph; } MOTOR_GPIO[MOTOR_N] = {
    { 12, 13 },  /* M1 */
    { 14, 21 },  /* M2 */
    {  9, 10 },  /* M3 */
    { 47, 11 },  /* M4 */
};

/* Wheel position -> motor channel (0..3 = M1..M4) and polarity (+1/-1).
 * Verified with `t` on the wired chassis: M1=FL, M2=FR, M3=RL(back left),
 * M4=RR, and all four spin the same way with no polarity flips. Any
 * future rewiring gets corrected once here so drive() stays sane. */
#define FL_CH 0
#define FR_CH 1
#define RL_CH 2
#define RR_CH 3
#define FL_POL (+1)
#define FR_POL (+1)
#define RL_POL (+1)
#define RR_POL (+1)

#define RUN_DEFAULT_S 3   /* auto-stop delay when a command gives no duration */

#define DEMO_JUMPER_GPIO 48   /* grounded at boot = run the floor demo */

/* Non-blocking poll of stdin; EOF (nothing waiting) clears the error flag
 * so later reads still work. */
static int key_poll(void)
{
    int c = fgetc(stdin);
    if (c == EOF) clearerr(stdin);
    return c;
}

/* Delay in small slices, bailing out early if a key arrives. */
static bool wait_or_key(int ms)
{
    for (int t = 0; t < ms; t += 50) {
        if (key_poll() != EOF) return true;
        int slice = (ms - t < 50) ? ms - t : 50;
        vTaskDelay(pdMS_TO_TICKS(slice));
    }
    return false;
}

/* ------------------------------------------------------------------ i2c */

#define I2C_SDA_GPIO 1
#define I2C_SCL_GPIO 2

static i2c_master_bus_handle_t i2c_bus;  /* NULL if bus init failed */

static void i2c_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = -1,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c bus init failed: %s", esp_err_to_name(err));
        i2c_bus = NULL;
    }
}

/* --------------------------------------------------------------- buzzer */

/* SparkFun Qwiic Buzzer: ATtiny with a register map. Registers 0x03..0x08
 * are freq MSB/LSB (Hz), volume (0-4), duration MSB/LSB (ms, 0 = until
 * stopped), active (1 = sound). Written as one sequential block. */
#define BUZZER_ADDR         0x34
#define BUZZER_REG_ID       0x00
#define BUZZER_ID           0x5E
#define BUZZER_REG_FREQ_MSB 0x03
#define BUZZER_REG_ACTIVE   0x08
#define BUZZER_DEFAULT_HZ   2730   /* the piezo's resonant frequency */

static i2c_master_dev_handle_t buzzer_dev;  /* NULL = not found at boot */

static void buzzer_init(void)
{
    if (!i2c_bus || i2c_master_probe(i2c_bus, BUZZER_ADDR, 100) != ESP_OK) {
        ESP_LOGW(TAG, "buzzer not found on i2c (0x%02X)", BUZZER_ADDR);
        return;
    }
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BUZZER_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(i2c_bus, &cfg, &buzzer_dev) != ESP_OK) {
        buzzer_dev = NULL;
        return;
    }
    uint8_t reg = BUZZER_REG_ID, id = 0;
    i2c_master_transmit_receive(buzzer_dev, &reg, 1, &id, 1, 100);
    ESP_LOGI(TAG, "buzzer found at 0x%02X (id 0x%02X%s)", BUZZER_ADDR, id,
             id == BUZZER_ID ? ", ok" : " — unexpected!");
}

static bool buzzer_beep(int hz, int ms, int vol)
{
    if (!buzzer_dev)
        return false;
    uint8_t cmd[7] = {
        BUZZER_REG_FREQ_MSB,
        hz >> 8, hz & 0xff,
        vol,
        ms >> 8, ms & 0xff,
        1,
    };
    return i2c_master_transmit(buzzer_dev, cmd, sizeof cmd, 100) == ESP_OK;
}

static bool buzzer_stop(void)
{
    if (!buzzer_dev)
        return false;
    uint8_t cmd[2] = { BUZZER_REG_ACTIVE, 0 };
    return i2c_master_transmit(buzzer_dev, cmd, sizeof cmd, 100) == ESP_OK;
}

/* ---------------------------------------------------------------- vocab */

/* Draft sound vocabulary: short square-wave phrases, R2-D2 school of
 * diction — rising = positive/asking, falling = negative/tired. */

#define VOCAB_VOL 4

typedef struct { uint16_t hz, ms, gap; } note_t;

static const note_t PH_HELLO[]    = { {1047,90,20},{1319,90,20},{1568,120,0} };
static const note_t PH_YES[]      = { {1175,80,25},{1760,140,0} };
static const note_t PH_NO[]       = { {494,120,30},{370,180,0} };
static const note_t PH_QUESTION[] = { {880,100,30},{988,80,20},{1480,180,0} };
static const note_t PH_HAPPY[]    = { {1047,70,15},{1319,70,15},{1568,70,15},
                                      {2093,140,20},{1568,70,0} };
static const note_t PH_SAD[]      = { {784,150,40},{659,150,40},{523,260,0} };
static const note_t PH_ALERT[]    = { {2400,90,60},{2400,90,60},{2400,90,0} };
static const note_t PH_TADA[]     = { {523,110,20},{659,110,20},{784,110,20},
                                      {1047,320,0} };

#define PHRASE(name, notes) { name, notes, sizeof(notes) / sizeof(note_t) }
static const struct { const char *name; const note_t *notes; int n; }
PHRASES[] = {
    PHRASE("hello",    PH_HELLO),
    PHRASE("yes",      PH_YES),
    PHRASE("no",       PH_NO),
    PHRASE("question", PH_QUESTION),
    PHRASE("happy",    PH_HAPPY),
    PHRASE("sad",      PH_SAD),
    PHRASE("alert",    PH_ALERT),
    PHRASE("ta-da",    PH_TADA),
};
#define PHRASE_N ((int)(sizeof PHRASES / sizeof PHRASES[0]))

/* Play one phrase; false if the buzzer is missing or a key aborted. */
static bool phrase_play(int idx)
{
    printf("%d: %s\n", idx, PHRASES[idx].name);
    for (int i = 0; i < PHRASES[idx].n; i++) {
        const note_t *n = &PHRASES[idx].notes[i];
        if (!buzzer_beep(n->hz, n->ms, VOCAB_VOL))
            return false;
        if (wait_or_key(n->ms + n->gap)) {
            buzzer_stop();
            return false;
        }
    }
    return true;
}

/* Glide: sweep between two frequencies in geometric steps ~20 ms apart
 * (pitch perception is logarithmic, so equal ratios sound like an even
 * slide). The buzzer runs continuously (duration 0) with the frequency
 * re-written each step — no per-note retrigger, so no gaps — and one
 * stop at the end. */
static bool glide(int from_hz, int to_hz, int ms)
{
    if (!buzzer_dev)
        return false;
    int steps = ms / 20;
    if (steps < 1) steps = 1;
    float hz = from_hz;
    float ratio = powf((float)to_hz / (float)from_hz, 1.0f / (float)steps);
    for (int i = 0; i <= steps; i++) {
        if (!buzzer_beep((int)(hz + 0.5f), 0, VOCAB_VOL))
            return false;
        if (wait_or_key(20)) {
            buzzer_stop();
            return false;
        }
        hz *= ratio;
    }
    buzzer_stop();
    return true;
}

static int rnd_range(int lo, int hi)
{
    return lo + esp_random() % (hi - lo + 1);
}

/* Random babble: random square-wave chirps for a while. Base frequency
 * is picked then shifted up 0-3 octaves so the spread sounds musical
 * rather than uniformly screechy (~200 Hz to ~3.2 kHz). */
static void babble(int secs)
{
    printf("babbling for %d s — any key stops.\n", secs);
    int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;
    while (esp_timer_get_time() < end) {
        int hz = rnd_range(200, 400) << rnd_range(0, 3);
        int ms = rnd_range(40, 250);
        if (!buzzer_beep(hz, ms, VOCAB_VOL))
            return;
        if (wait_or_key(ms + rnd_range(10, 130))) {
            buzzer_stop();
            return;
        }
    }
}

static void vocab_run_through(void)
{
    for (int i = 0; i < PHRASE_N; i++) {
        if (!phrase_play(i))
            return;
        if (wait_or_key(350))
            return;
    }
}

static int motor_pct[MOTOR_N];    /* last commanded values, for `v` */
static int64_t motor_stop_at_us;  /* 0 = no auto-stop pending */

static void motors_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < MOTOR_N; i++) {
        gpio_config_t ph = {
            .pin_bit_mask = 1ULL << MOTOR_GPIO[i].ph,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&ph));

        ledc_channel_config_t ch = {
            .gpio_num = MOTOR_GPIO[i].en,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }
}

static void motor_set(int idx, int pct)
{
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;
    motor_pct[idx] = pct;

    gpio_set_level(MOTOR_GPIO[idx].ph, pct < 0);
    int duty = (pct < 0 ? -pct : pct) * MOTOR_PWM_MAX / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, idx, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, idx);
}

static void motors_stop(void)
{
    for (int i = 0; i < MOTOR_N; i++)
        motor_set(i, 0);
    motor_stop_at_us = 0;
}

/* Arm the auto-stop: secs > 0 stops that many seconds from now, 0 runs
 * until `s`. */
static void run_for(int secs)
{
    if (secs > 0) {
        motor_stop_at_us = esp_timer_get_time() + (int64_t)secs * 1000000;
    } else {
        motor_stop_at_us = 0;
        printf("no auto-stop: running until `s`\n");
    }
}

/* Mecanum mix. x = strafe right, y = forward, r = rotate clockwise, each
 * -100..100; scaled down together if the mix overflows. Only meaningful
 * once the FL/FR/RL/RR mapping above matches the real wiring. */
static void drive(int x, int y, int r)
{
    int fl = y + x + r;
    int fr = y - x - r;
    int rl = y - x + r;
    int rr = y + x - r;

    int m = 100;
    if (abs(fl) > m) m = abs(fl);
    if (abs(fr) > m) m = abs(fr);
    if (abs(rl) > m) m = abs(rl);
    if (abs(rr) > m) m = abs(rr);

    motor_set(FL_CH, FL_POL * fl * 100 / m);
    motor_set(FR_CH, FR_POL * fr * 100 / m);
    motor_set(RL_CH, RL_POL * rl * 100 / m);
    motor_set(RR_CH, RR_POL * rr * 100 / m);
}

/* --------------------------------------------------------------- console */

/* Wheel-mapping test: each channel in turn, forward then reverse, so the
 * FL/FR/RL/RR macros can be corrected to match the real wiring. Any key
 * aborts. */
static void test_wheels(void)
{
    printf("wheel test: watch which wheel moves and which way.\n");
    printf("\"forward\" = PH pin low. any key aborts.\n");
    for (int i = 0; i < MOTOR_N; i++) {
        printf("M%d forward...\n", i + 1);
        motor_set(i, 40);
        if (wait_or_key(1500)) goto abort;
        motor_set(i, 0);
        if (wait_or_key(400)) goto abort;

        printf("M%d reverse...\n", i + 1);
        motor_set(i, -40);
        if (wait_or_key(1500)) goto abort;
        motor_set(i, 0);
        if (wait_or_key(400)) goto abort;
    }
    printf("wheel test done.\n");
    return;
abort:
    motors_stop();
    printf("wheel test aborted.\n");
}

/* Floor test: countdown long enough to unplug the USB lead and set the
 * robot down, then a short mecanum demo — forward/back, strafe both
 * ways, rotate both ways. Verifies the drive() mix and the roller
 * orientation, which the bench tests can't show. */
static void floor_test(int countdown)
{
    printf("floor test in %d s — unplug USB and set me down. any key aborts.\n",
           countdown);
    for (int s = countdown; s > 0; s--) {
        printf("%d...\n", s);
        if (wait_or_key(1000)) {
            printf("floor test aborted.\n");
            return;
        }
    }

    static const struct { int x, y, r, ms; const char *name; } SEQ[] = {
        {   0,  50,   0, 1000, "forward" },
        {   0, -50,   0, 1000, "backward" },
        {  50,   0,   0, 1000, "strafe right" },
        { -50,   0,   0, 1000, "strafe left" },
        {  50,  50,   0, 1000, "diagonal front-right" },
        { -50, -50,   0, 1000, "diagonal back-left" },
        { -50,  50,   0, 1000, "diagonal front-left" },
        {  50, -50,   0, 1000, "diagonal back-right" },
        {   0,  55,  30, 1500, "arc right, out" },
        {   0, -55, -30, 1500, "arc right, back" },
        {   0,  55, -30, 1500, "arc left, out" },
        {   0, -55,  30, 1500, "arc left, back" },
        {   0,   0,  50, 1000, "rotate cw" },
        {   0,   0, -50, 1000, "rotate ccw" },
    };
    for (size_t i = 0; i < sizeof SEQ / sizeof SEQ[0]; i++) {
        printf("%s...\n", SEQ[i].name);
        drive(SEQ[i].x, SEQ[i].y, SEQ[i].r);
        if (wait_or_key(SEQ[i].ms)) goto abort;
        motors_stop();
        if (wait_or_key(500)) goto abort;
    }
    printf("floor test done.\n");
    return;
abort:
    motors_stop();
    printf("floor test aborted.\n");
}

static void help(void)
{
    printf("commands:\n");
    printf("  t                     wheel test: each motor fwd then rev in turn\n");
    printf("  x [secs]              floor test: countdown (default 5), then\n");
    printf("                        fwd/back/strafe/rotate demo — unplug USB first\n");
    printf("  m <1-4> <pct> [secs]  one motor, pct -100..100\n");
    printf("  a <pct> [secs]        all motors\n");
    printf("  d <x> <y> <r> [secs]  mecanum drive: x strafe, y fwd, r rotate\n");
    printf("  b [hz] [ms] [vol]     beep (defaults %d Hz, 300 ms, vol 4;\n",
           BUZZER_DEFAULT_HZ);
    printf("                        ms 0 = continuous, `b 0` stops)\n");
    printf("  p [n]                 play vocab phrase n (0-%d), or all\n",
           PHRASE_N - 1);
    printf("  r [secs]              random babble (default 5 s)\n");
    printf("  g <hz1> <hz2> [ms]    glide between frequencies (default 300 ms)\n");
    printf("  s                     stop all motors\n");
    printf("  v                     show commanded motor values\n");
    printf("  ?                     this help\n");
    printf("[secs] defaults to %d; 0 = run until `s`.\n", RUN_DEFAULT_S);
}

static void handle_line(const char *line)
{
    int a, b, c, secs = RUN_DEFAULT_S;

    switch (line[0]) {
    case '?':
        help();
        break;
    case 's':
        motors_stop();
        printf("stopped.\n");
        break;
    case 'v':
        for (int i = 0; i < MOTOR_N; i++)
            printf("M%d: %d%%\n", i + 1, motor_pct[i]);
        break;
    case 't':
        test_wheels();
        break;
    case 'g': {
        int ms = 300;
        if (sscanf(line + 1, "%d %d %d", &a, &b, &ms) < 2) {
            printf("usage: g <from_hz> <to_hz> [ms]\n");
            break;
        }
        if (a < 40) a = 40;
        if (a > 10000) a = 10000;
        if (b < 40) b = 40;
        if (b > 10000) b = 10000;
        if (ms < 40) ms = 40;
        if (ms > 10000) ms = 10000;
        printf("glide %d -> %d Hz over %d ms\n", a, b, ms);
        if (!glide(a, b, ms))
            printf("(no buzzer or aborted)\n");
        break;
    }
    case 'r':
        a = 5;
        sscanf(line + 1, "%d", &a);
        if (a < 1) a = 1;
        if (a > 60) a = 60;
        babble(a);
        break;
    case 'p':
        if (sscanf(line + 1, "%d", &a) == 1) {
            if (a < 0 || a >= PHRASE_N) {
                printf("phrases are 0-%d\n", PHRASE_N - 1);
                break;
            }
            if (!phrase_play(a))
                printf("(no buzzer or aborted)\n");
        } else {
            vocab_run_through();
        }
        break;
    case 'b': {
        int hz = BUZZER_DEFAULT_HZ, ms = 300, vol = 4;
        sscanf(line + 1, "%d %d %d", &hz, &ms, &vol);
        if (hz < 0) hz = 0;
        if (hz > 0xffff) hz = 0xffff;
        if (ms < 0) ms = 0;
        if (ms > 0xffff) ms = 0xffff;
        if (vol < 0) vol = 0;
        if (vol > 4) vol = 4;
        bool ok;
        if (hz == 0) {
            ok = buzzer_stop();
            printf(ok ? "buzzer stopped.\n" : "no buzzer.\n");
        } else {
            ok = buzzer_beep(hz, ms, vol);
            printf(ok ? "beep: %d Hz, %d ms, vol %d\n" : "no buzzer.\n",
                   hz, ms, vol);
        }
        break;
    }
    case 'x':
        a = 5;
        sscanf(line + 1, "%d", &a);
        if (a < 1) a = 1;
        if (a > 60) a = 60;
        floor_test(a);
        break;
    case 'm':
        if (sscanf(line + 1, "%d %d %d", &a, &b, &secs) < 2 || a < 1 || a > 4) {
            printf("usage: m <1-4> <pct> [secs]\n");
            break;
        }
        motor_set(a - 1, b);
        run_for(secs);
        break;
    case 'a':
        if (sscanf(line + 1, "%d %d", &a, &secs) < 1) {
            printf("usage: a <pct> [secs]\n");
            break;
        }
        for (int i = 0; i < MOTOR_N; i++)
            motor_set(i, a);
        run_for(secs);
        break;
    case 'd':
        if (sscanf(line + 1, "%d %d %d %d", &a, &b, &c, &secs) < 3) {
            printf("usage: d <x> <y> <r> [secs]\n");
            break;
        }
        drive(a, b, c);
        run_for(secs);
        break;
    default:
        printf("? for help\n");
        break;
    }
}

void app_main(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    ESP_LOGI(TAG, "hello, I am Freddie 2");
    ESP_LOGI(TAG, "chip: %s, %d core(s), rev v%d.%d",
             CONFIG_IDF_TARGET, chip.cores,
             chip.revision / 100, chip.revision % 100);
    ESP_LOGI(TAG, "flash: %" PRIu32 " MB", flash_size / (1024 * 1024));
    ESP_LOGI(TAG, "PSRAM free: %u bytes",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    motors_init();
    ESP_LOGI(TAG, "motors ready (VM must be powered for wheels to turn)");

    i2c_init();
    buzzer_init();
    if (buzzer_dev)
        babble(10);

    /* Demo-on-boot jumper: if GPIO 48 (header P18, internal pull-up) is
     * patched to GND at power-on, run the floor demo after a countdown.
     * Lets the robot perform on battery alone, no console attached. */
    gpio_config_t jumper = {
        .pin_bit_mask = 1ULL << DEMO_JUMPER_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&jumper));
    vTaskDelay(pdMS_TO_TICKS(50));
    if (gpio_get_level(DEMO_JUMPER_GPIO) == 0) {
        ESP_LOGI(TAG, "demo jumper is grounded: floor demo after countdown");
        floor_test(10);
    }

    setvbuf(stdin, NULL, _IONBF, 0);
    help();

    char line[96];
    size_t len = 0;
    printf("> ");
    fflush(stdout);

    for (;;) {
        if (motor_stop_at_us && esp_timer_get_time() >= motor_stop_at_us) {
            motors_stop();
            printf("auto-stop.\n> ");
        }

        int ch = key_poll();
        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            putchar('\n');
            line[len] = '\0';
            if (len > 0)
                handle_line(line);
            len = 0;
            printf("> ");
        } else if (ch == 0x7f || ch == '\b') {
            if (len > 0) {
                len--;
                printf("\b \b");
            }
        } else if (len < sizeof(line) - 1 && isprint(ch)) {
            line[len++] = ch;
            putchar(ch);
        }
        fflush(stdout);
    }
}
