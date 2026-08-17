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
#include "driver/uart.h"
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
#define FL_CH 3
#define FR_CH 2
#define RL_CH 1
#define RR_CH 0
#define FL_POL (-1)
#define FR_POL (-1)
#define RL_POL (-1)
#define RR_POL (-1)

#define RUN_DEFAULT_S 3   /* auto-stop delay when a command gives no duration */

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

/* -------------------------------------------------- ld2410 human radar */

/* Hi-Link LD2410C 24 GHz presence sensor on UART1 (256000 8N1). It
 * streams a ~10 Hz binary report: header F4 F3 F2 F1, 2-byte length,
 * payload, tail F8 F7 F6 F5. Normal-mode payload: type 0x02, 0xAA,
 * state (bit0 moving, bit1 still), moving dist cm (LE16) + energy,
 * still dist + energy, overall detect dist. */

#define LD_UART    UART_NUM_1
#define LD_TX_GPIO 40   /* ESP TX -> sensor RX */
#define LD_RX_GPIO 41   /* ESP RX <- sensor TX */
#define LD_BAUD    256000

static bool ld_ok;  /* UART driver up (says nothing about wiring) */

typedef struct {
    uint8_t state;
    uint16_t mov_cm, still_cm, det_cm;
    uint8_t mov_energy, still_energy;
} ld_report_t;

static void ld_init(void)
{
    uart_config_t cfg = {
        .baud_rate = LD_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(LD_UART, 2048, 0, 0, NULL, 0) != ESP_OK ||
        uart_param_config(LD_UART, &cfg) != ESP_OK ||
        uart_set_pin(LD_UART, LD_TX_GPIO, LD_RX_GPIO,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGW(TAG, "ld2410 uart init failed");
        return;
    }
    ld_ok = true;
    ESP_LOGI(TAG, "ld2410 uart ready (TX GPIO %d, RX GPIO %d)",
             LD_TX_GPIO, LD_RX_GPIO);
}

/* Scan the UART stream for one well-formed normal-mode report. */
static bool ld_read(ld_report_t *out, int wait_ms)
{
    if (!ld_ok)
        return false;
    static const uint8_t HDR[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
    static const uint8_t TAIL[4] = { 0xF8, 0xF7, 0xF6, 0xF5 };
    int64_t end = esp_timer_get_time() + (int64_t)wait_ms * 1000;
    int match = 0, len = 0, got = -2;  /* got < 0: reading length bytes */
    uint8_t payload[32];

    while (esp_timer_get_time() < end) {
        uint8_t b;
        if (uart_read_bytes(LD_UART, &b, 1, pdMS_TO_TICKS(20)) != 1)
            continue;
        if (match < 4) {                       /* hunting for header */
            match = (b == HDR[match]) ? match + 1 : (b == HDR[0] ? 1 : 0);
            len = 0;
            got = -2;
        } else if (got < 0) {                  /* two length bytes, LE */
            len |= b << (8 * (2 + got));
            if (++got == 0 && (len < 11 || len > (int)sizeof(payload))) {
                match = 0;                     /* not a normal report */
            }
        } else if (got < len) {                /* payload */
            payload[got++] = b;
        } else {                               /* four tail bytes */
            if (b != TAIL[got - len]) {
                match = 0;
                continue;
            }
            if (++got - len == 4) {
                match = 0;
                if (payload[0] != 0x02 || payload[1] != 0xAA)
                    continue;                  /* engineering-mode etc. */
                out->state = payload[2];
                out->mov_cm = payload[3] | payload[4] << 8;
                out->mov_energy = payload[5];
                out->still_cm = payload[6] | payload[7] << 8;
                out->still_energy = payload[8];
                out->det_cm = payload[9] | payload[10] << 8;
                return true;
            }
        }
    }
    return false;
}

/* -------------------------------------------------------------- segments */

/* Everything the buzzer says is a sequence of segments. A segment glides
 * pitch hz0->hz1 and volume vol0->vol1 over ms, in SEG_STEP_MS steps —
 * a steady note is a glide that goes nowhere, a rest is hz0 == 0. On top
 * of the glide: vib = periodic pitch wobble (% depth, SEG_VIB_HZ rate),
 * trem = volume wobble (levels), jit = fresh random pitch scatter every
 * step (%), the R2-D2 warble. All three stack. */

#define SEG_STEP_MS 20
#define SEG_VIB_HZ  6

typedef struct {
    uint16_t hz0, hz1;   /* pitch start -> end; 0 = rest */
    uint8_t  vol0, vol1; /* volume start -> end (0-4) */
    uint16_t ms;
    uint8_t  vib;        /* vibrato depth, % of pitch */
    uint8_t  trem;       /* tremolo depth, volume levels */
    uint8_t  jit;        /* per-step random pitch scatter, % */
} seg_t;

#define SEG(a, b, v0, v1, ms, vib, trem, jit) { a, b, v0, v1, ms, vib, trem, jit }
#define NOTE(hz, vol, ms)          SEG(hz, hz, vol, vol, ms, 0, 0, 0)
#define REST(ms)                   SEG(0, 0, 0, 0, ms, 0, 0, 0)
#define GLIDE(a, b, vol, ms)       SEG(a, b, vol, vol, ms, 0, 0, 0)
#define SWOOP(a, b, v0, v1, ms)    SEG(a, b, v0, v1, ms, 0, 0, 0)
#define WARBLE(a, b, vol, ms, jit) SEG(a, b, vol, vol, ms, 0, 0, jit)
#define VIB_NOTE(hz, vol, ms, d)   SEG(hz, hz, vol, vol, ms, d, 0, 0)

static int rnd_range(int lo, int hi)
{
    return lo + esp_random() % (hi - lo + 1);
}

/* Play one segment; false if the buzzer is missing or a key aborted. */
static bool seg_play(const seg_t *s)
{
    if (s->hz0 == 0)
        return !wait_or_key(s->ms);
    if (!buzzer_dev)
        return false;

    int steps = s->ms / SEG_STEP_MS;
    if (steps < 1) steps = 1;
    float hz = s->hz0;
    float ratio = powf((float)s->hz1 / (float)s->hz0, 1.0f / (float)steps);

    for (int i = 0; i <= steps; i++) {
        float f = hz;
        if (s->vib)
            f *= 1.0f + s->vib / 100.0f *
                 sinf(6.2831853f * SEG_VIB_HZ * i * SEG_STEP_MS / 1000.0f);
        if (s->jit)
            f *= 1.0f + s->jit / 100.0f * (rnd_range(-1000, 1000) / 1000.0f);
        if (f < 40) f = 40;
        if (f > 10000) f = 10000;

        float t = (float)i / (float)steps;
        int vol = (int)(s->vol0 + (s->vol1 - (float)s->vol0) * t + 0.5f);
        if (s->trem && ((i * SEG_STEP_MS * SEG_VIB_HZ * 2 / 1000) & 1))
            vol -= s->trem;
        if (vol < 0) vol = 0;
        if (vol > 4) vol = 4;

        if (!buzzer_beep((int)(f + 0.5f), 0, vol))
            return false;
        if (wait_or_key(SEG_STEP_MS)) {
            buzzer_stop();
            return false;
        }
        hz *= ratio;
    }
    buzzer_stop();
    return true;
}

/* ---------------------------------------------------------------- vocab */

/* Draft sound vocabulary, R2-D2 school of diction — rising = positive/
 * asking, falling = negative/tired. All timings are draft; tuning them
 * is an ongoing pastime. */

static const seg_t PH_HELLO[]    = { NOTE(1047,4,90), REST(20),
                                     NOTE(1319,4,90), REST(20),
                                     NOTE(1568,4,120) };
static const seg_t PH_YES[]      = { NOTE(1175,4,80), REST(25),
                                     NOTE(1760,4,140) };
static const seg_t PH_NO[]       = { NOTE(494,4,120), REST(30),
                                     NOTE(370,4,180) };
static const seg_t PH_QUESTION[] = { NOTE(880,4,100), REST(30),
                                     NOTE(988,4,80), REST(20),
                                     GLIDE(1200,1750,4,180) };
static const seg_t PH_HAPPY[]    = { NOTE(1047,4,70), REST(15),
                                     NOTE(1319,4,70), REST(15),
                                     NOTE(1568,4,70), REST(15),
                                     NOTE(2093,4,140), REST(20),
                                     NOTE(1568,4,70) };
static const seg_t PH_SAD[]      = { NOTE(784,4,150), REST(40),
                                     NOTE(659,4,150), REST(40),
                                     SWOOP(523,440,4,3,260) };
static const seg_t PH_ALERT[]    = { NOTE(2400,4,90), REST(60),
                                     NOTE(2400,4,90), REST(60),
                                     NOTE(2400,4,90) };
static const seg_t PH_TADA[]     = { NOTE(523,4,110), REST(20),
                                     NOTE(659,4,110), REST(20),
                                     NOTE(784,4,110), REST(20),
                                     VIB_NOTE(1047,4,320,3) };
static const seg_t PH_UHOH[]     = { NOTE(587,4,200), REST(80),
                                     SWOOP(392,392,4,1,1000) };
static const seg_t PH_SCREAM[]   = { VIB_NOTE(2730,4,1000,3) };
static const seg_t PH_LOOK[]     = { SEG(2000,3000,4,4,100,50,1,50) };
static const seg_t PH_CONTENT[]  = { SEG(200,200,4,4,1000,10,1,10)};
static const seg_t PH_LAUGH[]    = { SEG(2000,3000,4,3,1000,10,1,10)};

#define PHRASE(name, segs) { name, segs, sizeof(segs) / sizeof(seg_t) }
static const struct { const char *name; const seg_t *segs; int n; }
PHRASES[] = {
    PHRASE("hello",    PH_HELLO),
    PHRASE("yes",      PH_YES),
    PHRASE("no",       PH_NO),
    PHRASE("question", PH_QUESTION),
    PHRASE("happy",    PH_HAPPY),
    PHRASE("sad",      PH_SAD),
    PHRASE("alert",    PH_ALERT),
    PHRASE("ta-da",    PH_TADA),
    PHRASE("uh-oh",    PH_UHOH),
    PHRASE("scream",   PH_SCREAM),
    PHRASE("look",     PH_LOOK),
    PHRASE("content",  PH_CONTENT),
    PHRASE("laugh",    PH_LAUGH),
};
#define PHRASE_N ((int)(sizeof PHRASES / sizeof PHRASES[0]))

static bool phrase_play(int idx)
{
    printf("%d: %s\n", idx, PHRASES[idx].name);
    for (int i = 0; i < PHRASES[idx].n; i++)
        if (!seg_play(&PHRASES[idx].segs[i]))
            return false;
    return true;
}

/* Random babble: a generator emitting random segments — mostly quick
 * chirps (base pitch shifted up 0-3 octaves so the spread sounds musical,
 * ~200 Hz to ~3.2 kHz), with the occasional short jittery swoop. */
static void babble(int secs)
{
    printf("babbling for %d s — any key stops.\n", secs);
    int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;
    while (esp_timer_get_time() < end) {
        int hz = rnd_range(200, 400) << rnd_range(0, 3);
        seg_t s;
        if (esp_random() % 5 == 0) {
            int hz2 = rnd_range(200, 400) << rnd_range(0, 3);
            s = (seg_t)WARBLE(hz, hz2, 4, rnd_range(80, 180), 4);
        } else {
            s = (seg_t)NOTE(hz, 4, rnd_range(40, 250));
        }
        if (!seg_play(&s))
            return;
        if (wait_or_key(rnd_range(10, 130)))
            return;
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
    printf("  q <hz0> <hz1> [vol0] [vol1] [ms] [vib%%] [trem] [jit%%]\n");
    printf("                        raw segment (hz0 0 = rest; defaults 4 4 200 0 0 0)\n");
    printf("  s                     stop all motors\n");
    printf("  v                     show commanded motor values\n");
    printf("  h                     stream human-radar readings until a key\n");
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
        seg_t s = GLIDE(a, b, 4, ms);
        if (!seg_play(&s))
            printf("(no buzzer or aborted)\n");
        break;
    }
    case 'h': {
        printf("human radar — any key stops.\n");
        bool seen = false;
        while (key_poll() == EOF) {
            ld_report_t r;
            /* The sensor streams ~10 reports/s but we display ~4/s;
             * drop the backlog each pass or the readout lags reality
             * by however much the UART ring buffer holds (~10 s). */
            uart_flush_input(LD_UART);
            if (!ld_read(&r, 500)) {
                if (!seen) {
                    printf("no data — check wiring/power.\n");
                    break;
                }
                continue;
            }
            seen = true;
            static const char *STATES[] =
                { "nobody", "moving", "still", "moving+still" };
            printf("%-12s det %3u cm | move %3u cm e%-3u | still %3u cm e%-3u\n",
                   STATES[r.state & 3], r.det_cm,
                   r.mov_cm, r.mov_energy, r.still_cm, r.still_energy);
            wait_or_key(250);
        }
        break;
    }
    case 'q': {
        seg_t s = { 0, 0, 4, 4, 200, 0, 0, 0 };
        int hz0, hz1, v0 = 4, v1 = 4, ms = 200, vib = 0, trem = 0, jit = 0;
        if (sscanf(line + 1, "%d %d %d %d %d %d %d %d",
                   &hz0, &hz1, &v0, &v1, &ms, &vib, &trem, &jit) < 2) {
            printf("usage: q <hz0> <hz1> [vol0] [vol1] [ms] [vib%%] [trem] [jit%%]\n");
            break;
        }
        s.hz0 = hz0 < 0 ? 0 : (hz0 > 10000 ? 10000 : hz0);
        s.hz1 = hz1 < 40 ? 40 : (hz1 > 10000 ? 10000 : hz1);
        s.vol0 = v0 < 0 ? 0 : (v0 > 4 ? 4 : v0);
        s.vol1 = v1 < 0 ? 0 : (v1 > 4 ? 4 : v1);
        s.ms = ms < 20 ? 20 : (ms > 10000 ? 10000 : ms);
        s.vib = vib < 0 ? 0 : (vib > 50 ? 50 : vib);
        s.trem = trem < 0 ? 0 : (trem > 4 ? 4 : trem);
        s.jit = jit < 0 ? 0 : (jit > 100 ? 100 : jit);
        printf("seg %u->%u Hz vol %u->%u %u ms vib %u trem %u jit %u\n",
               s.hz0, s.hz1, s.vol0, s.vol1, s.ms, s.vib, s.trem, s.jit);
        if (!seg_play(&s))
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
    ld_init();

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
