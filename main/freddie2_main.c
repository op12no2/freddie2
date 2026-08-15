/* Freddie 2 — motor bench-test console.
 *
 * Prints chip info at boot, then reads single-letter commands over the
 * USB-C console (type `?` for the list). Motor commands drive the Romeo
 * board's four DRV8876 channels in PH/EN mode (EN = PWM speed, PH =
 * direction). Motors auto-stop a few seconds after each command so a
 * forgotten command can't walk the robot off the bench.
 */

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
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
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

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
