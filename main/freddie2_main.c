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
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

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

#define PH_HELLO_INDEX 0

static const seg_t PH_HELLO[]    = { NOTE(880,4,100), REST(30), NOTE(988,4,80), REST(20), GLIDE(1200,1750,4,180) };

#define PHRASE(name, segs) { name, segs, sizeof(segs) / sizeof(seg_t) }
static const struct { const char *name; const seg_t *segs; int n; }
PHRASES[] = {
    PHRASE("hello",    PH_HELLO),
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
 * ~200 Hz to ~3.2 kHz), with the occasional short jittery swoop. False
 * if a key (or missing buzzer) cut it short. */
static bool babble(int secs)
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
            return false;
        if (wait_or_key(rnd_range(10, 130)))
            return false;
    }
    return true;
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

/* ----------------------------------------------------------------- watch */

/* Idle behavior: sit still and watch the radar; when somebody comes
 * close, greet them with a burst of babbles, then stay quiet until they
 * have clearly gone before re-arming. Polled from the main loop between
 * console commands — the distance is too noisy for anything finer than
 * "somebody is close / nobody is", so the debounce counters do the work:
 * a few consecutive close frames to greet, a few clear seconds to
 * re-arm. Each poll drains the UART backlog and judges the freshest
 * frame only. */

#define WATCH_POLL_MS  300   /* radar look interval while idle */
#define WATCH_CLOSE_CM 120   /* "visiting close" */
#define WATCH_IN_N       3   /* consecutive close polls to greet (~1 s) */
#define WATCH_OUT_N     15   /* consecutive clear polls to re-arm (~5 s) */

static bool watch_on = true;
static bool watch_occupied;   /* greeted; waiting for the visitor to leave */
static int watch_streak;
static int64_t watch_next_us;

/* The greeting: 2-5 babbles of 1-3 s each, breathing pauses between. */
static void watch_greet(void)
{
    phrase_play(PH_HELLO_INDEX);
    int n = rnd_range(2, 5);
    printf("visitor! %d babbles:\n", n);
    for (int i = 0; i < n; i++) {
        if (!babble(rnd_range(1, 3)))
            return;
        if (i < n - 1 && wait_or_key(rnd_range(300, 900)))
            return;
    }
}

/* One watch tick; true if it printed anything (so the caller can restore
 * the prompt). */
static bool watch_poll(void)
{
    if (!watch_on || !ld_ok)
        return false;
    int64_t now = esp_timer_get_time();
    if (now < watch_next_us)
        return false;
    watch_next_us = now + (int64_t)WATCH_POLL_MS * 1000;

    uart_flush_input(LD_UART);
    ld_report_t r;
    if (!ld_read(&r, 150))
        return false;
    bool close = (r.state & 3) && r.det_cm > 0 && r.det_cm <= WATCH_CLOSE_CM;

    if (!watch_occupied) {
        watch_streak = close ? watch_streak + 1 : 0;
        if (watch_streak >= WATCH_IN_N) {
            watch_greet();
            watch_occupied = true;
            watch_streak = 0;
            return true;
        }
    } else {
        watch_streak = close ? 0 : watch_streak + 1;
        if (watch_streak >= WATCH_OUT_N) {
            printf("visitor gone — watching again.\n");
            watch_occupied = false;
            watch_streak = 0;
            return true;
        }
    }
    return false;
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

/* --------------------------------------------------------------- esp-now */

/* Freddie 1 broadcasts a 21-byte ESP-NOW frame to ff:ff:ff:ff:ff:ff every
 * 100 ms, unencrypted, on channel 1 — no association, no AP, no IP stack.
 * `n` brings the radio up just long enough to listen, then takes it down
 * again: an unassociated station sits in receive at ~80 mA whether or not
 * anything is being heard, and Freddie 2's boot is meant to stay quiet.
 *
 * Wire format, little-endian, packed; a later ver may append fields but
 * the ones below never move, so `ver` mismatches are reported, not
 * rejected. */
#define BEACON_CHANNEL 1
#define BEACON_MAGIC   "FRED"
#define BEACON_VER     1
#define BEACON_LISTEN_S 5   /* default listen window for `n` */

typedef struct __attribute__((packed)) {
    char     magic[4];   /* "FRED", not NUL-terminated */
    uint8_t  ver;
    char     id[8];      /* "freddie", NUL-padded */
    uint32_t seq;        /* from 0 at boot; gaps are frames we lost */
    uint32_t up_ms;      /* sender's uptime, so a reboot reads as one */
} beacon_t;

/* The receive callback runs in the WiFi task, where printf and anything
 * else slow doesn't belong, so it only parks frames in this ring and the
 * console loop does the talking. Single producer, single consumer, free
 * running indices: no lock needed. */
#define RX_RING 32
typedef struct {
    uint8_t  mac[6];
    int8_t   rssi;
    beacon_t b;
} rx_frame_t;

static rx_frame_t rx_ring[RX_RING];
static volatile uint32_t rx_head, rx_tail;
static volatile uint32_t rx_dropped;   /* ring was full: console too slow */
static volatile uint32_t rx_alien;     /* ESP-NOW frames that weren't Freddie's */

static bool radio_ready;   /* WiFi stack initialised (once, lazily) */

static bool radio_fail(const char *what, esp_err_t err)
{
    printf("radio: %s failed (%s)\n", what, esp_err_to_name(err));
    return false;
}

/* NVS backs both radios' PHY calibration; whoever comes up first inits it. */
static bool nvs_init_once(void)
{
    static bool done;
    if (done) return true;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return radio_fail("nvs init", err);
    done = true;
    return true;
}

static void beacon_rx(const esp_now_recv_info_t *info,
                      const uint8_t *data, int len)
{
    if (len < (int)sizeof(beacon_t) || memcmp(data, BEACON_MAGIC, 4) != 0) {
        rx_alien++;
        return;
    }
    uint32_t head = rx_head;
    if (head - rx_tail >= RX_RING) {
        rx_dropped++;
        return;
    }
    rx_frame_t *f = &rx_ring[head % RX_RING];
    memcpy(f->mac, info->src_addr, sizeof(f->mac));
    f->rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
    memcpy(&f->b, data, sizeof(f->b));
    rx_head = head + 1;   /* publish last */
}

/* One-time: NVS (WiFi keeps its PHY calibration there), netif, event loop
 * and the WiFi driver itself. Left standing once up — it's the radio, not
 * the driver, that costs current. */
static bool radio_init(void)
{
    if (radio_ready) return true;
    if (!nvs_init_once()) return false;

    esp_err_t err;
    if ((err = esp_netif_init()) != ESP_OK)
        return radio_fail("netif init", err);
    if ((err = esp_event_loop_create_default()) != ESP_OK)
        return radio_fail("event loop", err);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if ((err = esp_wifi_init(&cfg)) != ESP_OK)
        return radio_fail("wifi init", err);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);   /* nothing to remember */
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK)
        return radio_fail("wifi mode", err);

    radio_ready = true;
    return true;
}

/* Channel is a property of a running radio, so it's re-stated every time.
 * Receiving a broadcast needs no peer registration — just a callback. */
static bool radio_listen_start(void)
{
    esp_err_t err;
    if (!radio_init()) return false;
    if ((err = esp_wifi_start()) != ESP_OK)
        return radio_fail("wifi start", err);
    if ((err = esp_wifi_set_channel(BEACON_CHANNEL, WIFI_SECOND_CHAN_NONE))
            != ESP_OK) {
        esp_wifi_stop();
        return radio_fail("set channel", err);
    }
    if ((err = esp_now_init()) != ESP_OK) {
        esp_wifi_stop();
        return radio_fail("esp-now init", err);
    }
    if ((err = esp_now_register_recv_cb(beacon_rx)) != ESP_OK) {
        esp_now_deinit();
        esp_wifi_stop();
        return radio_fail("register rx", err);
    }
    return true;
}

static void radio_listen_stop(void)
{
    esp_now_unregister_recv_cb();
    esp_now_deinit();
    esp_wifi_stop();   /* the radio genuinely goes down, not just the callback */
}

/* Listen for `secs` seconds (or until a key), printing every Freddie frame
 * as it lands and a summary at the end. Frames are only expected while the
 * other robot's beacon is on — silence here is as likely to be his `n` as
 * a problem at this end. */
static void beacon_listen(int secs)
{
    if (!radio_listen_start()) return;

    rx_head = rx_tail = 0;
    rx_dropped = rx_alien = 0;

    printf("listening for \"FRED\" on channel %d for %d s — any key stops.\n",
           BEACON_CHANNEL, secs);

    int64_t end_us = esp_timer_get_time() + (int64_t)secs * 1000000;
    uint32_t heard = 0, lost = 0, gaps = 0, reboots = 0, last_seq = 0;
    int rssi_min = 127, rssi_max = -128, rssi_sum = 0;
    bool have_last = false, said_who = false;

    while (esp_timer_get_time() < end_us) {
        if (key_poll() != EOF) break;

        while (rx_tail != rx_head) {
            rx_frame_t f = rx_ring[rx_tail % RX_RING];
            rx_tail++;

            if (!said_who) {
                printf("heard \"%.8s\" (ver %u) from "
                       "%02x:%02x:%02x:%02x:%02x:%02x\n",
                       f.b.id, f.b.ver, f.mac[0], f.mac[1], f.mac[2],
                       f.mac[3], f.mac[4], f.mac[5]);
                if (f.b.ver != BEACON_VER)
                    printf("(expected ver %d — fields may have moved)\n",
                           BEACON_VER);
                said_who = true;
            }

            if (have_last) {
                if (f.b.seq < last_seq) {
                    reboots++;
                    printf("  -- seq restarted: he rebooted\n");
                } else if (f.b.seq > last_seq + 1) {
                    gaps++;
                    lost += f.b.seq - last_seq - 1;
                }
            }
            last_seq = f.b.seq;
            have_last = true;

            heard++;
            if (f.rssi < rssi_min) rssi_min = f.rssi;
            if (f.rssi > rssi_max) rssi_max = f.rssi;
            rssi_sum += f.rssi;

            printf("seq %-6lu up %6lu.%lu s  rssi %4d\n",
                   (unsigned long)f.b.seq,
                   (unsigned long)(f.b.up_ms / 1000),
                   (unsigned long)((f.b.up_ms % 1000) / 100),
                   f.rssi);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    radio_listen_stop();

    if (heard == 0) {
        printf("nothing heard in %d s — is his beacon on, and on channel %d?\n",
               secs, BEACON_CHANNEL);
    } else {
        printf("%lu frames, %lu lost in %lu gap(s), rssi %d..%d avg %d\n",
               (unsigned long)heard, (unsigned long)lost,
               (unsigned long)gaps, rssi_min, rssi_max,
               rssi_sum / (int)heard);
        if (reboots)
            printf("%lu reboot(s) seen.\n", (unsigned long)reboots);
    }
    if (rx_dropped)
        printf("(%lu frames dropped: console couldn't keep up)\n",
               (unsigned long)rx_dropped);
    if (rx_alien)
        printf("(%lu other esp-now frame(s) on this channel, not Freddie's)\n",
               (unsigned long)rx_alien);
    printf("radio down.\n");
}

/* ------------------------------------------------------ ble console link */

/* A NimBLE GATT server that exposes the console to Web Bluetooth: one
 * service, one write characteristic, and every write is a console line
 * in exactly the prompt's syntax. The matching web app lives in docs/
 * (served by GitHub Pages), so a phone gets the whole vocabulary — hello,
 * babble, driving — for free as commands are added here.
 *
 * Advertising is off at boot (quiet-boot rule); `l` toggles it. Writes
 * arrive in the NimBLE host task, where slow work doesn't belong, so
 * they're parked in a one-line letterbox for the main loop — same
 * discipline as the esp-now ring. Two safety properties: motor commands
 * keep their auto-stop, and a disconnect stops the motors outright, so
 * a phone wandering out of range can't leave the robot driving. */

#define BLE_NAME "freddie2"

/* Service 46524544-4449-4532-8000-000000000001 — "FRED","DIE2" in ASCII
 * hex — and command characteristic ...0002. NimBLE takes 128-bit UUIDs
 * as bytes, least significant first. Must match docs/index.html. */
static const ble_uuid128_t BLE_SVC_UUID =
    BLE_UUID128_INIT(0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
                     0x32, 0x45, 0x49, 0x44, 0x44, 0x45, 0x52, 0x46);
static const ble_uuid128_t BLE_CMD_UUID =
    BLE_UUID128_INIT(0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
                     0x32, 0x45, 0x49, 0x44, 0x44, 0x45, 0x52, 0x46);

static bool ble_started;    /* NimBLE stack up (once, lazily) */
static bool ble_synced;     /* host ready; addresses sorted out */
static bool ble_want_adv;   /* `l` wants us advertising */
static uint8_t ble_addr_type;
static uint16_t ble_conn = BLE_HS_CONN_HANDLE_NONE;

/* Letterbox, host task -> main loop. The host task only writes when
 * `full` is clear and sets it last; the main loop copies the line out
 * before clearing. One slot is plenty: the main loop drains it every
 * ~20 ms, and a stream of drive vectors wants latest-wins anyway. */
static char ble_line[96];
static volatile bool ble_line_full;
static volatile int8_t ble_note;   /* +1 connected, -1 disconnected */

static int ble_gap_event(struct ble_gap_event *ev, void *arg);

static void ble_advertise(void)
{
    struct ble_hs_adv_fields fields = { 0 }, rsp = { 0 };

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)BLE_NAME;
    fields.name_len = strlen(BLE_NAME);
    fields.name_is_complete = 1;

    /* The 128-bit service UUID doesn't fit next to the name in the
     * 31-byte advertisement, so it rides in the scan response. */
    rsp.uuids128 = (ble_uuid128_t *)&BLE_SVC_UUID;
    rsp.num_uuids128 = 1;
    rsp.uuids128_is_complete = 1;

    struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc == 0) rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc == 0) rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER,
                                        &params, ble_gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY)
        printf("ble: advertise failed (rc %d)\n", rc);
}

static int ble_gap_event(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            ble_conn = ev->connect.conn_handle;
            ble_note = 1;
        } else if (ble_want_adv) {
            ble_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ble_conn = BLE_HS_CONN_HANDLE_NONE;
        motors_stop();   /* dead man's rule: no phone, no motion */
        ble_note = -1;
        if (ble_want_adv)
            ble_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (ble_want_adv)
            ble_advertise();
        break;
    }
    return 0;
}

static int ble_cmd_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
        return BLE_ATT_ERR_UNLIKELY;
    if (ble_line_full)   /* main loop busy; drop, latest-wins */
        return 0;

    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, ble_line, sizeof ble_line - 1, &len) != 0)
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    while (len && (ble_line[len - 1] == '\r' || ble_line[len - 1] == '\n'))
        len--;
    ble_line[len] = '\0';
    ble_line_full = true;   /* publish last */
    return 0;
}

static const struct ble_gatt_svc_def BLE_SVCS[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &BLE_SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &BLE_CMD_UUID.u,
                .access_cb = ble_cmd_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 },
        },
    },
    { 0 },
};

static void ble_on_reset(int reason)
{
    ble_synced = false;
}

static void ble_on_sync(void)
{
    if (ble_hs_util_ensure_addr(0) != 0 ||
        ble_hs_id_infer_auto(0, &ble_addr_type) != 0)
        return;
    ble_synced = true;
    if (ble_want_adv)
        ble_advertise();
}

static void ble_host_task(void *param)
{
    nimble_port_run();   /* returns only if the stack is stopped */
    nimble_port_freertos_deinit();
}

static bool ble_start(void)
{
    if (ble_started) return true;
    if (!nvs_init_once()) return false;

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        printf("ble: nimble init failed (%s)\n", esp_err_to_name(err));
        return false;
    }
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(BLE_SVCS);
    if (rc == 0) rc = ble_gatts_add_svcs(BLE_SVCS);
    if (rc == 0) rc = ble_svc_gap_device_name_set(BLE_NAME);
    if (rc != 0) {
        printf("ble: gatt setup failed (rc %d)\n", rc);
        return false;
    }

    nimble_port_freertos_init(ble_host_task);
    ble_started = true;
    return true;
}

static void handle_line(const char *line);

/* One ble tick from the main loop: relay connect/disconnect notes and
 * run a parked command line. True if it printed anything. */
static bool ble_poll(void)
{
    bool printed = false;

    int note = ble_note;
    if (note) {
        ble_note = 0;
        printf(note > 0 ? "ble: connected.\n"
                        : "ble: disconnected — motors stopped.\n");
        printed = true;
    }

    if (ble_line_full) {
        char line[sizeof ble_line];
        memcpy(line, ble_line, sizeof line);
        ble_line_full = false;
        printf("ble> %s\n", line);
        /* h and n hog the loop until a console key; l is the link's own
         * switch. Those stay console-only. */
        if (line[0] == 'h' || line[0] == 'n' || line[0] == 'l')
            printf("(console-only command, ignored)\n");
        else
            handle_line(line);
        printed = true;
    }
    return printed;
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
    printf("  n [secs]              listen for Freddie 1's esp-now beacon\n");
    printf("                        (default %d s; radio is off otherwise)\n",
           BEACON_LISTEN_S);
    printf("  w                     toggle the visitor watch (now %s)\n",
           watch_on ? "on" : "off");
    printf("  l                     toggle the ble link for the web app\n");
    printf("                        (advertises as \"%s\"; off at boot)\n",
           BLE_NAME);
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
    case 'l':
        if (!ble_started) {
            ble_want_adv = true;   /* before start: sync may beat us */
            if (ble_start())
                printf("ble on — advertising as \"%s\".\n", BLE_NAME);
            else
                ble_want_adv = false;
        } else if (ble_want_adv) {
            ble_want_adv = false;
            ble_gap_adv_stop();
            if (ble_conn != BLE_HS_CONN_HANDLE_NONE)
                ble_gap_terminate(ble_conn, BLE_ERR_REM_USER_CONN_TERM);
            printf("ble off.\n");
        } else {
            ble_want_adv = true;
            if (ble_synced)
                ble_advertise();
            printf("ble on — advertising as \"%s\".\n", BLE_NAME);
        }
        break;
    case 'w':
        watch_on = !watch_on;
        watch_occupied = false;
        watch_streak = 0;
        printf("watch %s.\n", watch_on ? "on" : "off");
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
    case 'n':
        a = BEACON_LISTEN_S;
        sscanf(line + 1, "%d", &a);
        if (a < 1) a = 1;
        if (a > 300) a = 300;
        beacon_listen(a);
        break;
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

    phrase_play(PH_HELLO_INDEX);

    char line[96];
    size_t len = 0;
    printf("> ");
    fflush(stdout);

    for (;;) {
        if (motor_stop_at_us && esp_timer_get_time() >= motor_stop_at_us) {
            motors_stop();
            printf("auto-stop.\n> ");
        }

        if (watch_poll()) {
            printf("> ");
            fflush(stdout);
        }

        if (ble_poll()) {
            printf("> ");
            fflush(stdout);
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
