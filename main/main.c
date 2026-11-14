#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"

/* ---------- Config ---------- */
#define PIN_LED1   GPIO_NUM_2   // Red    - Task A
#define PIN_LED2   GPIO_NUM_3   // Green  - Task B
#define PIN_LED3   GPIO_NUM_4   // Yellow - Task C
#define PIN_SDA    8
#define PIN_SCL    9
#define OLED_ADDR  0x3C
#define OLED_W     128
#define OLED_H     32
#define SCALE      2
#define TASK_PRIO  1            // beacon tasks: all equal
#define DISP_PRIO  5            // display task: higher, blocks on queue
#define BLINKS     5

#define MAXSW  200
#define MAXEV  100

#define MSG_DONE  3

/* ---------- OLED (SSD1306 128x32) ---------- */
static i2c_master_dev_handle_t oled;
static uint8_t fb[OLED_W * OLED_H / 8];
static uint8_t tx[1 + OLED_W * OLED_H / 8];

typedef struct { char c; uint8_t d[5]; } glyph_t;
static const glyph_t FONT[] = {
    {' ',{0x00,0x00,0x00,0x00,0x00}},
    {'A',{0x7E,0x11,0x11,0x11,0x7E}}, {'B',{0x7F,0x49,0x49,0x49,0x36}},
    {'C',{0x3E,0x41,0x41,0x41,0x22}}, {'D',{0x7F,0x41,0x41,0x22,0x1C}},
    {'E',{0x7F,0x49,0x49,0x49,0x41}}, {'G',{0x3E,0x41,0x49,0x49,0x7A}},
    {'I',{0x00,0x41,0x7F,0x41,0x00}}, {'K',{0x7F,0x08,0x14,0x22,0x41}},
    {'L',{0x7F,0x40,0x40,0x40,0x40}}, {'N',{0x7F,0x04,0x08,0x10,0x7F}},
    {'O',{0x3E,0x41,0x41,0x41,0x3E}}, {'R',{0x7F,0x09,0x19,0x29,0x46}},
    {'S',{0x46,0x49,0x49,0x49,0x31}}, {'T',{0x01,0x01,0x7F,0x01,0x01}},
    {'U',{0x3F,0x40,0x40,0x40,0x3F}},
};

static const uint8_t *glyph_lookup(char c) {
    for (size_t i = 0; i < sizeof(FONT) / sizeof(FONT[0]); i++)
        if (FONT[i].c == c) return FONT[i].d;
    return FONT[0].d;
}

static void oled_init(i2c_master_bus_handle_t bus) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = OLED_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &oled));

    static const uint8_t init_seq[] = {
        0x00,
        0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x02,
        0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x2E,
        0x21, 0x00, OLED_W - 1,            // column window, set once
        0x22, 0x00, (OLED_H / 8) - 1,      // page window, set once
        0xAF,
    };
    ESP_ERROR_CHECK(i2c_master_transmit(oled, init_seq, sizeof(init_seq), 100));
}

static void oled_flush(void) {
    tx[0] = 0x40;
    memcpy(&tx[1], fb, sizeof(fb));
    i2c_master_transmit(oled, tx, sizeof(tx), 100);
}

static void fb_pixel(int x, int y) {
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    fb[x + (y / 8) * OLED_W] |= (1 << (y % 8));
}

static void draw_char(int x, int y, char c) {
    const uint8_t *g = glyph_lookup(c);
    for (int col = 0; col < 5; col++)
        for (int row = 0; row < 7; row++)
            if (g[col] & (1 << row))
                for (int dx = 0; dx < SCALE; dx++)
                    for (int dy = 0; dy < SCALE; dy++)
                        fb_pixel(x + col * SCALE + dx, y + row * SCALE + dy);
}

static void draw_text(int x, int y, const char *s) {
    for (; *s; s++, x += 6 * SCALE) draw_char(x, y, *s);
}

static void oled_show(const char *l1, const char *l2) {
    memset(fb, 0, sizeof(fb));
    draw_text(0, 1, l1);
    draw_text(0, 17, l2);
    oled_flush();
}

/* ---------- Beacon tasks ---------- */
typedef struct {
    const char *name, *line1, *line2;
    gpio_num_t  led;
    uint32_t    half_ms;
    uint8_t     id;
    EventBits_t bit;
    int64_t     last;
    int         n;
    int64_t     out[MAXSW], in[MAXSW];
} beacon_t;

static beacon_t beacons[] = {
    { .name = "Task A", .line1 = "TASK A", .line2 = "RUNNING", .led = PIN_LED1, .half_ms = 200, .id = 0, .bit = (1 << 0) },
    { .name = "Task B", .line1 = "TASK B", .line2 = "RUNNING", .led = PIN_LED2, .half_ms = 300, .id = 1, .bit = (1 << 1) },
    { .name = "Task C", .line1 = "TASK C", .line2 = "RUNNING", .led = PIN_LED3, .half_ms = 100, .id = 2, .bit = (1 << 2) },
};
#define ALL_BITS ((1 << 0) | (1 << 1) | (1 << 2))

typedef struct { int64_t t; const char *name; const char *what; int blink; } ev_t;
static ev_t evs[MAXEV];
static int ev_n = 0;
static portMUX_TYPE ev_mux = portMUX_INITIALIZER_UNLOCKED;

static QueueHandle_t disp_q;
static EventGroupHandle_t start_grp;
static int tasks_done = 0;

static void ev_add(const beacon_t *b, const char *what, int blink) {
    int64_t t = esp_timer_get_time();
    portENTER_CRITICAL(&ev_mux);
    if (ev_n < MAXEV) evs[ev_n++] = (ev_t){t, b->name, what, blink};
    portEXIT_CRITICAL(&ev_mux);
}

// Display task: higher priority, sleeps on queue, wakes instantly.
static void display_task(void *pv) {
    int shown = -1;
    uint8_t msg;
    while (1) {
        if (xQueueReceive(disp_q, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg == shown) continue;
            shown = msg;
            if (msg == MSG_DONE) oled_show("ALL TASKS", "DONE");
            else oled_show(beacons[msg].line1, beacons[msg].line2);
        }
    }
}

// Busy-wait until absolute deadline (us). Task stays Ready -> time-sliced.
// A gap > 2 ms between polls = task was switched out.
static void wait_until(beacon_t *b, int64_t deadline) {
    int64_t now;
    while ((now = esp_timer_get_time()) < deadline) {
        if (b->last && (now - b->last) > 2000 && b->n < MAXSW) {
            b->out[b->n] = b->last;
            b->in[b->n]  = now;
            b->n++;
        }
        b->last = now;
    }
}

static void dump_all(void) {
    printf("==== EVENTS ====\n");
    for (int i = 0; i < ev_n; i++)
        printf("[%8lld ms] %s blink %d %s\n",
               evs[i].t / 1000, evs[i].name, evs[i].blink, evs[i].what);

    printf("==== SWITCHES (task preempted: ran / waited) ====\n");
    for (int t = 0; t < 3; t++) {
        beacon_t *b = &beacons[t];
        for (int k = 1; k < b->n; k++)
            printf("%s out@%lld ms  back@%lld ms  waited=%lld ms  ran=%lld ms\n",
                   b->name, b->out[k] / 1000, b->in[k] / 1000,
                   (b->in[k] - b->out[k]) / 1000,
                   (b->out[k] - b->in[k - 1]) / 1000);
    }
    printf("==== END ====\n");
}

static void beacon_task(void *pv) {
    beacon_t *b = (beacon_t *)pv;

    // All tasks start blinking together.
    xEventGroupSync(start_grp, b->bit, ALL_BITS, portMAX_DELAY);

    b->last = 0;
    b->n = 0;
    int64_t t = esp_timer_get_time();

    for (int i = 1; i <= BLINKS; i++) {
        gpio_set_level(b->led, 1);
        ev_add(b, "ON", i);
        xQueueSend(disp_q, &b->id, 0);          // never blocks

        t += (int64_t)b->half_ms * 1000;
        wait_until(b, t);

        gpio_set_level(b->led, 0);
        ev_add(b, "OFF", i);
        t += (int64_t)b->half_ms * 1000;
        wait_until(b, t);
    }

    gpio_set_level(b->led, 0);
    ev_add(b, "DONE", BLINKS);

    portENTER_CRITICAL(&ev_mux);
    int d = ++tasks_done;
    portEXIT_CRITICAL(&ev_mux);

    if (d == 3) {
        uint8_t m = MSG_DONE;
        xQueueSend(disp_q, &m, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));          // let OLED finish
        dump_all();
    }

    vTaskDelete(NULL);
}

void app_main(void) {
    // LEDs
    for (int i = 0; i < 3; i++) {
        gpio_reset_pin(beacons[i].led);
        gpio_set_direction(beacons[i].led, GPIO_MODE_OUTPUT);
        gpio_set_level(beacons[i].led, 0);
    }

    // I2C + OLED
    i2c_master_bus_config_t bc = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = PIN_SDA,
        .scl_io_num        = PIN_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &bus));
    oled_init(bus);
    oled_show("STARTING", "");

    disp_q    = xQueueCreate(8, sizeof(uint8_t));
    start_grp = xEventGroupCreate();

    xTaskCreate(display_task, "Display", 4096, NULL, DISP_PRIO, NULL);

    //vTaskDelay(pdMS_TO_TICKS(3000));   // let terminal reconnect

    // Same priority -> round-robin
    xTaskCreate(beacon_task, "Task A", 4096, &beacons[0], TASK_PRIO, NULL);
    xTaskCreate(beacon_task, "Task B", 4096, &beacons[1], TASK_PRIO, NULL);
    xTaskCreate(beacon_task, "Task C", 4096, &beacons[2], TASK_PRIO, NULL);
}