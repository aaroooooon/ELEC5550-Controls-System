/*
 * Weed detection on the Seeed XIAO ESP32S3 Sense.
 *
 * Every camera frame goes through the Edge Impulse FOMO model in
 * lib/weed-detection-cpp-mcu-v5-impulse-#1 (deploy v5: Brass Button, Burr
 * medic, Clover, Dandelion, Flat Weed, Tall Weed; 160x160 input). When one frame holds
 * WEED_TRIGGER_COUNT or more detections (all labels counted together), GPIO1 is
 * driven high for TRIGGER_PULSE_MS. Each further qualifying frame restarts that
 * timer, so the pin stays high while the patch is in view and drops
 * TRIGGER_PULSE_MS after the last qualifying frame.
 *
 * Frames and their detections are streamed down the USB-C cable; run
 * tools/view.py on the laptop to see the video with the boxes drawn on it.
 * Detection and GPIO1 carry on whether or not anything is plugged in.
 *
 * Wire format, one pair per frame, integers little-endian:
 *
 *     "DETS" | u32 length | detections payload, see build_detections()
 *     "JPGF" | u32 length | JPEG payload (starts FF D8, ends FF D9)
 *
 * A DETS packet is always sent immediately before its frame's JPGF; the viewer
 * relies on that to reject a cut-short DETS.
 *
 * The magics let the viewer resynchronise at any point, so it can be started
 * and stopped independently of the board, and stray console output in the
 * stream is skipped rather than being mistaken for data.
 *
 * This file is C++ because the Edge Impulse SDK is a C++ library: its headers
 * use templates, namespaces and std::function, so whatever includes
 * ei_run_classifier.h has to be compiled as C++.
 *
 * The previous versions are kept in extras/ (GPIO1 blink, WiFi stream, and the
 * plain USB stream this one grew out of). Earlier versions of this detection
 * firmware are archived in "detection model version control/"; its README lists
 * what changed between them.
 */

#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "jpeg_decoder.h"

/*
 * The SDK's headers carry code that trips the project's -Werror; the model's
 * own component builds with warnings off, and this does the same for the part
 * of it compiled here.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/image/processing.hpp"
#pragma GCC diagnostic pop

/* ------------------------------ trigger ------------------------------ */

#define TRIGGER_GPIO       GPIO_NUM_1
#define WEED_TRIGGER_COUNT 4       /* detections in one frame */
#define TRIGGER_PULSE_MS   2000

/* ------------------------------ stream ------------------------------- */

/*
 * Console logging and the video stream share one USB endpoint, so a log line
 * emitted mid-frame would corrupt that frame. Logs are silenced once streaming
 * starts. Set this to 0 to keep them: the viewer resynchronises on the magic,
 * so you lose the occasional frame but can watch ESP_LOG output in a monitor.
 */
#define STREAM_QUIET 1

/* Give up on a frame if the host has not drained the buffer within this long. */
#define USB_WRITE_TIMEOUT_MS 1000

/* TX ring buffer, and the largest single write allowed into it. See usb_write_all(). */
#define USB_TX_BUFFER_BYTES 8192
#define USB_CHUNK_BYTES     2048

#define JPEG_MAGIC      "JPGF"
#define DETS_MAGIC      "DETS"
#define FRAME_HDR_BYTES 8

/*
 * A VGA frame at quality 12 is 20-40 KB. A frame that does not fit is still
 * run through the model, just not streamed.
 */
#define STREAM_JPEG_MAX_BYTES (128 * 1024)

/*
 * FOMO scores a 20x20 grid over the 160x160 input, so it cannot report more
 * boxes than there are cells.
 */
#define STREAM_MAX_BOXES 400
#define DETS_BOX_BYTES   10
#define DETS_MAX_BYTES   (512 + STREAM_MAX_BOXES * DETS_BOX_BYTES)

static const char *TAG = "weed-det";

/* ---------------------------------------------------------------------
 * Camera pin map for the XIAO ESP32S3 Sense expansion board. Boards ship with
 * an OV2640 or an OV3660; the driver detects which.
 * ------------------------------------------------------------------ */
#define PWDN_GPIO_NUM  (-1)
#define RESET_GPIO_NUM (-1)
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

/*
 * The model sees the frame decoded at half size (VGA 640x480 -> 320x240), then
 * centre-cropped to a square and scaled to the model's input size (160x160 on
 * deploy v5, EI_CLASSIFIER_INPUT_WIDTH x _HEIGHT), matching the "fit shortest
 * axis" resize the impulse was trained with. Decoding at half size is done by
 * the JPEG decoder itself and is much cheaper than decoding the full frame.
 */
#define DECODE_SCALE     JPEG_IMAGE_SCALE_1_2
#define DECODE_DIVISOR   2
#define DECODE_MAX_BYTES ((640 / DECODE_DIVISOR) * (480 / DECODE_DIVISOR) * 3)

/* ------------------------------ camera ------------------------------- */

static esp_err_t camera_start(void)
{
    camera_config_t config = {};
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;

    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_VGA;   /* 640x480 */
    config.jpeg_quality = 12;            /* 0-63, lower is better quality */
    config.fb_count = 2;                 /* needs PSRAM */
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        /* The Sense module is mounted such that the image comes out flipped. */
        s->set_vflip(s, 1);
        s->set_hmirror(s, 1);
    }

    ESP_LOGI(TAG, "camera ready");
    return ESP_OK;
}

/* ------------------------------ trigger ------------------------------ */

static TaskHandle_t trigger_task_handle;
static std::atomic<bool> trigger_high{false};

/*
 * Owns GPIO1. Each notification is one qualifying frame: the pin goes high and
 * stays high until TRIGGER_PULSE_MS pass without another one. Doing this in a
 * task rather than a timer callback means a restart can never race with the
 * pin being switched off.
 */
static void trigger_task(void *arg)
{
    (void)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gpio_set_level(TRIGGER_GPIO, 1);
        trigger_high = true;

        while (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TRIGGER_PULSE_MS)) > 0) {
            /* retriggered: wait the full pulse length again */
        }

        gpio_set_level(TRIGGER_GPIO, 0);
        trigger_high = false;
    }
}

static esp_err_t trigger_start(void)
{
    gpio_reset_pin(TRIGGER_GPIO);
    gpio_set_direction(TRIGGER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIGGER_GPIO, 0);

    if (xTaskCreate(trigger_task, "trigger", 2048, NULL, 10, &trigger_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "could not start trigger task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ------------------------------ USB link ----------------------------- */

static esp_err_t usb_link_start(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size = USB_TX_BUFFER_BYTES;
    cfg.rx_buffer_size = 256;  /* nothing is sent back to the board */

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed: %s", esp_err_to_name(err));
    }
    return err;
}

/*
 * Chunking here is mandatory, not an optimisation.
 *
 * usb_serial_jtag_write_bytes() forwards the whole buffer to xRingbufferSend()
 * as a single item, and FreeRTOS rejects an item larger than the ring buffer
 * outright -- it returns immediately rather than blocking, since no amount of
 * draining could ever make room. A 25 KB VGA frame handed to an 8 KB TX buffer
 * therefore fails instantly and every frame is lost while the 8 byte header
 * still goes out, which looks like a stream of headers with no pixels.
 *
 * Writing in pieces comfortably smaller than the buffer keeps several chunks in
 * flight for the ISR to drain. Returns false if the host stopped reading.
 */
static bool usb_write_all(const uint8_t *data, size_t len)
{
    size_t sent = 0;

    while (sent < len) {
        size_t want = len - sent;
        if (want > USB_CHUNK_BYTES) {
            want = USB_CHUNK_BYTES;
        }

        int n = usb_serial_jtag_write_bytes(data + sent, want,
                                            pdMS_TO_TICKS(USB_WRITE_TIMEOUT_MS));
        if (n <= 0) {
            return false;
        }
        sent += (size_t)n;
    }

    return true;
}

static bool send_packet(const char *magic, const uint8_t *payload, size_t len)
{
    uint8_t hdr[FRAME_HDR_BYTES];

    memcpy(hdr, magic, 4);
    hdr[4] = (uint8_t)(len & 0xff);
    hdr[5] = (uint8_t)((len >> 8) & 0xff);
    hdr[6] = (uint8_t)((len >> 16) & 0xff);
    hdr[7] = (uint8_t)((len >> 24) & 0xff);

    return usb_write_all(hdr, sizeof(hdr)) && usb_write_all(payload, len);
}

/* ------------------------------ streaming ---------------------------- */

/*
 * A frame waiting to go down the cable: a copy of the JPEG plus its detections
 * packet. Two of them circulate between the detection loop and the stream task
 * through free_slots and ready_slots, so the detection loop never waits on USB.
 * If the host is slow, or the viewer is not running, frames simply are not
 * streamed; detection and GPIO1 carry on at full rate.
 */
struct stream_slot_t {
    uint8_t *jpeg;
    size_t jpeg_len;
    uint8_t *dets;
    size_t dets_len;
};

#define STREAM_SLOTS 2

static stream_slot_t slots[STREAM_SLOTS];
static QueueHandle_t free_slots;
static QueueHandle_t ready_slots;

static void stream_task(void *arg)
{
    (void)arg;

    while (true) {
        stream_slot_t *slot;
        xQueueReceive(ready_slots, &slot, portMAX_DELAY);

        /*
         * Nothing is plugged in, or the cable is charge-only: skip the write
         * rather than stalling on a buffer nobody is reading.
         */
        if (usb_serial_jtag_is_connected()) {
            bool ok = send_packet(DETS_MAGIC, slot->dets, slot->dets_len) &&
                      send_packet(JPEG_MAGIC, slot->jpeg, slot->jpeg_len);
            if (!ok) {
                /*
                 * The viewer is not draining the link: closed, paused, or the
                 * port is open in something that is not reading. Back off a
                 * little; the viewer resynchronises when it comes back.
                 */
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }

        xQueueSend(free_slots, &slot, portMAX_DELAY);
    }
}

static esp_err_t stream_start(void)
{
    free_slots = xQueueCreate(STREAM_SLOTS, sizeof(stream_slot_t *));
    ready_slots = xQueueCreate(STREAM_SLOTS, sizeof(stream_slot_t *));
    if (free_slots == NULL || ready_slots == NULL) {
        return ESP_ERR_NO_MEM;
    }

    for (int i = 0; i < STREAM_SLOTS; i++) {
        slots[i].jpeg = (uint8_t *)heap_caps_malloc(STREAM_JPEG_MAX_BYTES, MALLOC_CAP_SPIRAM);
        slots[i].dets = (uint8_t *)heap_caps_malloc(DETS_MAX_BYTES, MALLOC_CAP_SPIRAM);
        if (slots[i].jpeg == NULL || slots[i].dets == NULL) {
            return ESP_ERR_NO_MEM;
        }
        stream_slot_t *p = &slots[i];
        xQueueSend(free_slots, &p, 0);
    }

    /* Core 0, next to the camera driver; the model has core 1 to itself. */
    if (xTaskCreatePinnedToCore(stream_task, "stream", 4096, NULL, 5, NULL, 0) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ------------------------------ detections --------------------------- */

static inline uint8_t *put_u16(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
    return p + 2;
}

/* Region of the camera frame the model actually looks at, in frame pixels. */
struct roi_t {
    int x, y, w, h;
};

/*
 * Detections payload, integers little-endian:
 *
 *     offset  size  field
 *     ------  ----  -----
 *          0     2  frame width
 *          2     2  frame height
 *          4     8  model field of view in frame pixels: x, y, w, h (u16 each)
 *         12     2  model time for this frame, ms
 *         14     1  trigger threshold (WEED_TRIGGER_COUNT)
 *         15     1  flags: bit 0 = GPIO1 high
 *         16     1  label count L
 *                   L x { u8 name length, name bytes }
 *                   u16 box count N
 *                   N x { u8 label index, u8 confidence 0-255,
 *                         u16 x, u16 y, u16 w, u16 h }   -- frame pixels
 */
static size_t build_detections(uint8_t *out, const camera_fb_t *fb, const roi_t &roi,
                               const ei_impulse_result_t &result, uint32_t box_count,
                               uint32_t model_ms)
{
    uint8_t *p = out;

    p = put_u16(p, fb->width);
    p = put_u16(p, fb->height);
    p = put_u16(p, roi.x);
    p = put_u16(p, roi.y);
    p = put_u16(p, roi.w);
    p = put_u16(p, roi.h);
    p = put_u16(p, model_ms > 0xffff ? 0xffff : model_ms);
    *p++ = WEED_TRIGGER_COUNT;
    /*
     * A qualifying frame has only just notified trigger_task, which may not have
     * run yet; report the pin as high for it rather than a frame late.
     */
    *p++ = (trigger_high || box_count >= WEED_TRIGGER_COUNT) ? 0x01 : 0x00;

    *p++ = EI_CLASSIFIER_LABEL_COUNT;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        size_t n = strlen(ei_classifier_inferencing_categories[i]);
        if (n > 31) {
            n = 31;
        }
        *p++ = (uint8_t)n;
        memcpy(p, ei_classifier_inferencing_categories[i], n);
        p += n;
    }

    uint8_t *count_at = p;
    p += 2;
    uint32_t written = 0;

    for (uint32_t i = 0; i < result.bounding_boxes_count && written < box_count; i++) {
        const ei_impulse_result_bounding_box_t &bb = result.bounding_boxes[i];
        if (bb.value <= 0.0f) {
            continue;
        }

        uint8_t label = 0xff;
        for (size_t l = 0; l < EI_CLASSIFIER_LABEL_COUNT; l++) {
            if (strcmp(bb.label, ei_classifier_inferencing_categories[l]) == 0) {
                label = (uint8_t)l;
                break;
            }
        }

        /* Model input pixels -> frame pixels. */
        *p++ = label;
        *p++ = (uint8_t)(bb.value * 255.0f + 0.5f);
        p = put_u16(p, roi.x + bb.x * roi.w / EI_CLASSIFIER_INPUT_WIDTH);
        p = put_u16(p, roi.y + bb.y * roi.h / EI_CLASSIFIER_INPUT_HEIGHT);
        p = put_u16(p, bb.width * roi.w / EI_CLASSIFIER_INPUT_WIDTH);
        p = put_u16(p, bb.height * roi.h / EI_CLASSIFIER_INPUT_HEIGHT);
        written++;
    }
    put_u16(count_at, written);

    return (size_t)(p - out);
}

/* ------------------------------ inference ---------------------------- */

/*
 * Replaces the SDK's weak ei_calloc() with one that returns 16-byte aligned
 * memory.
 *
 * The model's scratch buffers go in its tensor arena when they fit, and are
 * ei_calloc()ed when they do not; on this model the first ESP-NN convolutions
 * always take the second route. The ESP32-S3 ESP-NN kernels use their scratch
 * buffer as-is with 128-bit vector loads and stores, which ignore the low four
 * address bits. The SDK's ESP32 port aligns ei_malloc() for exactly this reason
 * but not ei_calloc(), whose heap memory is only 4-byte aligned, so the kernels
 * wrote up to 12 bytes in front of the buffer, over the heap's bookkeeping, and
 * the board panicked when the model freed it after the first inference.
 * free() handles aligned allocations, so the SDK's ei_free() still pairs with this.
 */
void *ei_calloc(size_t nitems, size_t size)
{
    return heap_caps_aligned_calloc(16, nitems, size, MALLOC_CAP_DEFAULT);
}

/* Decoded frame; cropped and scaled in place, then its start is the model input. */
static uint8_t *rgb_buf;

/* Feeds the model: one float per pixel, holding the pixel packed as 0xRRGGBB. */
static int get_model_input(size_t offset, size_t length, float *out)
{
    const uint8_t *px = rgb_buf + offset * 3;
    for (size_t i = 0; i < length; i++, px += 3) {
        out[i] = (float)((px[0] << 16) | (px[1] << 8) | px[2]);
    }
    return 0;
}

/*
 * Decodes the frame at reduced size and turns it into the model input
 * in rgb_buf. Fills roi with the part of the frame that input covers.
 */
static bool prepare_model_input(const camera_fb_t *fb, roi_t *roi)
{
    esp_jpeg_image_cfg_t cfg = {};
    cfg.indata = fb->buf;
    cfg.indata_size = fb->len;
    cfg.outbuf = rgb_buf;
    cfg.outbuf_size = DECODE_MAX_BYTES;
    cfg.out_format = JPEG_IMAGE_FORMAT_RGB888;
    cfg.out_scale = DECODE_SCALE;

    esp_jpeg_image_output_t img = {};
    if (esp_jpeg_decode(&cfg, &img) != ESP_OK) {
        return false;
    }

    int crop_w, crop_h;
    ei::image::processing::calculate_crop_dims(img.width, img.height,
                                               EI_CLASSIFIER_INPUT_WIDTH,
                                               EI_CLASSIFIER_INPUT_HEIGHT,
                                               crop_w, crop_h);

    /* Same centre crop as crop_and_interpolate_rgb888(), scaled back up to the frame. */
    roi->x = ((img.width - crop_w) / 2) * fb->width / img.width;
    roi->y = ((img.height - crop_h) / 2) * fb->height / img.height;
    roi->w = crop_w * fb->width / img.width;
    roi->h = crop_h * fb->height / img.height;

    return ei::image::processing::crop_and_interpolate_rgb888(
               rgb_buf, img.width, img.height,
               rgb_buf, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT) == 0;
}

static void detect_task(void *arg)
{
    (void)arg;

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &get_model_input;

    uint32_t frames = 0;
    int64_t last_report = esp_timer_get_time();

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGE(TAG, "frame capture failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        roi_t roi;
        if (!prepare_model_input(fb, &roi)) {
            ESP_LOGE(TAG, "JPEG decode failed");
            esp_camera_fb_return(fb);
            continue;
        }

        /*
         * Keep a copy of the JPEG for the stream if a slot is free, so the
         * camera buffer can go back to the driver before the model runs.
         */
        stream_slot_t *slot = NULL;
        if (xQueueReceive(free_slots, &slot, 0) == pdTRUE) {
            if (fb->len <= STREAM_JPEG_MAX_BYTES) {
                memcpy(slot->jpeg, fb->buf, fb->len);
                slot->jpeg_len = fb->len;
            } else {
                xQueueSend(free_slots, &slot, 0);
                slot = NULL;
            }
        }

        ei_impulse_result_t result = {};
        EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
        if (err != EI_IMPULSE_OK) {
            ESP_LOGE(TAG, "run_classifier failed: %d", (int)err);
            if (slot != NULL) {
                xQueueSend(free_slots, &slot, 0);
            }
            esp_camera_fb_return(fb);
            continue;
        }

        uint32_t weeds = 0;
        for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
            if (result.bounding_boxes[i].value > 0.0f) {
                weeds++;
            }
        }

        if (weeds >= WEED_TRIGGER_COUNT) {
            xTaskNotifyGive(trigger_task_handle);
        }

        uint32_t model_ms = (uint32_t)((result.timing.dsp_us +
                                        result.timing.classification_us +
                                        result.timing.postprocessing_us) / 1000);

        if (slot != NULL) {
            uint32_t boxes = weeds > STREAM_MAX_BOXES ? STREAM_MAX_BOXES : weeds;
            slot->dets_len = build_detections(slot->dets, fb, roi, result, boxes, model_ms);
            xQueueSend(ready_slots, &slot, 0);
        }

        esp_camera_fb_return(fb);

        ESP_LOGD(TAG, "%" PRIu32 " weeds, model %" PRIu32 " ms", weeds, model_ms);
        if (++frames % 50 == 0) {
            int64_t now = esp_timer_get_time();
            ESP_LOGI(TAG, "%.1f fps, model %" PRIu32 " ms",
                     50.0f * 1000000.0f / (float)(now - last_report), model_ms);
            last_report = now;
        }

        /*
         * The loop can run flat out without ever blocking; yield a tick so the
         * idle task on this core runs and the task watchdog stays fed.
         */
        vTaskDelay(1);
    }
}

/* ------------------------------- main -------------------------------- */

extern "C" void app_main(void)
{
    if (trigger_start() != ESP_OK) {
        return;
    }

    if (camera_start() != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed; check the ribbon cable seating");
        return;
    }

    rgb_buf = (uint8_t *)heap_caps_malloc(DECODE_MAX_BYTES, MALLOC_CAP_SPIRAM);
    if (rgb_buf == NULL) {
        ESP_LOGE(TAG, "could not allocate decode buffer");
        return;
    }

    if (stream_start() != ESP_OK) {
        ESP_LOGE(TAG, "could not start stream");
        return;
    }

    /*
     * Printed before the driver takes over the endpoint, so it lands cleanly in
     * a plain serial monitor and tells you the board is alive.
     */
    printf("\n[weed-det] model \"%s\" (%d labels, %dx%d); GPIO1 high for %d ms "
           "on >= %d detections in a frame. Streaming over USB; run tools/view.py\n",
           EI_CLASSIFIER_PROJECT_NAME, EI_CLASSIFIER_LABEL_COUNT,
           EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT,
           TRIGGER_PULSE_MS, WEED_TRIGGER_COUNT);
    fflush(stdout);

#if STREAM_QUIET
    esp_log_level_set("*", ESP_LOG_NONE);
#endif

    if (usb_link_start() != ESP_OK) {
        return;
    }

    /*
     * The model's working memory (~470 KB) is allocated from PSRAM on the first
     * run. The stack only has to cover the SDK's call chain.
     */
    if (xTaskCreatePinnedToCore(detect_task, "detect", 16384, NULL, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "could not start detection task");
    }
}
