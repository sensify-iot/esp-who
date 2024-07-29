#include "who_camera.h"

#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "who_camera";
static QueueHandle_t xQueueFrameO = NULL;

static enum __cam_state_t state = RUNNING;
static bool _sleep = false;

enum __cam_state_t getCamState()
{
    return state;
}

void cam_setSleep(bool mode)
{
    _sleep = mode;
}

static void task_process_handler(void *arg)
{
    while (true)
    {
        switch(state)
        {
            case STOP:
            {
                vTaskDelay(pdMS_TO_TICKS(10));

                if(!_sleep)
                {
                    ESP_LOGI(TAG,"Cam task change to RUNNING");
                    state = RUNNING;
                }
                break;
            }

            case RUNNING:
            {
                camera_fb_t *frame = esp_camera_fb_get();
                if (frame)
                {
                    // printf("%d/%d -> Photo taken \t%db (fh: %d %d)\n",esp_log_timestamp(),pdMS_TO_TICKS(esp_log_timestamp()),frame->len,(uint32_t)heap_caps_get_free_size(MALLOC_CAP_8BIT),(uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
                    // Verificar el tamaño del buffer antes de enviarlo a la cola
                    if (frame->len != (frame->width * frame->height * 2)) // Suponiendo RGB565
                    {
                        ESP_LOGE(TAG, "Frame buffer size mismatch: expected %d, got %d", frame->width * frame->height * 2, frame->len);
                        esp_camera_fb_return(frame);
                        continue;
                    }
                    xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
                }
                else
                {
                    ESP_LOGI(TAG,"Cant get frame!");
                }

                if(_sleep)
                {
                    ESP_LOGI(TAG,"Cam task change to STOP");
                    state = STOP;
                }
                break;
            }
        }
    }
}

esp_err_t register_camera(const pixformat_t pixel_fromat,
                     const framesize_t frame_size,
                     const uint8_t fb_count,
                     const QueueHandle_t frame_o)
{
    ESP_LOGI(TAG, "Camera module is %s", CAMERA_MODULE_NAME);

#if CONFIG_CAMERA_MODULE_ESP_EYE || CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD
    /* IO13, IO14 is designed for JTAG by default,
     * to use it as generalized input,
     * firstly declair it as pullup input */
    gpio_config_t conf;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.pin_bit_mask = 1LL << 13;
    gpio_config(&conf);
    conf.pin_bit_mask = 1LL << 14;
    gpio_config(&conf);
#endif

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAMERA_PIN_D0;
    config.pin_d1 = CAMERA_PIN_D1;
    config.pin_d2 = CAMERA_PIN_D2;
    config.pin_d3 = CAMERA_PIN_D3;
    config.pin_d4 = CAMERA_PIN_D4;
    config.pin_d5 = CAMERA_PIN_D5;
    config.pin_d6 = CAMERA_PIN_D6;
    config.pin_d7 = CAMERA_PIN_D7;
    config.pin_xclk = CAMERA_PIN_XCLK;
    config.pin_pclk = CAMERA_PIN_PCLK;
    config.pin_vsync = CAMERA_PIN_VSYNC;
    config.pin_href = CAMERA_PIN_HREF;
    config.pin_sscb_sda = CAMERA_PIN_SIOD;
    config.pin_sscb_scl = CAMERA_PIN_SIOC;
    config.pin_pwdn = CAMERA_PIN_PWDN;
    config.pin_reset = CAMERA_PIN_RESET;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.pixel_format = pixel_fromat;
    config.frame_size = frame_size;
    config.jpeg_quality = 12;
    config.fb_count = fb_count;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return ESP_FAIL;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 0); //flip it back
    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_brightness(s, 1);  //up the blightness just a bit
        s->set_saturation(s, -2); //lower the saturation
    }

    xQueueFrameO = frame_o;
    xTaskCreatePinnedToCore(task_process_handler, TAG, 3 * 1024, NULL, 5, NULL, 1);
    return ESP_OK;
}

esp_err_t set_camera_framesize(framesize_t frame_size) {

    // Actualiza el framebuffer
    esp_camera_deinit();

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAMERA_PIN_D0;
    config.pin_d1 = CAMERA_PIN_D1;
    config.pin_d2 = CAMERA_PIN_D2;
    config.pin_d3 = CAMERA_PIN_D3;
    config.pin_d4 = CAMERA_PIN_D4;
    config.pin_d5 = CAMERA_PIN_D5;
    config.pin_d6 = CAMERA_PIN_D6;
    config.pin_d7 = CAMERA_PIN_D7;
    config.pin_xclk = CAMERA_PIN_XCLK;
    config.pin_pclk = CAMERA_PIN_PCLK;
    config.pin_vsync = CAMERA_PIN_VSYNC;
    config.pin_href = CAMERA_PIN_HREF;
    config.pin_sscb_sda = CAMERA_PIN_SIOD;
    config.pin_sscb_scl = CAMERA_PIN_SIOC;
    config.pin_pwdn = CAMERA_PIN_PWDN;
    config.pin_reset = CAMERA_PIN_RESET;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size = frame_size; // Set the new frame size
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    // Inicializa la cámara con el nuevo tamaño de frame
    if (esp_camera_init(&config) != ESP_OK) {
        ESP_LOGE(TAG, "Error al reinicializar la cámara");
        return ESP_FAIL;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 0); //flip it back

    ESP_LOGI(TAG, "Tamaño del frame cambiado y cámara reinicializada");
    return ESP_OK;
}