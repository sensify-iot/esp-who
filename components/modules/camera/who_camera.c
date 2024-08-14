#include "who_camera.h"

#include "esp_log.h"
#include "esp_system.h"

#include <string.h>

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
                    // printf("%d\t::: Photo taken \t%db (fh: %d %d)\n",esp_log_timestamp(),frame->len,(uint32_t)heap_caps_get_free_size(MALLOC_CAP_8BIT),(uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
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
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t register_camera(const pixformat_t pixel_fromat,
                     const framesize_t frame_size,
                     const uint8_t fb_count,
                     const camera_grab_mode_t grab_mode,
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
    config.grab_mode = grab_mode;

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

    if(frame_o)
    {
        ESP_LOGI(TAG,"Starting camera task process...");
        xQueueFrameO = frame_o;
        xTaskCreatePinnedToCore(task_process_handler, TAG, 3 * 1024, NULL, 18, NULL, 1);
    }

    return ESP_OK;
}

uint8_t* take_Photo(size_t* image_size, int quality)
{
    camera_fb_t* fb = NULL;

    ESP_LOGI(TAG,"Taking picture... (quality: %d)",quality);
    sensor_t * s = esp_camera_sensor_get();
    if(s==NULL)
    {
        return NULL;
    }

    if(quality > 60)
    {
        quality = 60;
    }

    if(quality < 5)
    {
        quality = 5;
    }

    s->set_quality(s,quality);

    fb = esp_camera_fb_get();
    if(fb == NULL){
        ESP_LOGE("ESPCAM","Camera capture failed");
        return NULL;
    }

    esp_camera_fb_return(fb);

    fb = esp_camera_fb_get();
    if(fb == NULL){
        ESP_LOGE("ESPCAM","Camera capture failed");
        return NULL;
    }


    // No es necesario convertir, ya está en JPEG
    *image_size = fb->len;
    
    uint8_t *image = (uint8_t *)malloc(fb->len);
    if(image != NULL)
    {
        memcpy(image, fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);

    return image;
}