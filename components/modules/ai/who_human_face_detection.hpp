#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_camera.h"

typedef void (*face_detected_cb)(camera_fb_t* frame, float score);

void set_faceD_callback(face_detected_cb cb);

bool is_processing_or_frames_left();

void register_human_face_detection(QueueHandle_t frame_i,
                                   QueueHandle_t event,
                                   QueueHandle_t result,
                                   QueueHandle_t frame_o = NULL,
                                   const bool camera_fb_return = false);
