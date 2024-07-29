#include "who_human_face_detection.hpp"

#include "esp_log.h"
#include "esp_camera.h"

#include "dl_image.hpp"
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"

#include "who_ai_utils.hpp"

#define TWO_STAGE_ON 0
#define SCORE_TH_DEFAULT    0.5

static const char *TAG = "human_face_detection";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

static bool gEvent = true;
static bool gReturnFB = true;

face_detected_cb _faceD_callback = nullptr;
bool is_processing = false;

static void task_process_handler(void *arg)
{
    camera_fb_t *frame = NULL;
    HumanFaceDetectMSR01 detector(0.3F, 0.3F, 10, 0.3F);
#if TWO_STAGE_ON
    HumanFaceDetectMNP01 detector2(0.4F, 0.3F, 10);
#endif

    while (true)
    {
        if (gEvent)
        {
            float score = 0;
            bool is_detected = false;
            if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
            {
                is_processing = true;
#if TWO_STAGE_ON
                std::list<dl::detect::result_t> &detect_candidates = detector.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3});
                std::list<dl::detect::result_t> &detect_results = detector2.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_candidates);
#else
                std::list<dl::detect::result_t> &detect_results = detector.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3});
#endif

                if (detect_results.size() > 0)
                {
                    draw_detection_result((uint16_t *)frame->buf, frame->height, frame->width, detect_results);
                    print_detection_result(detect_results);
                    is_detected = true;

                    for (auto it = detect_results.begin(); it != detect_results.end(); ++it)
                    {
                        if(it->score>score) score = it->score;
                    }

                    printf("Face detected [score: %f] ::: fh = %lub\n", score,(long unsigned)xPortGetFreeHeapSize());
                    
                    if(_faceD_callback)
                    {
                        _faceD_callback(frame,score);
                    }
                }
            }

            if (xQueueFrameO)
            {
                xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
            }
            else if (gReturnFB)
            {
                esp_camera_fb_return(frame);
            }
            else
            {
                free(frame);
            }

            if (xQueueResult)
            {
                xQueueSend(xQueueResult, &is_detected, portMAX_DELAY);
            }

            is_processing = false; // Termina de procesar el frame
        }
    }
}

static void task_event_handler(void *arg)
{
    while (true)
    {
        xQueueReceive(xQueueEvent, &(gEvent), portMAX_DELAY);
    }
}

void register_human_face_detection(const QueueHandle_t frame_i,
                                   const QueueHandle_t event,
                                   const QueueHandle_t result,
                                   const QueueHandle_t frame_o,
                                   const bool camera_fb_return)
{
    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    xQueueEvent = event;
    xQueueResult = result;
    gReturnFB = camera_fb_return;

    xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 0);
    if (xQueueEvent)
        xTaskCreatePinnedToCore(task_event_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);
}

void set_faceD_callback(face_detected_cb cb)
{
    _faceD_callback = cb;
}

bool is_processing_or_frames_left() {
    // Devuelve true si está procesando un frame o si quedan frames por procesar
    return (is_processing || (uxQueueMessagesWaiting(xQueueFrameI) > 0));
}
