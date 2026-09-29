#include <stdio.h>
#include <string.h>
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define LOG_OUTPUT_MODE true
#define LOG_RAW_SENSOR_VALUES false

#define LIGHT_CHANNEL ADC_CHANNEL_8
#define SOUND_CHANNEL ADC_CHANNEL_7

#define CONVERSION_FRAME_SIZE 128

static const char *TAG = "RhythmCal";

static adc_channel_t channel[2] = {LIGHT_CHANNEL, SOUND_CHANNEL};
static TaskHandle_t s_task_handle;

#pragma region "Continuous ADC"
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;

    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = CONVERSION_FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 40 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = channel[i];
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}
#pragma endregion

static void process_light_samples(int samples[], int count)
{
    if (count == 0)
        return;

    int sum = 0;
    for (int i = 0; i < count; i++)
    {
        sum += samples[i];
    }

    int average = sum / count;

#if (LOG_OUTPUT_MODE)
    ESP_LOGI(TAG, "Light: %d (%d samples)", average, count);
#endif
}

static void process_sound_samples(int samples[], int count)
{
    int sample_min = 4095;
    int sample_max = 0;

    for (int i = 0; i < count; i++)
    {
        int sample = samples[i];

        if (sample < sample_min)
            sample_min = sample;
        if (sample > sample_max)
            sample_max = sample;
    }

    int amp = sample_max - sample_min;

#if (LOG_OUTPUT_MODE)
    ESP_LOGI(TAG, "Sound: %d (%d samples)", amp, count);
#endif
}

void app_main(void)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[CONVERSION_FRAME_SIZE] = {0};
    memset(result, 0xcc, CONVERSION_FRAME_SIZE);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while (1)
    {
        // Wait for data to become available from continuous read.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1)
        {
            ret = adc_continuous_read(handle, result, CONVERSION_FRAME_SIZE, &ret_num, 0);
            if (ret == ESP_OK)
            {
                adc_continuous_data_t parsed_data[ret_num / SOC_ADC_DIGI_RESULT_BYTES];
                uint32_t num_parsed_samples = 0;

                esp_err_t parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);

                if (parse_ret == ESP_OK)
                {
                    int light_samples[num_parsed_samples];
                    int sound_samples[num_parsed_samples];
                    int light_sample_count = 0;
                    int sound_sample_count = 0;

                    for (int i = 0; i < num_parsed_samples; i++)
                    {
                        if (parsed_data[i].valid)
                        {
                            if (parsed_data[i].channel == LIGHT_CHANNEL)
                            {
                                light_samples[light_sample_count] = parsed_data[i].raw_data;
                                light_sample_count++;
                            }
                            else if (parsed_data[i].channel == SOUND_CHANNEL)
                            {
                                sound_samples[sound_sample_count] = parsed_data[i].raw_data;
                                sound_sample_count++;
                            }

#if (LOG_OUTPUT_MODE && LOG_RAW_SENSOR_VALUES)
                            ESP_LOGI(TAG, "ADC%d, Channel: %d, Value: %" PRIu32,
                                     parsed_data[i].unit + 1,
                                     parsed_data[i].channel,
                                     parsed_data[i].raw_data);
#endif
                        }
                        else
                        {
#if LOG_OUTPUT_MODE
                            ESP_LOGW(TAG, "Invalid data [ADC%d_Ch%d_%" PRIu32 "]",
                                     parsed_data[i].unit + 1,
                                     parsed_data[i].channel,
                                     parsed_data[i].raw_data);
#endif
                        }
                    }

                    process_light_samples(light_samples, light_sample_count);
                    process_sound_samples(sound_samples, sound_sample_count);
                }
                else
                {
#if LOG_OUTPUT_MODE
                    ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
#endif
                }

#if LOG_OUTPUT_MODE
                // Only needed because logging is slow.
                vTaskDelay(1);
#endif
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                // Read `CONVERSION_FRAME_SIZE` until API returns timeout, which means there's no available data.
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
