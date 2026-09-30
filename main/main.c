#include <stdio.h>
#include <string.h>
#include "class/hid/hid_device.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"

#define LOG_OUTPUT_MODE false
#define LOG_RAW_SENSOR_VALUES false
#define LOG_CURRENT_READINGS false

#define LIGHT_CHANNEL ADC_CHANNEL_8
#define SOUND_CHANNEL ADC_CHANNEL_7

#define CONVERSION_FRAME_SIZE 128

#define BASELINE_ALPHA 0.01
#define LIGHT_TRIGGER_THRESHOLD 200
#define SOUND_TRIGGER_THRESHOLD 200
#define TRIGGER_COOLDOWN_MS 100

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

static const char *TAG = "RhythmCal";

static char serial_number[13];
static const char *hid_string_descriptor[5] = {
    (char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "MrCapitalQ",         // 1: Manufacturer
    "RhythmCal",          // 2: Product
    serial_number,        // 3: Serial number from the chip's base MAC address
    "RhythmCal Sensor",   // 4: HID
};
static const uint8_t hid_report_descriptor[] = {TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD))};
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 1),
};

static adc_channel_t channel[2] = {LIGHT_CHANNEL, SOUND_CHANNEL};
static TaskHandle_t s_task_handle;

static int light_baseline = -1;
static int sound_baseline = -1;
static bool is_light_triggered = 0;
static bool is_sound_triggered = 0;
static int64_t last_light_trigger_time = 0;
static int64_t last_sound_trigger_time = 0;

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

#pragma region "Sensor Data Handling"
static inline void update_baseline(int *baseline, int reading)
{
    // Negative baseline means it hasn't been initialized so we set the first value as the starting baseline. Otherwise
    // update baseline based on exponential moving average (EMA).
    if (*baseline < 0)
        *baseline = reading;
    else
        *baseline = BASELINE_ALPHA * reading + (1 - BASELINE_ALPHA) * *baseline;
}

static int calculate_light_value(int samples[], int count)
{
    if (count == 0)
        return -1;

    int sum = 0;
    for (int i = 0; i < count; i++)
    {
        sum += samples[i];
    }

    return sum / count;
}

static int calculate_sound_value(int samples[], int count)
{
    if (count == 0)
        return -1;

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

    return sample_max - sample_min;
}

// Returns whether trigger state was changed.
static bool update_trigger_state(int current,
                                 int current_baseline,
                                 int threshold,
                                 int64_t now,
                                 bool *is_currently_triggered,
                                 int64_t *last_trigger_time)
{
    if (!*is_currently_triggered)
    {
        int delta = current - current_baseline;
        if (delta < 0)
            delta = 0;

        if (delta > threshold)
        {
            *is_currently_triggered = true;
            *last_trigger_time = now;

            return true;
        }
    }
    else if (now > ((*last_trigger_time) + (TRIGGER_COOLDOWN_MS * 1000)))
    {
        *is_currently_triggered = false;
        return true;
    }

    return false;
}
#pragma endregion

#pragma region "USB"
static void init_usb_hid()
{
    ESP_LOGI(TAG, "USB initialization");

    // Init serial based on mac address.
    uint8_t mac_address[6];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac_address));
    snprintf(serial_number,
             sizeof(serial_number),
             "%02X%02X%02X%02X%02X%02X",
             mac_address[0],
             mac_address[1],
             mac_address[2],
             mac_address[3],
             mac_address[4],
             mac_address[5]);

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();

    tusb_cfg.descriptor.device = NULL;
    tusb_cfg.descriptor.full_speed_config = hid_configuration_descriptor;
    tusb_cfg.descriptor.string = hid_string_descriptor;
    tusb_cfg.descriptor.string_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]);

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "USB initialization DONE");
}

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
}
#pragma endregion

void app_main(void)
{
#if (!LOG_OUTPUT_MODE)
    init_usb_hid();
#endif

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

                esp_err_t parse_ret = adc_continuous_parse_data(handle,
                                                                result,
                                                                ret_num,
                                                                parsed_data,
                                                                &num_parsed_samples);

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

                    int64_t now = esp_timer_get_time();
                    bool light_trigger_state_changed = false;
                    bool sound_trigger_state_changed = false;

                    int light = calculate_light_value(light_samples, light_sample_count);
                    if (light >= 0)
                    {
                        update_baseline(&light_baseline, light);
                        light_trigger_state_changed = update_trigger_state(light,
                                                                           light_baseline,
                                                                           LIGHT_TRIGGER_THRESHOLD,
                                                                           now,
                                                                           &is_light_triggered,
                                                                           &last_light_trigger_time);
                    }

                    int sound = calculate_sound_value(sound_samples, sound_sample_count);
                    if (sound >= 0)
                    {
                        update_baseline(&sound_baseline, sound);

                        sound_trigger_state_changed = update_trigger_state(sound,
                                                                           sound_baseline,
                                                                           SOUND_TRIGGER_THRESHOLD,
                                                                           now,
                                                                           &is_sound_triggered,
                                                                           &last_sound_trigger_time);
                    }

#if (LOG_OUTPUT_MODE)
                    if (light_trigger_state_changed && is_light_triggered)
                        ESP_LOGI(TAG, "LIGHT TRIGGER!");

                    if (sound_trigger_state_changed && is_sound_triggered)
                        ESP_LOGI(TAG, "SOUND TRIGGER!");
#else
                    // Update key press state based on current trigger state
                    if (light_trigger_state_changed || sound_trigger_state_changed)
                    {
                        if (is_light_triggered && !is_sound_triggered)
                        {
                            uint8_t keycode[6] = {HID_KEY_F13};
                            tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
                        }
                        else if (!is_light_triggered && is_sound_triggered)
                        {
                            uint8_t keycode[6] = {HID_KEY_F14};
                            tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
                        }
                        else if (is_light_triggered && is_sound_triggered)
                        {
                            uint8_t keycode[6] = {HID_KEY_F13, HID_KEY_F14};
                            tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
                        }
                        else
                            tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
                    }
#endif

#if (LOG_OUTPUT_MODE && LOG_CURRENT_READINGS)
                    ESP_LOGI(TAG, "Light baseline: %d, Light: %d (%d samples), Sound baseline: %d, Sound: %d (%d samples)",
                             light_baseline,
                             light,
                             light_sample_count,
                             sound_baseline,
                             sound,
                             sound_sample_count);
#endif
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
