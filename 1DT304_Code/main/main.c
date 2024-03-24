#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_system.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/Task.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <inttypes.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "esp_task_wdt.h"
#include "ttn.h"

// Sensor, led pins
#define TRIG_PIN GPIO_NUM_2
#define ECHO_PIN GPIO_NUM_4
#define LED_PIN GPIO_NUM_17

// Maximum brightness value for PWM
#define LED_MAX_BRIGHTNESS 255
static bool adjust_brightness = true;

#define ESP_CHANNEL 1
#define LED_STRIP 8
#define LED_STRIP_MAX_LEDS 1

// Mac address for other device
static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0x78, 0x21, 0x84, 0x9f, 0x02, 0x24};

static const char *TAG = "esp_now_init";

static esp_err_t init_wifi(void);
static int counter = 0;

#define DEFAULT_VREF 1100 // Voltage reference
#define NO_OF_SAMPLES 64  // Number of samples for ADC
#define LDR_THRESHOLD 75  // Threshold value for LDR (in mV)

// TTN info:
const char *appEui = "0000000000000000";
const char *devEui = "70B3D54993426209";
const char *appKey = "1B2C89AAAC5F6E2ED627260B53118597";

// Pins and other resources for LoRa
#define TTN_SPI_HOST SPI2_HOST
#define TTN_SPI_DMA_CHAN SPI_DMA_DISABLED
#define TTN_PIN_SPI_SCLK 5
#define TTN_PIN_SPI_MOSI 27
#define TTN_PIN_SPI_MISO 19
#define TTN_PIN_NSS 18
#define TTN_PIN_RXTX TTN_NOT_CONNECTED
#define TTN_PIN_RST 14
#define TTN_PIN_DIO0 26
#define TTN_PIN_DIO1 35

// Interval for sending data via LoRa in milliseconds
#define TX_INTERVAL 50

// ADC characteristics
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

/**
 * @brief Initialize ADC configuration.
 */
void initialize_adc()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, atten);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

/**
 * @brief Read the value from (LDR)
 * @return Voltage value read from the LDR sensor.
 */
uint32_t read_ldr_value()
{
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        adc_reading += adc1_get_raw(ADC1_CHANNEL_6);
    }
    adc_reading /= NO_OF_SAMPLES;
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return voltage;
}

/**
 * @brief Display the LDR value.
 * @param ldr_value The LDR value to display.
 */
void display_ldr_value(uint32_t ldr_value)
{
    printf("LDR Value: %ld mV\n", ldr_value);
}

/**
 * @brief Check if the LDR value is high.
 * @param ldr_value The LDR value to check.
 * @return true if the LDR value is high, false otherwise.
 */
bool is_ldr_value_high(uint32_t ldr_value)
{
    if (ldr_value > LDR_THRESHOLD)
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Initialize the ultrasonic sensor configuration.
 */
void ultrasonic_init()
{
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // Initialize LEDC peripheral
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = LED_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0};
    ledc_channel_config(&ledc_channel);
}

/**
 * @brief Measure distance using the ultrasonic sensor.
 * @return The measured distance in centimeters.
 */
uint32_t ultrasonic_measure()
{
    gpio_set_level(TRIG_PIN, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    gpio_set_level(TRIG_PIN, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(TRIG_PIN, 0);

    // Measure the length of echo signal
    uint32_t start_time = esp_timer_get_time();
    uint32_t echo_level = gpio_get_level(ECHO_PIN);
    while (echo_level == 0)
    {
        start_time = esp_timer_get_time();
        echo_level = gpio_get_level(ECHO_PIN);
    }

    uint32_t end_time = esp_timer_get_time();
    while (echo_level == 1)
    {
        end_time = esp_timer_get_time();
        echo_level = gpio_get_level(ECHO_PIN);
    }
    // Calculate duration of echo pulse in microseconds
    uint32_t pulse_duration = end_time - start_time;
    uint32_t distance = (pulse_duration * 0.0343) / 2;

    return distance;
}

// Implementation for wifi (ESP_NOW)

/**
 * @brief Callback function when data is received via ESP-NOW.
 * @param esp_now_info Information about the received data.
 * @param data Pointer to the received data.
 * @param data_len Length of the received data.
 */
void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    adjust_brightness = false;
    ESP_LOGI(TAG, "Data received from " MACSTR ": %.*s", MAC2STR(esp_now_info->src_addr), data_len, (char *)data);
    ESP_LOGI(TAG, "Turn on the led");
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 250);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    adjust_brightness = true;
}

/**
 * @brief Callback function after data transmission via ESP-NOW.
 * @param mac_addr MAC address of the receiver.
 * @param status Transmission status.
 */
void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "Data delivered successfully");
    }
    else
    {
        ESP_LOGI(TAG, "Failed to deliver data");
    }
}

/**
 * @brief Initialize ESP-NOW communication.
 */
static esp_err_t init_esp_now(void)
{
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);

    ESP_LOGI(TAG, "ESP-NOW initialized");
    return ESP_OK;
}

/**
 * @brief Register a peer device for ESP-NOW communication.
 * @param peer_addr MAC address of the peer device.
 * @return ESP_OK if successful, otherwise ESP_FAIL.
 */
static esp_err_t register_peer(uint8_t *peer_addr)
{
    esp_now_peer_info_t peer_info;
    memcpy(peer_info.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESP_CHANNEL;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    peer_info.encrypt = false;
    if (esp_now_add_peer(&peer_info) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add peer");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Send data using ESP-NOW.
 * @param data Pointer to the data to be sent.
 * @param len Length of the data.
 * @return ESP_OK if successful, otherwise ESP_FAIL.
 */
static esp_err_t esp_now_send_data(const uint8_t *data, size_t len)
{
    if (esp_now_send(peer_mac, data, len) != ESP_OK)
    {
        ESP_LOGI(TAG, "Send Error");
    }
    else
    {
        ESP_LOGI(TAG, "Data sent successfullyy");
    }
    return ESP_OK;
}

/**
 * @brief Initialize the ADC, ESP-NOW, and Wi-Fi.
 */
static esp_err_t init_wifi(void)
{
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_netif_init();
    esp_event_loop_create_default();
    nvs_flash_init();
    esp_wifi_init(&wifi_init_config);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_start();
    ESP_LOGI(TAG, "Wi-Fi initialized");
    return ESP_OK;
}

/**
 * @brief Task to send LoRaWAN messages periodically.
 * @param pvParameter Task parameter (unused).
 */
void sendMessages(void *pvParameter)
{
    while (1)
    {
        printf("Sending message...\n");
        // Convert byte
        uint8_t counter_bytes[sizeof(int)];
        memcpy(counter_bytes, &counter, sizeof(int));
        ttn_response_code_t res = ttn_transmit_message(counter_bytes, sizeof(int), 1, false);
        printf(res == TTN_SUCCESSFUL_TRANSMISSION ? "Message sent.\n" : "Transmission failed.\n");
        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL));
    }
}

/**
 * @brief Callback function for received LoRaWAN messages.
 * @param message Pointer to the received message.
 * @param length Length of the received message.
 * @param port Port number through which the message was received.
 */
void messageReceived(const uint8_t *message, size_t length, ttn_port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}

/**
 * @brief Initialize The Things Network (TTN) communication.
 */
void initializeTTN()
{
    esp_err_t err;
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);
    spi_bus_config_t spi_bus_config = {
        .miso_io_num = TTN_PIN_SPI_MISO,
        .mosi_io_num = TTN_PIN_SPI_MOSI,
        .sclk_io_num = TTN_PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    ttn_init();
    ttn_configure_pins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);
    ttn_provision(devEui, appEui, appKey);
    ttn_on_message(messageReceived);
}

/**
 * @brief Task to perform ultrasonic distance measurement.
 * @param pvParameters Task parameter.
 */
void ultrasonic_task(void *pvParameters)
{
    ultrasonic_init();
    initialize_adc();

    while (1)
    {
        uint32_t ldr_value = read_ldr_value();
        if (is_ldr_value_high(ldr_value))
        {
            // If LDR value is high, wait for a shorter duration before re-measuring
            vTaskDelay(pdMS_TO_TICKS(500)); // Shorter delay before re-measuring
            continue;                       // Skip the rest of the loop iteration
        }
        uint32_t distance = ultrasonic_measure();
        ESP_LOGI("Ultrasonic", "Measured Distance: %" PRIu32 " cm", distance);

        // Adjust LED brightness based on distance
        if (adjust_brightness)
        {
            if (distance < 100)
            { // If the distance is less than 100 cm
                // Calculate LED brightness based on distance
                uint8_t brightness = LED_MAX_BRIGHTNESS - (distance * 4);
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, brightness);
                // Update PWM signal
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
                // Sending number 4 to other device (Activation code)
                int data_to_send = 4;
                counter++;
                if (counter > 10)
                {
                    counter = 0;
                }
                esp_now_send_data((uint8_t *)&data_to_send, sizeof(int));
            }
            else
            {
                // If the distance is greater than or equal to 100 cm, turn off the LED
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 20);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Application entry point.
 */
void app_main(void)
{
    initializeTTN();
    if (counter > 10)
        printf("counter: %d\n", counter);
    {
        printf("Joining...\n");
        if (ttn_join())
        {
            printf("Joined.\n");
            xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void *)0, 3, NULL);
        }
        else
        {
            printf("Join failed. Goodbye\n");
        }
    }

    ESP_ERROR_CHECK(init_wifi());
    ESP_ERROR_CHECK(init_esp_now());
    ESP_ERROR_CHECK(register_peer(peer_mac));

    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 10, NULL);
}
