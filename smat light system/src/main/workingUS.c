#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"


#define TRIG_PIN GPIO_NUM_2
#define ECHO_PIN GPIO_NUM_4
#define LED_PIN GPIO_NUM_5 // Change this to the GPIO pin you connected the LED to

void ultrasonic_init() {
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); // Initialize LED pin

}

uint32_t ultrasonic_measure() {
    // Ensure trigger is low for a clean high pulse later
    gpio_set_level(TRIG_PIN, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS); // Wait 2 milliseconds
    gpio_set_level(TRIG_PIN, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS); // Keep trigger high for 10 milliseconds
    gpio_set_level(TRIG_PIN, 0);

    // Measure the length of echo signal
    uint32_t start_time = esp_timer_get_time();
    uint32_t echo_level = gpio_get_level(ECHO_PIN);
    while (echo_level == 0) {
        start_time = esp_timer_get_time();
        echo_level = gpio_get_level(ECHO_PIN);
    }

    uint32_t end_time = esp_timer_get_time();
    while (echo_level == 1) {
        end_time = esp_timer_get_time();
        echo_level = gpio_get_level(ECHO_PIN);
    }

    // Calculate duration of echo pulse in microseconds
    uint32_t pulse_duration = end_time - start_time;

    // Conversion from time to distance in centimeters
    // Speed of sound = 343 m/s = 0.0343 cm/µs
    // Distance = (Time * Speed of sound) / 2
    uint32_t distance = (pulse_duration * 0.0343) / 2;

    gpio_set_level(LED_PIN, 1);

    return distance;
}

void ultrasonic_task(void *pvParameters) {
    ultrasonic_init();
    while (1) {
        uint32_t distance = ultrasonic_measure();
        ESP_LOGI("Ultrasonic", "Measured Distance: %" PRIu32 " cm", distance);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main() {
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 10, NULL);
}