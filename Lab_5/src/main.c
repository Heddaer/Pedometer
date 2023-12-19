#include <stdio.h>
#include <stdlib.h>
#include <driver/i2c.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <esp_pm.h>
#include <freertos/FreeRTOS.h> 
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/semphr.h>
#include <math.h>

#include "getting_acceleration.h" 
#include "circular_buffer.h"

#define TASK_PRIORITY 10
#define ALGO_PRIORITY 9
#define LED_PRIORITY 8

// the reason why I have chosen this sampling frequency is because I take about 100 step/minut, about 2 steps/s 
// T=1/100
// f= 1/(1/100) = 100
// So my sampling period is 100
#define SAMPLING_PERIOD 100
// number of samples to he held inside the buffer: 20 because it is enough to get a valid step. 
#define BUFF_SIZE 20 
// I have chosen to run the algorithm every 2 s to get enough samples 
#define ALGO_PERIOD 2000
// minimum SD to avoid converging to 0
#define MIN_SD 600
// constant applied to SD to detect steps 
#define K 1.5
// minimum time between steps, 
//this value is chosen because we take 2 steps/s 
#define MIN_INTRA_STEP_TIME 500

#define STEPS_GOAL 20

#define LED_PIN 26 
#define BUTTON_PIN 14
#define PUSH_TIME_US 250000

struct circularBuffer buffer;
int step_count = 0; 
SemaphoreHandle_t xSemaphore = NULL;
static volatile uint64_t lastPush = -PUSH_TIME_US;


static void sampling_task(void *arg) 
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) 
    {
        // get the magnitude of the acceleration
        int acceleration = (int) calculatedAcceleration();
        //add acceleration value to buffer
        addElement(&buffer, acceleration);
        // print the content of the magnitude 
        //printBuffer(&buffer);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SAMPLING_PERIOD));
    }
}

static void algo_task(void *arg) 
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) 
    {
        if (buffer.numberOfElements > 0) 
        {
            // Compute mean
            int mean = 0;
            // Counting on head = 0 || head<tail
            for (int i = 0; i < BUFF_SIZE; i++)
            {
                mean += buffer.data[i];
            }
            mean = (mean / BUFF_SIZE);

            // Compute SD
            int sd = 0;
            for (int i = 0; i < BUFF_SIZE; i++)
            {
                sd += (pow((buffer.data[i] - mean), 2));
            }
            sd = sqrt(sd / BUFF_SIZE);

            if (sd < MIN_SD)
            {
                sd = MIN_SD;
            }

            // Count the steps, while also emptying the queue
            int lastStepTS = -MIN_INTRA_STEP_TIME;
            for (int i = 0; i < BUFF_SIZE; i++) 
            {
                // get sample, removing it from queue
                int sample = buffer.data[i];
                removeHead(&buffer);
                // if sample > mean + K * sd
                // AND if time between last step and this sample is > MIN_INTRA_STEP_TIME
                int validation = (mean + K * sd);
            
                if (sample > validation && (i * SAMPLING_PERIOD - lastStepTS) > MIN_INTRA_STEP_TIME)
                {
                    // step found!
                    step_count++;
                    lastStepTS = i * SAMPLING_PERIOD;
                }
            } 
        }
        printf("Steps: %d\n", step_count);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ALGO_PERIOD)); 
    }
}

// button pressed ISR
void button_isr_handler(void *arg) 
{
    // give semaphore (pay attention: this is in an ISR!)
    uint64_t now = esp_timer_get_time();

    if((now-lastPush)>PUSH_TIME_US)
    {
        lastPush = now;
        xSemaphoreGiveFromISR(xSemaphore, NULL);
    }
}

//task will react on button clicks
void led_task(void *arg) 
{
    while (1) 
    {
    // wait for semaphore
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
             // flash LED with sequence depending on if step_count > STEPS_GOAL
             if (step_count >= STEPS_GOAL)
             {
                 gpio_set_level(LED_PIN, 1);
             }

             else if (step_count < STEPS_GOAL/2)
             {
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(500/ portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 0); 
             }
             else
             {
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 0); 
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 0); 
             }
        } 
        else
        {
            //semaphore was not obtained (timeout)
        }
    }
}



void app_main()
{
    // configure light sleep mode with esp_pm_configure()
   /* esp_pm_config_esp32_t lightSleep;
    lightSleep.max_freq_mhz = 80;
    lightSleep.min_freq_mhz = 13;
    lightSleep.light_sleep_enable = true;
    esp_err_t res = esp_pm_configure(&lightSleep);
    ESP_ERROR_CHECK(res); */

    // Configure I2C & MPU6050
    initI2C();

    // Configure led and button
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_DEF_INPUT);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);

    // Create buffer
    int *buffer_data = (int *)malloc(BUFF_SIZE * sizeof(int));
    initCircularBuffer(&buffer, buffer_data, BUFF_SIZE);

    xSemaphore = xSemaphoreCreateBinary();

    // Create tasks
    xTaskCreate(sampling_task, "sampling", 2048, NULL, TASK_PRIORITY, NULL);
    xTaskCreate(algo_task, "algo", 2048, NULL, ALGO_PRIORITY, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, LED_PRIORITY, NULL);
}