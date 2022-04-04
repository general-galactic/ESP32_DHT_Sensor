// DHT11/22 Humidity Sensor API
//
// REFERENCES (and thanks)
// https://circuits4you.com/2019/01/25/esp32-dht11-22-humidity-temperature-sensor-interfacing-example/#google_vignette
// https://github.com/beegee-tokyo/DHTesp
// https://www.electronicwings.com/sensors-modules/dht11#:~:text=DHT11%20is%20a%20single%20wire,0%20to%2050%20%C2%B0C).

#include "sensors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include <stdbool.h>

#define LOG_TAG "DHT"

// Match to the DHT protocol. These are for the DHT
#define OUTPUT_HOLD_START_US            18000   // 18 ms to hold output low to request data
#define INPUT_DURATION_TIMEOUT_MS       100     // it takes longer to get the bits from the queue than it takes to receive the message
#define QUEUE_YIELD_MS                  5       // minimum time resolution for overall timeout detection, smaller enables faster reads
#define MINIMUM_HIGH_BIT_DURATION_US    50      // < 30 is a zero, >= 70us is a one

// Use the fastest run-time, most accurate microsecond timer available on your platform
#define GET_TIME_US                     esp_timer_get_time  // 

// Private functions
static void DHT_requestData(int gpio);
static void DHT_hold(int gpio);
static bool DHT_receiveData(int gpio, double *p_humdity, double *p_temperature_c);

//===================== INTERRUPT SECTION =======================//

// On each edge, send the duration of that edge to the queue
// The current time retreival needs to be FAST so choose GET_TIME_US() wisely

static uint64_t intrLastEdgeTimeUs = 0;     // initialize with the result of esp_timer_get_time()
static xQueueHandle gpio_evt_queue = NULL;  // will send edge times in microseconds to the recevier using this queue

static void IRAM_ATTR gpio_isr_handler(void* param){
    uint64_t currentTimeUs = GET_TIME_US();
    uint32_t diffTimeUs = (uint32_t)(currentTimeUs - intrLastEdgeTimeUs);
    intrLastEdgeTimeUs = currentTimeUs;
    xQueueSendFromISR(gpio_evt_queue, &diffTimeUs, NULL);
}

//===============================================================//

//---------------------------
// Private API
//---------------------------

static void DHT_requestData(int gpio){
    gpio_config_t io_conf = {};

    // Start sensor request by holding line low for a period
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << gpio);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_conf);

    gpio_set_level(gpio, 0);
    ets_delay_us(OUTPUT_HOLD_START_US);

    // Start receiving data via interrupt on edges
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << gpio);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}

static void DHT_hold(int gpio){
    gpio_config_t io_conf = {};
        
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << gpio);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(gpio, 1);    // hold high when not requesting data
}

/**
 * @brief Receive edges from the gpio interrupt and convert to humidity and temp bits
 * 
 * @param gpio, to which io the sensor is connected
 * @param p_humdity, will contain the humdity data
 * @param p_temperature_c, will contain the temperature data
 * @return true: the read was successfull and the returned data are valid
 * @return false: the read failed (likely timeout) and returned data are invalid
 */
static bool DHT_receiveData(int gpio, double *p_humdity, double *p_temperature_c){
    uint64_t startTimerUs = GET_TIME_US();
    uint32_t elapsedMs = 0;         // for timeout

    // Capture level durations ended at each edge (eg, high level duration end on a falling edge)
    // At each falling edge, the duration of the high level indicates the bit value
    // Note that the first 5 edges are the request and start bit
    // even edges are rising edges, and end the 54us low before each data bit
    // odd edges are falling edges, after the 5th falling edge, each data bit is defined by the length of the high

    uint8_t level = 0;              // the level we have been at (we already pulled the line low to request data, so starts low)
    uint32_t edgeCounter = 0;       // index of the edge that just happened
    uint32_t durationUs = 0;        // duration spend at that level
    uint8_t bitIndex = 0;

    uint16_t rawHumidity = 0;       // collected bits 0-15
    uint16_t rawTempC = 0;          // collected bits 16-32
    uint8_t checksum = 0;           // collected bits 33-40

    do{
        // will return false after QUEUE_YIELD_MS if no data received
        if( xQueueReceive(gpio_evt_queue, &durationUs, QUEUE_YIELD_MS / portTICK_PERIOD_MS) ) {
            // printf("Edge[%d] (%d) durationUs=%d\n", edgeCounter, level, durationUs);
            edgeCounter++;
            
            // Grab the data bits from the duration of the highs, 5th edge and beyond
            // First 16 bits are humdity (8.8)
            // Second 16 bits are tempC (8.8)
            // Last 8 bits a checksum
            if( level == 1 && edgeCounter >= 5 ){
                uint8_t bitValue = durationUs > MINIMUM_HIGH_BIT_DURATION_US;

                if( bitIndex < 16 ){
                    rawHumidity = (rawHumidity << 1) | bitValue;
                } else if (bitIndex < 32 ){
                    rawTempC = (rawTempC << 1) | bitValue;
                } else {
                    checksum = (checksum << 1 ) | bitValue;
                }

                bitIndex++;
            }

            level = !level;
        }

        elapsedMs = (esp_timer_get_time() - startTimerUs) / 1000;

    }while( edgeCounter < 85 && elapsedMs < INPUT_DURATION_TIMEOUT_MS );

    // Check for loop-ended on timeout
    if( edgeCounter < 85 && elapsedMs >= INPUT_DURATION_TIMEOUT_MS ){
        ESP_LOGE(LOG_TAG, "sensor read timed out: %d edges; %d bits; waitied for %dms\n", edgeCounter, bitIndex, elapsedMs);
        return false;
    }

    // Check data checksum
    uint8_t computedChecksum = ((rawHumidity >> 8) & 0xff) + (rawHumidity & 0xff) + ((rawTempC >> 8) & 0xff) + (rawTempC & 0xff);
    if( computedChecksum != checksum ){
        ESP_LOGE(LOG_TAG, "checksums do not match: computed=%d; received=%d\n", computedChecksum, checksum);
    }

    // Compute and return values
    *p_humdity = ((double)rawHumidity) / 256;
    *p_temperature_c = ((double)rawTempC) / 256;

    return true;
}

//---------------------------
// Public API
//---------------------------

void DHT_init(int gpio){
    gpio_install_isr_service(0);
    DHT_hold(gpio);
}

bool DHT_sample(int gpio, double *p_humdity, double *p_temperature_c ){
    // Create a queue to handle gpio event from isr and register handler
    gpio_evt_queue = xQueueCreate(100, sizeof(uint32_t));   // extra capacity for the edges because we are slow to retrieve them

    // Initialize the interrupt handler and hook it up
    intrLastEdgeTimeUs = GET_TIME_US();
    gpio_isr_handler_add(gpio, gpio_isr_handler, NULL);

    DHT_requestData(gpio);

    // Receive 40 bytes or timeout
    bool success = DHT_receiveData(gpio, p_humdity, p_temperature_c);
    if( success ){
        ESP_LOGI(LOG_TAG, "%.0f%% humidity, %.1fC, %.1fF\n", *p_humdity, *p_temperature_c, *p_temperature_c * 9 / 5 + 32);
    }
    
    // clean up interrupts and queue
    gpio_isr_handler_remove(gpio);
    vQueueDelete(gpio_evt_queue);

    DHT_hold(gpio);

    return true;
}
