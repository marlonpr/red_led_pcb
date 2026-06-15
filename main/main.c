#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include <stdint.h>

#define DATA_PIN    GPIO_NUM_25
#define CLOCK_PIN   GPIO_NUM_26
#define STROBE_PIN  GPIO_NUM_22

#define PWM_STEPS           16
#define PWM_STEP_TIME_US    500

#define DISPLAY_4DIGIT_COUNT  2
#define DISPLAY_3DIGIT_COUNT  2

#define FOUR_DIGIT_BYTES      4
#define THREE_DIGIT_BYTES     3

#define TOTAL_DIGITS \
    ((DISPLAY_4DIGIT_COUNT * FOUR_DIGIT_BYTES) + \
     (DISPLAY_3DIGIT_COUNT * THREE_DIGIT_BYTES))
	 
 const int segment_to_q[8] = {
     2, 0, 1, 3, 5, 4, 6, 7
 };

 const uint8_t digits_abcdefg[10] = {
     0b0111111, // 0
     0b0000101, // 1
     0b1101110, // 2
     0b1001111, // 3
     0b1010101, // 4
     0b1011011, // 5
     0b1111011, // 6
     0b0000111, // 7
     0b1111111, // 8
     0b1011111  // 9
 };

uint8_t digit_map[10];


 uint8_t digit_brightness[TOTAL_DIGITS] = {
     // 4-digit display 1
     120, 255, 255, 255,

     // 4-digit display 2
     120, 255, 255, 255,

     // 3-digit display 1
     255, 255, 255,

     // 3-digit display 2
     255, 255, 255
 };
	 
	 
 static inline void pulse_pin(gpio_num_t pin)
 {
     gpio_set_level(pin, 1);
     esp_rom_delay_us(1);
     gpio_set_level(pin, 0);
 }

 void build_digit_map(void)
 {
     for (int digit = 0; digit < 10; digit++) {
         uint8_t output = 0;

         for (int segment = 0; segment < 7; segment++) {
             if (digits_abcdefg[digit] & (1U << segment)) {
                 output |= (1U << segment_to_q[segment]);
             }
         }

         digit_map[digit] = output;
     }
 }

 void cd4094_shift(uint8_t data)
 {
     for (int bit = 7; bit >= 0; bit--) {
         gpio_set_level(DATA_PIN, (data >> bit) & 1U);
         pulse_pin(CLOCK_PIN);
     }
 }

 void cd4094_latch(void)
 {
     pulse_pin(STROBE_PIN);
 }
	 
	 
	 

	 void make_4digit_display_bytes(
	     int number,
	     bool dp_on,
	     uint8_t output[4])
	 {
	     if (number < 0) {
	         number = 0;
	     }

	     number %= 10000;

	     int digits[4] = {
	         (number / 1000) % 10,
	         (number / 100)  % 10,
	         (number / 10)   % 10,
	         number % 10
	     };

	     for (int i = 0; i < 4; i++) {
	         output[i] = digit_map[digits[i]];

	         // The only physical DP is connected to digit/register index 1.
	         if (i == 1 && dp_on) {
	             output[i] |= (1U << segment_to_q[7]);
	         }
	     }
	 }

	 void make_3digit_display_bytes(
	     int number,
	     bool dp_on,
	     uint8_t output[3])
	 {
	     if (number < 0) {
	         number = 0;
	     }

	     number %= 1000;

	     int digits[3] = {
	         (number / 100) % 10,
	         (number / 10)  % 10,
	         number % 10
	     };

	     for (int i = 0; i < 3; i++) {
	         output[i] = digit_map[digits[i]];

	         // One physical DP connected to digit/register index 1.
	         if (i == 1 && dp_on) {
	             output[i] |= (1U << segment_to_q[7]);
	         }
	     }
	 }
 
 
 
 
	 void display_four_numbers_pwm(
	     int number_4digit_1,
	     bool dp_on_4digit_1,

	     int number_4digit_2,
	     bool dp_on_4digit_2,

	     int number_3digit_1,
	     bool dp_on_3digit_1,

	     int number_3digit_2,
	     bool dp_on_3digit_2)
	 {
	     uint8_t display_bytes[TOTAL_DIGITS];

	     // Bytes 0–3
	     make_4digit_display_bytes(
	         number_4digit_1,
	         dp_on_4digit_1,
	         &display_bytes[0]);

	     // Bytes 4–7
	     make_4digit_display_bytes(
	         number_4digit_2,
	         dp_on_4digit_2,
	         &display_bytes[4]);

	     // Bytes 8–10
	     make_3digit_display_bytes(
	         number_3digit_1,
	         dp_on_3digit_1,
	         &display_bytes[8]);

	     // Bytes 11–13
	     make_3digit_display_bytes(
	         number_3digit_2,
	         dp_on_3digit_2,
	         &display_bytes[11]);

	     for (int pwm_step = 0; pwm_step < PWM_STEPS; pwm_step++) {

	         for (int digit = 0; digit < TOTAL_DIGITS; digit++) {

	             uint16_t pwm_level =
	                 ((uint16_t)digit_brightness[digit] * PWM_STEPS) / 255;

	             if (pwm_step < pwm_level) {
	                 cd4094_shift(display_bytes[digit]);
	             } else {
	                 cd4094_shift(0);
	             }
	         }

	         cd4094_latch();
	         esp_rom_delay_us(PWM_STEP_TIME_US);
	     }
	 }
 
 
 
 
 
 
 
 void app_main(void)
 {
     gpio_config_t io_conf = {};

     io_conf.pin_bit_mask =
         (1ULL << DATA_PIN) |
         (1ULL << CLOCK_PIN) |
         (1ULL << STROBE_PIN);

     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
     io_conf.intr_type = GPIO_INTR_DISABLE;

     ESP_ERROR_CHECK(gpio_config(&io_conf));

     gpio_set_level(DATA_PIN, 0);
     gpio_set_level(CLOCK_PIN, 0);
     gpio_set_level(STROBE_PIN, 0);

     build_digit_map();

     int counter_1 = 0;
     int counter_2 = 1000;
	 int counter_3 = 10;
	 int counter_4 = 100;

     bool dp_state = false;

     TickType_t last_dp_toggle = xTaskGetTickCount();
     TickType_t last_counter_update = xTaskGetTickCount();

     vTaskDelay(pdMS_TO_TICKS(100));

	 while (1) {
	     TickType_t now = xTaskGetTickCount();

	     if ((now - last_dp_toggle) >= pdMS_TO_TICKS(1000)) {
	         dp_state = !dp_state;
	         last_dp_toggle = now;
	     }

	     if ((now - last_counter_update) >= pdMS_TO_TICKS(1000)) {
	         counter_1 = (counter_1 + 1) % 10000;
	         counter_2 = (counter_2 + 10) % 10000;

	         counter_3 = (counter_3 + 1) % 1000;
	         counter_4 = (counter_4 + 5) % 1000;

	         last_counter_update = now;
	     }

		 display_four_numbers_pwm(
		     8888,
		     dp_state,

		     8888,
		     dp_state,

		     888,
		     dp_state,

		     888 ,
		     dp_state
		 );

	     vTaskDelay(pdMS_TO_TICKS(1));
	 }
 } 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

/*
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include <stdint.h>

#define DATA_PIN    GPIO_NUM_25
#define CLOCK_PIN   GPIO_NUM_26
#define STROBE_PIN  GPIO_NUM_22

#define DIGITS_PER_DISPLAY 4
#define DISPLAY_COUNT      2
#define TOTAL_DIGITS       (DIGITS_PER_DISPLAY * DISPLAY_COUNT)

#define PWM_STEPS           16
#define PWM_STEP_TIME_US    500

const int segment_to_q[8] = {
    2, 0, 1, 3, 5, 4, 6, 7
};

const uint8_t digits_abcdefg[10] = {
    0b0111111, // 0
    0b0000101, // 1
    0b1101110, // 2
    0b1001111, // 3
    0b1010101, // 4
    0b1011011, // 5
    0b1111011, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1011111  // 9
};

uint8_t digit_map[10];


 
 // indexes 0–3 = display 1
 // indexes 4–7 = display 2
 
uint8_t digit_brightness[TOTAL_DIGITS] = {
    120, 255, 255, 255,
    120, 255, 255, 255
};

static inline void pulse_pin(gpio_num_t pin)
{
    gpio_set_level(pin, 1);
    esp_rom_delay_us(1);
    gpio_set_level(pin, 0);
}

void build_digit_map(void)
{
    for (int digit = 0; digit < 10; digit++) {
        uint8_t output = 0;

        for (int segment = 0; segment < 7; segment++) {
            if (digits_abcdefg[digit] & (1U << segment)) {
                output |= (1U << segment_to_q[segment]);
            }
        }

        digit_map[digit] = output;
    }
}

void cd4094_shift(uint8_t data)
{
    for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(DATA_PIN, (data >> bit) & 1U);
        pulse_pin(CLOCK_PIN);
    }
}

void cd4094_latch(void)
{
    pulse_pin(STROBE_PIN);
}



void make_display_bytes(
    int number,
    bool dp_on,
    uint8_t output[4])
{
    if (number < 0) {
        number = 0;
    }

    number %= 10000;

    int digits[4] = {
        (number / 1000) % 10,
        (number / 100)  % 10,
        (number / 10)   % 10,
        number % 10
    };

    for (int i = 0; i < 4; i++) {
        output[i] = digit_map[digits[i]];

        // Each display has only one physical DP,
        // connected to digit/register index 1.
        if (i == 1 && dp_on) {
            output[i] |= (1U << segment_to_q[7]);
        }
    }
}


void display_two_numbers_pwm(
    int number_1,
    bool dp_on_1,
    int number_2,
    bool dp_on_2)
{
    uint8_t display_bytes[TOTAL_DIGITS];

    make_display_bytes(
        number_1,
        dp_on_1,
        &display_bytes[0]);

    make_display_bytes(
        number_2,
        dp_on_2,
        &display_bytes[4]);

    for (int pwm_step = 0; pwm_step < PWM_STEPS; pwm_step++) {

        for (int digit = 0; digit < TOTAL_DIGITS; digit++) {

            uint16_t pwm_level =
                ((uint16_t)digit_brightness[digit] * PWM_STEPS) / 255;

            if (pwm_step < pwm_level) {
                cd4094_shift(display_bytes[digit]);
            } else {
                cd4094_shift(0);
            }
        }

        cd4094_latch();
        esp_rom_delay_us(PWM_STEP_TIME_US);
    }
}

void app_main(void)
{
    gpio_config_t io_conf = {};

    io_conf.pin_bit_mask =
        (1ULL << DATA_PIN) |
        (1ULL << CLOCK_PIN) |
        (1ULL << STROBE_PIN);

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(DATA_PIN, 0);
    gpio_set_level(CLOCK_PIN, 0);
    gpio_set_level(STROBE_PIN, 0);

    build_digit_map();

    int counter_1 = 0;
    int counter_2 = 500;

    bool dp_state = false;

    TickType_t last_dp_toggle = xTaskGetTickCount();
    TickType_t last_counter_update = xTaskGetTickCount();

    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        TickType_t now = xTaskGetTickCount();

        if ((now - last_dp_toggle) >= pdMS_TO_TICKS(1000)) {
            dp_state = !dp_state;
            last_dp_toggle = now;
        }

        if ((now - last_counter_update) >= pdMS_TO_TICKS(1000)) {
            counter_1 = (counter_1 + 1) % 10000;
            counter_2 = (counter_2 + 10) % 10000;

            last_counter_update = now;
        }

		display_two_numbers_pwm(
		    counter_1,
		    dp_state,

		    counter_2,
		    !dp_state
		);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

*/














// ============ V1 ============

/*
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdint.h>

#define DATA_PIN    25
#define CLOCK_PIN   26
#define STROBE_PIN  22

const int segment_to_q[8] = {2,0,1,3,5,4,6,7};

const uint8_t digits_abcdefg[10] = {
    0b0111111,0b0000101,0b1101110,0b1001111,
    0b1010101,0b1011011,0b1111011,0b0000111,
    0b1111111,0b1011111
};

uint8_t digit_map[10];

// --- Per-digit brightness 0–255 ---
uint8_t digit_brightness[4] = {255,255,255,255};

// --- Build digit_map ---
void build_digit_map() {
    for(int d=0; d<10; d++) {
        uint8_t byte = 0;
        for(int seg=0; seg<7; seg++)
            if(digits_abcdefg[d] & (1 << seg))
                byte |= (1 << segment_to_q[seg]);
        digit_map[d] = byte;
    }
}

// --- Pulse a GPIO pin ---
static inline void pulse_pin(gpio_num_t pin) {
    gpio_set_level(pin,1);
    esp_rom_delay_us(1);
    gpio_set_level(pin,0);
}

// --- Shift 8 bits to CD4094 ---
void cd4094_shift(uint8_t data){
    for(int i=7;i>=0;i--){
        gpio_set_level(DATA_PIN,(data>>i)&1);
        pulse_pin(CLOCK_PIN);
    }
}

// --- Latch shifted data ---
void cd4094_latch(void){
    pulse_pin(STROBE_PIN);
}

// --- Display a 4-digit number with DP using PWM ---
void display_number_pwm(int number, int dp_on) {
    int digits[4] = {(number/1000)%10,(number/100)%10,(number/10)%10,number%10};
    const int PWM_STEPS = 16;

    for(int step=0; step<PWM_STEPS; step++) {
		for(int i=0;i<4;i++) {
		
		
		    uint8_t byte = digit_map[digits[i]];
		
		    if(i==1 && dp_on)
		        byte |= (1<<segment_to_q[7]);
		
		    if(step < ((digit_brightness[i]*PWM_STEPS)/255))
		        cd4094_shift(byte);
		    else
		        cd4094_shift(0);
		}

        cd4094_latch();
        esp_rom_delay_us(500);
    }
}

// --- Automatically adjust digit brightness ---
void adjust_brightness() {
    // Rough compensation: reduce first digit slightly
    digit_brightness[0] = 120; // adjust experimentally
    digit_brightness[1] = 255;
    digit_brightness[2] = 255;
    digit_brightness[3] = 255;
}

void app_main(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<DATA_PIN)|(1ULL<<CLOCK_PIN)|(1ULL<<STROBE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en =0,
        .pull_up_en=0,
        .intr_type=GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    build_digit_map();
    adjust_brightness();
	
    int counter = 0;
    int dp_state = 0;
    TickType_t last_dp_toggle = xTaskGetTickCount();
    TickType_t last_counter_update = xTaskGetTickCount();

	vTaskDelay(pdMS_TO_TICKS(100));

    while(1) 
{
        TickType_t now = xTaskGetTickCount();
		vTaskDelay(pdMS_TO_TICKS(1));

        // DP blink
        if(now - last_dp_toggle >= pdMS_TO_TICKS(1000)){
            dp_state = !dp_state;
            last_dp_toggle = now;
        }

        // Counter update
        if(now - last_counter_update >= pdMS_TO_TICKS(1000)){
            counter++;
            last_counter_update = now;
        }

        // Display with PWM
		display_number_pwm(counter, dp_state);  
    }
}

*/














/*
// Turn OFF the two left-most digits (digit 0 and 1)
if (i < 2) {
    cd4094_shift(0);
    continue;
}
*/













/*
// LED setup
gpio_reset_pin(LED_GPIO);
gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
gpio_set_level(LED_GPIO, 0);
*/









//LORA
/*
//-------------------------------------------------------------LORA--------------------------------------------------------
//✅ Unified Master/Slave Code
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora.h"

// ------------------ Config ------------------
#define LED_GPIO        GPIO_NUM_2
#define DEVICE_ID       0       // 0 = Master, 1..N-1 = Slave
#define NUM_DEVICES     3
#define FREQ_HZ         433000000
#define TX_INTERVAL_MS  1000
#define LOST_TIMEOUT_MS 10000
#define BLINK_PERIOD_MS 1000

static const char *TAG = "LORA_NODE";

// ------------------ IRQ flags (SX1278) ------------------
#define IRQ_TX_DONE       0x08
#define IRQ_RX_DONE       0x40
#define IRQ_VALID_HEADER  0x10
#define IRQ_CRC_ERROR     0x20
#define IRQ_RX_TIMEOUT    0x80
#define IRQ_ALL           0xFF

// ------------------ Globals ------------------
static uint32_t last_rx_tick = 0;
static uint32_t last_timestamp = 0;
static bool link_lost = false;

// ------------------ TX function (Master) ------------------
void send_timestamp(uint32_t ts)
{
    uint8_t buf[5];
    buf[0] = 0xAA;
    buf[1] = (ts >> 24) & 0xFF;
    buf[2] = (ts >> 16) & 0xFF;
    buf[3] = (ts >> 8) & 0xFF;
    buf[4] = ts & 0xFF;

    // Standby
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(1));

    // FIFO pointer
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x80);

    // Clear IRQs
    lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);

    // Write payload
    for (int i = 0; i < 5; i++)
        lora_write_reg(REG_FIFO, buf[i]);
    lora_write_reg(REG_PAYLOAD_LENGTH, 5);

    // PA config
    lora_write_reg(REG_PA_CONFIG, 0x8F);

    // Start TX
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    // Wait TX_DONE
    int timeout = 200;
    while (!(lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE) && timeout--) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------

*/








/*
static ds18b20_t sensor;

static int16_t current_temp = 0;
static bool temp_valid = false;
*/

/*
ds18b20_init(&sensor, GPIO_NUM_13); // Use GPIO4 with 4.7kΩ pull-up resistor


xTaskCreatePinnedToCore(temp_task,      "TempTask",      1024, NULL, 1, NULL, 1);
*/







/*
// LoRa init
if (lora_init() != ESP_OK) {

	    ESP_LOGE(TAG, "LoRa init failed");
    return;
}
lora_set_frequency(FREQ_HZ);

ESP_LOGI(TAG, "Device %d started on %.1f MHz", DEVICE_ID, FREQ_HZ / 1e6);
   

xTaskCreatePinnedToCore(lora_task,      "LoraTask",      4096, NULL, 1, NULL, 0);
*/









/*
void temp_task(void *arg)
{
    while (1) {
        int16_t t;
        if (ds18b20_read_temperature_int(&sensor, &t) == ESP_OK) 
        {
            current_temp = t;
            if (current_temp < -9)
            {
				current_temp = -9;
			}
            temp_valid = true;
        } else {
            temp_valid = false;
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // read every 5s
    }
}


static void send_packet(uint32_t counter)
{
    int16_t temp = current_temp;   // °C × 100

    uint8_t buf[8] = {
        0xAA,
        DEVICE_ID,
        (temp >> 8) & 0xFF,         // REAL temperature MSB
        temp & 0xFF,                // REAL temperature LSB
        (counter >> 24) & 0xFF,
        (counter >> 16) & 0xFF,
        (counter >> 8)  & 0xFF,
        counter & 0xFF
    };

    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(1));

    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x80);
    lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);

    for (int i = 0; i < sizeof(buf); i++)
        lora_write_reg(REG_FIFO, buf[i]);

    lora_write_reg(REG_PAYLOAD_LENGTH, sizeof(buf));
    lora_write_reg(REG_PA_CONFIG, 0x8F);
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    while (!(lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    lora_write_reg(REG_IRQ_FLAGS, IRQ_ALL);

    ESP_LOGI(TAG,
             "TX temp=%d.%02d°C counter=%lu",
             temp / 100,
             abs(temp % 100),
             counter);
}


void lora_task(void *arg)
{
    uint32_t counter = 0;

    while (1) {
        send_packet(counter++);
        gpio_set_level(LED_GPIO, !gpio_get_level(LED_GPIO));
        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
    }
}



*/
