#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include <stdint.h>

#define DATA_PIN    GPIO_NUM_25
#define CLOCK_PIN   GPIO_NUM_26
#define STROBE_PIN  GPIO_NUM_22

#define PWM_STEPS           32
#define PWM_STEP_TIME_US    250

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
     255, 255, 255, 255,

     // 4-digit display 2
     255, 255, 255, 255,

     // 3-digit display 1
     255, 255, 255,

     // 3-digit display 2
     255, 255, 255
 };
 
 // Global brightness for all displays: 0 = off, 255 = maximum
 volatile uint8_t global_brightness = 255;
 
 void set_brightness_level(uint8_t level)
 {
     if (level > 10) {
         level = 10;
     }

     global_brightness = (uint8_t)(((uint16_t)level * 255U) / 10U);
 }
	 
	 
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

			         uint32_t combined_brightness =
			             ((uint32_t)digit_brightness[digit] *
			              global_brightness) / 255U;

			         uint16_t pwm_level =
			             (combined_brightness * PWM_STEPS) / 255U;

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
	 
	 set_brightness_level(10);   // approximately 50%

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
		     counter_1,
		     dp_state,

		     counter_2,
		     dp_state,

		     counter_3,
		     dp_state,

		     counter_4,
		     dp_state
		 );

	     vTaskDelay(pdMS_TO_TICKS(1));
	 }
 } 
 
 

 
 
 