#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdint.h>

#define DATA_PIN    23
#define CLOCK_PIN   22
#define STROBE_PIN  21

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

            if(i==1 && dp_on) byte |= (1<<segment_to_q[7]);

            // Apply PWM based on per-digit brightness
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
    digit_brightness[0] = 220; // adjust experimentally
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

    while(1) {
        TickType_t now = xTaskGetTickCount();

        // DP blink
        if(now - last_dp_toggle >= pdMS_TO_TICKS(500)){
            dp_state = !dp_state;
            last_dp_toggle = now;
        }

        // Counter update
        if(now - last_counter_update >= pdMS_TO_TICKS(1000)){
            counter++;
            last_counter_update = now;
        }

        // Display with PWM
        display_number_pwm(counter%10000, dp_state);
    }
}



