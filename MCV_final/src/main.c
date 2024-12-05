#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"
#include <twi.h>
#include <oled.h>
#include <stdlib.h>
#include <gpio.h>

#define F_CPU 16000000UL  // Define the clock frequency (adjust if needed)

#define DHT_ADR 0x5c
#define DHT_HUM_MEM 0
#define DHT_TEMP_MEM 2
#define LED_PIN PD3  // Use OC2B (PD3 on ATmega328) for PWM control of LED
#define FAN_PIN PD5
#define HEATER_PIN PD6

volatile uint8_t update_oled = 0;
volatile uint8_t dht12_values[5];
volatile uint8_t adc_channel = 0;

// ADC Min/Max thresholds and corresponding PWM values
volatile uint16_t adc_min = 50;
volatile uint16_t adc_max = 300;
volatile uint8_t pwm_min = 0;  // Minimum PWM value (when ADC is at max)
volatile uint8_t pwm_max = 255;  // Maximum PWM value (when ADC is at min)

volatile uint16_t last_adc_value = 0;
volatile uint8_t last_pwm_duty = 0;

volatile uint8_t max_humidity = 60;
volatile uint8_t min_humidity = 30;

volatile uint8_t max_temp = 30;
volatile uint8_t min_temp = 10;
volatile uint8_t temp_thr = 25;

volatile uint16_t soil_value = 0;

void pwm_init(void) {
    // Set LED_PIN as output
    DDRD |= (1 << LED_PIN);

    // Set Fast PWM mode, non-inverting on OC2B (PD3), prescaler = 64
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2B1);
    TCCR2B = (1 << CS22);  // Prescaler 64, Fast PWM mode
}

void set_pwm_duty(uint8_t duty) {
    OCR2B = duty;  // Set PWM duty cycle on OC2B
}

ISR(TIMER1_OVF_vect)
{
    static uint8_t n_ovfs = 0;
    n_ovfs++;

    if (n_ovfs >= 20) {  // 1 second interval (assuming 1 overflow every 16ms)
        n_ovfs = 0;

        // Start ADC conversion
        ADCSRA |= (1 << ADSC);

        if(adc_channel == 0){
            ADMUX =(ADMUX & 0xF0) | 0x01;
            adc_channel = 1;
        }else{
            ADMUX =(ADMUX & 0xF0) | 0x00;
            adc_channel = 0;
        }

        if ((dht12_values[0] > max_humidity) || (dht12_values[2] > max_temp) )
        {
            GPIO_write_high(&PORTD, FAN_PIN);
        } else if (dht12_values[0] <= min_humidity)
        {
            GPIO_write_low(&PORTD, FAN_PIN);
        }

        if (dht12_values[2] < min_temp)
        {
            GPIO_write_high(&PORTD, HEATER_PIN);
        } else if (dht12_values[2] >= temp_thr)
        {
            GPIO_write_low(&PORTD, HEATER_PIN);
        }
        

        // Print ADC value and PWM duty once every second
        char string[5];  // Increase buffer size for ADC values and PWM duty
        itoa(last_adc_value, string, 10);

        itoa(last_pwm_duty, string, 10);

        // Read DHT values
        twi_start();
        if (twi_write((DHT_ADR << 1) | TWI_WRITE) == 0) {
            twi_write(DHT_HUM_MEM);
            twi_stop();
            twi_start();
            if (twi_write((DHT_ADR << 1) | TWI_READ) == 0) {
                dht12_values[0] = twi_read(TWI_ACK);  // Humidity integer part
                dht12_values[1] = twi_read(TWI_ACK);  // Humidity decimal part
                dht12_values[2] = twi_read(TWI_ACK);  // Temperature integer part
                dht12_values[3] = twi_read(TWI_NACK); // Temperature decimal part
                update_oled = 1;
            }
        }
        twi_stop();
    }
}

ISR(ADC_vect)
{
    uint16_t adc_value = ADC;
    if(adc_channel==1){

    // Store ADC value
    last_adc_value = adc_value;

    // Map ADC range (adc_min to adc_max) to PWM duty cycle (pwm_min to pwm_max)
    if (adc_value <= adc_min) {
        last_pwm_duty = pwm_max;  // Max brightness for min input
    } else if (adc_value >= adc_max) {
        last_pwm_duty = pwm_min;  // Min brightness for max input
    } else {
        // Calculate scaled PWM duty using precise arithmetic
        last_pwm_duty = pwm_max - ((uint32_t)(adc_value - adc_min) * (pwm_max - pwm_min) / (adc_max - adc_min));
    }

    // Set PWM duty cycle
    set_pwm_duty(last_pwm_duty);
    }else{

        soil_value = (adc_value)/10;
    }

    // Update UART and OLED in main loop
    update_oled = 1;
}

void display_brightness(void) {
    char string[5];
    uint8_t brightness = (uint8_t)((last_pwm_duty * 100UL) / 255);

    oled_gotoxy(14, 5);
    oled_puts("    ");
    itoa(brightness, string, 10);
    oled_gotoxy(14, 5);
    oled_puts(string);
}

int main(void)
{
    // Configure ADC
    ADMUX |= (1 << REFS0);  // AVcc as reference
    ADMUX &= ~(1 << MUX3 | 1 << MUX2 | 1 << MUX1 | 1 << MUX0);  // ADC0
    ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC with interrupt

    // GPIO definition
    GPIO_mode_output(&DDRD, FAN_PIN);
    GPIO_mode_output(&DDRD, HEATER_PIN);

    // uart_init(UART_BAUD_SELECT(9600, F_CPU));
    twi_init();
    oled_init(OLED_DISP_ON);
    oled_clrscr();

    pwm_init();  // Initialize PWM for LED control

    oled_gotoxy(0, 1);
    oled_puts("Temperature:");
    oled_gotoxy(18, 1);
    oled_puts("[C]");
    oled_gotoxy(0, 3);
    oled_puts("Humidity:");
    oled_gotoxy(18, 3);
    oled_puts("[\%]");
    oled_gotoxy(0, 5);
    oled_puts(" Brightness:");
    oled_gotoxy(18, 5);
    oled_puts("[\%]");
    oled_gotoxy(0, 7);
    oled_puts(" Soil:");
    oled_gotoxy(18, 7);
    oled_puts("[\%]");

    oled_display();
    
    TIM1_ovf_33ms();
    TIM1_ovf_enable();

    sei();

    while (1)
    {
        if (update_oled) {
            char string[5];

            // Update OLED for temperature and humidity
            oled_gotoxy(14, 1);
            itoa(dht12_values[2], string, 10);
            oled_puts(string);
            oled_puts(".");
            itoa(dht12_values[3], string, 10);
            oled_puts(string);

            oled_gotoxy(14, 3);
            itoa(dht12_values[0], string, 10);
            oled_puts(string);
            oled_puts(".");
            itoa(dht12_values[1], string, 10);
            oled_puts(string);

            oled_gotoxy(14, 7);
            oled_puts("    ");
            oled_gotoxy(14, 7);
            itoa(soil_value, string, 10);
            oled_puts(string);
            
            // Update OLED for brightness
            display_brightness();
            oled_display();

            update_oled = 0;
        }
    }
    return 0;
}
