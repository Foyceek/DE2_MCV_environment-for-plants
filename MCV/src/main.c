#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"
#include <twi.h>
#include <oled.h>
#include <stdlib.h>
#include <util/delay.h>  // Include the delay header

#define F_CPU 16000000UL  // Define the clock frequency (adjust if needed)

#define DHT_ADR 0x5c
#define DHT_HUM_MEM 0
#define DHT_TEMP_MEM 2
#define LED_PIN PD3  // Use OC2B (PD3 on ATmega328) for PWM control of LED

#define UART_BAUD_SELECT(baudRate, xtalCpu) ((xtalCpu) / (16UL * (baudRate)) - 1)

volatile uint8_t update_oled = 0;
volatile uint8_t dht12_values[5];

// ADC Min/Max thresholds and corresponding PWM values
volatile uint16_t adc_min = 0;
volatile uint16_t adc_max = 100;
volatile uint8_t pwm_min = 0;  // Minimum PWM value (when ADC is at max)
volatile uint8_t pwm_max = 255;  // Maximum PWM value (when ADC is at min)

volatile uint16_t last_adc_value = 0;
volatile uint8_t last_pwm_duty = 0;

void uart_init(uint16_t ubrr) {
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print_string(const char *str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

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

        // Print ADC value and PWM duty once every second
        char string[5];  // Increase buffer size for ADC values and PWM duty
        itoa(last_adc_value, string, 10);
        uart_print_string("ADC: ");
        uart_print_string(string);
        uart_print_string(" ");

        itoa(last_pwm_duty, string, 10);
        uart_print_string("PWM Duty: ");
        uart_print_string(string);
        uart_print_string("\n");

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
            } else {
                uart_print_string("Error reading DHT12 data.\n");
            }
        } else {
            uart_print_string("Error communicating with DHT12.\n");
        }
        twi_stop();
    }
}

ISR(ADC_vect)
{
    uint16_t adc_value = ADC;

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

    // Update UART and OLED in main loop
    update_oled = 1;
}

void display_brightness(void) {
    char string[5];
    uint8_t brightness = (uint8_t)((last_pwm_duty * 100UL) / 255);

    itoa(brightness, string, 10);
    oled_gotoxy(16, 6);
    oled_puts(string);
}

int main(void)
{
    // Configure ADC
    ADMUX |= (1 << REFS0);  // AVcc as reference
    ADMUX &= ~(1 << MUX3 | 1 << MUX2 | 1 << MUX1 | 1 << MUX0);  // ADC0
    ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC with interrupt

    uart_init(UART_BAUD_SELECT(9600, F_CPU));
    twi_init();
    oled_init(OLED_DISP_ON);
    oled_clrscr();

    pwm_init();  // Initialize PWM for LED control

    oled_gotoxy(0, 2);
    oled_puts("Temperature:");
    oled_gotoxy(18, 2);
    oled_puts("[C]");
    oled_gotoxy(0, 4);
    oled_puts("  Humidity:");
    oled_gotoxy(18, 4);
    oled_puts("[\%]");
    oled_gotoxy(0, 6);
    oled_puts(" Brightness:");
    oled_gotoxy(18, 6);
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
            oled_gotoxy(14, 2);
            itoa(dht12_values[2], string, 10);
            oled_puts(string);
            oled_puts(".");
            itoa(dht12_values[3], string, 10);
            oled_puts(string);

            oled_gotoxy(14, 4);
            itoa(dht12_values[0], string, 10);
            oled_puts(string);
            oled_puts(".");
            itoa(dht12_values[1], string, 10);
            oled_puts(string);

            // Update OLED for brightness
            display_brightness();
            oled_display();

            // Add a short delay to ensure OLED updates properly
            _delay_ms(50);

            update_oled = 0;
        }
    }
    return 0;
}
