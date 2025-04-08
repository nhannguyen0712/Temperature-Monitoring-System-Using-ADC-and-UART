/*
 * TEMP_MONITOR.c
 *
 * Created: 3/19/2025 5:32:07 AM
 * Author : dainhan
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

// --- F_CPU Setup ---
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// --- UART Config ---
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

// --- LCD Pins on PORTC ---
#define LCD_RS PC0
#define LCD_EN PC1
#define LCD_D4 PC2
#define LCD_D5 PC3
#define LCD_D6 PC4
#define LCD_D7 PC5

// --- Function Prototypes ---
void UART_init(void);
void uart_transmit(unsigned char data);
void uart_print_string(const char *str);
void ADC_init(void);
uint16_t ADC_read(uint8_t channel);
uint8_t getTemperature(void);
void LCD_init(void);
void LCD_command(uint8_t cmd);
void LCD_data(uint8_t data);
void LCD_print(const char *str);
void LCD_clear(void);
void LCD_set_cursor(uint8_t col, uint8_t row);

// --- Main ---
int main(void) {
	char buffer[32];
	uint8_t temperature;

	UART_init();
	ADC_init();
	LCD_init();
	LCD_clear();

	uart_print_string("Temperature Monitor (ATmega324PA)\r\n");

	while (1) {
		temperature = getTemperature();

		// LCD
		LCD_set_cursor(0, 0);
		sprintf(buffer, "Temp: %d C   ", temperature);
		LCD_print(buffer);

		// UART
		sprintf(buffer, "Temperature: %d C\r\n", temperature);
		uart_print_string(buffer);

		_delay_ms(1000);
	}
}

// --- UART Setup ---
void UART_init(void) {
	UBRR0H = (UBRR_VALUE >> 8);
	UBRR0L = UBRR_VALUE;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	DDRD |= (1 << PD1);  // TXD0 as output
	DDRD &= ~(1 << PD0); // RXD0 as input
}

void uart_transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void uart_print_string(const char *str) {
	while (*str) uart_transmit(*str++);
}

// --- ADC ---
void ADC_init(void) {
	ADMUX = (1 << REFS0); // AVcc reference
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Prescaler = 64
}

uint16_t ADC_read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

uint8_t getTemperature(void) {
	uint16_t adc_val = ADC_read(0); // LM35 on ADC0 (PA0)
	uint16_t mv = adc_val * 5000UL / 1024;
	return mv / 10;
}

// --- LCD 4-bit Interface ---
void LCD_command(uint8_t cmd) {
	PORTC &= ~(1 << LCD_RS);
	PORTC = (PORTC & 0x03) | (cmd & 0xF0);
	PORTC |= (1 << LCD_EN); _delay_us(1); PORTC &= ~(1 << LCD_EN);
	_delay_us(50);
	PORTC = (PORTC & 0x03) | ((cmd << 4) & 0xF0);
	PORTC |= (1 << LCD_EN); _delay_us(1); PORTC &= ~(1 << LCD_EN);
	_delay_ms(2);
}

void LCD_data(uint8_t data) {
	PORTC |= (1 << LCD_RS);
	PORTC = (PORTC & 0x03) | (data & 0xF0);
	PORTC |= (1 << LCD_EN); _delay_us(1); PORTC &= ~(1 << LCD_EN);
	_delay_us(50);
	PORTC = (PORTC & 0x03) | ((data << 4) & 0xF0);
	PORTC |= (1 << LCD_EN); _delay_us(1); PORTC &= ~(1 << LCD_EN);
	_delay_ms(2);
}

void LCD_init(void) {
	DDRC |= (1 << LCD_RS) | (1 << LCD_EN) |
	(1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);

	_delay_ms(20);
	LCD_command(0x33); LCD_command(0x32);
	LCD_command(0x28); LCD_command(0x0C); LCD_command(0x06); LCD_command(0x01);
	_delay_ms(2);
}

void LCD_clear(void) {
	LCD_command(0x01);
	_delay_ms(2);
}

void LCD_set_cursor(uint8_t col, uint8_t row) {
	uint8_t offsets[] = {0x00, 0x40, 0x14, 0x54};
	LCD_command(0x80 | (col + offsets[row]));
}

void LCD_print(const char *str) {
	while (*str) LCD_data(*str++);
}

