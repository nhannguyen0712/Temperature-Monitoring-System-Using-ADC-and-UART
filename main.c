#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

// ------------------ CONFIG ------------------
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define LCD_RS PB0
#define LCD_EN PB1
#define LCD_D4 PD4
#define LCD_D5 PD5
#define LCD_D6 PD6
#define LCD_D7 PD7

#define RTC_ADDRESS     0x68
#define RTC_WRITE_ADDR  (RTC_ADDRESS << 1)
#define RTC_READ_ADDR   ((RTC_ADDRESS << 1) | 1)

// ------------------ GLOBALS ------------------
char date[20];
char time_str[20];

// ------------------ FUNCTION PROTOTYPES ------------------
void LCD_init(void);
void LCD_command_4bit_init(uint8_t cmd_nibble);
void LCD_command(uint8_t cmd);
void LCD_data(uint8_t data);
void LCD_print(const char *str);
void LCD_clear(void);
void LCD_set_cursor(uint8_t col, uint8_t row);

void I2C_init(void);
uint8_t I2C_start(uint8_t address);
void I2C_stop(void);
uint8_t I2C_write(uint8_t data);
uint8_t I2C_read_ack(void);
uint8_t I2C_read_nack(void);
void getDateTime(void);
uint8_t bcd_to_decimal(uint8_t bcd);

void ADC_init(void);
uint16_t ADC_read(uint8_t channel);
uint8_t getTemperature(void);

// ------------------ MAIN ------------------
int main(void) {
	char buffer[20];
	uint8_t temp;

	LCD_init();
	I2C_init();
	ADC_init();
	LCD_clear();

	while (1) {
		temp = getTemperature();
		getDateTime();

		LCD_set_cursor(0, 0); // Row 1: Temp
		sprintf(buffer, "Temp: %d C     ", temp);
		LCD_print(buffer);

		LCD_set_cursor(0, 1); // Row 2: Time & Date
		sprintf(buffer, "%s %s", time_str, date);
		LCD_print(buffer);

		_delay_ms(1000);
	}
}

// ------------------ LCD ------------------
void LCD_init(void) {
	DDRB |= (1 << LCD_RS) | (1 << LCD_EN);
	DDRD |= (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);

	_delay_ms(50);
	LCD_command_4bit_init(0x30); _delay_ms(5);
	LCD_command_4bit_init(0x30); _delay_us(150);
	LCD_command_4bit_init(0x30); _delay_us(150);
	LCD_command_4bit_init(0x20); _delay_us(150);

	LCD_command(0x28); // 4-bit, 2 line
	LCD_command(0x0C); // Display ON, cursor OFF
	LCD_command(0x06); // Entry mode
	LCD_command(0x01); // Clear display
	_delay_ms(2);
}

void LCD_command_4bit_init(uint8_t cmd_nibble) {
	PORTB &= ~(1 << LCD_RS);
	PORTD = (PORTD & 0x0F) | (cmd_nibble & 0xF0);
	PORTB |= (1 << LCD_EN);
	_delay_us(1);
	PORTB &= ~(1 << LCD_EN);
	_delay_us(50);
}

void LCD_command(uint8_t cmd) {
	PORTB &= ~(1 << LCD_RS);
	PORTD = (PORTD & 0x0F) | (cmd & 0xF0);
	PORTB |= (1 << LCD_EN);
	_delay_us(1);
	PORTB &= ~(1 << LCD_EN);
	_delay_us(1);
	PORTD = (PORTD & 0x0F) | ((cmd << 4) & 0xF0);
	PORTB |= (1 << LCD_EN);
	_delay_us(1);
	PORTB &= ~(1 << LCD_EN);
	if (cmd == 0x01 || cmd == 0x02)
	_delay_ms(2);
	else
	_delay_us(40);
}

void LCD_data(uint8_t data) {
	PORTB |= (1 << LCD_RS);
	PORTD = (PORTD & 0x0F) | (data & 0xF0);
	PORTB |= (1 << LCD_EN);
	_delay_us(1);
	PORTB &= ~(1 << LCD_EN);
	_delay_us(1);
	PORTD = (PORTD & 0x0F) | ((data << 4) & 0xF0);
	PORTB |= (1 << LCD_EN);
	_delay_us(1);
	PORTB &= ~(1 << LCD_EN);
	_delay_us(40);
}

void LCD_print(const char *str) {
	while (*str)
	LCD_data(*str++);
}

void LCD_clear(void) {
	LCD_command(0x01);
	_delay_ms(2);
}

void LCD_set_cursor(uint8_t col, uint8_t row) {
	const uint8_t row_offsets[] = {0x00, 0x40};
	if (row >= 2) row = 1;
	LCD_command(0x80 | (col + row_offsets[row]));
}

// ------------------ I2C + RTC ------------------
void I2C_init(void) {
	TWSR = 0x00;
	TWBR = ((F_CPU / 100000UL) - 16) / 2;
	TWCR = (1 << TWEN);
}

uint8_t I2C_start(uint8_t address) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	TWDR = address;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return 0;
}

void I2C_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

uint8_t I2C_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return 0;
}

uint8_t I2C_read_ack(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

uint8_t I2C_read_nack(void) {
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

void getDateTime(void) {
	uint8_t s, m, h, d, dt, mo, y;

	I2C_start(RTC_WRITE_ADDR);
	I2C_write(0x00); // Start from seconds register
	I2C_start(RTC_READ_ADDR);
	s = I2C_read_ack();
	m = I2C_read_ack();
	h = I2C_read_ack();
	d = I2C_read_ack(); // day of week
	dt = I2C_read_ack();
	mo = I2C_read_ack();
	y = I2C_read_nack();
	I2C_stop();

	if (s & 0x80) {
		strcpy(date, "RTC HALTED");
		strcpy(time_str, "--:--:--");
		return;
	}

	s = bcd_to_decimal(s & 0x7F);
	m = bcd_to_decimal(m);
	h = bcd_to_decimal(h & 0x3F);
	dt = bcd_to_decimal(dt);
	mo = bcd_to_decimal(mo);
	y = bcd_to_decimal(y);

	sprintf(time_str, "%02d:%02d:%02d", h, m, s);
	sprintf(date, "%02d/%02d/%02d", dt, mo, y);
}

uint8_t bcd_to_decimal(uint8_t bcd) {
	return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// ------------------ ADC ------------------
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
	return mv / 10; // LM35 = 10mV per °C
}
