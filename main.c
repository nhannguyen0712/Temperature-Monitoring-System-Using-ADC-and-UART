#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// --- Clock Frequency ---
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// --- LCD Pin Definitions (PORTB & PORTD) ---
#define LCD_RS PB0
#define LCD_EN PB1
#define LCD_D4 PD4
#define LCD_D5 PD5
#define LCD_D6 PD6
#define LCD_D7 PD7

// --- UART Settings ---
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

// --- Function Prototypes ---
void UART_init(void);
void uart_transmit(unsigned char data);
void uart_print_string(const char *str);
void LCD_init(void);
void LCD_command_4bit_init(uint8_t cmd_nibble);
void LCD_command(uint8_t cmd);
void LCD_data(uint8_t data);
void LCD_print(const char *str);
void LCD_clear(void);
void LCD_set_cursor(uint8_t col, uint8_t row);
void ADC_init(void);
uint16_t ADC_read(uint8_t channel);
uint8_t getTemperature(void);

// --- Main ---
int main(void) {
    char buffer[32];

    UART_init();
    LCD_init();
    ADC_init();

    LCD_clear();
    LCD_set_cursor(0, 0);
    LCD_print("Temp Monitor Ready");
    uart_print_string("Temperature Monitor Started\r\n");

    _delay_ms(1000);
    LCD_clear();

    while (1) {
        uint8_t temperature = getTemperature();

        // Print to LCD
        LCD_set_cursor(0, 0);
        sprintf(buffer, "Temp: %d C   ", temperature);
        LCD_print(buffer);

        // Print to UART
        sprintf(buffer, "Temperature: %d C\r\n", temperature);
        uart_print_string(buffer);

        _delay_ms(1000);
    }
}

// --- UART ---
void UART_init(void) {
    UBRR0H = (UBRR_VALUE >> 8);
    UBRR0L = UBRR_VALUE;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    DDRD &= ~(1 << PD0); // RX input
    DDRD |= (1 << PD1);  // TX output
}

void uart_transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print_string(const char *str) {
    while (*str) uart_transmit(*str++);
}

// --- LCD ---
void LCD_init(void) {
    DDRB |= (1 << LCD_RS) | (1 << LCD_EN);
    DDRD |= (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);

    _delay_ms(50);
    LCD_command_4bit_init(0x30); _delay_ms(5);
    LCD_command_4bit_init(0x30); _delay_us(150);
    LCD_command_4bit_init(0x30); _delay_us(150);
    LCD_command_4bit_init(0x20); _delay_us(150);
    LCD_command(0x28); LCD_command(0x08); LCD_command(0x01); _delay_ms(2);
    LCD_command(0x06); LCD_command(0x0C);
}

void LCD_command_4bit_init(uint8_t cmd_nibble) {
    PORTB &= ~(1 << LCD_RS);
    PORTD = (PORTD & 0x0F) | (cmd_nibble & 0xF0);
    PORTB |= (1 << LCD_EN); _delay_us(1);
    PORTB &= ~(1 << LCD_EN); _delay_us(50);
}

void LCD_command(uint8_t cmd) {
    PORTB &= ~(1 << LCD_RS);
    PORTD = (PORTD & 0x0F) | (cmd & 0xF0);
    PORTB |= (1 << LCD_EN); _delay_us(1);
    PORTB &= ~(1 << LCD_EN); _delay_us(1);
    PORTD = (PORTD & 0x0F) | ((cmd << 4) & 0xF0);
    PORTB |= (1 << LCD_EN); _delay_us(1);
    PORTB &= ~(1 << LCD_EN);
    if (cmd == 0x01 || cmd == 0x02) _delay_ms(2);
    else _delay_us(40);
}

void LCD_data(uint8_t data) {
    PORTB |= (1 << LCD_RS);
    PORTD = (PORTD & 0x0F) | (data & 0xF0);
    PORTB |= (1 << LCD_EN); _delay_us(1);
    PORTB &= ~(1 << LCD_EN); _delay_us(1);
    PORTD = (PORTD & 0x0F) | ((data << 4) & 0xF0);
    PORTB |= (1 << LCD_EN); _delay_us(1);
    PORTB &= ~(1 << LCD_EN); _delay_us(40);
}

void LCD_print(const char *str) {
    while (*str) LCD_data(*str++);
}

void LCD_clear(void) {
    LCD_command(0x01);
    _delay_ms(2);
}

void LCD_set_cursor(uint8_t col, uint8_t row) {
    const uint8_t offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row > 3) row = 3;
    LCD_command(0x80 | (offsets[row] + col));
}

// --- ADC ---
void ADC_init(void) {
    ADMUX = (1 << REFS0); // AVcc reference
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Prescaler = 64
}

uint16_t ADC_read(uint8_t channel) {
    ADMUX = (1 << REFS0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

uint8_t getTemperature(void) {
    uint16_t adc_val = ADC_read(0); // Read from ADC0 (PA0)
    uint16_t mv = adc_val * 5000UL / 1024;
    return mv / 10; // LM35: 10mV per °C
}
