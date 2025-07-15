/* 
  I got a lot of the code and information in this program from http://www.rjhcoding.com/avrc-uart.php
*/
#include <avr/io.h>
#include <stdio.h>
#include "UART.h"

#define F_CPU 16000000L
#define UBRR_9600 103

void uart_init(uint16_t baudRate)
{
    // Set the baud rate (baud rate is symbols per second, or pulses per second)
    // Using the equation in the data sheet, the value in the baud rate register should be:
    uint16_t ubrr = F_CPU / 16 / baudRate - 1;

    // Write value to baud rate registers
    UBRR0L = (uint8_t)(ubrr & 0xFF);
    UBRR0H = (uint8_t)(ubrr >> 8);

    // Enable the transmitter and receiver
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
}

// ATMEGA328P does not have a hardware divider, so division is very expensive. This alternative
// init() function initiates the uart with a hardcoded UBRR equivalent to a baud rate of 9600
void uart_init_9600()
{
    uint16_t ubrr = UBRR_9600;

    // Write value to baud rate registers
    UBRR0L = (uint8_t)(ubrr & 0xFF);
    UBRR0H = (uint8_t)(ubrr >> 8);

    // Enable the transmitter and receiver
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
}

//*************** Character and string transmission ***************
void uart_txChar(unsigned char c)
{
    // Wait while the transmit buffer is not empty
    while (!(UCSR0A & (1 << UDRE0)));

    // Load data into transmit register
    UDR0 = c;
}

void uart_txString(const char* s)
{
    // Transmit characters until NULL is reached
    while (*s != '\0') {
        uart_txChar(*s++);
    }
}

//*************** Integer transmission ***************

// This function was written by the author of: http://www.rjhcoding.com/avrc-uart.php
void uart_txU8(uint8_t val)
{
    uint8_t dig1 = '0', dig2 = '0';

    // Count value in 100s place
    while(val >= 100)
    {
        val -= 100;
        dig1++;
    }

    // Count value in 10s place
    while(val >= 10)
    {
        val -= 10;
        dig2++;
    }

    // Transmit first digit (or ignore leading zeros)
    if(dig1 != '0') uart_txChar(dig1);

    // Transmit second digit (or ignore leading zeros)
    if((dig1 != '0') || (dig2 != '0')) uart_txChar(dig2);

    // Transmit final digit
    uart_txChar(val + '0');
}

// This function was written by the author of: http://www.rjhcoding.com/avrc-uart.php
void uart_txU16(uint16_t val)
{
    uint8_t dig1 = '0', dig2 = '0', dig3 = '0', dig4 = '0';

    // Count value in 10000s place
    while(val >= 10000)
    {
        val -= 10000;
        dig1++;
    }

    // Count value in 1000s place
    while(val >= 1000)
    {
        val -= 1000;
        dig2++;
    }

    // Count value in 100s place
    while(val >= 100)
    {
        val -= 100;
        dig3++;
    }

    // count value in 10s place
    while(val >= 10)
    {
        val -= 10;
        dig4++;
    }

    uint8_t prevPrinted = 0;

    // Transmit first digit (or ignore leading zeros)
    if(dig1 != '0') {uart_txChar(dig1); prevPrinted = 1;}

    // Transmit second digit (or ignore leading zeros)
    if(prevPrinted || (dig2 != '0')) {uart_txChar(dig2); prevPrinted = 1;}

    // Transmit third digit (or ignore leading zeros)
    if(prevPrinted || (dig3 != '0')) {uart_txChar(dig3); prevPrinted = 1;}

    // Transmit third digit (or ignore leading zeros)
    if(prevPrinted || (dig4 != '0')) {uart_txChar(dig4); prevPrinted = 1;}

    // Transmit final digit
    uart_txChar(val + '0');
}

void uart_txFormatted(const char* format, ...) 
{
    char buffer[64];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    uart_txString(buffer);
}
