#ifndef uart_h
#define uart_h

void uart_init(uint16_t baudRate);              // Initialise with the passed baud rate

void uart_init_9600();                          // Initialise with a baud rate of 9600

void uart_txChar(unsigned char c);              // Transmit char

void uart_txString(const char* s);              // Transmit string

void uart_txU8(uint8_t val);                    // Transmit 8-bit unsigned int

void uart_txU16(uint16_t val);                  // Transmit 16-bit unsigned int

void uart_txFormatted(const char* format, ...); // Transmit formatted output

#endif