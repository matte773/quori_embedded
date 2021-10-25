#include <Arduino.h>                              // required before wiring_private.h
#include <wiring_private.h>

#define FORWARD 0x86
#define REVERSE 0x85
// VARIBLE IDs defined at: https://www.pololu.com/docs/0J77/6.4
#define ERROR_STATUS 0
#define VIN_STATUS 23

//static const char MKR1000_LED       = 6;

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (1ul)                // Pin description number for PIO_SERCOM on D1
#define PIN_SERIAL2_TX       (0ul)                // Pin description number for PIO_SERCOM on D0
#define PAD_SERIAL2_TX       (UART_TX_PAD_0)      // SERCOM pad 0 TX
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_1)    // SERCOM pad 1 RX

// Serial3 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL3_RX       (5ul)                // Pin description number for PIO_SERCOM on D5
#define PIN_SERIAL3_TX       (4ul)                // Pin description number for PIO_SERCOM on D4
#define PAD_SERIAL3_TX       (UART_TX_PAD_2)      // SERCOM pad 2 TX
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3 RX

void serial_setup();

void serial_motor_controls(int motor_tag, 
                        float DIR_PACK,
                        int serial_val); 

unsigned int getVIN();
bool isMotorErr();
int readByte();
int getVariable(unsigned char variableID);
