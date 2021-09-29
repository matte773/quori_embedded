#ifndef ARDUINO_MLX90363_H
#define ARDUINO_MLX90363_H

#include <Arduino.h>
#include "inttypes.h"

/* MLX90363 class.
 * Used for communication with the MLX90363 with SPI
 */
class MLX90363
{
public:
    MLX90363(uint8_t ss_pin);
    static void InitializeSPI(int,int,int);
    void SetZeroPosition();
    void SetZeroPosition(int16_t);
    bool SendGET3();
    bool SendNOP();
    int64_t ReadAngle();
    void PrintReceiveBuffer();
    double MLX90363::WholeMessage();
private:
    uint8_t slave_select;
    uint8_t receive_buffer[8];
    uint8_t send_buffer[8];
    uint8_t crc;
    int16_t raw_value;
    int64_t summed_value;
    int16_t zero_position;
    int i,j;
    bool SendSPI();
    bool Checksum(uint8_t *message);
};

#endif //ARDUINO_MLX90363_H
