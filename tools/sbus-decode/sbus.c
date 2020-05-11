#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
  uint16_t channel0 : 11;
  uint16_t channel1 : 11;
  uint16_t channel2 : 11;
  uint16_t channel3 : 11;
  uint16_t channel4 : 11;
  uint16_t channel5 : 11;
  uint16_t channel6 : 11;
  uint16_t channel7 : 11;
  uint16_t channel8 : 11;
  uint16_t channel9 : 11;
  uint16_t channel10 : 11;
  uint16_t channel11 : 11;
  uint16_t channel12 : 11;
  uint16_t channel13 : 11;
  uint16_t channel14 : 11;
  uint16_t channel15 : 11;
  uint8_t flags;
  uint8_t rssi;
} __attribute__((__packed__)) SbusChannels_t;

#define SBUS_MIN 192
#define SBUS_MAX 1792

int main()
{
  SbusChannels_t channels;
  memset(&channels, 0, sizeof(channels));

  channels.channel0 = 1;
  channels.channel1 = 2;
  channels.channel2 = 3;
  channels.channel3 = 4;

  fwrite(&channels, 1, sizeof(channels), stdout);

  return 0;
}