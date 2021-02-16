#include <bcm2835.h>
#include <stdio.h>

int main() {

  if (!bcm2835_init()) {
    printf("bcm2835_init failed\n");
    return 1;
  }

  if (!bcm2835_spi_begin()) {
    printf("bcm2835_spi_begin failed\n");
    return 1;
  }

  if (!bcm2835_aux_spi_begin()) {
    printf("bcm2835_spi_begin failed\n");
    return 1;
  }

  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS1);
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW);

  bcm2835_aux_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);

  int hz = 16;
  int periods = 16 * 10;
  int steps = 30;

  int step_delay = 1000000 / (steps + 1) / hz;
  uint16_t step_incr = 0xFFFF / steps;
  uint16_t cur_mag;
  
  for (int i = 0; i < periods; i++) {

    cur_mag = 0;

    for (int j = 0; j < steps + 1; j++) {

      bcm2835_aux_spi_write(cur_mag);
      bcm2835_delayMicroseconds(step_delay);
      
      cur_mag += step_incr;

    }
  }

  bcm2835_aux_spi_write(0x0000);
  bcm2835_spi_end();

  return 0;
}
