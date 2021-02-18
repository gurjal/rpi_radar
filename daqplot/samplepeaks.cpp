#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>

int main() {
  
  char softSpan_data[] = { 0x00, 0x10 };
  char send_data[]     = { 0x00, 0x00 };
  char read_data[]     = { 0x00, 0x00 };
  int len = 2;
  //char softSpan_data[] = { 0x00, 0x10, 0x00 };
  //char send_data[]     = { 0x00, 0x00, 0x00 };
  //char read_data[]     = { 0x00, 0x00, 0x00 };
  //int len = 3;

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

  // Conversion pin set to output
  bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_33, BCM2835_GPIO_FSEL_OUTP);

  // dac aux inits
  bcm2835_aux_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);

  // dac variables
  int hz = 16;
  int periods = 16 * 5;
  int steps = 50;
  uint16_t adc_data[periods] = {0};

  // adc variables

  // calculate step variables
  int step_delay = 1000000 / (steps + 1) / hz;
  uint16_t step_incr = 0xFFFF / steps;
  uint16_t cur_mag;

  for (int i = 0; i < periods; i++) {

    cur_mag = 0;

    for (int j = 0; j < steps + 1; j++) {

      bcm2835_aux_spi_write(cur_mag);
      //bcm2835_aux_spi_write(0xFFFF);
      //bcm2835_aux_spi_write(0x0000);
      bcm2835_delayMicroseconds(step_delay);

      cur_mag += step_incr;

      //if (j == 0) {
      //  bcm2835_spi_transfernb(softSpan_data, read_data, len);
      //  bcm2835_gpio_set(RPI_BPLUS_GPIO_J8_33);
      //  bcm2835_gpio_clr(RPI_BPLUS_GPIO_J8_33);
      //  bcm2835_spi_transfernb(send_data, read_data, len);
      //  adc_data[i] = (read_data[1] << 8 | read_data[0]);
      //  printf("%02x%02x\n", read_data[1], read_data[0]);
      //  //printf("%02x%02x%02x\n", read_data[2], read_data[1], read_data[0]);
      //}

    }

    bcm2835_spi_transfernb(softSpan_data, read_data, len);
    bcm2835_gpio_set(RPI_BPLUS_GPIO_J8_33);
    bcm2835_gpio_clr(RPI_BPLUS_GPIO_J8_33);
    bcm2835_spi_transfernb(send_data, read_data, len);
    adc_data[i] = (read_data[1] << 8 | read_data[0]);
    //printf("%02x%02x\n\n", read_data[1], read_data[0]);
    //printf("%02x%02x%02x\n\n", read_data[2], read_data[1], read_data[0]);

  }

  bcm2835_aux_spi_write(0x0000);
  bcm2835_spi_end();

  for (int i = 0; i < periods; i++)
    printf("%04x\n", adc_data[i]);

  return 0;
}
