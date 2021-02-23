#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>

#include "pbPlots/pbPlots.hpp"
#include "pbPlots/supportLib.hpp"

using namespace std;

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

  // adc spi inits
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
  int periods = 16;
  int steps = 128;

  // adc variables
  uint16_t adc_data[ periods * steps ] = {0};

  // calculate step variables
  int tune = 20;
  int step_delay = (1000000 / steps / hz) - tune;
  uint16_t step_incr = 0xFFFF / steps;
  uint16_t cur_mag;

  bcm2835_aux_spi_write(0x0000);

  for (int p = 0; p < periods; p++, cur_mag = 0) {
    for (int s = 0; s < steps; s++) {

      bcm2835_aux_spi_write(cur_mag);

      bcm2835_spi_transfernb(softSpan_data, read_data, len);
      bcm2835_gpio_set(RPI_BPLUS_GPIO_J8_33);
      bcm2835_gpio_clr(RPI_BPLUS_GPIO_J8_33);
      bcm2835_spi_transfernb(send_data, read_data, len);
      adc_data[ (p * steps) + s ] = (uint16_t)(read_data[0] << 8 | read_data[1]);
      //printf("%f\n", (0xFFFF - adc_data[ (p * steps) + s ]) * (5.12f/65536.0f));

      bcm2835_delayMicroseconds(step_delay);
      cur_mag += step_incr;
    }
  }

  bcm2835_spi_end();

	vector<double> xs, ys;

  //for (uint16_t data : adc_data)
  //  printf("%f\n", (0xFFFF - data) * (5.12f/65536.0f));

  for (int i = 0; i < (periods * steps); i++) {
    ys.push_back( (double)(0xFFFF - adc_data[i]) * (5.12f/65536.0f) );
    xs.push_back(i);
  }

  printf("%04x\n", step_incr);

  RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

	ScatterPlotSeries *series = GetDefaultScatterPlotSeriesSettings();
	series->xs = &xs;
	series->ys = &ys;

	//series->linearInterpolation = true;
	//series->lineType = toVector(L"dashed");
	//series->lineThickness = 2;
	//series->color = GetGray(0.3);

	ScatterPlotSettings *settings = GetDefaultScatterPlotSettings();
	settings->width = 600;
	settings->height = 400;
	settings->autoBoundaries = true;
	settings->autoPadding = true;
	settings->xMin = 0;
	settings->xMax = (periods * steps);
	settings->yMin = 0;
	settings->yMax = 6;
	//settings->title = toVector(L"x^2 - 2");
	//settings->xLabel = toVector(L"X axis");
	//settings->yLabel = toVector(L"Y axis");
	settings->scatterPlotSeries->push_back(series);

	DrawScatterPlotFromSettings(imageReference, settings);

	vector<double> *pngdata = ConvertToPNG(imageReference->image);
	WriteToFile(pngdata, "plot.png");
	DeleteImage(imageReference->image);

  return 0;
}
