#include <stdio.h>
#include <vector>
#include <thread>

#include <bcm2835.h>

#include <fftw3.h>

#include "pbPlots/pbPlots.hpp"
#include "pbPlots/supportLib.hpp"

using namespace std;

#define ADC_DATA_SIZE 2048
#define REAL 0
#define IMAG 1

 // dac variables
volatile int _hz = 16;
volatile int _steps = 128;

// calculate step variables
volatile int _tune = 0;
volatile int _step_delay = (1000000 / _steps / _hz) - _tune;
volatile uint16_t _step_incr = 0xFFFF / _steps;

// adc variables
volatile uint16_t _adc_data[ADC_DATA_SIZE] = {0};
volatile char _softSpan_data[] = { 0x00, 0x10 };
volatile char _send_data[]     = { 0x00, 0x00 };
volatile char _read_data[]     = { 0x00, 0x00 };
volatile int _len = 2;
//volatile char _softSpan_data[] = { 0x00, 0x10, 0x00 };
//volatile char _send_data[]     = { 0x00, 0x00, 0x00 };
//volatile char _read_data[]     = { 0x00, 0x00, 0x00 };
//volatile int _len = 3;

// fftw
volatile fftw_complex *_in;
volatile fftw_complex *_out;
volatile fftw_plan _p;
volatile double _fft_mag[ADC_DATA_SIZE];
volatile double _fft_ang[ADC_DATA_SIZE];

inline double mag(double real, double imag) {
  return sqrt((real * real) + (imag * imag));
}

inline double ang(double real, double imag) {
  return atan(imag / real);
}

int spi_init() {

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

  // gain control
  // ABGn
  //bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_08, BCM2835_GPIO_FSEL_OUTP);
  //bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_10, BCM2835_GPIO_FSEL_OUTP);
  //bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_13, BCM2835_GPIO_FSEL_OUTP);

  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_08, LOW); //G2
  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_10, HIGH); //G1
  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_13, HIGH); //G0

  // CDGn
  //bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_03, BCM2835_GPIO_FSEL_OUTP);
  //bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_05, BCM2835_GPIO_FSEL_OUTP);
  //bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_07, BCM2835_GPIO_FSEL_OUTP);

  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_03, HIGH); //G2
  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_05, HIGH); //G1
  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_07, HIGH); //G0

  // allocating space to fftw vars
  _in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * ADC_DATA_SIZE);
  _out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * ADC_DATA_SIZE);
  
  // create fftw plan
  _p = fftw_plan_dft_1d(ADC_DATA_SIZE, (fftw_complex*)_in, (fftw_complex*)_out, FFTW_FORWARD, FFTW_ESTIMATE);


  return 0;
}

void run_dac() {

  int cur_mag = 0;

  for (int s = 0; s < _steps; s++) {
    bcm2835_aux_spi_write(cur_mag);
    bcm2835_delayMicroseconds(_step_delay);
    cur_mag += _step_incr;
  }

}

void run_adc() {

  for (int i = 0; i < _steps; i++) {

    bcm2835_spi_transfernb((char*)_softSpan_data, (char*)_read_data, (int)_len);
    bcm2835_gpio_set(RPI_BPLUS_GPIO_J8_33);
    bcm2835_gpio_clr(RPI_BPLUS_GPIO_J8_33);
    bcm2835_spi_transfernb((char*)_send_data, (char*)_read_data, (int)_len);
    printf("%04x\n",  (uint16_t)(_read_data[0] << 8 | _read_data[1]));
    bcm2835_delayMicroseconds(_step_delay);

  }

}

void run_daq() {

  uint16_t cur_mag = 0;

  for (int p = 0; p < _hz; p++, cur_mag = 0) {
    for (int s = 0; s < _steps; s++) {

      // dac write
      bcm2835_aux_spi_write(cur_mag);

      // adc read
      bcm2835_spi_transfernb((char*)_softSpan_data, (char*)_read_data, (int)_len);
      bcm2835_gpio_set(RPI_BPLUS_GPIO_J8_33);
      bcm2835_gpio_clr(RPI_BPLUS_GPIO_J8_33);
      bcm2835_spi_transfernb((char*)_send_data, (char*)_read_data, (int)_len);
      //_adc_data[(p * _steps) + s] = (uint16_t)(_read_data[0] << 8 | _read_data[1]);
      //_in[](double)(0xFFFF - _adc_data[i]) * (5.12f/65536.0f);
      _in[ (p * _steps) + s ][REAL] = (double)((_read_data[0] << 8) | (_read_data[1])) * (5.12f/65536.0f);;
      //printf("%04x\n",  (uint16_t)(_read_data[0] << 8 | _read_data[1]));

      bcm2835_delayMicroseconds(_step_delay);
      cur_mag += _step_incr;
    }
  }
}

void run_fft() {
  //_in[](double)(0xFFFF - _adc_data[i]) * (5.12f/65536.0f);

  fftw_execute(_p);

  for (int i = 0; i < ADC_DATA_SIZE; i++) {
    _fft_mag[i] = mag(_out[i][REAL], _out[i][IMAG]);
    _fft_ang[i] = ang(_out[i][REAL], _out[i][IMAG]);

  }

  for (double fft : _fft_mag)
    printf("%f\n", fft);

}

void plot_adc() {

	vector<double> xs, ys;

  //for (uint16_t data : adc_data)
  //  printf("%f\n", (0xFFFF - data) * (5.12f/65536.0f));

  for (int i = 0; i < ADC_DATA_SIZE; i++) {

    //printf("%f\n", (0xFFFF - _adc_data[i]) * (5.12f/65536.0f));
    //printf("%04x\n", _adc_data[i]);
    ys.push_back( (double)(0xFFFF - _adc_data[i]) * (5.12f/65536.0f) );
    xs.push_back(i);

  }

  //printf("%04x\n", step_incr);

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
	settings->autoBoundaries = false;
	settings->autoPadding = true;
	settings->xMin = 0;
	settings->xMax = ADC_DATA_SIZE;
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

}

void plot_fft() {

	vector<double> xs, ys;
  int trim_samples = 3;

  //for (uint16_t data : adc_data)
  //  printf("%f\n", (0xFFFF - data) * (5.12f/65536.0f));

  for (int i = 0; i < ADC_DATA_SIZE; i++) {

    //printf("%f\n", (0xFFFF - _adc_data[i]) * (5.12f/65536.0f));
    //printf("%04x\n", _adc_data[i]);
    //ys.push_back( (double)(0xFFFF - _adc_data[i]) * (5.12f/65536.0f) );
    ys.push_back((double)_fft_mag[i]);
    xs.push_back(i);

  }
  
  for ( int i = 0; i < trim_samples; i++ ) {
    ys[i] = 0;
    ys[ADC_DATA_SIZE - i - 1] = 0;
  }

  //printf("%04x\n", step_incr);

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
	settings->xMax = ADC_DATA_SIZE;
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

}

int main() {


  // zeroing imaginary nums for fft
  //for (int i = 0; i < ADC_DATA_SIZE; i++) {
  //  //_in[i][IMAG] = 0;
  //}

  if ( spi_init() ) return 1;

  for ( int i = 0; i < 5; i++ )
    run_daq();

  for (int i = 0; i < 1; i++) {

    thread t_daq(run_daq);
    thread t_fft(run_fft);
    thread t_plot(plot_fft);
    t_daq.join();
    t_fft.join();
    t_plot.join();

  }

  printf("ABGn pins\n");
  printf("g2 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_08));
  printf("g1 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_10));
  printf("g0 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_13));
  printf("CDGn pins\n");
  printf("g2 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_03));
  printf("g1 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_05));
  printf("g0 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_07));

  bcm2835_aux_spi_write(0x0000);

  bcm2835_spi_end();
  bcm2835_close();

  // free fftw resources
  fftw_destroy_plan(_p);
  fftw_free((fftw_complex*)_in);
  fftw_free((fftw_complex*)_out);

  return 0;
}
