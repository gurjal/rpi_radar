#include <stdio.h>
#include <vector>
#include <thread>
#include <sstream>
#include <iostream>

#include <bcm2835.h>

#include <fftw3.h>

#include "pbPlots/pbPlots.hpp"
#include "pbPlots/supportLib.hpp"

using namespace std;

#define REAL 0
#define IMAG 1

#define HZ 16
#define STEPS 128
#define PERIODS 16
#define TUNE 0

// calculate step variables
volatile int _step_delay = (1000000 / STEPS / HZ) - TUNE;
volatile uint16_t _step_incr = 0xFFFF / STEPS;

// adc variables
volatile uint16_t _adc_data[STEPS] = {0};
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

volatile fftw_complex *_in_v[PERIODS];
volatile fftw_complex *_out_v[PERIODS];
volatile fftw_plan _p_v[PERIODS];

volatile double _fft_mag[STEPS] = {0};
volatile double _freespace_fft_mag[STEPS] = {0};
volatile double _fft_ang[STEPS] = {0};

volatile double _hamming[STEPS] = {0};

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

  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_08, HIGH); //G2
  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_10, HIGH); //G1
  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_13, HIGH); //G0

  // CDGn
  //bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_03, BCM2835_GPIO_FSEL_OUTP);
  //bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_05, BCM2835_GPIO_FSEL_OUTP);
  //bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_07, BCM2835_GPIO_FSEL_OUTP);

  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_03, HIGH); //G2
  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_05, LOW); //G1
  //bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_07, HIGH); //G0

  // allocating space to fftw vars
  //_in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * STEPS);
  //_out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * STEPS);
  for (int i = 0; i < PERIODS; i++) {
    _in_v[i]  = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * STEPS);
    _out_v[i] = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * STEPS);
  }
  
  // create fftw plan
  //_p = fftw_plan_dft_1d(STEPS, (fftw_complex*)_in, (fftw_complex*)_out, FFTW_FORWARD, FFTW_ESTIMATE);

  for (int i = 0; i < PERIODS; i++)
    _p_v[i] = fftw_plan_dft_1d(STEPS, (fftw_complex*)_in_v[i], (fftw_complex*)_out_v[i], FFTW_FORWARD, FFTW_ESTIMATE);

  for ( int n = 0; n < STEPS; n++ ) {
    _hamming[n] = (0.54 - (0.46 * cos((double)(2 * 3.14159265 * n)/(double)STEPS)));
    //printf("%f\n", _hamming[n]);
  }


  return 0;
}

void run_dac() {

  int cur_mag = 0;

  for (int s = 0; s < STEPS; s++) {
    bcm2835_aux_spi_write(cur_mag);
    bcm2835_delayMicroseconds(_step_delay);
    cur_mag += _step_incr;
  }

}

void run_adc() {

  for (int i = 0; i < STEPS; i++) {

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

  for (int p = 0; p < PERIODS; p++, cur_mag = 0) {
    for (int s = 0; s < STEPS; s++) {

      // dac write
      bcm2835_aux_spi_write(cur_mag);

      // adc read
      bcm2835_spi_transfernb((char*)_softSpan_data, (char*)_read_data, (int)_len);
      bcm2835_gpio_set(RPI_BPLUS_GPIO_J8_33);
      bcm2835_gpio_clr(RPI_BPLUS_GPIO_J8_33);
      bcm2835_spi_transfernb((char*)_send_data, (char*)_read_data, (int)_len);
      //_adc_data[(p * STEPS) + s] = (uint16_t)(_read_data[0] << 8 | _read_data[1]);
      //_in[](double)(0xFFFF - _adc_data[i]) * (5.12f/65536.0f);
      _in_v[p][s][REAL] = (double)((double)((0xFFFF - ((_read_data[0] << 8) | (_read_data[1]))) * (5.12f/65536.0f)) - 2.5f);
      //printf("%04x\n",  (uint16_t)(_read_data[0] << 8 | _read_data[1]));

      bcm2835_delayMicroseconds(_step_delay);
      cur_mag += _step_incr;
    }
  }
}

void run_fft() {

  double total_fft_mag;

  //_in[](double)(0xFFFF - _adc_data[i]) * (5.12f/65536.0f);

  for (int i = 0; i < PERIODS; i++)
    fftw_execute(_p_v[i]);

  for (int s = 0; s < STEPS; s++, total_fft_mag = 0) {
    for (int p = 0; p < PERIODS; p++) {

      total_fft_mag += mag(_out_v[p][s][REAL], _out_v[p][s][IMAG]);
      
    }

    //_prev_fft_mag[s] = _fft_mag[s];
    _fft_mag[s] = _hamming[s] * (total_fft_mag / PERIODS);
  }
}

void calibrate_freespace_fft(double freespace_weight) {


  printf("Callibrating freespace\n");

  int calibration_cycles = 10;
  double total_fft_mag;


  //for (int i = 0; i < 5; i++)
  //  run_daq();


  //_in[](double)(0xFFFF - _adc_data[i]) * (5.12f/65536.0f);
  for (int c = 0; c < calibration_cycles; c++) {

    printf("%d\n", calibration_cycles - c);

    run_daq();

    for (int i = 0; i < PERIODS; i++)
      fftw_execute(_p_v[i]);

    for (int s = 0; s < STEPS; s++, total_fft_mag = 0) {
      for (int p = 0; p < PERIODS; p++) {

        total_fft_mag += mag(_out_v[p][s][REAL], _out_v[p][s][IMAG]);
        
      }

      //_prev_fft_mag[s] = _fft_mag[s];
      _freespace_fft_mag[s] += _hamming[s] * (total_fft_mag / PERIODS);
    }
  }

  for (int s = 0; s < STEPS; s++) {    
    _freespace_fft_mag[s] /= calibration_cycles;
    _freespace_fft_mag[s] *= freespace_weight;
  }


  printf("Done\n");

}

void plot_adc(bool overwrite) {

	vector<double> xs, ys;

  static int n = 0;
  ostringstream filename;

  filename << "screens/raw_voltage_plot" << n << ".png";

  //for (uint16_t data : adc_data)
  //  printf("%f\n", (0xFFFF - data) * (5.12f/65536.0f));

  for (int p = 0; p < PERIODS; p++) {
    for (int s = 0; s < STEPS; s++) {

    //printf("%f\n", _in_v[p][s][REAL]);
    //printf("%04x\n", _adc_data[i]);
    ys.push_back( (double)_in_v[p][s][REAL] );
    //ys.push_back( (double)_in_v[1][s][REAL] );
    xs.push_back( (p * STEPS) + s );
    //xs.push_back(s);
    }
  }


  //printf("%04x\n", step_incr);

  RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

	ScatterPlotSeries *series = GetDefaultScatterPlotSeriesSettings();
	series->xs = &xs;
	series->ys = &ys;

	ScatterPlotSettings *settings = GetDefaultScatterPlotSettings();
	settings->width = 800;
	settings->height = 600;
	settings->autoBoundaries = true;
	settings->autoPadding = true;
	settings->xMin = 0;
	settings->xMax = STEPS;
	settings->yMin = 2.5;
	settings->yMax = 2.7;


	//series->linearInterpolation = true;
	//series->lineType = toVector(L"dashed");
	//series->lineThickness = 2;
	//series->color = GetGray(0.3);

	//settings->title = toVector(L"x^2 - 2");
	//settings->xLabel = toVector(L"X axis");
	//settings->yLabel = toVector(L"Y axis");
	settings->scatterPlotSeries->push_back(series);

	DrawScatterPlotFromSettings(imageReference, settings);

	vector<double> *pngdata = ConvertToPNG(imageReference->image);
	WriteToFile(pngdata, filename.str());
	DeleteImage(imageReference->image);

  if (!overwrite) n++;

}

void plot_fft(bool overwrite) {

	vector<double> xs, ys;

  static int n = 0;
  ostringstream filename;

  filename << "screens/fft_plot" << n << ".png";

  //for (uint16_t data : adc_data)
  //  printf("%f\n", (0xFFFF - data) * (5.12f/65536.0f));

  for (int s = 0; s < STEPS; s++) {

    //printf("%f\n", (0xFFFF - _adc_data[i]) * (5.12f/65536.0f));
    //printf("%04x\n", _adc_data[i]);
    //ys.push_back( (double)(0xFFFF - _adc_data[i]) * (5.12f/65536.0f) );
    ys.push_back((double)_fft_mag[s] - (double)(_freespace_fft_mag[s]));
    xs.push_back(s);

  }
  
  //printf("%04x\n", step_incr);

  RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

	ScatterPlotSeries *series = GetDefaultScatterPlotSeriesSettings();
	series->xs = &xs;
	series->ys = &ys;
	series->linearInterpolation = true;
	//series->color = GetGray(0.3);

	//series->lineType = toVector(L"dashed");
	//series->lineThickness = 2;

	ScatterPlotSettings *settings = GetDefaultScatterPlotSettings();
	settings->width = 800;
	settings->height = 600;
	settings->autoBoundaries = false;
	settings->autoPadding = true;
	settings->xMin = 0;
	settings->xMax = STEPS;
	settings->yMin = 0;
	settings->yMax = 0.3;
	//settings->title = toVector(L"x^2 - 2");
	//settings->xLabel = toVector(L"X axis");
	//settings->yLabel = toVector(L"Y axis");
	settings->scatterPlotSeries->push_back(series);

	DrawScatterPlotFromSettings(imageReference, settings);

	vector<double> *pngdata = ConvertToPNG(imageReference->image);
	WriteToFile(pngdata, filename.str());
	DeleteImage(imageReference->image);

  if (!overwrite) n++;

}

int main() {


  // zeroing imaginary nums for fft
  //for (int i = 0; i < STEPS; i++) {
  //  //_in[i][IMAG] = 0;
  //}

  if ( spi_init() ) return 1;

  //calibrate_freespace_fft();
  for (int i = 0; i < 5; i++)
    run_daq();

  //calibrate_freespace_fft(0.80f);

  char c;

  // control loop
  while(1) {
  //for ( int i = 0; i < 160; i++) {

    if (cin.get() == 'q')
      break;

    thread t_daq(run_daq);
    t_daq.join();

    thread t_fft(run_fft);
    t_fft.join();

    thread t_plot(plot_fft, true);
    t_plot.join();

  }

  //printf("ABGn pins\n");
  //printf("g2 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_08));
  //printf("g1 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_10));
  //printf("g0 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_13));
  //printf("CDGn pins\n");
  //printf("g2 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_03));
  //printf("g1 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_05));
  //printf("g0 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_07));

  //printf("03 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_03));
  //printf("05 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_05));
  //printf("07 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_07));
  //printf("08 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_08));
  //printf("10 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_10));
  //printf("11 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_11));
  //printf("12 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_12));
  //printf("13 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_13));
  //printf("15 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_15));
  //printf("16 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_16));
  //printf("18 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_18));
  //printf("19 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_19));
  //printf("21 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_21));
  //printf("22 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_22));
  //printf("23 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_23));
  //printf("24 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_24));
  //printf("26 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_26));
  //printf("29 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_29));
  //printf("31 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_31));
  //printf("32 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_32));
  //printf("33 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_33));
  //printf("35 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_35));
  //printf("36 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_36));
  //printf("37 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_37));
  //printf("38 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_38));
  //printf("40 %d\n", bcm2835_gpio_lev(RPI_BPLUS_GPIO_J8_40));

  bcm2835_aux_spi_write(0x0000);

  bcm2835_spi_end();
  bcm2835_close();

  // free fftw resources
  fftw_destroy_plan(_p);
  fftw_free((fftw_complex*)_in);
  fftw_free((fftw_complex*)_out);

  return 0;
}
