#ifndef LTC2357_16_
#define LTC2357_16_

#include <bcm2835.h>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include <cmath>
#include <fftw3.h>

#define REAL 0
#define IMAG 1
#define _SPI_0 0
#define _SPI_1 1
#define _ADC_CONV RPI_BPLUS_GPIO_J8_33
#define _ADC_BUSY RPI_BPLUS_GPIO_J8_29

#define SAMPLE_SIZE 2144

class ltc2357_16 {
public:
  int init();
  void set_softspan(char _softSpan_data[]);
  void set_tx_data(char _send_data[]);
  void set_rx_data(char _read_data[]);
  void set_msg_len(int _len);
  void set_sample_rate(int _hz);
  void read_adc_fft();
  void close();
  double get_fft_mag_avg();
  int get_fft_largest_freq();
  ltc2357_16();
  ~ltc2357_16();

private:
  char *softSpan_data = nullptr;
  char *send_data = nullptr;
  char *read_data = nullptr;
  int len = 0;
  int sample_delay_clk_ticks = CLOCKS_PER_SEC / SAMPLE_SIZE;
  double dft_hz_step = (double)(1.0 / SAMPLE_SIZE);
  double fft_mag[SAMPLE_SIZE];
  double fft_ang[SAMPLE_SIZE];
//  double fft_mag_avg;
  fftw_complex *in;
  fftw_complex *out;
  fftw_plan p;
};

#endif

inline double mag(double real, double imag) {
  return sqrt((real * real) + (imag * imag));
}

inline double ang(double real, double imag) { return atan(imag / real); }

// bcm2835 inits
// returns 0 on success
//
int ltc2357_16::init() {

  if (!bcm2835_init()) {
    printf("bcm2835_init failed\n");
    return 1;
  }

  if (!bcm2835_spi_begin()) {
    printf("bcm2835_spi_begin failed\n");
    return 1;
  }

  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS1);
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW);

  // Conversion pin set to output
  bcm2835_gpio_fsel(_ADC_CONV, BCM2835_GPIO_FSEL_OUTP);

  // allocating space to fftw vars
  in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * SAMPLE_SIZE);
  out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * SAMPLE_SIZE);

  // create fftw plan
  p = fftw_plan_dft_1d(SAMPLE_SIZE, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

  // zeroing imaginary nums for fft
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    in[i][IMAG] = 0;
  }

  return 0;
}

/* Message data example
 *
 * 24 bit data
 * char softSpan_data[] = { 0x00, 0x10, 0x00 };
 * char send_data[]     = { 0x00, 0x00, 0x00 };
 * char read_data[]     = { 0x00, 0x00, 0x00 };
 *
 * 16 bit data
 * channel 0
 * char softSpan_data[] = { 0x00, 0x10 };
 * char send_data[]     = { 0x00, 0x00 };
 * char read_data[]     = { 0x00, 0x00 };
 */

// Sets softspan data
//
void ltc2357_16::set_softspan(char _softSpan_data[]) {
  this->softSpan_data = _softSpan_data;
}

// Sets transmit data
//
void ltc2357_16::set_tx_data(char _send_data[]) {
  this->send_data = _send_data;
}

// Sets recieve data
//
void ltc2357_16::set_rx_data(char _read_data[]) {
  this->read_data = _read_data;
}

// Sets message length
//
void ltc2357_16::set_msg_len(int _len) { this->len = _len; }

// Reads samples from adc then runs fft on sample set
//
void ltc2357_16::read_adc_fft() {
  // Read ADC loop
  // Taking [SAMPLE_SIZE] number of samples

  clock_t t;

  //std::cout << sample_delay_clk_ticks << std::endl;

  for (int i = 0; i < SAMPLE_SIZE; i++) {
    
    t = clock();

    // spi transfer
    bcm2835_spi_transfernb(softSpan_data, read_data, len);
    bcm2835_gpio_set(_ADC_CONV);
    //fflush(stdout);
    bcm2835_gpio_clr(_ADC_CONV);
    //fflush(stdout);
    bcm2835_spi_transfernb(send_data, read_data, len);

    // reading 16 bit voltage data into fftw_complex input real component
    in[i][REAL] = (read_data[0] << 8) | (read_data[1]);

    while(clock() - t < sample_delay_clk_ticks);
    //usleep(sample_delay_clk_ticks);
  }

  // execute fftw plan
  fftw_execute(p);

  // calculate magnitude and angle of fft
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    fft_mag[i] = mag(out[i][REAL], out[i][IMAG]);
    fft_ang[i] = ang(out[i][REAL], out[i][IMAG]);
  }
}

// Close spi connection and free resources
//
void ltc2357_16::close() {
  // close spi connection
  bcm2835_spi_end();
  bcm2835_close();

  // free fftw resources
  fftw_destroy_plan(p);
  fftw_free(in);
  fftw_free(out);
}

// Gets average fft magnitude
//
double ltc2357_16::get_fft_mag_avg() { 

  double fft_mag_avg = 0;

  // calculate average of fft magnitudes
  for (int i = 0; i < SAMPLE_SIZE; i++)
    fft_mag_avg += fft_mag[i];
  fft_mag_avg = (5.0 / 65536.0) * (fft_mag_avg / (SAMPLE_SIZE));

  return fft_mag_avg;
}

int ltc2357_16::get_fft_largest_freq() {

  double max_mag = 0;
  int max_mag_idx = 0;

  for (int i = 48; i < (SAMPLE_SIZE - 48); i++) {
    std::cout << fft_mag[i] << std::endl;
    if (fft_mag[i] > max_mag) {
      max_mag = fft_mag[i];
      max_mag_idx = i;
    }
  }

  max_mag_idx++;
  //std::cout << max_mag_idx << std::endl;
  //std::cout << max_mag << std::endl;

  return max_mag_idx;
}

ltc2357_16::ltc2357_16() {}

ltc2357_16::~ltc2357_16() {}
