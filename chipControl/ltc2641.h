#ifndef SPIDEVICE_H_
#define SPIDEVICE_H_

#include <bcm2835.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define LEN 2
#define SPI_DEVICE_PATH "/dev/spidev1.0"

class ltc2641 {
public:
  int init();
  void set_hz(int _hz);
  void set_digital_steps_per_period_and_hz(int _steps, int _hz, int tune);
  void tune_delay(int tune);
  void start_dac(int periods = INT32_MAX);
  void close();
  ltc2641();
  ~ltc2641();

private:
  int spi_fd;
  uint8_t mode = SPI_MODE_0;
  uint32_t speed = 500000;
  uint8_t delay = 0;
  uint8_t bits_per_word = 8;
  struct spi_ioc_transfer tf;
  uint8_t tx[2];
  uint8_t tx_state[2];
  uint8_t rx[2];
  int hz;
  int steps;
  int micro_delay;
  int bcm_micro_delay;
  int while_micro_delay;
  int half_micro_delay;
  uint8_t increment[2];
};

#endif

// dac_ltc2641.open();
// dac_ltc2641.setMode(mode);
// dac_ltc2641.setSpeed(speed);
// dac_ltc2641.setDelay(delay);

int ltc2641::init() {

  // Open spi device
  spi_fd = ::open(SPI_DEVICE_PATH, O_RDWR);
  if (spi_fd == -1) {
    perror("cant open spi device. use sudo dummy");
    return -1;
  }

  // Set spi mode
  if (ioctl(spi_fd, SPI_IOC_WR_MODE, &this->mode) == -1) {
    perror("cant set spi mode");
    return -1;
  }
  if (ioctl(spi_fd, SPI_IOC_RD_MODE, &this->mode) == -1) {
    perror("cant set spi mode");
    return -1;
  }

  // Set spi speed
  if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &this->speed) == -1) {
    perror("cant set max speed");
    return -1;
  }
  if (ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &this->speed) == -1) {
    perror("cant set max speed");
    return -1;
  }

  // Set bits per word
  if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &this->bits_per_word) == -1) {
    perror("cant set bits per word");
    return -1;
  }
  if (ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &this->bits_per_word) == -1) {
    perror("cant set bits per word");
    return -1;
  }

  memset(&tf, 0, sizeof(tf));
  tf.tx_buf = (uint64_t)tx;
  tf.rx_buf = (uint64_t)rx;
  tf.len = LEN;
  tf.delay_usecs = delay;
  tf.speed_hz = speed;
  tf.bits_per_word = bits_per_word;
  tf.cs_change = 0; // 1 to leave CS low // O to leave CS high

  return 0;
}

void ltc2641::set_digital_steps_per_period_and_hz(int _steps, int _hz, int tune) {
  steps = _steps;
  hz = _hz;
  micro_delay = 1000000 / hz / steps;
  bcm_micro_delay = (micro_delay / 4) + tune;
  while_micro_delay = bcm_micro_delay * 3;
  half_micro_delay = (micro_delay / 2);
  increment[0] = ((0xFFFF / steps) >> 8);
  increment[1] = (0x00FF & (0xFFFF / steps));
}

void ltc2641::start_dac(int periods) {

  clock_t t;
  clock_t tune;
  tune = clock();

  for (int j = 0; j < periods; j++) {
    for (int i = 0; i < steps; i++) {

      t = clock();

      //printf("t at start -> %d\n", t);

      // printf("%x, %x, %x, %x\n", tx[0], tx[1], tx[2], tx[3]);

      // prepare transfer data
      tx[0] = tx_state[0];
      tx[1] = tx_state[1];
      tf.rx_buf = (uint64_t)tx;

      // spi transfer
      if (ioctl(this->spi_fd, SPI_IOC_MESSAGE(1), &tf) == -1) {
        perror("cant send spi message");
        return;
      }

      // increment transfer data
      tx_state[0] = tx_state[0] + increment[0];
      tx_state[1] = tx_state[1] + increment[1];

      // wait for clock cycles per step

      //printf("%ld  ..waiting..  ", clock() - t);

      // bcm delay
      //bcm2835_delayMicroseconds(micro_delay);

      // while delay
      //while ((clock() - t) < micro_delay);

      // 1/4 bcm delay and 3/4 while delay
      bcm2835_delayMicroseconds(bcm_micro_delay);
      while ((clock() - t) < while_micro_delay);

      // half bcm delay and half while delay
      //usleep(half_micro_delay);
      //while ((clock() - t) < half_micro_delay);

      //printf("t at end -> %d\n", t);
      //printf("%ld\n", clock() - t);
    }
    // reset transfer data
    tx_state[0] = 0;
    tx_state[1] = 0;
  }
  //printf("%ld\n", clock() - tune);
  //printf("%d\n", half_micro_delay);
  //printf("0x%x\n", increment[0]);
  //printf("0x%x\n", increment[1]);
}

void ltc2641::close() {
  ::close(this->spi_fd);
  this->spi_fd = -1;
}

ltc2641::ltc2641() {}

ltc2641::~ltc2641() { this->close(); }
