#include "../chipControl/ltc2641.h"
#include "../chipControl/ltc2357-16.h"

#include <thread>
#include <time.h>

using namespace std;

void timer_ithink() {
  sleep(1);
}

void run_dac_in_parrallel(int steps, int hz, int tune, int secs) {

  char softSpan_data[] = {0x00, 0x10};
  char send_data[] = {0x00, 0x00};
  char read_data[] = {0x00, 0x00};

  ltc2641 dac;
  ltc2357_16 adc;

  dac.init();
  dac.set_digital_steps_per_period_and_hz(steps, hz, tune);

  adc.init();
  
  adc.set_softspan(softSpan_data);
  adc.set_tx_data(send_data);
  adc.set_rx_data(read_data);
  adc.set_msg_len(2);


  thread t1(&ltc2641::start_dac, &dac, 16*secs);
  //dac.start_dac(16 * secs);

  sleep(2);

  thread t2(&ltc2357_16::read_adc_fft, &adc);
  //adc.read_adc_fft();
  t2.join();

  t1.join();


  cout << adc.get_fft_mag_avg() << endl;
  //cout << adc.get_fft_largest_freq() << endl;

  //dac.start_dac(16*10);



  dac.close();
  adc.close();

}

void tune_dac(int steps, int target_hz) {

  int current_hz = 0;
  int tuning_step[] = {1, 5, 10, 25, 50, 100};
  int step_idx = 5;
  int current_tuning = 100;
  double dft_mult = (double)(100) * (double)(1.0/2144.0);
  int secs = 3;

  char softSpan_data[] = {0x00, 0x10};
  char send_data[] = {0x00, 0x00};
  char read_data[] = {0x00, 0x00};

  ltc2641 dac;
  ltc2357_16 adc;

  dac.init();
  adc.init();
  
  adc.set_softspan(softSpan_data);
  adc.set_tx_data(send_data);
  adc.set_rx_data(read_data);
  adc.set_msg_len(2);


  cout << "Starting tuning step size is -> " << tuning_step[step_idx] << '\n' << endl;

  while (1) {

    dac.set_digital_steps_per_period_and_hz(20, 16, current_tuning);

    thread t1(&ltc2641::start_dac, &dac, 16*secs);
    sleep(secs - (secs - 1));

    thread t2(&ltc2357_16::read_adc_fft, &adc);
    t2.join();
    t1.join();

    current_hz = 2144 * adc.get_fft_largest_freq();

    cout << "Current Hz -> " << current_hz << '\n' \
      << "Current Tuning -> " << current_tuning << '\n' \
      << "Current Step Size -> " << tuning_step[step_idx] << '\n' \
      << endl;

    if (current_hz > target_hz) {
      if (step_idx == 0)
        break;
      else {
        current_tuning += tuning_step[step_idx];
        current_tuning -= tuning_step[--step_idx];
      }
    } else
        current_tuning -= tuning_step[step_idx];
  }

  dac.close();
  adc.close();

  cout << "Optimal tuning is " << current_tuning << endl;
}

int main() {

  //time_t timer;
  //time(&timer);
  //clock_t t = clock();
  //double test = 1.0/2144.0;
  


  //tune_dac(10, 16);
  run_dac_in_parrallel(30, 16, 20, 10);
  
  //cout << clock() - t << " clk tick runtime" << endl;
  //cout << time(NULL) - timer << " second runtime" << endl;
  return 0;
}
