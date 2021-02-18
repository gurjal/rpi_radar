#include "../chipControl/ltc2641.h"
#include "../chipControl/ltc2357-16.h"

#include <thread>
#include <time.h>

using namespace std;

void run_dac_in_parrallel() {

  char softSpan_data[] = {0x00, 0x10};
  char send_data[] = {0x00, 0x00};
  char read_data[] = {0x00, 0x00};

  //ltc2641 dac;
  ltc2357_16 adc;

  //dac.init();
  //dac.set_digital_steps_per_period_and_hz(10, 16, -23);

  adc.init();
  
  adc.set_softspan(softSpan_data);
  adc.set_tx_data(send_data);
  adc.set_rx_data(read_data);
  adc.set_msg_len(2);


  //thread t1(&ltc2641::start_dac, &dac, 16*4);
  //sleep(2);

  thread t2(&ltc2357_16::read_adc_fft, &adc);
  t2.join();
  //t1.join();

  cout << adc.get_fft_mag_avg() << endl;
  cout << adc.get_fft_largest_freq() << endl;

  //dac.start_dac(16*10);



  //dac.close();
  adc.close();

}

int main() {

  time_t timer;
  time(&timer);
  clock_t t = clock();

  while (time(NULL) - timer < 20)
  run_dac_in_parrallel();

  cout << clock() - t << " clk tick runtime" << endl;
  cout << time(NULL) - timer << " second runtime" << endl;
  return 0;
}
