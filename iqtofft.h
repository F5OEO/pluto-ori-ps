#ifndef IQTOFFT
#define IQTOFFT

void init_fft(uint16_t fft_size, uint16_t average);
uint16_t * iqtofft(short *bufferiq, uint16_t RxSize);
void update_web_param(float freqrx,float span);
void publishwebfft(uint16_t *powfftdb);
#endif