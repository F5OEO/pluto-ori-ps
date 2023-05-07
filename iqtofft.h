#ifndef IQTOFFT
#define IQTOFFT

void init_fft(uint16_t fft_size, uint16_t average);
void iqtofft(short *bufferiq, uint16_t RxSize);
void update_web_param(float freqrx,float span);

#endif