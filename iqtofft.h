#ifndef IQTOFFT
#define IQTOFFT

void init_fft(uint16_t fft_size, uint16_t average);
uint16_t *iqtofft(short *bufferiq, uint16_t RxSize, int NbSweep,size_t *bin);
void update_web_param(float freqrx,float span);
void publishwebfft(uint16_t *powfftdb,int Totalbin);
size_t PrepareSpan (uint64_t CenterFrequency, uint64_t SR, uint64_t span);
void RecallFastlockTune(int fastlock_profile_no);
#endif