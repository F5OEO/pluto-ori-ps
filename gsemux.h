#ifndef GSEMUX
#define GSEMUX   

extern void init_gsemux(char *mcast_rxgse,char * mcast_rxiface,char *tunip,long tun_read_timeout);
extern void setpaddinggse();
extern void setgsemodcod(uint Constellation, uint CodeRate, uint FrameType);
#endif