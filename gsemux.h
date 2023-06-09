#ifndef GSEMUX
#define GSEMUX   

extern void init_gsemux(char *mcast_rxgse,char * mcast_rxiface,char *tunip,long tun_read_timeout);
extern void setpaddinggse();
extern void setgsemodcod(uint Constellation, uint CodeRate, uint FrameType,uint Pilots);
extern void setgsesr(uint sr);
extern void setbbframemcast(char * mcast_rx);
extern uint8_t m_variable_gse_coderate;
extern uint32_t m_MaxBBFrameByte;
extern uint32_t m_UsedBBFrameByte;
#endif