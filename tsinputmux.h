#ifndef TSMUX
#define TSMUX   
//#include "./dvbs2neon/dvbs2neon.h"

extern void init_tsmux(char *mcast_ts,char * mcast_iface);
extern void settsmodcode();
extern void setpaddingts();
extern uint8_t BBFrameNeonBuff[144000] __attribute__((aligned(128)));
extern uint8_t symbolbuff[144 * 1024] __attribute__((aligned(16)));
extern void setneonmodcod(uint Constellation,uint CodeRate,uint FrameType,uint Pilots);
extern uint8_t m_variable_ts_coderate;
extern void settssource(int tssource,char *arg);

extern int m_tssource;
extern char m_mcast_ts[255];
extern char m_mcast_iface[255];
extern char m_ts_filename[255];
#endif