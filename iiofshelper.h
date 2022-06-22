#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <errno.h>
float readiio(const char *sPath);
size_t readiiohex(const char *sPath);
void writeiiohex(const char *sPath, size_t xValue);
void writeiiohex(const char *sPath, size_t Address, size_t xValue);
size_t ReadRegister( size_t Address);
void WriteRegister(size_t Address, size_t RegisterValue);
