#include "iiofshelper.h"
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>

float readiio(const char *sPath)
{
    FILE *fdread = NULL;
    fdread = fopen(sPath, "r");
    char svalue[255];
    fgets(svalue, 255, fdread);
    fclose(fdread);
    float fvalue = atof(svalue);
    return fvalue;
}

size_t readiiohex(const char *sPath)
{
    FILE *fdread = NULL;
    fdread = fopen(sPath, "r");
    char svalue[255];
    fgets(svalue, 255, fdread);
    fprintf(stderr,"read %s\n",svalue);
    fclose(fdread);
    size_t xvalue;
    sscanf(svalue + 2, "%x", &xvalue);
    return xvalue;
}

void writeiiohex(const char *sPath, size_t xValue)
{

    //fprintf(stderr,"%s %x \n",sPath,xValue);
    FILE *fdwrite = NULL;

    fdwrite = fopen(sPath, "w");

    fprintf(fdwrite, "0x%x\n", xValue);
    fclose(fdwrite);
}

void writeiiohex(const char *sPath, size_t Address, size_t xValue)
{
    FILE *fdwrite = NULL;
    fdwrite = fopen(sPath, "w");
    if(fdwrite==NULL)
    {
      fprintf(stderr,"Writeiiohex error\n");  
    }
    //fprintf(stderr,"Writeiiohex 0x%x 0x%x\n",Address,xValue);
    fprintf(fdwrite, "0x%x 0x%x\n", Address, xValue);
    fclose(fdwrite);
}

static int fdregister=NULL;
    static unsigned page_addr, page_offset;
	static void *ptr=NULL;
static unsigned page_size;

void InitDevMem(size_t Address)
{
    

    page_size=sysconf(_SC_PAGESIZE);

    fdregister = open ("/dev/mem", O_RDWR);
    unsigned gpio_addr = Address; //Fixme if high mem
    page_addr = (gpio_addr & (~(page_size-1)));
	
	ptr = mmap(NULL, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, fdregister, page_addr);
    
}    


size_t ReadRegister( size_t Address)
{
   // writeiiohex(sPath, Address);
   // size_t Value = readiiohex(sPath);
    InitDevMem(Address);
    page_offset = Address - page_addr;
    size_t value = *((unsigned *)(ptr + page_offset));

    munmap(ptr, page_size);
    close(fdregister);
    return value;
}

void WriteRegister(size_t Address, size_t RegisterValue)
{
     InitDevMem(Address);
     page_offset = Address - page_addr;
     *((unsigned *)(ptr + page_offset)) = RegisterValue;
     munmap(ptr, page_size);
    close(fdregister);
    //writeiiohex(sPath, Address, RegisterValue);
}