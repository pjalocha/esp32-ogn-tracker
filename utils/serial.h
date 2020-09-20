#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

class SerialPort
{ public:

   int DeviceHandle;

  public:

   SerialPort()
     { DeviceHandle=(-1); }

   ~SerialPort()
     { Close(); }

   int isOpen(void) const { return DeviceHandle>=0; }

   int Open(const char *DeviceName="/dev/ttyS0", int BaudRate=9600)
     { Close();

       DeviceHandle=open(DeviceName, O_RDWR | O_NOCTTY | O_NDELAY );
       if(DeviceHandle<0) return -1;

       int Speed; // speed_t Speed;
            if(BaudRate==4800) Speed=B4800;
       else if(BaudRate==9600) Speed=B9600;
       else if(BaudRate==19200) Speed=B19200;
       else if(BaudRate==38400) Speed=B38400;
       else if(BaudRate==57600) Speed=B57600;
       else if(BaudRate==115200) Speed=B115200;
       else if(BaudRate==230400) Speed=B230400;
       // else if(BaudRate==128000) Speed=B128000;
       // else if(BaudRate==256000) Speed=B256000;
       else return -2;

       struct termios Options;
       tcgetattr(DeviceHandle, &Options);

       if(cfsetispeed(&Options, Speed)<0) return -2;
       if(cfsetospeed(&Options, Speed)<0) return -2;
       // cfsetspeed(&Options, Speed);

       Options.c_cflag |= (CLOCAL | CREAD);

       Options.c_cflag &= ~PARENB;   // 8-bits, no parity
       Options.c_cflag &= ~CSTOPB;
       Options.c_cflag &= ~CSIZE;
       Options.c_cflag |= CS8;

       Options.c_cc[VTIME] = 0;
       Options.c_cc[VMIN]  = 1;

       tcsetattr(DeviceHandle, TCSANOW, &Options);
/*
       int Bytes=sizeof(Options);
       uint8_t *Ptr = (uint8_t *)(&Options);
       printf("Options[%d] =", Bytes);
       for( ; Bytes; Bytes--, Ptr++)
       { printf(" %02X", *Ptr); }
       printf("\n");
*/
     return 0; }

   int OpenFileForRead(const char *FileName)
     { Close();
       DeviceHandle=open(FileName, O_RDONLY);
       if(DeviceHandle<0) return -1;
       return 0; }

   int Close(void)
     { if(DeviceHandle>=0)
       { close(DeviceHandle); DeviceHandle=(-1); }
       return 0; }

   int Read(char *Buffer, size_t MaxChars)
     { int Len=read(DeviceHandle, Buffer, MaxChars);
       return Len<=0 ? 0:Len; }

   int Read(char &Char)
     { return Read(&Char, 1); }

   int Write(const char *Buffer, size_t Chars)
     { int Len=write(DeviceHandle, Buffer, Chars);
       return Len<=0 ? 0:Len; }

   int Write(const char *String)
     { return Write(String, strlen(String)); }

   int Write(char Char)
     { return Write(&Char,1); }

} ;

