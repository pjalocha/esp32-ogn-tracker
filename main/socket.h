#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

class Socket
{ public:
   struct addrinfo *Host;
   int Link;

  public:
   Socket()
   { Host=0;
     Link=(-1); }

  ~Socket()
   { Disconnect(); }

   uint32_t getIP(void) const
   { if(Host==0) return 0;
     uint32_t *IP = (uint32_t *)(Host->ai_addr->sa_data+2);
     return IP[0]; }

   uint16_t getPort(void) const
   { if(Host==0) return 0;
     uint16_t *Port = (uint16_t *)(Host->ai_addr->sa_data);
     return Port[0]; }

   int Connect(const char *HostName, const char *ServiceName)
   { const struct addrinfo Hints = { ai_flags:0, ai_family:AF_INET, ai_socktype:SOCK_STREAM,
                                     ai_protocol:0, ai_addrlen:0, ai_addr:0,
                                     ai_canonname:0, ai_next:0 };
     int Err = getaddrinfo(HostName, ServiceName, &Hints, &Host);
     if((Err!=0) || (Host==0)) { Disconnect(); return -1; }
     Link = socket(Host->ai_family, Host->ai_socktype, 0);
     if(Link<0) { Disconnect(); return -2; }
     Err=connect(Link, Host->ai_addr, Host->ai_addrlen);
     if(Err!=0) { Disconnect(); return -3; }
     return Link; }

   void Disconnect(void)
   { if(Link>=0) { close(Link); Link=(-1); }
     if(Host) { freeaddrinfo(Host); Host=0; }
   }

   bool isConnected(void) { return Link>=0; }

   int setBlocking(int Block=1)
   { int Flags = fcntl(Link, F_GETFL, 0);
     if(Block) Flags &= ~O_NONBLOCK;
          else Flags |=  O_NONBLOCK;
     return fcntl(Link, F_SETFL, Flags); }

   int setNonBlocking(void)
   { return setBlocking(0); }

   int Send(void *Buff, int Len)
   { if(!isConnected()) return -1;
     return send(Link, Buff, Len, 0); }
     // return write(Link, Buff, Len); }

   int Receive(void *Buff, int Len)
   { if(!isConnected()) return -1;
     int Ret=recv(Link, Buff, Len-1, 0);
     if(Ret>=0) return Ret;
     return errno==EWOULDBLOCK ? 0:Ret; }
     // return read(Link, Buff, Len); }

   int setReceiveTimeout(int Sec)
   { struct timeval Timeout = { tv_sec:Sec, tv_usec:0 };
     return setsockopt(Link, SOL_SOCKET, SO_RCVTIMEO, &Timeout, sizeof(Timeout)); }

} ;


class DataServer
{ public:
   static const int MaxConn = 2;
   int Link;
   int Client[MaxConn];

  public:
   DataServer()
   { Link=(-1);
     for(int Idx=0; Idx<MaxConn; Idx++)
       Client[Idx]=(-1);
   }

   int Listen(int Port)
   { struct sockaddr_in dest_addr;
     dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
     dest_addr.sin_family = AF_INET;
     dest_addr.sin_port = htons(Port);
     Link = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
     if(Link<0) return -1;
     int Err = bind(Link, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
     if(Err!=0) { close(Link); Link=(-1); return -1; }
     Err = listen(Link, MaxConn);
     if(Err!=0) { close(Link); Link=(-1); return -1; }
     setBlocking(Link, 0);
     return Link; }

   static int setBlocking(int Link, int Block=1)
   { int Flags = fcntl(Link, F_GETFL, 0);
     if(Block) Flags &= ~O_NONBLOCK;
          else Flags |=  O_NONBLOCK;
     return fcntl(Link, F_SETFL, Flags); }

   int Accept(void)                                // check for new clients
   { struct sockaddr_in6 source_addr;
     uint addr_len = sizeof(source_addr);
     int New=accept(Link, (struct sockaddr *)&source_addr, &addr_len); // attempt to accept a new client
     if(New<0) return -1;                          // if nobody waiting then give up
     int Idx;
     for(Idx=0; Idx<MaxConn; Idx++)                // go through the list
     { if(Client[Idx]<0) break; }                  // stop on the first free slot
     if(Idx>=MaxConn) { close(New); return -1; }   // if no free slots then close the new client and give up
     Client[Idx]=New; return New; }

   // static int Send(int Link, void *Buff, int Len)
   // { if(Link<0) return -1;
   //   return send(Link, Buff, Len, 0); }

   int Send(const char *Buff)
   { return Send(Buff, strlen(Buff)); }

   int Send(const void *Buff, int Len)
   { if(Link<0) return -1;
     int Count=0;
     for(int Idx=0; Idx<MaxConn; Idx++)
     { if(Client[Idx]<0) continue;
       int Sent=send(Client[Idx], Buff, Len, 0);
       if(Sent!=Len) { close(Client[Idx]); Client[Idx]=(-1); continue; }
       Count++; }
     return Count; }

} ;
