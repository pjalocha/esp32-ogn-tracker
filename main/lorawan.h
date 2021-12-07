#ifndef __LORAWAN_H__
#define __LORAWAN_H__

#include <stdint.h>

#ifdef WITH_ESP32
#include "nvs.h"
#endif

#include "LoRaMacCrypto.h"

#include "rfm.h"

class LoRaWANnode
{ public:
   static const uint8_t Chans=8;
   uint64_t AppEUI;        // from application registration: Application identification
   uint8_t  AppKey[16];    // from device registration: application encryption/decryption key
   uint64_t DevEUI;        // Device identification (MAC)
   uint32_t DevNonce;      // unique counter kept by the device for Join-Requests
   uint8_t  NetSesKey[16]; // from Join-Accept: Network Session Key
   uint8_t  AppSesKey[16]; // from Join-Accept: App Session Key
   uint32_t JoinNonce;     // from Join-Accept: unique must not be reused
   uint32_t HomeNetID;     // from Join-Accept: Home Network ID
   uint32_t DevAddr;       // from Join-Accept: Device Address
   uint8_t  DLsetting;     // from Join-Accept: DownLink configuration: OptNeg:1 | RX1 data rate offset:3 | RX2 data rate:4
   uint8_t  RxDelay;       // from Join-Accept: RFU:4 | Del:4  Del=1..15s for the RX1, RX2 delay is Del+1
   uint8_t  State;         // 0:disconencted, 1:join-request sent, 2:join-accept received, 3:uplink-packet sent
   uint8_t  Chan;          // [0..7] Current channel being used
   uint32_t UpCount;       // [seq] Uplink frame counter: reset when joining the network
   uint32_t DnCount;       // [seq] Downlink frame counter: reset when joining the network
   uint32_t TxCount;       // [packets] transmitted to the network
   uint32_t LastTx;        // [sec] last transmission
   uint32_t RxCount;       // [packets] received from the network
   uint32_t LastRx;        // [sec] when last heard from the network
    int8_t  RxSNR;         // [0.25dB] SNR on receive
    int8_t  RxRSSI;        // [dBm] signal strength
   union
   { uint8_t  Flags;
     struct
     { bool RxACK :1;    // received ACK
       bool TxACK :1;    // ACK to be transmitted
       bool RxPend:1;    // more frames pending for reception
     } ;
   } ;
   uint8_t  Spare;
   uint8_t  Packet[40];    // generic packet for storage/processing

  public:
   LoRaWANnode() { Reset(); }

   void Reset(void)
   { State=0; DevNonce=0; JoinNonce=0;
     LastTx=0; TxCount=0; LastRx=0; RxCount=0; Flags=0; }

   void Reset(uint64_t MAC, uint8_t *AppKey=0)
   { AppEUI=0x70B3D57ED0035895;
     DevEUI=MAC;
     if(AppKey) memcpy(this->AppKey, AppKey, 16);
     Reset(); }

   void Disconnect(void)
   { State=0; }

   uint8_t incrChan(uint8_t Step=1) { Chan+=Step; if(Chan>=Chans) Chan-=Chans; return Chan; }

   int Save(FILE *File) { return fwrite(this, sizeof(LoRaWANnode), 1, File); }
   int Save(const char *FileName)
   { FILE *File=fopen(FileName, "wb"); if(File==0) return 0;
     int Written=Save(File); fclose(File); return Written; }

   int Restore(FILE *File) { return fread(this, sizeof(LoRaWANnode), 1, File); }
   int Restore(const char *FileName)
   { FILE *File=fopen(FileName, "rb"); if(File==0) return 0;
     int Read=Restore(File); fclose(File); return Read; }

   static int ReadHex(uint8_t *Data, int Len, const char *Inp)
   { int Bytes=0;
     for( ; Bytes<Len; )
     { int8_t H = Read_Hex1(*Inp++); if(H<0) break;
       int8_t L = Read_Hex1(*Inp++); if(L<0) break;
       Data[Bytes++] = (H<<4) | L; }
     return Bytes; }

   // int readAppEUI(const char *Inp) { return ReadHex(&AppEUI, 8, Inp); }
   int readAppKey(const char *Inp) { return ReadHex(AppKey,16, Inp); }
   // int readDevEUI(const char *Inp) { return ReadHex(&DevEUI, 8, Inp); }

  template<class Type>
   static Type readInt(const uint8_t *Inp, int Len=sizeof(Type))
   { int Idx=Len-1;
     Type Value = Inp[Idx];
     for( ; Idx>0; )
     { Idx--; Value<<=8; Value|=Inp[Idx]; }
     return Value; }

  template<class Type>
   static int writeInt(uint8_t *Out, Type Value, int Len=sizeof(Type), bool Rev=0)
   { if(Rev)
     { for( int Idx=Len; Idx>0; )
       { Out[--Idx] = Value; Value>>=8; }
     }
     else
     { for(int Idx=0; Idx<Len; )
       { Out[Idx++] = Value; Value>>=8; }
     }
     return Len; }

   int getJoinRequest(uint8_t *Req)
   { Req[0] = 0x00;                                // MHDR, Join-Request: 000 000 00
     memcpy(Req+1, &AppEUI, 8);                    // AppEUI
     memcpy(Req+9, &DevEUI, 8);                    // DevEUI
     DevNonce++;                                   // increment DevNonce for a new request
     Req[17] = DevNonce;                           // DevNonce
     Req[18] = DevNonce>>8;
     uint32_t MIC=0;
     LoRaMacJoinComputeMic(Req, 19, AppKey, &MIC); // compute MIC
     memcpy(Req+19, &MIC, 4);                      // append MIC
     State=1;                                      // State: Join-Request sent
     return 23; }                                  // 23 bytes packet length

   int getJoinRequest(uint8_t **Req) { int Len=getJoinRequest(Packet); *Req = Packet; return Len; }

   int procJoinAccept(const RFM_LoRa_RxPacket &RxPacket)
   { int Ret=procJoinAccept(RxPacket.Byte, RxPacket.Len); if(Ret<0) return Ret;
     RxSNR  = RxPacket.SNR;
     RxRSSI = RxPacket.RSSI;
     return Ret; }

   int procJoinAccept(const uint8_t *PktData, int PktLen)                        // process Join-Accept packet (5sec after Join-Request)
   { if(PktLen<13) return -1;
     uint8_t Type = PktData[0]>>5; if(Type!=1) return -1;
     Packet[0] = PktData[0];
     LoRaMacJoinDecrypt(PktData+1, PktLen-1, AppKey, Packet+1);                  // decrypt the Join-Accept packet
     uint32_t MIC=0;
     LoRaMacJoinComputeMic(Packet, PktLen-4, AppKey, &MIC);                      // Compute MIC
     if(memcmp( Packet+PktLen-4, &MIC, 4)) return -1;                            // Compare with the packet
     LoRaMacJoinComputeSKeys(AppKey, Packet+1, DevNonce, NetSesKey, AppSesKey);  // derive Network Session and Application Session keys
     JoinNonce = readInt<uint32_t>(Packet+1, 3);                                 // this should be not smaller than the previous one
     HomeNetID = readInt<uint32_t>(Packet+4, 3);
     DevAddr   = readInt<uint32_t>(Packet+7, 4);
     DLsetting = Packet[11];
     RxDelay   = Packet[12];
     State = 2;                                                                  // State = accepted on network
     UpCount = 0;
     DnCount = 0;
#ifdef WITH_PRINTF
     printf("Accept[%d] ", PktLen-4);
     for(int Idx=0; Idx<PktLen-4; Idx++)
       printf("%02X", Packet[Idx]);
     printf(" HomeNetID:%06X, DevAddr:%08X, DL:%02X, RxDelay:%02X\n", HomeNetID, DevAddr, DLsetting, RxDelay);
     printf("NetSesKey: "); PrintHex(NetSesKey, 16); printf("\n");
     printf("AppSesKey: "); PrintHex(AppSesKey, 16); printf("\n");
#endif
     return 0; }

   int getDataPacket(uint8_t *Packet, const uint8_t *Data, int DataLen, uint8_t Port=1, bool Confirm=0)
   { if(State<2) return 0;                                   // not joined to the network yet
     uint8_t Type = Confirm?0x04:0x02;                       // request confirmation or not ?
     int PktLen=0;
     Packet[PktLen++] = Type<<5;                             // packet-type
     PktLen+=writeInt(Packet+PktLen, DevAddr, 4);            // Device Address
     uint8_t Ctrl=0;                                         // Frame Control
     if(TxACK) { Ctrl|=0x20; TxACK=0; }                      // if there is ACK to be transmitted
     Packet[PktLen++] = Ctrl;                                // Frame Control: ADR | ADR-ACK-Req | ACK | ClassB | FOptsLen[4]
     PktLen+=writeInt(Packet+PktLen, UpCount, 2);            // uplink frame counter
     Packet[PktLen++] = Port;                                // port
     LoRaMacPayloadEncrypt(Data, DataLen, AppSesKey, DevAddr, 0, UpCount, Packet+PktLen); PktLen+=DataLen; // copy+encrypt user data
     uint32_t MIC=0;
     LoRaMacComputeMic(Packet, PktLen, NetSesKey, DevAddr, 0x00, UpCount, &MIC); // calc. MIC
     memcpy(Packet+PktLen, &MIC, 4); PktLen+=4;               // append MIC
     UpCount++; State=3; return PktLen; }                     // return the packet size

   int getDataPacket(uint8_t **Pkt, const uint8_t *Data, int DataLen, uint8_t Port=1, bool Confirm=0)
   { int Len=getDataPacket(Packet, Data, DataLen, Port, Confirm); *Pkt = Packet; return Len; }

   int procRxData(const RFM_LoRa_RxPacket &RxPacket)
   { int Ret = procRxData(RxPacket.Byte, RxPacket.Len); if(Ret<0) return Ret;
     RxSNR  += (RxPacket.SNR-RxSNR+1)/2;                            // if good packet then update the signal statistics
     RxRSSI += (RxPacket.RSSI-RxRSSI+1)/2;
     return Ret; }

   int procRxData(const uint8_t *PktData, int PktLen)
   { if(PktLen<12) return -1;
     uint8_t Type = PktData[0]>>5; if(Type!=3 && Type!=5) return -1; // Frame Type: 3=unconfirmed data downlink, 5=confirmed data downlink
     uint32_t Addr=readInt<uint32_t>(PktData+1, 4);                  // device address
     if(Addr!=DevAddr) return 0;                                     // check if packet is for us (it could be for somebody else)
     uint8_t Ctrl = PktData[5];                                      // Frame Control: ADR | RFU | ACK | FPending | FOptLen[4]
     uint32_t Count = readInt<uint32_t>(PktData+6, 2);               // download counter
     int16_t CountDiff = Count-DnCount;                              // how many we have missed ?
     if(CountDiff<=0) return -1;                                     // attempt to reuse the counter: drop this packet
     uint32_t MIC=0;
     LoRaMacComputeMic(PktData, PktLen-4, NetSesKey, Addr, 0x01, Count, &MIC);
     if(memcmp(PktData+PktLen-4, &MIC, 4)) return -1;                // give up if MIC does not match
     uint8_t OptLen = Ctrl&0x0F;                                     // Options: how many bytes
     uint8_t DataOfs = 8 + OptLen;                                   // where the port byte should be
     if(OptLen) procRxOpt(PktData+8, OptLen);                        // process the options (these are not encrypted)
     uint8_t DataLen = PktLen-DataOfs-4;                             // number of bytes of the user data field
     if(DataLen)                                                     // if non-zero
     { Packet[0] = PktData[DataOfs];                                 // copy port number
       LoRaMacPayloadDecrypt(PktData+DataOfs+1, DataLen-1, AppSesKey, Addr, 0x01, Count, Packet+1); } // decrypt and copy the user data
#ifdef WITH_PRINTF
     printf("RxData: [%d] ", DataLen);
     for(int Idx=0; Idx<DataLen; Idx++)
       printf("%02X", Packet[Idx]);
     printf("\n");
#endif
     DnCount += CountDiff;                                           // update the download sequence counter
     if(Ctrl&0x40) RxACK=1;                                          // we got ACK to our ACK request
     if(Type==5) TxACK=1;                                            // if ACK requested
     RxPend = Ctrl&0x10;                                             // is there more data pending to be received on next round ?
     State=2;
     // if(DataLen==0)
     // { Packet[0]=0x00; DataLen++;                                    // copy MAC commands to user buffer for debug
     //   for(uint8_t Idx=0; Idx<OptLen; Idx++)
     //   { Packet[DataLen++] = PktData[8+Idx]; }
     // }
     return DataLen; }

     void procRxOpt(const uint8_t *Opt, uint8_t Len)                 // process the options
     { }
     // { Format_String(CONS_UART_Write, "LoRaWAN Opt: ");
     //   for(uint8_t Idx=0; Idx<Len; Idx++)
     //     Format_Hex(CONS_UART_Write, Opt[Idx]);
     //   Format_String(CONS_UART_Write, "\n"); }

#ifdef WITH_ESP32
  esp_err_t WriteToNVS(const char *Name="LoRaWAN", const char *NameSpace="TRACKER")
  { nvs_handle Handle;
    esp_err_t Err = nvs_open(NameSpace, NVS_READWRITE, &Handle);
    if(Err!=ESP_OK) return Err;
    Err = nvs_set_blob(Handle, Name, this, sizeof(LoRaWANnode)-40);
    if(Err==ESP_OK) Err = nvs_commit(Handle);
    nvs_close(Handle);
    return Err; }

  esp_err_t ReadFromNVS(const char *Name="LoRaWAN", const char *NameSpace="TRACKER")
  { nvs_handle Handle;
    esp_err_t Err = nvs_open(NameSpace, NVS_READWRITE, &Handle);
    if(Err!=ESP_OK) return Err;
    size_t Size=0;
    Err = nvs_get_blob(Handle, Name,    0, &Size);                  // get the Size of the blob in the Flash
    if( (Err==ESP_OK) && (Size<=(sizeof(LoRaWANnode)-40)) )
      Err = nvs_get_blob(Handle, Name, this, &Size);                // read the Blob from the Flash
    nvs_close(Handle);
    return Err; }
#endif // WITH_ESP32

} ;

#endif // __LORAWAN_H__
