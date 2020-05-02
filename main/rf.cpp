#include "hal.h"
#include "rf.h"

#include "timesync.h"
#include "lowpass2.h"

// ===============================================================================================

// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
static const uint8_t OGN1_SYNC[8] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A };
// OGNv2 SYNC:       0xF56D3738 encoded in Machester
static const uint8_t OGN2_SYNC[8] = { 0x55, 0x99, 0x96, 0x59, 0xA5, 0x95, 0xA5, 0x6A };

#ifdef WITH_OGN1
static const uint8_t *OGN_SYNC = OGN1_SYNC;
#endif

#ifdef WITH_OGN2
static const uint8_t *OGN_SYNC = OGN2_SYNC;
#endif

       RFM_TRX           TRX;               // radio transceiver

     //  uint8_t   RX_AverRSSI;               // [-0.5dBm] average RSSI
     // int8_t       RF_Temp;               // [degC] temperature of the RF chip: uncalibrated

static uint32_t  RF_SlotTime;               // [sec] UTC time which belongs to the current time slot (0.3sec late by GPS UTC)
       FreqPlan  RF_FreqPlan;               // frequency hopping pattern calculator

       FIFO<RFM_RxPktData, 16> RF_RxFIFO;   // buffer for received packets
       FIFO<OGN_TxPacket<OGN_Packet>, 4> RF_TxFIFO;   // buffer for transmitted packets

#ifdef WITH_FANET
       FIFO<FANET_RxPacket, 8> FNT_RxFIFO;
       FIFO<FANET_Packet, 4> FNT_TxFIFO;
#endif

       uint16_t TX_Credit  =0;              // counts transmitted packets vs. time to avoid using more than 1% of the time

       uint8_t RX_OGN_Packets=0;            // [packets] counts received packets
static LowPass2<uint32_t, 4,2,4> RX_RSSI;   // low pass filter to average the RX noise

static Delay<uint8_t, 64> RX_OGN_CountDelay;
       uint16_t           RX_OGN_Count64=0; // counts received packets for the last 64 seconds

      uint32_t RX_Random=0x12345678;        // Random number from LSB of RSSI readouts

//       void XorShift32(uint32_t &Seed)      // simple random number generator
// { Seed ^= Seed << 13;
//   Seed ^= Seed >> 17;
//   Seed ^= Seed << 5; }

static uint8_t RX_Channel=0;                // (hopping) channel currently being received

static void SetTxChannel(uint8_t TxChan=RX_Channel)         // default channel to transmit is same as the receive channel
{
#ifdef WITH_RFM69
  TRX.WriteTxPower(Parameters.getTxPower(), Parameters.isTxTypeHW()); // set TX for transmission
#endif
#if defined(WITH_RFM95) || defined(WITH_SX1272)
  TRX.WriteTxPower(Parameters.getTxPower());                          // set TX for transmission
#endif
  TRX.setChannel(TxChan&0x7F);
  TRX.WriteSYNC(8, 7, OGN_SYNC); }                              // Full SYNC for TX

static void SetRxChannel(uint8_t RxChan=RX_Channel)
{ TRX.WriteTxPowerMin();                                        // setup for RX
  TRX.setChannel(RxChan&0x7F);
  TRX.WriteSYNC(7, 7, OGN_SYNC); }                              // Shorter SYNC for RX

static uint8_t ReceivePacket(void)                              // see if a packet has arrived
{ if(!TRX.DIO0_isOn()) return 0;                                // DIO0 line HIGH signals a new packet has arrived
#ifdef WITH_LED_RX
  LED_RX_Flash(20);
#endif
  uint8_t RxRSSI = TRX.ReadRSSI();                              // signal strength for the received packet
  RX_Random = (RX_Random<<1) | (RxRSSI&1);                      // use the lowest bit to add entropy

  RFM_RxPktData *RxPkt = RF_RxFIFO.getWrite();
  RxPkt->Time    = RF_SlotTime;                                 // store reception time
  RxPkt->msTime = TimeSync_msTime(); if(RxPkt->msTime<200) RxPkt->msTime+=1000;
  RxPkt->Channel = RX_Channel;                                  // store reception channel
  RxPkt->RSSI    = RxRSSI;                                      // store signal strength
  TRX.ReadPacketOGN(RxPkt->Data, RxPkt->Err);                      // get the packet data from the FIFO
  // PktData.Print();                                           // for debug

  RF_RxFIFO.Write();                                            // complete the write to the receiver FIFO
  // TRX.WriteMode(RFM69_OPMODE_RX);                            // back to receive (but we already have AutoRxRestart)
  return 1; }                                                   // return: 1 packet we have received

static uint32_t ReceiveUntil(TickType_t End)
{ uint32_t Count=0;
  for( ; ; )
  { Count+=ReceivePacket();
    int32_t Left = End-xTaskGetTickCount();
    if(Left<=0) break;
    vTaskDelay(1); }
  return Count; }

// static uint32_t ReceiveFor(TickType_t Ticks)                     // keep receiving packets for given period of time
// { return ReceiveUntil(xTaskGetTickCount()+Ticks); }

static uint8_t Transmit(uint8_t TxChan, const uint8_t *PacketByte, uint8_t Thresh, uint8_t MaxWait=7)
{
  if(PacketByte==0) return 0;                                   // if no packet to send: simply return

  if(MaxWait)
  { for( ; MaxWait; MaxWait--)                                  // wait for a given maximum time for a free radio channel
    {
#ifdef WITH_RFM69
      TRX.TriggerRSSI();
#endif
      vTaskDelay(1);
      uint8_t RxRSSI=TRX.ReadRSSI();
      RX_Random = (RX_Random<<1) | (RxRSSI&1);
      if(RxRSSI>=Thresh) break; }
    if(MaxWait==0) return 0; }

#ifdef WITH_LED_TX
  LED_TX_Flash(20);
#endif
  TRX.WriteMode(RF_OPMODE_STANDBY);                              // switch to standby
  // vTaskPrioritySet(0, tskIDLE_PRIORITY+2);
  vTaskDelay(1);
  SetTxChannel(TxChan);

  TRX.ClearIrqFlags();
  TRX.WritePacketOGN(PacketByte);                                   // write packet into FIFO
  TRX.WriteMode(RF_OPMODE_TRANSMITTER);                          // transmit
  vTaskDelay(5);                                                 // wait 5ms
  uint8_t Break=0;
  for(uint16_t Wait=400; Wait; Wait--)                           // wait for transmission to end
  { // if(!TRX.DIO0_isOn()) break;
    // uint8_t  Mode=TRX.ReadMode();
    uint16_t Flags=TRX.ReadIrqFlags();
    // if(Mode!=RF_OPMODE_TRANSMITTER) break;
    if(Flags&RF_IRQ_PacketSent) Break++;
    if(Break>=2) break; }
  TRX.WriteMode(RF_OPMODE_STANDBY);                              // switch to standy
  // vTaskPrioritySet(0, tskIDLE_PRIORITY+2);

  SetRxChannel();
  TRX.WriteMode(RF_OPMODE_RECEIVER);                             // back to receive mode
  return 1; }
                                                                           // make a time-slot: listen for packets and transmit given PacketByte$
static void TimeSlot(uint8_t TxChan, uint32_t SlotLen, const uint8_t *PacketByte, uint8_t Rx_RSSI, uint8_t MaxWait=8, uint32_t TxTime=0)
{ TickType_t Start = xTaskGetTickCount();                                  // when the slot started
  TickType_t End   = Start + SlotLen;                                      // when should it end
  uint32_t MaxTxTime = SlotLen-8-MaxWait;                                  // time limit when transmision could start
  if( (TxTime==0) || (TxTime>=MaxTxTime) ) TxTime = RX_Random%MaxTxTime;   // if TxTime out of limits, setup a random TxTime
  TickType_t Tx    = Start + TxTime;                                       // Tx = the moment to start transmission
  ReceiveUntil(Tx);                                                        // listen until this time comes
  if( (TX_Credit) && (PacketByte) )                                        // when packet to transmit is given and there is still TX credit left:
    TX_Credit-=Transmit(TxChan, PacketByte, Rx_RSSI, MaxWait);             // attempt to transmit the packet
  ReceiveUntil(End);                                                       // listen till the end of the time-slot
}

static void SetFreqPlan(void)                                // set the RF TRX according to the selected frequency hopping plan
{ TRX.setBaseFrequency(RF_FreqPlan.BaseFreq);                // set the base frequency (recalculate to RFM69 internal synth. units)
  TRX.setChannelSpacing(RF_FreqPlan.ChanSepar);              // set the channel separation
  TRX.setFrequencyCorrection(Parameters.RFchipFreqCorr);     // set the fine correction (to counter the Xtal error)
}

static uint8_t StartRFchip(void)
{ TRX.WriteMode(RF_OPMODE_STANDBY);
  vTaskDelay(1);
  TRX.RESET(1);                                              // RESET active
  vTaskDelay(1);                                             // wait 10ms
  TRX.RESET(0);                                              // RESET released
  vTaskDelay(5);                                             // wait 10ms
  SetFreqPlan();                                             // set TRX base frequency and channel separation after the frequency hopping plan
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  TRX.PrintReg(CONS_UART_Write);
  xSemaphoreGive(CONS_Mutex);
#endif
// #ifdef WITH_RFM95
//   TRX.WriteDefaultReg();
// #endif
  TRX.ConfigureOGN(0, OGN_SYNC);                                // setup RF chip parameters and set to channel #0
  TRX.WriteMode(RF_OPMODE_STANDBY);                          // set RF chip mode to STANDBY
  uint8_t Version = TRX.ReadVersion();
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  TRX.PrintReg(CONS_UART_Write);
  Format_String(CONS_UART_Write, "StartRFchip() v");
  Format_Hex(CONS_UART_Write, Version);
  CONS_UART_Write(' ');
  Format_UnsDec(CONS_UART_Write, RF_FreqPlan.BaseFreq, 4, 3);
  CONS_UART_Write('+');
  Format_UnsDec(CONS_UART_Write, (uint16_t)RF_FreqPlan.Channels, 2);
  CONS_UART_Write('x');
  Format_UnsDec(CONS_UART_Write, RF_FreqPlan.ChanSepar, 4, 3);
  Format_String(CONS_UART_Write, "kHz\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  return Version; }                                          // read the RF chip version and return it

extern "C"
 void vTaskRF(void* pvParameters)
{
  RF_RxFIFO.Clear();                      // clear receive/transmit packet FIFO's
  RF_TxFIFO.Clear();
#ifdef WITH_FANET
  FNT_RxFIFO.Clear();
  FNT_TxFIFO.Clear();
#endif

#ifdef USE_BLOCK_SPI
  TRX.TransferBlock = RFM_TransferBlock;
#else
  TRX.Select       = RFM_Select;
  TRX.Deselect     = RFM_Deselect;
  TRX.TransferByte = RFM_TransferByte;
#endif
  TRX.DIO0_isOn    = RFM_IRQ_isOn;
  TRX.RESET        = RFM_RESET;

  RF_FreqPlan.setPlan(Parameters.FreqPlan);     // 1 = Europe/Africa, 2 = USA/CA, 3 = Australia and South America

  vTaskDelay(5);

  for( ; ; )
  { uint8_t ChipVersion = StartRFchip();

    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "TaskRF: v");
    Format_Hex(CONS_UART_Write, ChipVersion);
    CONS_UART_Write('\r'); CONS_UART_Write('\n');
    xSemaphoreGive(CONS_Mutex);

    if( (ChipVersion!=0x00) && (ChipVersion!=0xFF) ) break;  // only break the endless loop then an RF chip is detected
    vTaskDelay(1000);
  }

  TX_Credit      = 0;    // count slots and packets transmitted: to keep the rule of 1% transmitter duty cycle
  RX_OGN_Packets = 0;    // count received packets per every second (two time slots)

  RX_OGN_Count64 = 0;
  RX_OGN_CountDelay.Clear();

  RX_Channel = RF_FreqPlan.getChannel(TimeSync_Time(), 0, 1);                  // set initial RX channel
  SetRxChannel();
  TRX.WriteMode(RF_OPMODE_RECEIVER);

  RX_RSSI.Set(2*112);

  for( ; ; )
  {

// #ifdef DEBUG_PRINT
    // RF_Print();
// #endif

    uint32_t RxRssiSum=0; uint16_t RxRssiCount=0;                              // measure the average RSSI for lower frequency
    do
    { ReceivePacket();                                                         // keep checking for received packets
#ifdef WITH_RFM69
      TRX.TriggerRSSI();
#endif
      vTaskDelay(1);
      uint8_t RxRSSI=TRX.ReadRSSI();                                           // measure the channel noise level
      RX_Random = (RX_Random<<1) | (RxRSSI&1);
      RxRssiSum+=RxRSSI; RxRssiCount++;
    } while(TimeSync_msTime()<270);                                                  // until 300ms from the PPS
    RX_RSSI.Process(RxRssiSum/RxRssiCount);                                    // [-0.5dBm] average noise on channel

    TRX.WriteMode(RF_OPMODE_STANDBY);                                         // switch to standy
    vTaskDelay(1);

    if(PowerMode==0)
    { TRX.WriteMode(RF_OPMODE_SLEEP);
      while(PowerMode==0)
        vTaskDelay(1);
      TRX.WriteMode(RF_OPMODE_STANDBY);
      vTaskDelay(1); }

    SetFreqPlan();

    TRX.averRSSI=RX_RSSI.getOutput();

    RX_OGN_Count64 += RX_OGN_Packets - RX_OGN_CountDelay.Input(RX_OGN_Packets); // add OGN packets received, subtract packets received 64 seconds ago

    RX_OGN_Packets=0;                                                           // clear the received packet count

    StartRFchip();                                                             // reset and rewrite the RF chip config

#ifdef WITH_RFM69
    TRX.TriggerTemp();                                                         // trigger RF chip temperature readout
    vTaskDelay(1); // while(TRX.RunningTemp()) taskYIELD();                    // wait for conversion to be ready
    TRX.ReadTemp();                                                            // [degC] read RF chip temperature
#endif
#ifdef WITH_RFM95
    TRX.ReadTemp();                                                            // [degC] read RF chip temperature
#endif
    TRX.chipTemp+=Parameters.RFchipTempCorr;
                                                                               // Note: on RFM95 temperature sens does not work in STANDBY
    RF_SlotTime = TimeSync_Time();
    uint8_t TxChan = RF_FreqPlan.getChannel(RF_SlotTime, 0, 1);                // tranmsit channel
    RX_Channel = TxChan;
    SetRxChannel();
                                                                               // here we can read the chip temperature
    TRX.WriteMode(RF_OPMODE_RECEIVER);                                         // switch to receive mode
    vTaskDelay(1);

    RxRssiSum=0; RxRssiCount=0;                                                // measure the average RSSI for the upper frequency
    do
    { ReceivePacket();                                                         // check for packets being received ?
#ifdef WITH_RFM69
      TRX.TriggerRSSI();                                                       // start RSSI measurement
#endif
      vTaskDelay(1);
      uint8_t RxRSSI=TRX.ReadRSSI();                                           // read RSSI
      RX_Random = (RX_Random<<1) | (RxRSSI&1);                                 // take lower bit for random number generator
      RxRssiSum+=RxRSSI; RxRssiCount++;
    } while(TimeSync_msTime()<350);                                            // keep going until 400 ms after PPS
    RX_RSSI.Process(RxRssiSum/RxRssiCount);                                    // [-0.5dBm] average noise on channel

    TX_Credit+=2; if(TX_Credit>7200) TX_Credit=7200;                           // count the transmission credit

    XorShift32(RX_Random);
    uint32_t TxTime = (RX_Random&0x3F)+1; TxTime*=6; TxTime+=50;               // random transmission time: (1..64)*6+50 [ms]

    const uint8_t *TxPktData0=0;
    const uint8_t *TxPktData1=0;
    const OGN_TxPacket<OGN_Packet> *TxPkt0 = RF_TxFIFO.getRead(0);             // get 1st packet from TxFIFO
    const OGN_TxPacket<OGN_Packet> *TxPkt1 = RF_TxFIFO.getRead(1);             // get 2nd packet from TxFIFO
    if(TxPkt0) TxPktData0=TxPkt0->Byte();                                      // if 1st is not NULL then get its data
    if(TxPkt1) TxPktData1=TxPkt1->Byte();                                      // if 2nd if not NULL then get its data
          else TxPktData1=TxPktData0;                                          // but if NULL then take copy of the 1st packet

    if(TxPkt0)                                                                 // if 1st packet is not NULL
    { if( (RX_Channel!=TxChan) && (TxPkt0->Packet.Header.Relay==0) )
      { const uint8_t *Tmp=TxPktData0; TxPktData0=TxPktData1; TxPktData1=Tmp; } // swap 1st and 2nd packet data
    }
    TimeSlot(TxChan, 800-TimeSync_msTime(), TxPktData0, TRX.averRSSI, 0, TxTime); // run a Time-Slot till 0.800sec

    TRX.WriteMode(RF_OPMODE_STANDBY);                                          // switch to receive mode
    TxChan = RF_FreqPlan.getChannel(RF_SlotTime, 1, 1);                        // transmit channel
    RX_Channel = TxChan;

#if defined(WITH_FANET) && defined(WITH_RFM95)
    const FANET_Packet *FNTpkt = FNT_TxFIFO.getRead(0);                        // read the packet from the FANET transmitt queue
    if(FNTpkt)                                                                 // was there any ?
    { TRX.SetLoRa();                                                           // switch TRX to LoRa
      TRX.ConfigureFNT();                                                      // configure for FANET
      TRX.WriteMode(RF_OPMODE_LORA_RX_CONT);
      vTaskDelay(2);
      for(uint8_t Wait=50; Wait; Wait--)
      { vTaskDelay(1);
        uint8_t Stat = TRX.ReadByte(REG_LORA_MODEM_STATUS);
        if((Stat&0x0B)==0) break; }
      TRX.WriteMode(RF_OPMODE_LORA_STANDBY);
      TRX.SendPacketFNT(FNTpkt->Byte, FNTpkt->Len);
      FNT_TxFIFO.Read();                                                        // remove the last packet from the queue
/*
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "TRX: ");
      Format_Hex(CONS_UART_Write, TRX.ReadMode());
      CONS_UART_Write(':');
      Format_Hex(CONS_UART_Write, TRX.ReadByte(REG_LORA_IRQ_FLAGS));
      CONS_UART_Write(' ');
      Format_Hex(CONS_UART_Write, TRX.ReadByte(REG_LORA_PREAMBLE_LSB));
      CONS_UART_Write(':');
      Format_Hex(CONS_UART_Write, TRX.ReadByte(REG_LORA_SYNC));
      CONS_UART_Write(' ');
      Format_Hex(CONS_UART_Write, TRX.ReadByte(REG_LORA_MODEM_CONFIG1));
      CONS_UART_Write(':');
      Format_Hex(CONS_UART_Write, TRX.ReadByte(REG_LORA_MODEM_CONFIG2));
      Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex);
*/
      vTaskDelay(8);
      for( uint8_t Wait=50; Wait; Wait--)
      { vTaskDelay(1);
        uint8_t Mode=TRX.ReadMode();
        if(Mode!=RF_OPMODE_LORA_TX) break; }
      TRX.SetFSK();
      TRX.ConfigureOGN(0, OGN_SYNC);
    }
#endif

    SetRxChannel();
    TRX.WriteMode(RF_OPMODE_RECEIVER);                                         // switch to receive mode

    XorShift32(RX_Random);
    TxTime = (RX_Random&0x3F)+1; TxTime*=6;

    TimeSlot(TxChan, 1250-TimeSync_msTime(), TxPktData1, TRX.averRSSI, 0, TxTime);

    if(TxPkt0) RF_TxFIFO.Read();
    if(TxPkt1) RF_TxFIFO.Read();

  }

}

// ======================================================================================
