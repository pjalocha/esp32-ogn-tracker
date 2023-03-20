#include "hal.h"
#include "rf.h"

#include "timesync.h"
#include "lowpass2.h"

#ifdef WITH_LORAWAN
#include "lorawan.h"
#endif

// ===============================================================================================

// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
static const uint8_t OGN1_SYNC[10] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A, 0x00, 0x00 };
// OGNv2 SYNC:       0xF56D3738 encoded in Machester
static const uint8_t OGN2_SYNC[10] = { 0x55, 0x99, 0x96, 0x59, 0xA5, 0x95, 0xA5, 0x6A, 0x00, 0x00 };

// ADS-L SYNC:       0xF5724B18 encoded in Manchester (fixed packet length 0x18 is included)
static const uint8_t ADSL_SYNC[10] = { 0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65, 0xA9, 0x6A, 0x00, 0x00 };

// PilotAWare SYNC including the packet size and CRC seed
static const uint8_t PAW_SYNC [10] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x18, 0x71, 0x00, 0x00 };

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

       FIFO<RFM_FSK_RxPktData, 16> RF_RxFIFO;         // buffer for received packets
       FIFO<OGN_TxPacket<OGN_Packet>, 4> RF_TxFIFO;   // buffer for transmitted packets

#ifdef WITH_ADSL
       FIFO<ADSL_Packet, 4> ADSL_TxFIFO;
#endif

#ifdef WITH_FANET
       FIFO<FANET_RxPacket, 8> FNT_RxFIFO;
       FIFO<FANET_Packet, 4> FNT_TxFIFO;
#endif

       int32_t TX_Credit  = 0;              // [ms] counts transmitter time avoid using more than 1% or air time

       uint8_t RX_OGN_Packets=0;            // [packets] counts received packets
static LowPass2<uint32_t, 4,2,4> RX_RSSI;   // low pass filter to average the RX noise

static Delay<uint8_t, 64> RX_OGN_CountDelay;
       uint16_t           RX_OGN_Count64=0; // counts received packets for the last 64 seconds

      uint32_t RX_Random=0x12345678;        // Random number from LSB of RSSI readouts

static uint8_t RX_Channel=0;                // (hopping) channel currently being received

static void SetTxChannel(uint8_t TxChan=RX_Channel, const uint8_t *SYNC=OGN_SYNC)         // default channel to transmit is same as the receive channel
{
#ifdef WITH_SX1262
  TRX.OGN_Configure(TxChan&0x7F, SYNC);
  TRX.WaitWhileBusy_ms(2);
  TRX.WriteTxPower(Parameters.TxPower);
  TRX.WaitWhileBusy_ms(2);
#else // WITH_SX1262
#ifdef WITH_RFM69
  TRX.WriteTxPower(Parameters.TxPower, Parameters.RFchipTypeHW); // set TX for transmission
#endif
#if defined(WITH_RFM95) || defined(WITH_SX1272)
  TRX.WriteTxPower(Parameters.TxPower);                         // set TX for transmission
#endif
  TRX.setChannel(TxChan&0x7F);
  TRX.FSK_WriteSYNC(8, 7, SYNC);                                // Full SYNC for TX
#endif // WITH_SX1262
}

static void SetRxChannel(uint8_t RxChan=RX_Channel, const uint8_t *SYNC=OGN_SYNC)
{
#ifdef WITH_SX1262
  // TRX.OGN_Configure(RxChan&0x7F, SYNC);
  TRX.setChannel(RxChan&0x7F);
  TRX.FSK_WriteSYNC(7, 7, SYNC);                                // Shorter SYNC for RX
#else
  TRX.WriteTxPowerMin();                                        // setup for RX
  TRX.setChannel(RxChan&0x7F);
  TRX.FSK_WriteSYNC(7, 7, SYNC);                                // Shorter SYNC for RX
#endif
}

static uint8_t ReceivePacket(void)                              // see if a packet has arrived
{
#ifdef DEBUG_PRINT
  uint16_t Mask=TRX.ReadIrqFlags();
  // static uint16_t BackOff=0;
  if(Mask /* || BackOff==0 */ )
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "IRQ:0x");
    Format_Hex(CONS_UART_Write, Mask);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
    TRX.ClearIrqFlags(0xFFFF); /* BackOff=100; */ }
  // else BackOff--;
#endif
  if(!TRX.DIO0_isOn()) return 0;                                // DIO0 line HIGH signals a new packet has arrived
#ifdef WITH_LED_RX
  LED_RX_Flash(20);
#endif
  uint8_t RxRSSI = TRX.ReadRSSI();                              // signal strength for the received packet
  RX_Random = (RX_Random<<1) | (RxRSSI&1);                      // use the lowest bit to add entropy

  RFM_FSK_RxPktData *RxPkt = RF_RxFIFO.getWrite();
  RxPkt->Time    = RF_SlotTime;                                 // store reception time
  RxPkt->msTime = TimeSync_msTime(); if(RxPkt->msTime<200) RxPkt->msTime+=1000;
  RxPkt->Channel = RX_Channel;                                  // store reception channel
  RxPkt->RSSI    = RxRSSI;                                      // store signal strength
  TRX.OGN_ReadPacket(RxPkt->Data, RxPkt->Err);                  // get the packet data from the FIFO
  // RxPkt->Print(CONS_UART_Write);                                // for debug

  RF_RxFIFO.Write();                                            // complete the write to the receiver FIFO
  TRX.setModeRX();                                              // back to receive (but we already have AutoRxRestart)
  TRX.ClearIrqFlags();
  return 1; }                                                   // return: 1 packet we have received

static uint32_t ReceiveUntil(TickType_t End)
{ uint32_t Count=0;
  for( ; ; )
  { Count+=ReceivePacket();
    int32_t Left = End-xTaskGetTickCount();
    if(Left<=0) break;
    vTaskDelay(1); }
#ifdef WITH_SX1262
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "ReceiveUntil() => 0x");
  Format_Hex(CONS_UART_Write, TRX.ReadIrqFlags());
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
#endif
  return Count; }

// static uint32_t ReceiveFor(TickType_t Ticks)                     // keep receiving packets for given period of time
// { return ReceiveUntil(xTaskGetTickCount()+Ticks); }

#ifdef WITH_ADSL
static uint8_t Transmit(uint8_t TxChan, const ADSL_Packet *Packet, uint8_t Thresh, uint8_t MaxWait=7)
{
  if(Packet==0) return 0;                                   // if no packet to send: simply return

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
  TRX.setModeStandby();                                        // switch to standby
  vTaskDelay(1);
  TRX.FSK_Configure(TxChan&0x7F, ADSL_SYNC, ADSL_Packet::TxBytes-3);
  SetTxChannel(TxChan, ADSL_SYNC);

#ifdef WITH_SX1262
#else // not WITH_SX1262
  TRX.ClearIrqFlags();
  TRX.FSK_WritePacket(&(Packet->Version), ADSL_Packet::TxBytes-3);       // write packet into FIFO (apply Manchester conversion)
  TRX.setModeTX();                                               // transmit
  vTaskDelay(5);                                                 // wait 5ms (about the OGN packet time)
  uint8_t Break=0;
  for(uint16_t Wait=400; Wait; Wait--)                           // wait for transmission to end
  { uint16_t Flags=TRX.ReadIrqFlags();
    if(Flags&RF_IRQ_PacketSent) Break++;
    if(Break>=2) break; }
  TRX.setModeStandby();                                       // switch to standy
  // vTaskPrioritySet(0, tskIDLE_PRIORITY+2);
#endif

  TRX.FSK_Configure(RX_Channel&0x7F, OGN_SYNC, OGN_TxPacket<OGN_Packet>::Bytes);
  SetRxChannel();
  TRX.setModeRX();                                            // back to receive mode
  TRX.ClearIrqFlags();

  return 1; }
#endif

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
  TRX.setModeStandby();                                        // switch to standby
  // vTaskPrioritySet(0, tskIDLE_PRIORITY+2);
  vTaskDelay(1);
  SetTxChannel(TxChan);

#ifdef WITH_SX1262
  TRX.OGN_WritePacket(PacketByte);
  uint16_t PreFlags=TRX.ReadIrqFlags();
  TRX.ClearIrqFlags();
  TRX.WaitWhileBusy_ms(10);
  TickType_t TxDur = xTaskGetTickCount();
  TRX.setModeTX(0);
  TRX.WaitWhileBusy_ms(2);
  for( uint16_t Wait=7; Wait; Wait--)
  { vTaskDelay(1);
    uint16_t Flags=TRX.ReadIrqFlags();
    if(Flags&IRQ_TXDONE) break; }
  TxDur = xTaskGetTickCount()-TxDur;
  TRX.setModeStandby();
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "0x");
  Format_Hex(CONS_UART_Write, PreFlags);
  Format_String(CONS_UART_Write, " => Transmit() => 0x");
  Format_Hex(CONS_UART_Write, TRX.getStatus());
  Format_String(CONS_UART_Write, " ");
  Format_UnsDec(CONS_UART_Write, TxDur);
  Format_String(CONS_UART_Write, "ms\n");
  xSemaphoreGive(CONS_Mutex);
#endif
#else // not WITH_SX1262
  TRX.ClearIrqFlags();
  TRX.FSK_WritePacket(PacketByte, OGN_TxPacket<OGN_Packet>::Bytes); // write packet into FIFO (apply Manchester conversion)
  TRX.setModeTX();                                               // transmit
  vTaskDelay(5);                                                 // wait 5ms (about the OGN packet time)
  uint8_t Break=0;
  for(uint16_t Wait=400; Wait; Wait--)                           // wait for transmission to end
  { uint16_t Flags=TRX.ReadIrqFlags();
    if(Flags&RF_IRQ_PacketSent) Break++;
    if(Break>=2) break; }
  TRX.setModeStandby();                                       // switch to standy
  // vTaskPrioritySet(0, tskIDLE_PRIORITY+2);
#endif

  SetRxChannel();
  TRX.setModeRX();                                            // back to receive mode
  TRX.ClearIrqFlags();
#ifdef WITH_SX1262
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "setModeRX() => 0x");
  Format_Hex(CONS_UART_Write, TRX.getStatus());
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
#endif
  return 1; }
                                                                           // make a time-slot: listen for packets and transmit given PacketByte$
static void TimeSlot(uint8_t TxChan, uint32_t SlotLen, const uint8_t *PacketByte, uint8_t Rx_RSSI, uint8_t MaxWait=8, uint32_t TxTime=0)
{ TickType_t Start = xTaskGetTickCount();                                  // when the slot started
  TickType_t End   = Start + SlotLen;                                      // when should it end
  uint32_t MaxTxTime = SlotLen-8-MaxWait;                                  // time limit when transmision could start
  if( (TxTime==0) || (TxTime>=MaxTxTime) ) TxTime = RX_Random%MaxTxTime;   // if TxTime out of limits, setup a random TxTime
  TickType_t Tx    = Start + TxTime;                                       // Tx = the moment to start transmission
  ReceiveUntil(Tx);                                                        // listen until this time comes
  if( (TX_Credit>0) && Parameters.TxPower!=(-32) && PacketByte )           // when packet to transmit is given and there is still TX credit left:
    if(Transmit(TxChan, PacketByte, Rx_RSSI, MaxWait)) TX_Credit-=5;       // attempt to transmit the packet
  ReceiveUntil(End); }                                                     // listen till the end of the time-slot

#ifdef WITH_ADSL
static void TimeSlot(uint8_t TxChan, uint32_t SlotLen, const ADSL_Packet *Packet, uint8_t Rx_RSSI, uint8_t MaxWait=8, uint32_t TxTime=0)
{ TickType_t Start = xTaskGetTickCount();                                  // when the slot started
  TickType_t End   = Start + SlotLen;                                      // when should it end
  uint32_t MaxTxTime = SlotLen-8-MaxWait;                                  // time limit when transmision could start
  if( (TxTime==0) || (TxTime>=MaxTxTime) ) TxTime = RX_Random%MaxTxTime;   // if TxTime out of limits, setup a random TxTime
  TickType_t Tx    = Start + TxTime;                                       // Tx = the moment to start transmission
  ReceiveUntil(Tx);                                                        // listen until this time comes
  if( (TX_Credit>0) && Parameters.TxPower!=(-32) && Packet )              // when packet to transmit is given and there is still TX credit left:
    if(Transmit(TxChan, Packet, Rx_RSSI, MaxWait)) TX_Credit-=5;           // attempt to transmit the packet
  ReceiveUntil(End); }                                                     // listen till the end of the time-slot
#endif

static void SetFreqPlanOGN(void)                             // set the RF TRX according to the selected frequency hopping plan
{ TRX.setBaseFrequency(RF_FreqPlan.BaseFreq);                // set the base frequency (recalculate to RFM69 internal synth. units)
  TRX.setChannelSpacing(RF_FreqPlan.ChanSepar);              // set the channel separation
  TRX.setFrequencyCorrection(Parameters.RFchipFreqCorr);     // set the fine correction (to counter the Xtal error)
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "SetFreqPlanOGN() ");
  Format_UnsDec(CONS_UART_Write, TRX.BaseFrequency);
  Format_String(CONS_UART_Write, " + [");
  Format_UnsDec(CONS_UART_Write, TRX.ChannelSpacing);
  Format_String(CONS_UART_Write, "] [32MHz/2^27]\n");
  xSemaphoreGive(CONS_Mutex);
#endif
}

static void SetFreqPlanWAN(void)                             // set the LoRaWAN EU frequency plan: 8 LoRa channels
{ TRX.setBaseFrequency(867100000);
  TRX.setChannelSpacing(   200000);
  TRX.setFrequencyCorrection(Parameters.RFchipFreqCorr); }

/*
#ifdef WITH_SX1262
static int WaitWhileBusy(uint32_t MaxTime)
{ if(!TRX.readBusy()) return 0;
  uint32_t Count=0;
  for( ; Count<MaxTime; )
  { Count++; if(!TRX.readBusy()) break; }
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "WaitWhileBusy(");
  Format_UnsDec(CONS_UART_Write, MaxTime);
  Format_String(CONS_UART_Write, ") => ");
  Format_UnsDec(CONS_UART_Write, Count);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  return Count; }
#endif
*/
                                                             // some LoRaWAN variables
static uint8_t StartRFchip(void)
{ TRX.setModeStandby();
  vTaskDelay(1);
  TRX.RESET(1);                                              // RESET active: LOW for RFM95 and SX1262
  vTaskDelay(1);                                             // 100us for SX1262, p.50
  TRX.RESET(0);                                              // RESET released
#ifdef WITH_SX1262
  TRX.WaitWhileBusy_ms(20);
  // if(TRX.readBusy()) Format_String(CONS_UART_Write, "StartRFchip() sx1262 BUSY after RESET(0)\n");
  //               else Format_String(CONS_UART_Write, "StartRFchip() sx1262 NOT busy after reset\n");
  TRX.setRegulator(1);
  TRX.WaitWhileBusy_ms(10);
  TRX.setRFswitchDIO2(1);       // enable DIO0 as RF switch
  TRX.WaitWhileBusy_ms(10);
  //if(TRX.readBusy()) Format_String(CONS_UART_Write, "StartRFchip() sx1262 BUSY after setRFswitchDIO2()\n");
  // WaitWhileBusy(100);
  TRX.setTCXOctrlDIO3(2);  // TCXO control: 2=1.8V
  TRX.WaitWhileBusy_ms(10);
  TRX.Calibrate(0x7F);          // 
  TRX.WaitWhileBusy_ms(10);
  //if(TRX.readBusy()) Format_String(CONS_UART_Write, "StartRFchip() sx1262 BUSY after setTCXOctrlDIO3()\n");
  // WaitWhileBusy(100);
  TRX.setModeStandby(0);
  TRX.WaitWhileBusy_ms(10);
  // if(TRX.readBusy()) Format_String(CONS_UART_Write, "StartRFchip() sx1262 BUSY after setModeStandby()\n");
  // TRX.WaitWhileBusy(100);
  // if(TRX.readBusy()) { Format_String(CONS_UART_Write, "StartRFchip() sx1262 still busy !\n"); }
  TRX.setFSK();
  TRX.WaitWhileBusy_ms(10);
#ifdef DEBUG_PRINT
  { uint8_t *SYNC = TRX.Regs_Read(REG_SYNCWORD0, 8);
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "StartRFchip() => 0x");
    Format_Hex(CONS_UART_Write, TRX.getStatus());
    Format_String(CONS_UART_Write, " => 0x");
    for(int Idx=0; Idx<8; Idx++)
    { Format_Hex(CONS_UART_Write, SYNC[Idx]); }             // SYNC after chip reset can serve as a detection method for sx1262
    Format_String(CONS_UART_Write, "\n");                   // it should be: 0x9723522556536564
    xSemaphoreGive(CONS_Mutex); }
#endif
  TRX.StopOnPreamble(0);
#else // WITH_SX1262                                        // not WITH_SX1262
  vTaskDelay(5);                                            // wait 10ms
#endif
  SetFreqPlanOGN();                                         // set TRX base frequency and channel separation after the frequency hopping plan
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  TRX.PrintReg(CONS_UART_Write);
  xSemaphoreGive(CONS_Mutex);
#endif
// #ifdef WITH_RFM95
//   TRX.WriteDefaultReg();
// #endif
  TRX.FSK_Configure(0, OGN_SYNC, OGN_TxPacket<OGN_Packet>::Bytes); // setup RF chip parameters and set to channel #0
#ifdef WITH_SX1262
  TRX.WaitWhileBusy_ms(5);
#ifdef DEBUG_PRINT
  { uint8_t *SYNC = TRX.Regs_Read(REG_SYNCWORD0, 8);
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "FSK_Configure() => 0x");
    Format_Hex(CONS_UART_Write, TRX.getStatus());
    Format_String(CONS_UART_Write, " => 0x");
    for(int Idx=0; Idx<8; Idx++)
    { Format_Hex(CONS_UART_Write, SYNC[Idx]); }
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex); }
#endif
  TRX.CalibrateImage();
  TRX.WaitWhileBusy_ms(20);
  if(TRX.readBusy()) Format_String(CONS_UART_Write, "StartRFchip() sx1262 BUSY after OGN_Configure() and Calibrate()\n");
#endif
  TRX.setModeStandby();                                        // set RF chip mode to STANDBY
  uint8_t Version = TRX.ReadVersion();
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
#ifdef WITH_RFM95
  TRX.PrintReg(CONS_UART_Write);
#endif
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

#ifdef WITH_LORAWAN
static uint8_t WAN_BackOff = 60;                             // back-off timer
static TickType_t WAN_RespTick = 0;                          // when to expect the WAN response
static RFM_LoRa_RxPacket WAN_RxPacket;                       // packet received from WAN
// static WAN_Setup()
// static WAN_Back()
#endif

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
  TRX.Select       = RFM_Select;                // [call]
  TRX.Deselect     = RFM_Deselect;              // [call]
  TRX.TransferByte = RFM_TransferByte;          // [call]
#endif
  TRX.DIO0_isOn    = RFM_IRQ_isOn;              // [call] read IRQ (it is DIO1 for sx1262)
#ifdef WITH_SX1262
  TRX.Busy_isOn    = RFM_Busy_isOn;             // [call] read Busy
#endif
  TRX.Delay_ms     = RFM_Delay;                 // [call] delay by N miliseconds
  TRX.RESET        = RFM_RESET;                 // [call] chip reset control

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
  TRX.setModeRX();
  TRX.ClearIrqFlags();

  RX_RSSI.Set(2*112);

  for( ; ; )
  {

// #ifdef DEBUG_PRINT
    // RF_Print();
// #endif

    // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    // Format_UnsDec(CONS_UART_Write, xTaskGetTickCount(), 4, 3);
    // Format_String(CONS_UART_Write, " => RF-slot\n");
    // xSemaphoreGive(CONS_Mutex);

#ifdef WITH_LORAWAN
    bool WANrx=0;
    int WAN_RespLeft = WAN_RespTick-xTaskGetTickCount();        // [tick] how much time left before expected response
    if(WANdev.State==1 || WANdev.State==3)                      // if State indicates we are waiting for the response
    { if(WAN_RespLeft<=5) WANdev.State--;                       // if time below 5 ticks we have not enough time
      else if(WAN_RespLeft<200) { WANrx=1; }                    // if more than 200ms then we can't wait this long now
      if(WANrx==0 && WAN_RespLeft<=5)
      { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_UnsDec(CONS_UART_Write, xTaskGetTickCount(), 4, 3);
        Format_String(CONS_UART_Write, "s LoRaWAN missed Rx: ");
        Format_SignDec(CONS_UART_Write, WAN_RespLeft);
        Format_String(CONS_UART_Write, "ms\n");
        xSemaphoreGive(CONS_Mutex); }
    }

    if(WANrx)                                              // if reception expected from WAN
    { int RxLen=0;
      TRX.setModeStandby();                                // TRX to standby
      TRX.setLoRa();                                       // switch to LoRa mode (through sleep)
      TRX.setModeLoRaStandby();                            // TRX in LoRa (not FSK) standby
      SetFreqPlanWAN();                                    // WAN frequency plan
      TRX.WAN_Configure();                                 // LoRa for WAN config.
      TRX.setChannel(WANdev.Chan);                         // set the channel
      TRX.LoRa_InvertIQ(1); TRX.LoRa_setCRC(0); TRX.LoRa_setIRQ(0); // setup for WAN RX
      TRX.setModeLoRaRXsingle();                           // wait for a single packet
      int Wait=WAN_RespLeft+100;                           // 100ms timeout after the expected reception
      for( ; Wait>0; Wait--)
      { vTaskDelay(1);
        if(TRX.readIRQ()) break; }                         // IRQ signals packet reception
      if(Wait)                                             // if no timeout thus a packet has been received.
      { TRX.LoRa_ReceivePacket(WAN_RxPacket);
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_UnsDec(CONS_UART_Write, xTaskGetTickCount(), 4, 3);
        Format_String(CONS_UART_Write, "s LoRaWAN Rx[");
        CONS_UART_Write('0'+WANdev.State);
        Format_String(CONS_UART_Write, "]: ");
        Format_UnsDec(CONS_UART_Write, (uint16_t)WAN_RxPacket.Len);
        Format_String(CONS_UART_Write, "B/");
        Format_UnsDec(CONS_UART_Write, (unsigned)Wait);    // amount of timeout left
        Format_String(CONS_UART_Write, "ms\n");
        xSemaphoreGive(CONS_Mutex);
             if(WANdev.State==1) WANdev.procJoinAccept(WAN_RxPacket);    // if join-request state then expect a join-accept packet
        else if(WANdev.State==3) RxLen=WANdev.procRxData(WAN_RxPacket);  // if data send then respect ACK and/or downlink data packet
        WANdev.RxSilent=0;
      }
      else                                                 // if no packet received then retreat the State
      { WANdev.State--;
        WANdev.RxSilent++; if(WANdev.RxSilent>30) WANdev.Disconnect(); } // count silence when reception expected, if too many then disconnect
      TRX.setFSK();                                        // back to FSK
      SetFreqPlanOGN();                                    // OGN frequency plan
      TRX.FSK_Configure(0, OGN_SYNC, OGN_TxPacket<OGN_Packet>::Bytes); // OGN config
      SetRxChannel();
      TRX.setModeRX();                                     // switch to receive mode
      TRX.ClearIrqFlags();
      WANdev.WriteToNVS();                                 // store new WAN state in flash
      if(RxLen>0)                                          // if Downlink data received
      { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, "LoRaWAN Msg: ");
        // Format_UnsDec(CONS_UART_Write, (uint16_t)RxLen);
        // Format_String(CONS_UART_Write, "B");
        for(int Idx=0; Idx<RxLen; Idx++)
        { Format_Hex(CONS_UART_Write, WANdev.Packet[Idx]); } //
        Format_String(CONS_UART_Write, "\n");
        xSemaphoreGive(CONS_Mutex); }
    }
    else                                                     // if no WAN reception expected or possible
#endif // WITH_LORAWAN
    // if(TimeSync_msTime()<260);
    { uint32_t RxRssiSum=0; uint16_t RxRssiCount=0;          // measure the average RSSI for lower frequency
      do
      { ReceivePacket();                                     // keep checking for received packets
#ifdef WITH_RFM69
        TRX.TriggerRSSI();
#endif
        vTaskDelay(1);
        uint8_t RxRSSI=TRX.ReadRSSI();                       // measure the channel noise level
        RX_Random = (RX_Random<<1) | (RxRSSI&1);
        RxRssiSum+=RxRSSI; RxRssiCount++;
      } while(TimeSync_msTime()<270);                        // until 300ms from the PPS
      RX_RSSI.Process(RxRssiSum/RxRssiCount);                // [-0.5dBm] average noise on channel
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "RSSI: ");
      Format_UnsDec(CONS_UART_Write, RxRssiSum);
      Format_String(CONS_UART_Write, "/");
      Format_UnsDec(CONS_UART_Write, RxRssiCount);
      Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex);
#endif
    }
// #endif // WITH_LORAWAN

    TRX.setModeStandby();                                                      // switch to standy
    vTaskDelay(1);

    if(PowerMode==0)
    { TRX.setModeSleep();
      while(PowerMode==0)
        vTaskDelay(1);
      TRX.setModeStandby();
      vTaskDelay(1); }

    SetFreqPlanOGN();

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
    TRX.setModeRX();                                                           // switch to receive mode
    TRX.ClearIrqFlags();                                                       // here we can read the chip temperature
    vTaskDelay(1);

    uint32_t RxRssiSum=0; uint16_t RxRssiCount=0;                              // measure the average RSSI for the upper frequency
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

    TX_Credit+=10; if(TX_Credit>3600000) TX_Credit=600000;                     // [ms] count the transmission credit

    XorShift32(RX_Random);
    uint32_t TxTime = (RX_Random&0x3F)+1; TxTime*=6; TxTime+=50;               // random transmission time: (1..64)*6+50 [ms]
#ifdef WITH_ADSL
    bool ADSL_Slot = RX_Random&0x40;
    ADSL_Packet *ADSL_TxPkt = ADSL_TxFIFO.getRead();
    // if(ADSL_TxPkt)
    // { Format_String(CONS_UART_Write, "ADSL:");
    //   CONS_UART_Write('0'+ADSL_Slot);
    //   Format_String(CONS_UART_Write, "\n"); }
#endif
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
#ifdef WITH_ADSL
    if(ADSL_Slot==0 && ADSL_TxPkt)
      TimeSlot(TxChan, 800-TimeSync_msTime(), ADSL_TxPkt, TRX.averRSSI, 0, TxTime);
    else
#endif
      TimeSlot(TxChan, 800-TimeSync_msTime(), TxPktData0, TRX.averRSSI, 0, TxTime); // run a Time-Slot till 0.800sec

    TRX.setModeStandby();
    TxChan = RF_FreqPlan.getChannel(RF_SlotTime, 1, 1);                        // transmit channel
    RX_Channel = TxChan;

#if defined(WITH_FANET) && defined(WITH_RFM95)
    const FANET_Packet *FNTpkt = FNT_TxFIFO.getRead(0);                        // read the packet from the FANET transmitt queue
    if(FNTpkt)                                                                 // was there any ?
    { TRX.setLoRa();                                                           // switch TRX to LoRa
      TRX.FNT_Configure();                                                     // configure for FANET
      // TRX.setChannel(0);                                                      // configure for FANET
      TRX.WriteTxPower(Parameters.TxPower);                                    // transmission power
      TRX.setModeLoRaRXcont();                                                 // continous receiver mode
      vTaskDelay(2);
      for(uint8_t Wait=50; Wait; Wait--)                                       //
      { vTaskDelay(1);                                                         // every milisecond
        uint8_t Stat = TRX.ReadByte(REG_LORA_MODEM_STATUS);                    // receiver status
        if((Stat&0x0B)==0) break; }                                            // 0:signal-det, 1:signal-sync, 3:header-valid
      TRX.setModeLoRaStandby();
      TRX.FNT_SendPacket(FNTpkt->Byte, FNTpkt->Len);                           // transmit the FANET packet
      FNT_TxFIFO.Read();                                                       // remove the last packet from the queue
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
      { vTaskDelay(1); if(!TRX.isModeLoRaTX()) break; }
        // uint8_t Mode=TRX.ReadMode();
        // if(Mode!=RF_OPMODE_LORA_TX) break; }
      TRX.setFSK();
      TRX.FSK_Configure(0, OGN_SYNC, OGN_TxPacket<OGN_Packet>::Bytes);
    }
#endif // WITH_FANET

    SetRxChannel();
    TRX.setModeRX();                                                         // switch to receive mode
    TRX.ClearIrqFlags();

    XorShift32(RX_Random);
    TxTime = (RX_Random&0x3F)+1; TxTime*=6;                                    // [ms] (1..64)*6 = 6..384ms
    uint16_t SlotEnd = 1240;
#ifdef WITH_LORAWAN
    bool WANtx = 0;
    if(WAN_BackOff) WAN_BackOff--;
    else if(Parameters.TxPower!=(-32))                                         // decide to transmit in this slot
    { if(WANdev.State==0 || WANdev.State==2)                                   //
      { WANtx=1; SlotEnd=1220; }
    }
#endif
#ifdef WITH_ADSL
    if(ADSL_Slot==1 && ADSL_TxPkt)
      TimeSlot(TxChan, SlotEnd-TimeSync_msTime(), ADSL_TxPkt, TRX.averRSSI, 0, TxTime);
    else
#endif
      TimeSlot(TxChan, SlotEnd-TimeSync_msTime(), TxPktData1, TRX.averRSSI, 0, TxTime);

#ifdef WITH_PAW
   static uint8_t PAWtxBackOff = 4;
#ifdef WITH_LORAWAN
   if(!WANtx && TxPkt0 && TxPkt0->Packet.Header.AddrType && WANdev.State!=1 && WANdev.State!=3)         // if no WAN transmission/reception scheduled
#else
   if(TxPkt0 && TxPkt0->Packet.Header.AddrType)
#endif
   { PAW_Packet Packet; Packet.Clear();
     OGN1_Packet TxPkt = TxPkt0->Packet;
     TxPkt.Dewhiten();                                                // de-whiten the OGN packet so it can be converted to PAW format
     XorShift32(RX_Random);                                           // convert PAW to OGN
     if(PAWtxBackOff==0 && Parameters.TxPower!=(-32) && TxPkt.Header.NonPos==0 && !TxPkt.Header.Relay && Packet.Copy(TxPkt) && TxPkt.Position.Time<60)
     { TRX.setModeStandby();                                          //
       TRX.PAW_Configure(PAW_SYNC);
       TRX.WriteTxPower(Parameters.TxPower+6);                        //
       vTaskDelay(RX_Random&0x3F);                                    //
       TRX.ClearIrqFlags();
       TRX.PAW_WritePacket(Packet.Byte, 24);                          //
       TRX.setModeTX();                                               // 
       vTaskDelay(8);                                                 // wait 8ms (about the PAW packet time)
#ifdef WITH_SX1262
       TRX.WaitWhileBusy_ms(2);
       for( uint16_t Wait=7; Wait; Wait--)
       { vTaskDelay(1);
         uint16_t Flags=TRX.ReadIrqFlags();
         if(Flags&IRQ_TXDONE) break; }
#else
       uint8_t Break=0;
       for(uint16_t Wait=400; Wait; Wait--)                           // wait for transmission to end
       { uint16_t Flags=TRX.ReadIrqFlags();
         if(Flags&RF_IRQ_PacketSent) Break++;
         if(Break>=2) break; }
#endif
       TRX.setModeStandby();
       TRX.FSK_Configure(0, OGN_SYNC, OGN_TxPacket<OGN_Packet>::Bytes);
       PAWtxBackOff = 2+(RX_Random%5); XorShift32(RX_Random);
       TX_Credit-=8; }
   }
   if(PAWtxBackOff) PAWtxBackOff--;
#endif

#ifdef WITH_LORAWAN
    if(WANtx)
    { TRX.setModeStandby();                              // TRX to standby
      TRX.setLoRa();                                       // switch to LoRa mode (through sleep)
      TRX.setModeLoRaStandby();                          // TRX in standby
      SetFreqPlanWAN();                                    // WAN frequency plan
      TRX.WAN_Configure();                                 // LoRa for WAN config.
      XorShift32(RX_Random);                               // random
      WANdev.Chan = RX_Random&7;                           // choose random channel
      TRX.setChannel(WANdev.Chan);                         // set the channel
      TRX.LoRa_InvertIQ(0); TRX.LoRa_setCRC(1);            // setup for WAN TX
      TRX.WriteTxPower(Parameters.TxPower);                // transmit power
      int RespDelay=0;
      int TxPktLen=0;
      if(WANdev.State==0)
      { uint8_t *TxPacket; TxPktLen=WANdev.getJoinRequest(&TxPacket); // produce Join-Request packet
        TRX.LoRa_SendPacket(TxPacket, TxPktLen); RespDelay=5000;      // transmit join-request packet
        WAN_BackOff=48+(RX_Random%21); XorShift32(RX_Random);
      } else if(WANdev.State==2)
      { const uint8_t *PktData=TxPktData0;
        if(PktData==0) PktData=TxPktData1;
        if(PktData)                                                   // if there is a packet to transmit
        { OGN1_Packet *OGN = (OGN1_Packet *)PktData; if(!OGN->Header.Encrypted) OGN->Dewhiten();
          uint8_t *TxPacket;
          bool Short = !OGN->Header.NonPos && !OGN->Header.Encrypted
                     && OGN->Header.AddrType==3 && OGN->Header.Address==(uint32_t)(getUniqueAddress()&0x00FFFFFF);
          if(Short)
          { TxPktLen=WANdev.getDataPacket(&TxPacket, PktData+4, 16, 1, ((RX_Random>>16)&0xF)==0x8 ); }
          else
          { TxPktLen=WANdev.getDataPacket(&TxPacket, PktData, 20, 1, ((RX_Random>>16)&0xF)==0x8 ); }
          TRX.LoRa_SendPacket(TxPacket, TxPktLen); RespDelay = WANdev.RxDelay&0x0F; if(RespDelay<1) RespDelay=1; RespDelay*=1000;
          WAN_BackOff=50+(RX_Random%21); XorShift32(RX_Random); }
      }
      if(RespDelay)
      { vTaskDelay(8);
        uint8_t Wait = 100;                                  // [ms]
        for( ; Wait; Wait--)                                 // wait for the end of transmission
        { vTaskDelay(1); if(!TRX.isModeLoRaTX()) break; }
          // uint8_t Mode=TRX.ReadMode();
          // if(Mode!=RF_OPMODE_LORA_TX) break; }
        WAN_RespTick=xTaskGetTickCount()+RespDelay;          // when to expect the response after the end of transmitted packet
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_UnsDec(CONS_UART_Write, xTaskGetTickCount(), 4, 3);
        Format_String(CONS_UART_Write, "s LoRaWAN Tx[");
        CONS_UART_Write('0'+WANdev.State);
        Format_String(CONS_UART_Write, "]: ");
        Format_UnsDec(CONS_UART_Write, (unsigned)TxPktLen);  // packet size
        Format_String(CONS_UART_Write, "B/");
        Format_UnsDec(CONS_UART_Write, (unsigned)Wait);      // amount of timeout left
        Format_String(CONS_UART_Write, "ms\n");
        xSemaphoreGive(CONS_Mutex);
      }
      TRX.setFSK();                                        // back to FSK
      SetFreqPlanOGN();                                    // OGN frequency plan
      TRX.FSK_Configure(0, OGN_SYNC, OGN_TxPacket<OGN_Packet>::Bytes);                      // OGN config
      SetRxChannel();
      TRX.setModeRX();                                   // switch to receive mode
      TRX.ClearIrqFlags();
    }
#endif

#ifdef WITH_ADSL
    if(ADSL_TxPkt) ADSL_TxFIFO.Read();
#endif
    if(TxPkt0) RF_TxFIFO.Read();
    if(TxPkt1) RF_TxFIFO.Read();

  }

}

// ======================================================================================
