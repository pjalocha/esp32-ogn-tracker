#include <ctype.h>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>

#include <vector>
#include <algorithm>

#include "hal.h"

#include "esp_http_server.h"

#include "format.h"
#include "rf.h"
#include "proc.h"
#include "gps.h"
#include "log.h"
#include "http.h"
#include "ognconv.h"

// #define DEBUG_PRINT

#ifdef WITH_HTTP

// ============================================================================================================

// generic HTML list for submit forms
static void SelectList(httpd_req_t *Req, const char *Name, const char **List, int Size, int Sel=0)
{ char Line[64]; int Len;
  Len =Format_String(Line, "<select name=\"");
  Len+=Format_String(Line+Len, Name);
  Len+=Format_String(Line+Len, "\">\n");
  httpd_resp_send_chunk(Req, Line, Len);
  for(int Idx=0; Idx<Size; Idx++)
  { Len =Format_String(Line, "<option value=\"");
    Len+=Format_UnsDec(Line+Len, (uint16_t)Idx);
    Len+=Format_String(Line+Len, "\"");
    if(Idx==Sel) Len+=Format_String(Line+Len, " selected>");
            else Len+=Format_String(Line+Len, ">");
    Len+=Format_String(Line+Len, List[Idx]);
    Len+=Format_String(Line+Len, "</option>\n");
    httpd_resp_send_chunk(Req, Line, Len); }
  httpd_resp_sendstr_chunk(Req, "</select>\n"); }

// HTML form for the Info parameters
static void ParmForm_Info(httpd_req_t *Req)
{
  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"Info\">\n<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Info</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Pilot</td><td><input type=\"text\" name=\"Pilot\" size=\"10\" value=\"");
  if(Parameters.Pilot[0]) httpd_resp_sendstr_chunk(Req, Parameters.Pilot);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Crew</td><td><input type=\"text\" name=\"Crew\" size=\"10\" value=\"");
  if(Parameters.Crew[0]) httpd_resp_sendstr_chunk(Req, Parameters.Crew);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Base airfield</td><td><input type=\"text\" name=\"Base\" size=\"10\" value=\"");
  if(Parameters.Base[0]) httpd_resp_sendstr_chunk(Req, Parameters.Base);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Registration</td><td><input type=\"text\" name=\"Reg\" size=\"10\" value=\"");
  if(Parameters.Reg[0]) httpd_resp_sendstr_chunk(Req, Parameters.Reg);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Manufacturer</td><td><input type=\"text\" name=\"Manuf\" size=\"10\" value=\"");
  if(Parameters.Manuf[0]) httpd_resp_sendstr_chunk(Req, Parameters.Manuf);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Model</td><td><input type=\"text\" name=\"Model\" size=\"10\" value=\"");
  if(Parameters.Model[0]) httpd_resp_sendstr_chunk(Req, Parameters.Model);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Type</td><td><input type=\"text\" name=\"Type\" size=\"10\" value=\"");
  if(Parameters.Type[0]) httpd_resp_sendstr_chunk(Req, Parameters.Type);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }

// HTML form for the Aircraft identification: address, address-type, aircraft-type
static void ParmForm_Acft(httpd_req_t *Req)
{ char Line[16];

  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"Acft\">\n<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Aircraft</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Address</td><td><input type=\"text\" name=\"Address\" size=\"10\" value=\"0x");
  Format_Hex(Line, (uint8_t)(Parameters.Address>>16)); Format_Hex(Line+2, (uint16_t)Parameters.Address);
  httpd_resp_send_chunk(Req, Line, 6);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  const char *AddrTypeTable[4] = { "Random", "ICAO", "FLARM", "OGN" } ;
  httpd_resp_sendstr_chunk(Req, "<tr><td>Addr-Type</td><td>\n");
  SelectList(Req, "AddrType", AddrTypeTable, 4, Parameters.AddrType);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n");

  const char *AcftTypeTable[16] = { "Unknown", "(moto)Glider", "Tow-plane", "Helicopter", "Parachute", "Drop-plane", "Hang-glider", "Para-glider",
                                    "Powered-aircraft", "Jet-aircraft", "UFO", "Balloon", "Airship", "UAV/drone", "Ground support", "Static object" } ;
  httpd_resp_sendstr_chunk(Req, "<tr><td>Acft-Type</td><td>\n");
  SelectList(Req, "AcftType", AcftTypeTable, 16, Parameters.AcftType);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }

static void ParmForm_GPS(httpd_req_t *Req)  // produce HTML form for GPS parameters
{ char Line[16]; int Len;

  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"GPS\">\n<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
#ifdef WITH_GPS_UBX
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>GPS: UBX</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");
#else
#ifdef WITH_GPS_MTK
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>GPS: MTK</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");
#else
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>GPS</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");
#endif
#endif

  httpd_resp_sendstr_chunk(Req, "<tr><td>Nav. rate [Hz]</td><td><input type=\"text\" name=\"NavRate\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.NavRate);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Nav. mode</td><td><input type=\"text\" name=\"NavMode\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.NavMode);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Geoid-Separ.</td><td>\n");

  const char *GeoidSeparTable[2] = { "GPS", "Override" } ;
  SelectList(Req, "manGeoidSepar", GeoidSeparTable, 2, Parameters.manGeoidSepar);

  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"GeoidSepar\" size=\"3\" value=\"");
  Len=Format_SignDec(Line, Parameters.GeoidSepar, 2, 1, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">\n");

  httpd_resp_sendstr_chunk(Req, "</td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>GNSS mode</td><td><input type=\"text\" name=\"GNSS\" size=\"10\" value=\"0x");
  Len=Format_Hex(Line, Parameters.GNSS);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>PPS delay [ms]</td><td><input type=\"text\" name=\"PPSdelay\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.PPSdelay);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }

static void ParmForm_Other(httpd_req_t *Req)  // produce HTML form for aircraft parameters
{ char Line[16]; int Len;

  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"Other\">\n<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Other</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");

  const char *FreqPlanTable[6] = { "Auto", "Europe/Africa", "USA/Canada", "Australia/Chile", "New Zeeland", "Izrael" };
  httpd_resp_sendstr_chunk(Req, "<tr><td>Freq. plan</td><td>\n");
  SelectList(Req, "FreqPlan", FreqPlanTable, 6, Parameters.FreqPlan);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Tx power [dBm]</td><td><input type=\"text\" name=\"TxPower\" size=\"10\" value=\"");
  Len=Format_SignDec(Line, (int16_t)Parameters.TxPower, 1, 0, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Freq.corr. [ppm]</td><td><input type=\"text\" name=\"RFchipFreqCorr\" size=\"10\" value=\"");
  Len=Format_SignDec(Line, Parameters.RFchipFreqCorr, 2, 1, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Console baud</td><td><input type=\"text\" name=\"CONbaud\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.CONbaud);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Verbose</td><td><input type=\"text\" name=\"Verbose\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, (uint16_t)Parameters.Verbose);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Page sel. mask</td><td><input type=\"text\" name=\"PageMask\" size=\"10\" value=\"0x");
  Len=Format_Hex(Line, Parameters.PageMask);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }

#ifdef WITH_STRATUX
static void ParmForm_Stratux(httpd_req_t *Req) // Connection to Stratux WiFi parameters and options
{ char Line[16]; int Len;

  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"Stratux\">\n<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Stratux</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>SSID</td><td><input type=\"text\" name=\"StratuxWIFI\" size=\"10\" value=\"");
  if(Parameters.StratuxWIFI[0]) httpd_resp_sendstr_chunk(Req, Parameters.StratuxWIFI);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Password</td><td><input type=\"text\" name=\"StratuxPass\" size=\"10\" value=\"");
  if(Parameters.StratuxPass[0]) httpd_resp_sendstr_chunk(Req, Parameters.StratuxPass);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>TCP host</td><td><input type=\"text\" name=\"StratuxHost\" size=\"10\" value=\"");
  if(Parameters.StratuxHost[0]) httpd_resp_sendstr_chunk(Req, Parameters.StratuxHost);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>TCP port</td><td><input type=\"text\" name=\"StratuxPort\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.StratuxPort);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Tx power [dBm]</td><td><input type=\"text\" name=\"StratuxTxPwr\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, (10*Parameters.StratuxTxPwr+2)>>2, 2, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Min. RSSI [dBm]</td><td><input type=\"text\" name=\"StratuxMinSig\" size=\"10\" value=\"");
  Len=Format_SignDec(Line, Parameters.StratuxMinSig, 1, 0, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }
#endif

#ifdef WITH_AP
static void ParmForm_AP(httpd_req_t *Req) // Wi-Fi access point parameters { char Line[16]; int Len;
{ char Line[16]; int Len;

  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"AP\">\n<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Wi-Fi AP</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>SSID</td><td><input type=\"text\" name=\"APname\" size=\"10\" value=\"");
  if(Parameters.APname[0]) httpd_resp_sendstr_chunk(Req, Parameters.APname);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Password</td><td><input type=\"text\" name=\"APpass\" size=\"10\" value=\"");
  if(Parameters.APpass[0]) httpd_resp_sendstr_chunk(Req, Parameters.APpass);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Data port</td><td><input type=\"text\" name=\"APport\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.APport);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Tx power [dBm]</td><td><input type=\"text\" name=\"APtxPwr\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, (10*Parameters.APtxPwr+2)>>2, 2, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Min. RSSI [dBm]</td><td><input type=\"text\" name=\"APminSig\" size=\"10\" value=\"");
  Len=Format_SignDec(Line, Parameters.APminSig, 1, 0, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }
#endif

// ============================================================================================================

static void Table_GPS(httpd_req_t *Req)
{ char Line[128]; int Len;
  uint32_t Time=TimeSync_Time();
  uint32_t Sec = (Time-1)%60;
  GPS_Position *GPS = GPS_getPosition(Sec); if(GPS==0) return;

  httpd_resp_sendstr_chunk(Req, "<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n\
<tr><th>GPS</th><td></td></tr>\n");

  Len =Format_String(Line, "<tr><td>Date</td><td align=\"right\">");
  Len+=Format_UnsDec(Line+Len, GPS->Year+2000 , 4); Line[Len++]='.';
  Len+=Format_UnsDec(Line+Len, GPS->Month, 2); Line[Len++]='.';
  Len+=Format_UnsDec(Line+Len, GPS->Day  , 2);
  Len+=Format_String(Line+Len, "</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Time</td><td align=\"right\">");
  Len+=Format_UnsDec(Line+Len, GPS->Hour , 2); Line[Len++]=':';
  Len+=Format_UnsDec(Line+Len, GPS->Min  , 2); Line[Len++]=':';
  Len+=Format_UnsDec(Line+Len, GPS->Sec  , 2); Line[Len++]='.';
  Len+=Format_UnsDec(Line+Len, GPS->mSec, 3);
  Len+=Format_String(Line+Len, "</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len=Format_String(Line, "<td>Lock</td><td align=\"right\">");
  if(GPS->FixMode>=2) { strcpy(Line+Len, "0-D "); Line[Len]='0'+GPS->FixMode; }
                 else { strcpy(Line+Len, "--- "); }
  Len+=4;
  Len+=Format_String(Line+Len, " Hdop");
  Len+=Format_UnsDec(Line+Len, GPS->HDOP, 2, 1);
  Len+=Format_String(Line+Len, "</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len=Format_String(Line, "<td>Satellites</td><td align=\"right\">");
  Len+=Format_UnsDec(Line+Len, GPS->Satellites);
  Len+=Format_String(Line+Len, "sats ");
  Len+=Format_UnsDec(Line+Len, ((uint16_t)10*GPS_SatSNR+2)/4, 2, 1);
  Len+=Format_String(Line+Len, "dB</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Latitude</td><td align=\"right\">");
  Len+=Format_SignDec(Line+Len, GPS->Latitude/6, 7, 5);
  Len+=Format_String(Line+Len, "&deg;</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Longitude</td><td align=\"right\">");
  Len+=Format_SignDec(Line+Len, GPS->Longitude/6, 8, 5);
  Len+=Format_String(Line+Len, "&deg;</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Altitude</td><td align=\"right\">");
  Len+=Format_SignDec(Line+Len, GPS->Altitude, 2, 1);
  Len+=Format_String(Line+Len, " m</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Geoid Separ.</td><td align=\"right\">");
  Len+=Format_SignDec(Line+Len, GPS->GeoidSeparation, 2, 1);
  Len+=Format_String(Line+Len, " m</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  if(GPS->hasBaro)
  { Len =Format_String(Line, "<tr><td>Pressure Alt.</td><td align=\"right\">");
    Len+=Format_SignDec(Line+Len, GPS->StdAltitude, 2, 1);
    Len+=Format_String(Line+Len, " m</td></tr>\n");
    httpd_resp_send_chunk(Req, Line, Len);

    Len =Format_String(Line, "<tr><td>Pressure</td><td align=\"right\">");
    Len+=Format_SignDec(Line+Len, (GPS->Pressure+2)/4, 3, 2);
    Len+=Format_String(Line+Len, " hPa</td></tr>\n");
    httpd_resp_send_chunk(Req, Line, Len);

    Len =Format_String(Line, "<tr><td>Temperature</td><td align=\"right\">");
    Len+=Format_SignDec(Line+Len, GPS->Temperature, 2, 1);
    Len+=Format_String(Line+Len, " &#x2103;</td></tr>\n");
    httpd_resp_send_chunk(Req, Line, Len); }

  Len =Format_String(Line, "<tr><td>Climb rate</td><td align=\"right\">");
  Len+=Format_SignDec(Line+Len, GPS->ClimbRate, 2, 1);
  Len+=Format_String(Line+Len, " m/s</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Hor. speed</td><td align=\"right\">");
  Len+=Format_UnsDec(Line+Len, GPS->Speed, 2, 1);
  Len+=Format_String(Line+Len, " m/s</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Hor. track</td><td align=\"right\">");
  Len+=Format_UnsDec(Line+Len, GPS->Heading, 4, 1);
  Len+=Format_String(Line+Len, "&deg;</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  httpd_resp_sendstr_chunk(Req, "</table>\n"); }

// -------------------------------------------------------------------------------------------------------------

#ifdef WITH_LOOKOUT
static void Table_LookOut(httpd_req_t *Req)
{ char Line[128]; int Len;
  httpd_resp_sendstr_chunk(Req, "<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>LookOut</th><td align=\"center\"></td><td align=\"center\"></td></tr>\n");

  for( uint8_t Idx=0; Idx<Look.MaxTargets; Idx++)
  { const LookOut_Target *Tgt = Look.Target+Idx; if(!Tgt->Alloc) continue;
    Len =Format_String(Line, "<tr><td>");
    Len+=Format_Hex(Line+Len, Tgt->ID, 7);
    Len+=Format_String(Line+Len, "</td><td>");
    Len+=Format_UnsDec(Line+Len, Tgt->TimeMargin>>1, 2);
    Len+=Format_String(Line+Len, "s</td><td>");
    Len+=Format_UnsDec(Line+Len, ((Tgt->HorDist>>1)+5)/10, 2, 2);
    Len+=Format_String(Line+Len, "km</td></tr>\n");
    httpd_resp_send_chunk(Req, Line, Len); }

  httpd_resp_sendstr_chunk(Req, "</table>\n"); }
#endif

// -------------------------------------------------------------------------------------------------------------

static void Table_Relay(httpd_req_t *Req)
{ char Line[128]; int Len;
  httpd_resp_sendstr_chunk(Req, "<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Relay</th><td align=\"center\">Rank</td><td align=\"center\">[sec]</td></tr>\n");

  for( uint8_t Idx=0; Idx<RelayQueueSize; Idx++)
  { OGN_RxPacket<OGN_Packet> *Packet = RelayQueue.Packet+Idx; if(Packet->Rank==0) continue;
    Len =Format_String(Line, "<tr><td>");
    Line[Len++]='0'+Packet->Packet.Header.AddrType;
    Line[Len++]=':';
    Len+=Format_Hex(Line+Len, Packet->Packet.Header.Address, 6);
    Len+=Format_String(Line+Len, "</td><td>");
    Len+=Format_Hex(Line+Len, Packet->Rank);
    Len+=Format_String(Line+Len, "</td><td>");
    Len+=Format_UnsDec(Line+Len, Packet->Packet.Position.Time, 2);
    Len+=Format_String(Line+Len, "</td></tr>\n");
    httpd_resp_send_chunk(Req, Line, Len); }

  httpd_resp_sendstr_chunk(Req, "</table>\n"); }

// -------------------------------------------------------------------------------------------------------------

static void Table_RF(httpd_req_t *Req)
{ char Line[128]; int Len;

  httpd_resp_sendstr_chunk(Req, "<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  Len=Format_String(Line, "<tr><th><b>RF chip</th><td align=\"center\">");
#ifdef WITH_RFM69
  Len+=Format_String(Line+Len, "RFM69");
#endif
#ifdef WITH_RFM95
  Len+=Format_String(Line+Len, "sx1276");
#endif
#ifdef WITH_SX1272
  Len+=Format_String(Line+Len, "sx1272");
#endif
  Len+=Format_String(Line+Len, "</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Tx power</td><td align=\"right\">");
  Len+=Format_SignDec(Line+Len, (int16_t)Parameters.TxPower);
  Len+=Format_String(Line+Len, "dBm</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Freq. corr.</td><td align=\"right\">");
  Len+=Format_SignDec(Line+Len, (int32_t)Parameters.RFchipFreqCorr, 2, 1);
  Len+=Format_String(Line+Len, "ppm</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Rx noise</td><td align=\"right\">");
  Len+=Format_SignDec(Line+Len, -5*TRX.averRSSI, 2, 1);
  Len+=Format_String(Line+Len, "dBm</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Rx rate</td><td align=\"right\">");
  Len+=Format_UnsDec(Line+Len, RX_OGN_Count64);
  Len+=Format_String(Line+Len, "/min</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Temperature</td><td align=\"right\">");
  Len+=Format_SignDec(Line+Len, (int16_t)TRX.chipTemp);
  Len+=Format_String(Line+Len, "&deg;C</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Rx queue</td><td align=\"right\">");
  Len+=Format_UnsDec(Line+Len, RF_RxFIFO.Full());
  Len+=Format_String(Line+Len, "pkt</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Band</td><td align=\"right\">");
  Len+=Format_UnsDec(Line+Len, (uint16_t)(RF_FreqPlan.getCenterFreq()/100000), 3, 1);
  Len+=Format_String(Line+Len, "MHz</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  httpd_resp_sendstr_chunk(Req, "</table>\n"); }

static uint8_t BattCapacity(uint16_t mVolt)
{ if(mVolt>=4100) return 100;
  if(mVolt<=3600) return   0;
  return (mVolt-3600+2)/5; }

// -------------------------------------------------------------------------------------------------------------

static void Table_Batt(httpd_req_t *Req)
{ char Line[128]; int Len;

  httpd_resp_sendstr_chunk(Req, "<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Battery</th><td></td></tr>\n");

  Len =Format_String(Line, "<td>Voltage</td><td align=\"right\">");
#ifdef WITH_MAVLINK
  Len+=Format_UnsDec(Line+Len, MAVLINK_BattVolt, 4, 3);
#else
  Len+=Format_UnsDec(Line+Len, BatteryVoltage>>8, 4, 3);        // print the battery voltage readout
#endif
  Len+=Format_String(Line+Len, " V</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>Capacity</td><td align=\"right\">");
#ifdef WITH_MAVLINK
  uint8_t Cap=MAVLINK_BattCap;                         // [%] from the drone's telemetry
#else
  uint8_t Cap=BattCapacity(BatteryVoltage>>8);         // [%] est. battery capacity based on the voltage readout
#endif
  Len+=Format_UnsDec(Line+Len, (uint16_t)Cap);
  Len+=Format_String(Line+Len, " %</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

#ifdef WITH_BQ
  uint8_t Status = BQ.readStatus();                    // read status register
  uint8_t State = (Status>>4)&0x03;                    // charging status
  const char *StateName[4] = { "Charge OFF" , "Pre-charge", "Charging", "Full" } ;
  Len =Format_String(Line, "<tr><td>State</td><td align=\"right\">");
  Len+=Format_String(Line+Len, StateName[State]);
  Len+=Format_String(Line+Len, "</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);
#endif

#ifdef WITH_AXP
  uint16_t InpCurr=AXP.readBatteryInpCurrent(); // [mA]
  uint16_t OutCurr=AXP.readBatteryOutCurrent(); // [mA]
  uint16_t Vbus=AXP.readVbusVoltage();          // [mV]
  uint16_t VbusCurr=AXP.readVbusCurrent();      // [mA]
   int16_t Temp=AXP.readTemperature();          // [0.1degC]
  uint32_t InpCharge=AXP.readBatteryInpCharge();
  uint32_t OutCharge=AXP.readBatteryOutCharge();

  int16_t Current = InpCurr-OutCurr;
  Len =Format_String(Line, "<tr><td>Current</td><td align=\"right\">");
  Len+=Format_SignDec(Line+Len, Current, 3);
  Len+=Format_String(Line+Len, " mA</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  int32_t Charge = InpCharge-OutCharge;
  Len =Format_String(Line, "<tr><td>Charge</td><td align=\"right\">");
  Len+=Format_SignDec(Line+Len, (((int64_t)Charge<<12)+562)/1125, 2, 1);
  Len+=Format_String(Line+Len, " mAh</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>USB volt.</td><td align=\"right\">");
  Len+=Format_UnsDec(Line+Len, Vbus, 4, 3);
  Len+=Format_String(Line+Len, " V</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);

  Len =Format_String(Line, "<tr><td>USB curr.</td><td align=\"right\">");
  Len+=Format_UnsDec(Line+Len, VbusCurr, 4, 3);
  Len+=Format_String(Line+Len, " A</td></tr>\n");
  httpd_resp_send_chunk(Req, Line, Len);
#endif

  httpd_resp_sendstr_chunk(Req, "</table>\n"); }

// -------------------------------------------------------------------------------------------------------------

static void Top_Bar(httpd_req_t *Req)
{ char Line[32]; int Len;

  httpd_resp_sendstr_chunk(Req, "<h2>OGN-Tracker</h2>\n");
  httpd_resp_sendstr_chunk(Req, "<b>EUID: ");
  Len=Format_Hex(Line, getUniqueID());
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "</b><br />\n");

  httpd_resp_sendstr_chunk(Req, "<table border=\"1\" cellspacing=\"5\" cellpadding=\"5\"><tr>\n\
<th><a href=\"/\">Status</a></th>\n\
<th><a href=\"/parm.html\">Configuration</a></th>\n\
<th><a href=\"/log.html\">Log files</a></th>\n\
</tr></table><br />\n");
}

// ============================================================================================================

static esp_err_t parm_get_handler(httpd_req_t *Req)
{ // char Line[32]; int Len;
  bool Restart=0;
  uint16_t URLlen=httpd_req_get_url_query_len(Req);
  if(URLlen)
  { char *URL = (char *)malloc(URLlen+1);
    httpd_req_get_url_query_str(Req, URL, URLlen+1);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "parm_get_handler() => [");
    Format_SignDec(CONS_UART_Write, URLlen);
    Format_String(CONS_UART_Write, "] ");
    Format_String(CONS_UART_Write, URL);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    char *Line=URL;
    Restart = strstr(Line,"Restart=1");
    for( ; ; )
    { Parameters.ReadLine(Line);
      Line = strchr(Line, '&'); if(Line==0) break;
      Line++; }
    free(URL);
    Parameters.WriteToNVS(); }
  httpd_resp_sendstr_chunk(Req, "\
<!DOCTYPE html>\n\
<html>\n\
<head>\n\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n\
</head>\n\
<body>\n\
<title>OGN-Tracker configuration</title>\n\
");

  Top_Bar(Req);

  httpd_resp_sendstr_chunk(Req, "<table>\n<tr><td>\n");
  ParmForm_Acft(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
  ParmForm_Info(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
  ParmForm_GPS(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
#ifdef WITH_AP
  ParmForm_AP(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
#endif
#ifdef WITH_STRATUX
  ParmForm_Stratux(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
#endif
  ParmForm_Other(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
  httpd_resp_sendstr_chunk(Req, "\
<form action=\"/parm.html\" method=\"get\">\n\
<input type=\"submit\" value=\"Reset to defaults\">\n\
<input type=\"hidden\" name=\"Defaults\" value=\"1\">\n\
</form>\n");
  // httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\">\n");
  // httpd_resp_sendstr_chunk(Req, "<input type=\"submit\" value=\"Reset to defaults\">\n");
  // httpd_resp_sendstr_chunk(Req, "<input type=\"hidden\" name=\"Defaults\" value=\"1\">\n");
  // httpd_resp_sendstr_chunk(Req, "</form>\n");
  httpd_resp_sendstr_chunk(Req, "\
<form action=\"/parm.html\" method=\"get\">\n\
<input type=\"submit\" value=\"Restart\">\n\
<input type=\"hidden\" name=\"Restart\" value=\"1\">\n\
</form>\n");
  // httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\">\n");
  // httpd_resp_sendstr_chunk(Req, "<input type=\"submit\" value=\"Restart\">\n");
  // httpd_resp_sendstr_chunk(Req, "<input type=\"hidden\" name=\"Restart\" value=\"1\">\n");
  // httpd_resp_sendstr_chunk(Req, "</form>\n");
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n</table>\n");
  httpd_resp_sendstr_chunk(Req, "</body>\n</html>\n");
  httpd_resp_send_chunk(Req, 0, 0);

  if(Restart)
  {
#ifdef WITH_SPIFFS
    FlashLog_SaveReq=1;
#endif
    vTaskDelay(1000);
    esp_restart(); }

  return ESP_OK; }

static esp_err_t top_get_handler(httpd_req_t *Req)
{
  httpd_resp_sendstr_chunk(Req, "\
<!DOCTYPE html>\n\
<html><body>\n\
<title>OGN-Tracker status</title>\n\
");

  Top_Bar(Req);

  Table_GPS(Req);
  httpd_resp_sendstr_chunk(Req, "<br />\n");
  Table_RF(Req);
  httpd_resp_sendstr_chunk(Req, "<br />\n");
  Table_Batt(Req);
#ifdef WITH_LOOKOUT
  httpd_resp_sendstr_chunk(Req, "<br />\n");
  Table_LookOut(Req);
#endif
  httpd_resp_sendstr_chunk(Req, "<br />\n");
  Table_Relay(Req);

  httpd_resp_sendstr_chunk(Req, "</body></html>\n");
  httpd_resp_sendstr_chunk(Req, 0);
  return ESP_OK; }

static int Format_DateTime(char *Out, time_t Time)
{ struct tm *TM = gmtime(&Time);
  int Len=Format_UnsDec(Out, (uint16_t)1900+TM->tm_year, 4);
  Out[Len++]='.';
  Len+=Format_UnsDec(Out+Len, (uint16_t)1+TM->tm_mon, 2);
  Out[Len++]='.';
  Len+=Format_UnsDec(Out+Len, (uint16_t)TM->tm_mday, 2);
  Out[Len++]=' ';
  Len+=Format_UnsDec(Out+Len, (uint16_t)TM->tm_hour, 2);
  Out[Len++]=':';
  Len+=Format_UnsDec(Out+Len, (uint16_t)TM->tm_min, 2);
  Out[Len++]=':';
  Len+=Format_UnsDec(Out+Len, (uint16_t)TM->tm_sec, 2);
  return Len; }

static int LogFileName(char *Name, time_t Time, const char *Ext=0)
{ int Len=Parameters.getAprsCall(Name);
  Name[Len++]='_';
  struct tm *TM = gmtime(&Time);
  Len+=Format_UnsDec(Name+Len, (uint16_t)1900+TM->tm_year, 4);
  Name[Len++]='.';
  Len+=Format_UnsDec(Name+Len, (uint16_t)1+TM->tm_mon, 2);
  Name[Len++]='.';
  Len+=Format_UnsDec(Name+Len, (uint16_t)TM->tm_mday, 2);
  Name[Len++]='_';
  Len+=Format_UnsDec(Name+Len, (uint16_t)TM->tm_hour, 2);
  Len+=Format_UnsDec(Name+Len, (uint16_t)TM->tm_min, 2);
  Len+=Format_UnsDec(Name+Len, (uint16_t)TM->tm_sec, 2);
  if(Ext) Len+=Format_String(Name+Len, Ext);
  Name[Len]=0; return Len; }

// send give log file in the IGC format
static esp_err_t SendLog_IGC(httpd_req_t *Req, const char *FileName, uint32_t FileTime)
{ char ContDisp[64];
  char Line[1000]; int Len=0;
  Len=Format_String(ContDisp, "attachement; filename=\""); Len+=LogFileName(ContDisp+Len, FileTime, ".IGC"); ContDisp[Len++]='\"'; ContDisp[Len]=0;
  httpd_resp_set_hdr(Req, "Content-Disposition", ContDisp);
  httpd_resp_set_type(Req, "text/plain");
  FILE *File = fopen(FileName, "rb"); if(File==0) { httpd_resp_send_chunk(Req, 0, 0); return ESP_OK; }
  // here we should write the IGC header
  OGN_LogPacket<OGN_Packet> Packet;
  for( ; ; )
  { if(fread(&Packet, Packet.Bytes, 1, File)!=1) break;          // read the next packet
    if(!Packet.isCorrect()) continue;
    uint32_t Time = Packet.getTime(FileTime);                    // [sec] get exact time from short time in the packet and the file start time
    Len+=Format_String(Line+Len, "LGNE ");                       // attach APRS as LGNE record
    char *APRS=Line+Len;
    Len+=Packet.Packet.WriteAPRS(Line+Len, Time);                // packet in the APRS format
    bool Own = Packet.Packet.Header.Address==Parameters.Address && Packet.Packet.Header.AddrType==Parameters.AddrType;
    if(Own && !Packet.Packet.Header.NonPos && !Packet.Packet.Header.Encrypted)
      Len+=APRS2IGC(Line+Len, APRS, GPS_GeoidSepar);             // IGC B-record
    if(Len>=800) { httpd_resp_send_chunk(Req, Line, Len); Len=0; } // when more than 800 bytes then write this part to the socket
    vTaskDelay(1); }
  fclose(File);
  if(Len) { httpd_resp_send_chunk(Req, Line, Len); Len=0; }
  httpd_resp_send_chunk(Req, 0, 0);
  return ESP_OK; }

// send give log file in the APRS format
static esp_err_t SendLog_APRS(httpd_req_t *Req, const char *FileName, uint32_t FileTime)
{ char ContDisp[64];
  char Line[1000]; int Len=0;
  Len=Format_String(ContDisp, "attachement; filename=\""); Len+=LogFileName(ContDisp+Len, FileTime, ".aprs"); ContDisp[Len++]='\"'; ContDisp[Len]=0;
  httpd_resp_set_hdr(Req, "Content-Disposition", ContDisp);
  httpd_resp_set_type(Req, "text/plain");
  FILE *File = fopen(FileName, "rb"); if(File==0) { httpd_resp_send_chunk(Req, 0, 0); return ESP_OK; }
  OGN_LogPacket<OGN_Packet> Packet;
  for( ; ; )
  { if(fread(&Packet, Packet.Bytes, 1, File)!=1) break;          // read the next packet
    if(!Packet.isCorrect()) continue;
    uint32_t Time = Packet.getTime(FileTime);                    // [sec] get exact time from short time in the packet and the file start time
    Len+=Packet.Packet.WriteAPRS(Line+Len, Time);                // print the packet in the APRS format
    if(Len>850) { httpd_resp_send_chunk(Req, Line, Len); Len=0; }
    vTaskDelay(1); }
  fclose(File);
  if(Len) { httpd_resp_send_chunk(Req, Line, Len); Len=0; }
  httpd_resp_send_chunk(Req, 0, 0);
  return ESP_OK; }

// send given log file in the TLG (binary) format
static esp_err_t SendLog_TLG(httpd_req_t *Req, const char *FileName, uint32_t FileTime)
{ char ContDisp[64];
  char Line[512]; int Len;
  Len=Format_String(ContDisp, "attachement; filename=\"");
  Len+=Parameters.getAprsCall(ContDisp+Len); ContDisp[Len++]='_';
  Len+=Format_Hex(ContDisp+Len, FileTime);
  Len+=Format_String(ContDisp+Len, ".TLG"); ContDisp[Len++]='\"'; ContDisp[Len]=0;
  httpd_resp_set_hdr(Req, "Content-Disposition", ContDisp);
  httpd_resp_set_type(Req, "application/octet-stream");
  FILE *File = fopen(FileName, "rb"); if(File==0) { httpd_resp_send_chunk(Req, 0, 0); return ESP_OK; }
  for( ; ; )
  { Len=fread(Line, 1, 512, File); if(Len<=0) break;
    httpd_resp_send_chunk(Req, Line, Len);
    vTaskDelay(1); }
  fclose(File);
  httpd_resp_send_chunk(Req, 0, 0);
  return ESP_OK; }

// handle the HTTP request for the log files page
static esp_err_t log_get_handler(httpd_req_t *Req)
{ char FullName[32]; char Line[256]; struct stat Stat;
  const char *Path= "/spiffs";

  uint16_t URLlen=httpd_req_get_url_query_len(Req);
  if(URLlen)
  { char *URL = (char *)malloc(URLlen+1);
    httpd_req_get_url_query_str(Req, URL, URLlen+1);
    char Name[16]; bool SendFile = httpd_query_key_value(URL, "File", Name, 16)==ESP_OK;
    char Format[8] = { 0 }; httpd_query_key_value(URL, "Format", Format, 8);
    free(URL);
    if(SendFile)
    { AddPath(FullName, Name, Path);
      uint32_t Time=FlashLog_ReadShortFileTime(Name);
      if(Time)
      { if(strcmp(Format, "APRS")==0) return SendLog_APRS(Req, FullName, Time);
        if(strcmp(Format, "IGC" )==0) return SendLog_IGC(Req, FullName, Time);
        return SendLog_TLG(Req, FullName, Time); }
    }
  }
  httpd_resp_sendstr_chunk(Req, "\
<!DOCTYPE html>\n\
<html>\n\
<head>\n\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n\
</head>\n\
<body>\n\
<title>OGN-Tracker log files</title>\n\
");

  Top_Bar(Req);

  std::vector<uint32_t> FileList;                               // list of log files
  DIR *Dir=opendir(Path);                                       // open the log file directory
  if(Dir==0)
  { httpd_resp_sendstr_chunk(Req, "<p>Cannot open the log directory !</p>\n");
    httpd_resp_send_chunk(Req, 0, 0);
    return ESP_OK; }
  for( ; ; )
  { struct dirent *Ent = readdir(Dir); if(!Ent) break;        // read next directory entry, break if all read
    if(Ent->d_type != DT_REG) continue;                       // skip non-regular files
    char *Name = Ent->d_name;
    uint32_t Time=FlashLog_ReadShortFileTime(Name);           // read time from the file name
    if(Time==0) continue;                                     // skip if not .TLG format
    FileList.push_back(Time); }
  closedir(Dir);
  std::sort(FileList.begin(), FileList.end());

  httpd_resp_sendstr_chunk(Req, "<table>\n<tr><th>File</th><th></th><th></th><th>[KB]</th><th>Date</th></tr>\n");
  for(size_t Idx=0; Idx<FileList.size(); Idx++)
  { uint32_t Time=FileList[Idx];
    char Name[16];
    FlashLog_ShortFileName(Name, Time);
    AddPath(FullName, Name, Path);
    uint32_t Size=0;
    if(stat(FullName, &Stat)>=0)                              // get file info
      Size = Stat.st_size;
    int Len=Format_String(Line, "<tr><td><a href=\"/log.html?File=");
    Len+=Format_String(Line+Len, Name);
    Len+=Format_String(Line+Len, "\">");
    Len+=Format_String(Line+Len, Name);
    Len+=Format_String(Line+Len, "</a></td><td><a href=\"/log.html?Format=APRS&File=");
    Len+=Format_String(Line+Len, Name);
    Len+=Format_String(Line+Len, "\">APRS</a></td>");
    Len+=Format_String(Line+Len, "<td><a href=\"/log.html?Format=IGC&File=");
    Len+=Format_String(Line+Len, Name);
    Len+=Format_String(Line+Len, "\">IGC</a></td><td align=\"center\">");
    Len+=Format_UnsDec(Line+Len, (Size+512)>>10);
    Len+=Format_String(Line+Len, "</td><td>");
    Len+=Format_DateTime(Line+Len, Time);
    Len+=Format_String(Line+Len, "</td></tr>\n");
    httpd_resp_send_chunk(Req, Line, Len);
    vTaskDelay(1); }
  httpd_resp_sendstr_chunk(Req, "</table>\n");

  httpd_resp_sendstr_chunk(Req, "</body>\n</html>\n");
  httpd_resp_send_chunk(Req, 0, 0);
  return ESP_OK; }

static esp_err_t logo_get_handler(httpd_req_t *Req)
{ extern const uint8_t OGN_logo_jpg[]   asm("_binary_OGN_logo_240x240_jpg_start");
  extern const uint8_t OGN_logo_end[]   asm("_binary_OGN_logo_240x240_jpg_end");
  const int OGN_logo_size = OGN_logo_end-OGN_logo_jpg;
  httpd_resp_set_type(Req, "image/jpeg");
  httpd_resp_send(Req, (const char *)OGN_logo_jpg, OGN_logo_size);
  return ESP_OK; }

static const httpd_uri_t HTTPtop =
{ .uri       = "/",
  .method    = HTTP_GET,
  .handler   = top_get_handler,
  .user_ctx  = 0 };

static const httpd_uri_t HTTPlogo =
{ .uri       = "/logo.jpeg",
  .method    = HTTP_GET,
  .handler   = logo_get_handler,
  .user_ctx  = 0 };

static const httpd_uri_t HTTPparm =
{ .uri       = "/parm.html",
  .method    = HTTP_GET,
  .handler   = parm_get_handler,
  .user_ctx  = 0 };

static const httpd_uri_t HTTPlog =
{ .uri       = "/log.html",
  .method    = HTTP_GET,
  .handler   = log_get_handler,
  .user_ctx  = 0 };

static httpd_handle_t HTTPserver = 0;

esp_err_t HTTP_Start(int MaxSockets, int Port)
{ httpd_config_t Config = HTTPD_DEFAULT_CONFIG();
  Config.server_port      = Port;
  Config.task_priority    = tskIDLE_PRIORITY+3;
  Config.max_open_sockets = MaxSockets;
  esp_err_t Err=httpd_start(&HTTPserver, &Config); if(Err!=ESP_OK) return Err;
  httpd_register_uri_handler(HTTPserver, &HTTPtop);  // top URL
  httpd_register_uri_handler(HTTPserver, &HTTPparm); // parameters URL
  httpd_register_uri_handler(HTTPserver, &HTTPlog);  // log files URL
  httpd_register_uri_handler(HTTPserver, &HTTPlogo); // OGN logo
  return Err; }

void HTTP_Stop(void)
{ if(HTTPserver) httpd_stop(HTTPserver); HTTPserver=0; }

// ============================================================================================================

#endif // WITH_HTTP
