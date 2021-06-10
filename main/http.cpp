#include <ctype.h>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>

#include <vector>
#include <algorithm>

#include "mbedtls/md5.h"

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

static void Begin_Control_Row(httpd_req_t *Req, const char *Label)
{
  httpd_resp_sendstr_chunk(Req, "<div class=\"control-row\">\n<label>");
  httpd_resp_sendstr_chunk(Req, Label);
  httpd_resp_sendstr_chunk(Req, "</label><div class=\"input\">\n");
}
static void End_Control_Row(httpd_req_t *Req)
{
  httpd_resp_sendstr_chunk(Req, "\n</div></div>\n");
}

// HTML form for the Info parameters
static void ParmForm_Info(httpd_req_t *Req)
{
  httpd_resp_sendstr_chunk(Req, "<h2>Info</h2>");
  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"POST\" id=\"Info\">\n");

  Begin_Control_Row(Req, "Pilot");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"Pilot\" size=\"10\" value=\"");
  if(Parameters.Pilot[0]) httpd_resp_sendstr_chunk(Req, Parameters.Pilot);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Crew");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"Crew\" size=\"10\" value=\"");
  if(Parameters.Crew[0]) httpd_resp_sendstr_chunk(Req, Parameters.Crew);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Base airfield");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"Base\" size=\"10\" value=\"");
  if(Parameters.Base[0]) httpd_resp_sendstr_chunk(Req, Parameters.Base);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Registration");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"Reg\" size=\"10\" value=\"");
  if(Parameters.Reg[0]) httpd_resp_sendstr_chunk(Req, Parameters.Reg);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Manufacturer");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"Manuf\" size=\"10\" value=\"");
  if(Parameters.Manuf[0]) httpd_resp_sendstr_chunk(Req, Parameters.Manuf);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Model");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"Model\" size=\"10\" value=\"");
  if(Parameters.Model[0]) httpd_resp_sendstr_chunk(Req, Parameters.Model);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Type");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"Type\" size=\"10\" value=\"");
  if(Parameters.Type[0]) httpd_resp_sendstr_chunk(Req, Parameters.Type);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  httpd_resp_sendstr_chunk(Req, "<div class=\"submit-row\"><input type=\"submit\" value=\"Apply\"></div>\n");
  httpd_resp_sendstr_chunk(Req, "</form>\n"); }

// HTML form for the Aircraft identification: address, address-type, aircraft-type
static void ParmForm_Acft(httpd_req_t *Req)
{ char Line[16];

  httpd_resp_sendstr_chunk(Req, "<h2>Aircraft</h2>");
  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"POST\" id=\"Acft\">\n");

  Begin_Control_Row(Req, "Address");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"Address\" size=\"10\" value=\"0x");
  Format_Hex(Line, (uint8_t)(Parameters.Address>>16)); Format_Hex(Line+2, (uint16_t)Parameters.Address);
  httpd_resp_send_chunk(Req, Line, 6);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Addr-Type");
  const char *AddrTypeTable[4] = { "Random", "ICAO", "FLARM", "OGN" } ;
  SelectList(Req, "AddrType", AddrTypeTable, 4, Parameters.AddrType);
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Acft-Type");
  const char *AcftTypeTable[16] = { "Unknown", "(moto)Glider", "Tow-plane", "Helicopter", "Parachute", "Drop-plane", "Hang-glider", "Para-glider",
                                    "Powered-aircraft", "Jet-aircraft", "UFO", "Balloon", "Airship", "UAV/drone", "Ground support", "Static object" } ;
  SelectList(Req, "AcftType", AcftTypeTable, 16, Parameters.AcftType);
  End_Control_Row(Req);

  httpd_resp_sendstr_chunk(Req, "<div class=\"submit-row\"><input type=\"submit\" value=\"Apply\"></div>\n");
  httpd_resp_sendstr_chunk(Req, "</form>\n"); }

static void ParmForm_GPS(httpd_req_t *Req)  // produce HTML form for GPS parameters
{ char Line[16]; int Len;

#ifdef WITH_GPS_UBX
  httpd_resp_sendstr_chunk(Req, "<h2>GPS: UBX</h2>");
#else
#ifdef WITH_GPS_MTK
  httpd_resp_sendstr_chunk(Req, "<h2>GPS: MTK</h2>");
#else
  httpd_resp_sendstr_chunk(Req, "<h2>GPS</h2>");
#endif
#endif
  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"POST\" id=\"GPS\">\n");


  Begin_Control_Row(Req, "Nav. rate [Hz]");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"NavRate\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.NavRate);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Nav. mode");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"NavMode\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.NavMode);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Geoid-Separ.");

  const char *GeoidSeparTable[2] = { "GPS", "Override" } ;
  SelectList(Req, "manGeoidSepar", GeoidSeparTable, 2, Parameters.manGeoidSepar);

  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"GeoidSepar\" size=\"3\" value=\"");
  Len=Format_SignDec(Line, Parameters.GeoidSepar, 2, 1, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">\n");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "GNSS mode");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"GNSS\" size=\"10\" value=\"0x");
  Len=Format_Hex(Line, Parameters.GNSS);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "PPS delay [ms]");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"PPSdelay\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.PPSdelay);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  httpd_resp_sendstr_chunk(Req, "<div class=\"submit-row\"><input type=\"submit\" value=\"Apply\"></div>\n");
  httpd_resp_sendstr_chunk(Req, "</form>\n"); }

static void ParmForm_Other(httpd_req_t *Req)  // produce HTML form for aircraft parameters
{ char Line[16]; int Len;

  httpd_resp_sendstr_chunk(Req, "<h2>Other</h2>");
  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"POST\" id=\"Other\">\n");

  Begin_Control_Row(Req, "Freq. plan");
  const char *FreqPlanTable[6] = { "Auto", "Europe/Africa", "USA/Canada", "Australia/Chile", "New Zeeland", "Izrael" };
  SelectList(Req, "FreqPlan", FreqPlanTable, 6, Parameters.FreqPlan);
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Tx power [dBm]");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"TxPower\" size=\"10\" value=\"");
  Len=Format_SignDec(Line, (int16_t)Parameters.TxPower, 1, 0, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Freq.corr. [ppm]");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"RFchipFreqCorr\" size=\"10\" value=\"");
  Len=Format_SignDec(Line, Parameters.RFchipFreqCorr, 2, 1, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Console baud");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"CONbaud\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.CONbaud);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Verbose");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"Verbose\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, (uint16_t)Parameters.Verbose);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Page sel. mask");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"PageMask\" size=\"10\" value=\"0x");
  Len=Format_Hex(Line, Parameters.PageMask);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Initial Page");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"InitialPage\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, (uint8_t)Parameters.InitialPage);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  httpd_resp_sendstr_chunk(Req, "<div class=\"submit-row\"><input type=\"submit\" value=\"Apply\"></div>\n");
  httpd_resp_sendstr_chunk(Req, "</form>\n"); }

#ifdef WITH_STRATUX
static void ParmForm_Stratux(httpd_req_t *Req) // Connection to Stratux WiFi parameters and options
{ char Line[16]; int Len;

  httpd_resp_sendstr_chunk(Req, "<h2>Stratux</h2>");
  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"POST\" id=\"Stratux\">\n");

  Begin_Control_Row(Req, "SSID");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"StratuxWIFI\" size=\"10\" value=\"");
  if(Parameters.StratuxWIFI[0]) httpd_resp_sendstr_chunk(Req, Parameters.StratuxWIFI);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Password");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"StratuxPass\" size=\"10\" value=\"");
  if(Parameters.StratuxPass[0]) httpd_resp_sendstr_chunk(Req, Parameters.StratuxPass);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "TCP host");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"StratuxHost\" size=\"10\" value=\"");
  if(Parameters.StratuxHost[0]) httpd_resp_sendstr_chunk(Req, Parameters.StratuxHost);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "TCP port");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"StratuxPort\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.StratuxPort);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Tx power [dBm]");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"StratuxTxPwr\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, (10*Parameters.StratuxTxPwr+2)>>2, 2, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Min. RSSI [dBm]");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"StratuxMinSig\" size=\"10\" value=\"");
  Len=Format_SignDec(Line, Parameters.StratuxMinSig, 1, 0, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  httpd_resp_sendstr_chunk(Req, "<div class=\"submit-row\"><input type=\"submit\" value=\"Apply\"></div>\n");
  httpd_resp_sendstr_chunk(Req, "</form>\n"); }
#endif

#ifdef WITH_AP
static void ParmForm_AP(httpd_req_t *Req) // Wi-Fi access point parameters { char Line[16]; int Len;
{ char Line[16]; int Len;

  httpd_resp_sendstr_chunk(Req, "<h2>Wi-Fi AP</h2>");
  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"POST\" id=\"AP\">\n");

  Begin_Control_Row(Req, "SSID");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"APname\" size=\"10\" value=\"");
  if(Parameters.APname[0]) httpd_resp_sendstr_chunk(Req, Parameters.APname);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Password");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"APpass\" size=\"10\" value=\"");
  if(Parameters.APpass[0]) httpd_resp_sendstr_chunk(Req, Parameters.APpass);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Data port");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"APport\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.APport);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Tx power [dBm]");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"APtxPwr\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, (10*Parameters.APtxPwr+2)>>2, 2, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  Begin_Control_Row(Req, "Min. RSSI [dBm]");
  httpd_resp_sendstr_chunk(Req, "<input type=\"text\" name=\"APminSig\" size=\"10\" value=\"");
  Len=Format_SignDec(Line, Parameters.APminSig, 1, 0, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">");
  End_Control_Row(Req);

  httpd_resp_sendstr_chunk(Req, "<div class=\"submit-row\"><input type=\"submit\" value=\"Apply\"></div>\n");
  httpd_resp_sendstr_chunk(Req, "</form>\n"); }
#endif



static void ParmForm_Restart(httpd_req_t *Req)
{
  httpd_resp_sendstr_chunk(Req, "\
<form action=\"/parm.html\" method=\"POST\" onsubmit=\"return confirm('Are you sure to restart?')\">\n\
<input type=\"submit\" value=\"Restart\">\n\
<input type=\"hidden\" name=\"Restart\" value=\"1\">\n\
</form>\n");
}

// ============================================================================================================

static void Table_GPS(httpd_req_t *Req)
{ char Line[128]; int Len;
  uint32_t Time=TimeSync_Time();
  uint32_t Sec = (Time-1)%60;
  GPS_Position *GPS = GPS_getPosition(Sec); if(GPS==0) return;

  httpd_resp_sendstr_chunk(Req, "<h2>GPS</h2>");
  httpd_resp_sendstr_chunk(Req, "<table class=\"table table-striped table-bordered\">\n");

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
  httpd_resp_sendstr_chunk(Req, "<h2>LookOut</h2>");
  httpd_resp_sendstr_chunk(Req, "<table class=\"table table-striped table-bordered\">\n");
  httpd_resp_sendstr_chunk(Req, "<thead><tr><th>LookOut</th><th>Time Margin</th><th>Distance</th></tr></thead>\n<tbody>\n");

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

  httpd_resp_sendstr_chunk(Req, "</tbody>\n</table>\n"); }
#endif

// -------------------------------------------------------------------------------------------------------------

static void Table_Relay(httpd_req_t *Req)
{ char Line[128]; int Len;
  httpd_resp_sendstr_chunk(Req, "<h2>Relay</h2>");
  httpd_resp_sendstr_chunk(Req, "<table class=\"table table-striped table-bordered\">\n");
  httpd_resp_sendstr_chunk(Req, "<thead><tr><th>Relay</th><th>Rank</th><th>[sec]</th></tr></thead>\n<tbody>\n");

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

  httpd_resp_sendstr_chunk(Req, "</tbody>\n</table>\n"); }

// -------------------------------------------------------------------------------------------------------------

static void Table_RF(httpd_req_t *Req)
{ char Line[128]; int Len;

  httpd_resp_sendstr_chunk(Req, "<h2>RF chip</h2>");
  httpd_resp_sendstr_chunk(Req, "<table class=\"table table-striped table-bordered\">\n");
  Len=Format_String(Line, "<tr><td>RF chip</td><td align=\"right\">");
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

  httpd_resp_sendstr_chunk(Req, "<h2>Battery</h2>");
  httpd_resp_sendstr_chunk(Req, "<table class=\"table table-striped table-bordered\">\n");

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
#ifdef WITH_SPIFFS
static void Table_SPIFFS(httpd_req_t *Req)
{ char Line[128]; int Len;

  httpd_resp_sendstr_chunk(Req, "<h2>SPIFFS</h2>");
  httpd_resp_sendstr_chunk(Req, "<table class=\"table table-striped table-bordered\">\n");

  size_t Total, Used;
  if(SPIFFS_Info(Total, Used)==0)                            // get the SPIFFS usage summary
  { 
    Len =Format_String(Line, "<tr><td>Free</td><td align=\"right\">");
    Len+=Format_UnsDec(Line+Len, (Total-Used)/1024);
    Len+=Format_String(Line+Len, " kB</td></tr>\n");
    httpd_resp_send_chunk(Req, Line, Len);

    Len =Format_String(Line, "<tr><td>Used</td><td align=\"right\">");
    Len+=Format_UnsDec(Line+Len, Used/1024);
    Len+=Format_String(Line+Len, " kB</td></tr>\n");
    httpd_resp_send_chunk(Req, Line, Len);

    Len =Format_String(Line, "<tr><td>Total</td><td align=\"right\">");
    Len+=Format_UnsDec(Line+Len, Total/1024);
    Len+=Format_String(Line+Len, " kB</td></tr>\n");
    httpd_resp_send_chunk(Req, Line, Len);
  }
  httpd_resp_sendstr_chunk(Req, "</table>\n"); }
#endif

// -------------------------------------------------------------------------------------------------------------

static void Html_Start(httpd_req_t *Req, const char *Title, const uint8_t ActiveMenuIndex)
{
  httpd_resp_sendstr_chunk(Req, "\
<!DOCTYPE html>\n\
<html>\n\
<head>\n\
<title>");
  httpd_resp_sendstr_chunk(Req, Title);
  httpd_resp_sendstr_chunk(Req, "</title>\n\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n\
<meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\" />\n\
<style>\n\
html {margin: 0px;}\n\
body {margin: 8px;}\n\
h1#page-title {margin: 0; font-size: 28px; line-height: 36px;}\n\
h2 {margin: 0.7em 0 0.3em 0;}\n\
#top-menu {display: flex;margin-bottom: 8px;background: #cbcbcb;}\n\
#top-menu > div > a,#top-menu > div > a:link {padding: 10px;display: block;color: #000000;}\n\
#top-menu > div > a.active, #top-menu > div > a:hover {color: #f3f3f3;background: #2d2d2d;}\n\
#content {padding-bottom: 30px;}\n\
.table{border-collapse:collapse;border-spacing:0;empty-cells:show;border:1px solid #cbcbcb}.table td,.table th{border-left:1px solid #cbcbcb;border-bottom-width:0;border-right-width:0;border-top-width:0;font-size:inherit;margin:0;padding:6px;overflow:visible}.table thead{background-color:#e0e0e0;color:#000;text-align:left;vertical-align:bottom}.table td{background-color:transparent}.table-striped tr:nth-child(2n-1) td{background-color:#f2f2f2}.table-bordered td{border-bottom:1px solid #cbcbcb}.table-bordered tbody>tr:last-child>td{border-bottom-width:0}form{margin:0 0 20px 0}form .control-row{display:flex;margin:6px 0}form .control-row label{width:120px;text-align:right;margin-right:8px;display:block;font-weight:700}form .submit-row{padding-left:128px}\n\
</style>\
</head>\n\
<body>\n\
");
  httpd_resp_sendstr_chunk(Req, "<h1 id=\"page-title\">OGN-Tracker</h1>\n");

  httpd_resp_sendstr_chunk(Req, "<div id=\"top-menu\">\n");

  httpd_resp_sendstr_chunk(Req, "<div><a href=\"/\"");
  if(ActiveMenuIndex==1) httpd_resp_sendstr_chunk(Req, " class=\"active\"");
  httpd_resp_sendstr_chunk(Req, ">Status</a></div>\n");

  httpd_resp_sendstr_chunk(Req, "<div><a href=\"/parm.html\"");
  if(ActiveMenuIndex==2) httpd_resp_sendstr_chunk(Req, " class=\"active\"");
  httpd_resp_sendstr_chunk(Req, ">Configuration</a></div>\n");

  httpd_resp_sendstr_chunk(Req, "<div><a href=\"/log.html\"");
  if(ActiveMenuIndex==3) httpd_resp_sendstr_chunk(Req, " class=\"active\"");
  httpd_resp_sendstr_chunk(Req, ">Log files</a></div>\n");

  httpd_resp_sendstr_chunk(Req, "</div>\n");

  httpd_resp_sendstr_chunk(Req, "<div id=\"content\">\n");
}

static void Html_End(httpd_req_t *Req)
{
  httpd_resp_sendstr_chunk(Req, "</div>\n</body>\n</html>\n");
  httpd_resp_send_chunk(Req, 0, 0);
}

// ============================================================================================================

static esp_err_t parm_post_handler(httpd_req_t *Req)
{
  bool Restart=0;
  /* Destination buffer for content of HTTP POST request.
   * httpd_req_recv() accepts char* only, but content could
   * as well be any binary data (needs type casting).
   * In case of string data, null termination will be absent, and
   * content length would give length of string */
  char content[1024];
  char *URL = (char *)malloc(Req->content_len+1);

  /* Truncate if content length larger than the buffer */
  size_t recv_size = std::min(Req->content_len, sizeof(content));

  int ret = httpd_req_recv(Req, URL, recv_size);
  if (ret <= 0) {  /* 0 return value indicates connection closed */
    /* Check if timeout occurred */
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
      /* In case of timeout one can choose to retry calling
       * httpd_req_recv(), but to keep it simple, here we
       * respond with an HTTP 408 (Request Timeout) error */
      httpd_resp_send_408(Req);
    }
    /* In case of error, returning ESP_FAIL will
     * ensure that the underlying socket is closed */
    return ESP_FAIL;
  }
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "parm_post_handler() => [");
  Format_UnsDec(CONS_UART_Write, Req->content_len, 1);
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
  Parameters.WriteToNVS();

  if(Restart)
  {
#ifdef WITH_SPIFFS
    FlashLog_SaveReq=1;
#endif
    vTaskDelay(1000);
    esp_restart(); }

  // redirect to get handler
  httpd_resp_set_type(Req, "text/html");
  httpd_resp_set_status(Req, "302 Found");
  httpd_resp_set_hdr(Req, "Location", "/parm.html");
  httpd_resp_send(Req, NULL, 0);
  return ESP_OK;
}

static esp_err_t parm_get_handler(httpd_req_t *Req)
{
  Html_Start(Req, "OGN-Tracker configuration", 2);

  ParmForm_Acft(Req);
  ParmForm_Info(Req);
  ParmForm_GPS(Req);
#ifdef WITH_AP
  ParmForm_AP(Req);
#endif
#ifdef WITH_STRATUX
  ParmForm_Stratux(Req);
#endif
  ParmForm_Other(Req);

  ParmForm_Restart(Req);

  Html_End(Req);

  return ESP_OK; }

static esp_err_t top_get_handler(httpd_req_t *Req)
{
  Html_Start(Req, "OGN-Tracker status", 1);

  char Line[32]; int Len;
  httpd_resp_sendstr_chunk(Req, "<b>EUID: ");
  Len=Format_Hex(Line, getUniqueID());
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "</b><br />\n");

  Table_GPS(Req);
  Table_RF(Req);
  Table_Batt(Req);
#ifdef WITH_SPIFFS
  Table_SPIFFS(Req);
#endif
#ifdef WITH_LOOKOUT
  Table_LookOut(Req);
#endif
  Table_Relay(Req);

  Html_End(Req);
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
  mbedtls_md5_context MD5;
  mbedtls_md5_starts_ret(&MD5);
  char Line[1000]; int Len=0;
  Len=Format_String(ContDisp, "attachement; filename=\""); Len+=LogFileName(ContDisp+Len, FileTime, ".IGC"); ContDisp[Len++]='\"'; ContDisp[Len]=0;
  httpd_resp_set_hdr(Req, "Content-Disposition", ContDisp);
  httpd_resp_set_type(Req, "text/plain");
  FILE *File = fopen(FileName, "rb"); if(File==0) { httpd_resp_send_chunk(Req, 0, 0); return ESP_OK; }
  Len=Format_String(Line, "AXXX ESP32-OGN-TRACKER\nHFFXA020\n");         // IGC file header
  Len+=Format_String(Line+Len, "HFDTE");
  GPS_Time Time; Time.setUnixTime(FileTime);
  Len+=Format_UnsDec(Line+Len, (uint16_t)Time.Day  , 2);
  Len+=Format_UnsDec(Line+Len, (uint16_t)Time.Month, 2);
  Len+=Format_UnsDec(Line+Len, (uint16_t)Time.Year , 2);
  Line[Len++]='\n';
  if(Parameters.Pilot[0])
  { Len+=Format_String(Line+Len, "HFPLTPILOTINCHARGE:");
    Len+=Format_String(Line+Len, Parameters.Pilot);
    Line[Len++]='\n'; }
  if(Parameters.Manuf[0])
  { Len+=Format_String(Line+Len, "HFGTYGLIDERTYPE:");
    Len+=Format_String(Line+Len, Parameters.Manuf);
    if(Parameters.Model[0])
    { Len+=Format_String(Line+Len, " ");
      Len+=Format_String(Line+Len, Parameters.Model); }
    Line[Len++]='\n'; }
  if(Parameters.Reg[0])
  { Len+=Format_String(Line+Len, "HFGIDGLIDERID:");
    Len+=Format_String(Line+Len, Parameters.Reg);
    Line[Len++]='\n'; }
#ifdef WITH_FollowMe
  Len+=Format_String(Line+Len, "HFRHWHARDWAREVERSION:FollowMe\n");
#else
  Len+=Format_String(Line+Len, "HFRHWHARDWAREVERSION:ESP32+LoRa\n");     // hardware version
#endif
  Len+=Format_String(Line+Len, "HFRFWFIRMWAREVERSION:ESP32-OGN-TRACKER " __DATE__ " " __TIME__ "\n");
#ifdef WITH_BMP180
  Len+=Format_String(Line+Len, "HFPRSPRESSALTSENSOR:BMP180\n");        // pressure sensor
#endif
#ifdef WITH_BMP280
  Len+=Format_String(Line+Len, "HFPRSPRESSALTSENSOR:BMP280\n");        // pressure sensor
#endif
#ifdef WITH_BME280
  Len+=Format_String(Line+Len, "HFPRSPRESSALTSENSOR:BME280/BMP280\n"); // pressure sensor
#endif
  Len+=Format_String(Line+Len, "LOGN");
  Len+=Format_HHMMSS(Line+Len, FileTime);
  Len+=Format_String(Line+Len, "ID ");
  Len+=Format_Hex(Line+Len, Parameters.AcftID);
  Line[Len++]='\n';
  uint64_t MAC = getUniqueID();
  Len+=Format_String(Line+Len, "LOGN");
  Len+=Format_HHMMSS(Line+Len, FileTime);
  Len+=Format_String(Line+Len, "MAC ");
  Len+=Format_Hex(Line+Len, (uint16_t)(MAC>>32));                  // ESP32 48-bit ID
  Len+=Format_Hex(Line+Len, (uint32_t) MAC     );
  Line[Len++]='\n';
  httpd_resp_send_chunk(Req, Line, Len);
  mbedtls_md5_update_ret(&MD5, (uint8_t *)Line, Len);
  OGN_LogPacket<OGN_Packet> Packet;
  Len=0;
  for( ; ; )
  { if(fread(&Packet, Packet.Bytes, 1, File)!=1) break;          // read the next packet
    if(!Packet.isCorrect()) continue;
    uint32_t Time = Packet.getTime(FileTime);                    // [sec] get exact time from short time in the packet and the file start time
    Len+=Format_String(Line+Len, "LGNE ");                       // attach APRS as LGNE record
    char *APRS=Line+Len;
    Len+=Packet.Packet.WriteAPRS(Line+Len, Time);                // packet in the APRS format
    bool Own = Packet.Packet.Header.Address==Parameters.Address && Packet.Packet.Header.AddrType==Parameters.AddrType; // 
    if(Own && !Packet.Packet.Header.NonPos && !Packet.Packet.Header.Encrypted)
      Len+=APRS2IGC(Line+Len, APRS, GPS_GeoidSepar);             // IGC B-record
    if(Len>=800)                                                 // when more than 800 bytes then write this part to the socket
    { httpd_resp_send_chunk(Req, Line, Len);
      mbedtls_md5_update_ret(&MD5, (uint8_t *)Line, Len);
      Len=0; vTaskDelay(1); }
  }
  fclose(File);
  if(Len)
  { httpd_resp_send_chunk(Req, Line, Len);
    mbedtls_md5_update_ret(&MD5, (uint8_t *)Line, Len);
    Len=0; }
  uint8_t Digest[16];
  mbedtls_md5_finish_ret(&MD5, Digest);
  Len=0; Line[Len++]='G';                                        // G-record, not signed yet, just MD5
  for(int Idx=0; Idx<16; Idx++)
    Len+=Format_Hex(Line+Len, Digest[Idx]);
  Line[Len++]='\n'; Line[Len]=0;
  httpd_resp_send_chunk(Req, Line, Len);
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
  Len=0;
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
  Html_Start(Req, "OGN-Tracker log files", 3);

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

  httpd_resp_sendstr_chunk(Req, "<table class=\"table table-bordered table-striped\">\n<thead><tr><th>File</th><th></th><th></th><th>[KB]</th><th>Date</th></tr></thead>\n<tbody>\n");
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
  httpd_resp_sendstr_chunk(Req, "</tbody></table>\n");

  Html_End(Req);
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

static const httpd_uri_t HTTPparmPost =
{ .uri       = "/parm.html",
  .method    = HTTP_POST,
  .handler   = parm_post_handler,
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
  httpd_register_uri_handler(HTTPserver, &HTTPparmPost); // parameters URL
  httpd_register_uri_handler(HTTPserver, &HTTPlog);  // log files URL
  httpd_register_uri_handler(HTTPserver, &HTTPlogo); // OGN logo
  return Err; }

void HTTP_Stop(void)
{ if(HTTPserver) httpd_stop(HTTPserver); HTTPserver=0; }

// ============================================================================================================

#endif // WITH_HTTP
