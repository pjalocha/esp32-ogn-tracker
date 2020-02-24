#if defined(WITH_ST7789) || defined(WITH_ILI9341)

#include "st7789.h"

void LCD_LogoPage_Draw(void);
void LCD_LogoPage_Draw(uint32_t Time, GPS_Position *GPS);
void LCD_LogoPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange);

void LCD_GPSpage_Draw(void);
void LCD_GPSpage_Draw(uint32_t Time, GPS_Position *GPS);
void LCD_GPSpage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange);

void LCD_RFpage_Draw(void);
void LCD_RFpage_Draw(uint32_t Time, GPS_Position *GPS);
void LCD_RFpage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange);

void LCD_BattPage_Draw(void);
void LCD_BattPage_Draw(uint32_t Time, GPS_Position *GPS);
void LCD_BattPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange);

void LCD_ParmPage_Draw(void);
void LCD_ParmPage_Draw(uint32_t Time, GPS_Position *GPS);
void LCD_ParmPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange);

void LCD_BaroPage_Draw(void);
void LCD_BaroPage_Draw(uint32_t Time, GPS_Position *GPS);
void LCD_BaroPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange);

void LCD_SysPage_Draw(void);
void LCD_SysPage_Draw(uint32_t Time, GPS_Position *GPS);
void LCD_SysPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange);

void LCD_LookPage_Draw(void);
void LCD_LookPage_Draw(uint32_t Time, GPS_Position *GPS);
void LCD_LookPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange);

void LCD_RelayPage_Draw(void);
void LCD_RelayPage_Draw(uint32_t Time, GPS_Position *GPS);
void LCD_RelayPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange);

#endif
