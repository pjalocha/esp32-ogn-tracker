
#ifdef WITH_OLED

int OLED_DisplayStatus  (uint32_t Time, uint8_t LineIdx=0);
int OLED_DisplayPosition(GPS_Position *GPS=0, uint8_t LineIdx=2);

#endif

#ifdef WITH_U8G2_OLED

void OLED_DrawStatus   (u8g2_t *OLED, uint32_t Time, uint8_t LineIdx=0);
void OLED_DrawPosition (u8g2_t *OLED, GPS_Position *GPS=0, uint8_t LineIdx=2);

void OLED_DrawLogo     (u8g2_t *OLED, GPS_Position *GPS=0);
void OLED_DrawGPS      (u8g2_t *OLED, GPS_Position *GPS=0);
void OLED_DrawRF       (u8g2_t *OLED, GPS_Position *GPS=0);
void OLED_DrawRelay    (u8g2_t *OLED, GPS_Position *GPS=0);
void OLED_DrawLookout  (u8g2_t *OLED, GPS_Position *GPS=0);
void OLED_DrawTrafWarn (u8g2_t *OLED, GPS_Position *GPS=0);
void OLED_DrawBaro     (u8g2_t *OLED, GPS_Position *GPS=0);
void OLED_DrawBattery  (u8g2_t *OLED, GPS_Position *GPS=0);
void OLED_DrawStatusBar(u8g2_t *OLED, GPS_Position *GPS=0);
void OLED_DrawSystem   (u8g2_t *OLED, GPS_Position *GPS=0);
void OLED_DrawID       (u8g2_t *OLED, GPS_Position *GPS=0);

#endif
