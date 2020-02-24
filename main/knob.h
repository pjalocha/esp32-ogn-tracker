#ifdef WITH_KNOB

extern volatile uint8_t KNOB_Tick;

#ifdef __cplusplus
  extern "C"
#endif
void vTaskKNOB(void* pvParameters);

#endif

