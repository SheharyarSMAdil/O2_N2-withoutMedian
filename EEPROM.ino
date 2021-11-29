int eepromPointer = EEPROM.length();                // maximum value of eeprom according to ic
const uint8_t noEepromVar = 4;
enum eepromEnum {EXIST = 0, SET_ZERO, SET_SPAN, T_FLOW};
uint16_t eepromAdd[noEepromVar];

void eeW(int _e, float _val) {
  eepromPointer -= sizeof(_val);
  eepromAdd[_e] = eepromPointer;
  EEPROM.put(eepromPointer, _val);
}

void eeW(int _e, int _val) {
  eepromPointer -= sizeof(_val);
  eepromAdd[_e] = eepromPointer;
  EEPROM.put(eepromPointer, _val);
}

void eepromDefault() {

  // Serial.println(EEPROM.read(eepromPointer-sizeof(int)));
  // if(EEPROM.read(eepromPointer-sizeof(int))!=1){
  //   eeW(EXIST,1);
  //   eeW(SET_ZERO,zero_volts);
  //   eeW(SET_SPAN,span_volts);
  //   eeW(T_FLOW,TOTAL_FLOW);
  // }

  //   EEPROM.get(eepromAdd[SET_ZERO], zero_volts);
  //   EEPROM.get(eepromAdd[SET_SPAN], span_volts);
  //   EEPROM.get(eepromAdd[T_FLOW],TOTAL_FLOW);

  if (EEPROM.read(E_SET_ZERO) == 255)
  {
    EEPROM.put(E_SET_ZERO, zero_volts);
  }
  if (EEPROM.read(E_SET_SPAN) == 255)
  {
    EEPROM.put(E_SET_SPAN, span_volts);
  }
  if (EEPROM.read(E_TOTAL_FLOW) == 255)
  {
    EEPROM.put(E_TOTAL_FLOW, TOTAL_FLOW);
  }
  EEPROM.get(E_SET_ZERO, zero_volts);
  EEPROM.get(E_SET_SPAN, span_volts);
  EEPROM.get(E_TOTAL_FLOW, TOTAL_FLOW);

}
