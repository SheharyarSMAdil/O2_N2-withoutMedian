#define MFC_O2 Serial1
#define MFC_N2 Serial2
#define O2_Serial Serial3
#define EE_TOTALFLOW 0

#include "Statistic.h"
#include "command.h"
#include <EEPROM.h>
#include <LiquidCrystal.h>
Statistic myStats;

const int rs = 22, en = 23, d4 = 24, d5 = 25, d6 = 26, d7 = 27;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// #define FLOW_OFF 0.0

#define E_UART_ANALYSER 20
#define E_SET_ZERO 24
#define E_SET_SPAN 28
#define E_TOTAL_FLOW 32

// ##################### PCB BUTTON PINOUT ################
#define B_OFF 6
#define B_SET0 7
#define B_SET100 8
#define B_PASSN2 9
#define B_PASSO2 10
#define B_RESET 11

#define lcdBackLight 28
#include <Adafruit_ADS1X15.h>
#include <Wire.h>

Adafruit_ADS1115 ads;

// float mfc_error = 0.15;

bool uart_analyser = false;

long int wait_time = 15000;

float mfc_error = 0;

int itt_count = 0;

float zero_volts = 0;
float span_volts = 1000;
float raw_volts = 0;

bool forced_o2 = false;

const int sample = 100;
const int midSample = 10;

// const float TOTALTOTAL_FLOW = 10.0;

//int analyserFlow;
float analyserFlow;
const String int_str = ":0680010121";
const String zero_str = "!L0.0\r";
const String span_str = "!H100.0\r";
const String reset_str = "!R\r";

long int lt1 = 0, lt_disp;
bool ask_mfc1 = true;
float O2;
const float FLOW_OFF = 0.0;
float TOTAL_FLOW = 10.0;
double sum_o2 = 0;
int counter = 0;
int counter2 = 0;
float pre_o2 = 0;

float o2_array[100];
float o2_array2[50];
float globalError = NULL;
bool bad_command = false;
bool set_target = false;
float target_o2 = 0;
float O2flow;
float N2flow;
long int lt2 = 0;
bool softConnect = false;

//float TOTAL_FLOW = 10.0; // comment out in future
bool start_up = true;

//  ################################# SETUP INITIALISED  ###########################
void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);

  // set adc
  ads.setGain(GAIN_TWO); // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  ads.begin();

  MFC_O2.begin(38400);
  MFC_N2.begin(38400);
  O2_Serial.begin(19200);

  // while(!MFC_O2 && MFC_N2 && O2_Serial){
  //   ;
  // }

  myStats.clear();
  pinMode(B_OFF, INPUT);
  pinMode(B_SET0, INPUT);
  pinMode(B_SET100, INPUT);
  pinMode(B_PASSN2, INPUT);
  pinMode(B_PASSO2, INPUT);
  pinMode(B_RESET, INPUT);
  pinMode(lcdBackLight, OUTPUT);
  digitalWrite(lcdBackLight, HIGH);


  MFC_N2.println(getRequestString(FLOW_OFF));
  MFC_O2.println(getRequestString(FLOW_OFF));
  set_target = false;

  Serial.println("TF_" + String(TOTAL_FLOW));
  eepromDefault();
  Serial.println("TF_" + String(TOTAL_FLOW));


  if (uart_analyser) {
    wait_time = 10000;
    mfc_error = 0;
  } else {
    wait_time = 25000;
    mfc_error = 0.15;
  }
}

//  ############################### LOOP INITIALISED ###########################

void loop() {

  if (start_up) {
    if (millis() > 10000) {
      MFC_O2.println(":058001010400");
      MFC_N2.println(":058001010400");
      start_up = false;
    }
  }
  handleSerial();

  if (millis() - lt1 > 1000) {
    if (ask_mfc1) {
      MFC_O2.println(":06800401210120");
      ask_mfc1 = false;
    } else {
      MFC_N2.println(":06800401210120");
      ask_mfc1 = true;
    }

    //        Serial.print(O2flow, 4); Serial.print("\t\t");
    //        Serial.print(N2flow, 4); Serial.print("\t\t");
    //        Serial.print(N2flow + O2flow, 4); Serial.print("\t\t");
    //        Serial.print(O2); Serial.print('\t');

    if (softConnect) {
      WriteS(100, W_O2_FLOW, String(O2flow, 1));
      WriteS(100, W_N2_FLOW, String(N2flow, 1));
      WriteS(100, W_TOTAL_FLOW, String(O2flow + N2flow, 1));
      WriteS(100, W_O2_CONC, String(O2, 1));
      WriteS(100, W_ANALYSER_FLOW, String(analyserFlow));
      WriteS(100, W_FLOW_SET, String(TOTAL_FLOW));
    }

    if (set_target) {
      Serial.println(target_o2);
      // Serial.println("@100:02con:" + String(target_o2) + ":@");
      Serial.println("#####");
      // WriteS(100,"O2con",String(target_o2));
    }

    if (bad_command) {
      // Serial.println("bad command");
    }
    lt1 = millis();
  }

  readMFC1();
  readMFC2();

  if (uart_analyser) {
    readO2();
  } else {
    read_PM11E();
  }

  if (millis() - lt_disp > 1000) {
    int16_t adc1;
    float volts1;
    adc1 = ads.readADC_SingleEnded(1);
    volts1 = ads.computeVolts(adc1);
    // Serial.println("VOLDS V1:"+String(volts1));
    analyserFlow = CalAnal2(volts1);
    //    analyserFlow = round((volts1 - 0.5) * 1 / 2 * 1000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(String(O2, 1));
    lcd.setCursor(10, 0);
    lcd.print(String(O2flow, 2));
    lcd.setCursor(0, 1);
    lcd.print(analyserFlow);
    lcd.setCursor(10, 1);

    lcd.print(String(N2flow, 2));

    PID();
    lt_disp = millis();
  }

  if (!digitalRead(B_OFF)) {
    //    lcd.clear();
    //    lcd.print(digitalRead(B_OFF));
    //    delay(2000);
    //    lcd.clear();
    MFC_N2.println(getRequestString(FLOW_OFF));
    MFC_O2.println(getRequestString(FLOW_OFF));
    set_target = false;
  }
  if (!digitalRead(B_SET0)) {
    if (uart_analyser) {
      O2_Serial.print(zero_str);
    } else {
      set_PM11e_zero();
    }
  }
  if (!digitalRead(B_SET100)) {
    if (uart_analyser) {
      O2_Serial.print(span_str);
    } else {
      set_PM11e_span();
    }
  }
  if (!digitalRead(B_PASSN2)) {

    lcd.clear();
    lcd.print(digitalRead(B_PASSN2));
    delay(2000);
    lcd.clear();
    // Serial.println("passing N2");
    MFC_O2.println(getRequestString(FLOW_OFF));
    MFC_N2.println(getRequestString(TOTAL_FLOW));
  }
  if (!digitalRead(B_PASSO2)) {
    lcd.clear();
    lcd.print(digitalRead(B_PASSO2));
    delay(2000);
    lcd.clear();
    // Serial.println("passing O2");
    MFC_O2.println(getRequestString(TOTAL_FLOW));
    MFC_N2.println(getRequestString(FLOW_OFF));
  }
  if (!digitalRead(B_RESET)) {
    if (uart_analyser) {
      O2_Serial.print(reset_str);
    }
  }
}

int CalAnal(float v) {
  int ret = 0;
  if (v <= 0.5) {
    ret = 0;
  } else if (v > 0.5 && v <= 1.6) {
    ret = (v - 0.5) * (0.25) / (1.1) * 1000;
  } else if (v > 1.6 && v <= 2.1) {
    ret = (v - 1.6) * (0.25) / (2.1 - 1.6) * 1000 + 250;
  } else if (v > 2.1 && v <= 0.75) {
    ret = (v - 2.1) * (0.25) / (2.31 - 2.1) * 1000 + 500;
  } else if (v > 0.75 && v <= 1) {
    ret = (v - 0.75) * (0.25) / (2.5 - 2.31) * 1000 + 750;
  } else {
    ret = 1000;
  }

  if (ret <= 4)
    ret = 0;
  return ret;
}

float CalAnal2(float v) {
  Serial.print("anal. volt: ");
  Serial.println(v);
  v = round(v * 100) / 100.0;
  if (v < 1.01) v = 1 ;

  float ret = 0;
  //   ret = -0.251 + 0.2517 * v - 0.0005 * v * v;
  ret = 250 * (v - 1);
  if(ret<3){
    ret=0;
  }
  return ret;
}
