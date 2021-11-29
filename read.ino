void readO2() {

  if (O2_Serial.available()) {
    String x;
    long lts = millis();
    x = O2_Serial.readStringUntil('\r');
    if (x.indexOf("B") > -1) {
      bad_command = true;
    } else {
      bad_command = false;
    }
    x.remove(x.length() - 3, x.length());
    x.trim();

    float temp_o2 = x.toFloat();
    o2_array[counter] = temp_o2;
    // sum_o2 += temp_o2;                  // old logic
    counter++;
    if (counter == sample) {

      int array_length = sizeof(o2_array) / sizeof(o2_array[0]);
      qsort(o2_array, array_length, sizeof(o2_array[0]), sort_desc);

      int rStart = (sample / 2) - midSample / 2;
      int meanArray[midSample];

      for (int i = rStart; i < rStart + midSample; i++)
      {
        sum_o2 += o2_array[i];
      }

      O2 = sum_o2 / midSample;

      //for (int i = 0; i < 50; i++) {
      //        Serial.print(o2_array[i]); Serial.print(",");
      //}
      // Serial.println();

      O2 = round(O2 * 10) / 10.0;

       if (O2 > 101) {
      O2 = 101;
    } else if(O2 < -1) {
      O2 = -1;
    }
      sum_o2 = 0;
      counter = 0;

      
      if (pre_o2 == 0) pre_o2 = O2;
      float diff = abs(pre_o2 - O2);

      if (diff <= 0.5) {
        O2 = 0.95 * O2 + 0.05 * pre_o2;

      } else if (diff < 1) {
        O2 = 0.8 * O2 + 0.2 * pre_o2;
      }
      else if (diff < 2) {
        O2 = 0.5 * O2 + 0.5 * pre_o2;
      }
      //Serial.print("diff: "); Serial.println(diff);
      if (forced_o2) {


        O2 = target_o2;
      }




      pre_o2 = O2;
    }
  }
}


void read_PM11E() {

  int16_t adc2;
  float volts2;

  adc2 = ads.readADC_SingleEnded(2);

  volts2 = ads.computeVolts(adc2) * 1000;


  o2_array[counter] = volts2;

  counter++;
  if (counter == sample) {

    double sum_volts = 0;
    for (int i = 0; i < sample; i++) {
      sum_volts += o2_array[i];
    }

    counter = 0;



    raw_volts = sum_volts / sample;

    O2 = (raw_volts - zero_volts) / (span_volts - zero_volts) * 100;

    O2 = round(O2 * 10) / 10.0;
    
    if (O2 > 101) {
      O2 = 101;
    } else if(O2 < -1) {
      O2 = -1;
    }

    if (pre_o2 == 0) pre_o2 = O2;
    float diff = abs(pre_o2 - O2);

    if (diff <= 0.5) {
      O2 = 0.95 * O2 + 0.05 * pre_o2;

    } else if (diff < 1) {
      O2 = 0.8 * O2 + 0.2 * pre_o2;
    }
    else if (diff < 2) {
      O2 = 0.5 * O2 + 0.5 * pre_o2;
    }

    if (forced_o2) {
      O2 = target_o2;
    }

    o2_array2[counter2] = O2;
    counter2++;
    if (counter2 == 50) {
      counter2 = 0;
    }
    myStats.clear();
    for (int i = 0; i < 50; i++) {
      myStats.add(o2_array2[i]);
    }
    //Serial.println(myStats.pop_stdev(), 2);
    //Serial.println(myStats.average(), 2);

    //Serial.println(raw_volts);
  }
}



void set_PM11e_zero() {
  zero_volts = raw_volts;
   EEPROM.put(E_SET_ZERO, zero_volts);
  // eeW(SET_ZERO,zero_volts);
  
}

void set_PM11e_span() {
  span_volts = raw_volts;
   EEPROM.put(E_SET_SPAN, span_volts);
  // eeW(SET_SPAN,span_volts);
}


void PID() {
  if (set_target) {
    float error = O2 - target_o2;
    //  Serial.println(O2);

    //        Serial.print("error: ");
    //         Serial.println(error);



    // if (abs(target_o2 - O2) > 0.05) {

    // }

    

    if (millis() - lt2 > wait_time) {

      float sd = myStats.pop_stdev();
      //   Serial.print("sd :"); Serial.println(sd);
      if (sd < 0.08) {
        if (abs(error) <= 0.1) {
          forced_o2 = true;
        } else {
          forced_o2 = false;
        }


        //if (diff < 0.5) {

        // Serial.println(O2);
        // Serial.println(target_o2);
        //Serial.print("error f: ");
        //Serial.println(error);

        //Serial.println("GE: " + String(globalError));

        if (abs(error) > 0.1) {
//          if (itt_count == 0) {
//            mfc_error = error;
//          }
          itt_count++;
          //            Serial.println(F("performing..........................."));
          float O2_flow = O2flow - (error*0.95) / 100.0 * TOTAL_FLOW;
          float N2_flow = N2flow + (error*0.95) / 100.0 * TOTAL_FLOW;

          //            Serial.println("N2 flow: "); Serial.println(N2_flow);
          //            Serial.println("O2 flow: "); Serial.println(O2_flow);

          MFC_N2.println(getRequestString(N2_flow));
          MFC_O2.println(getRequestString(O2_flow));
        } else {
          WriteS(100, W_ACHIEVED, _blank);
          //globalError = error;
          mfc_error = (target_o2 / 100.0 * TOTAL_FLOW - O2flow)*10;
          Serial.print("MFC error: "); Serial.println(mfc_error);
        }

        lt2 = millis();
        //}


        //}
      }

    }
    //}

  } else {
    if (forced_o2)
      forced_o2 = false;
  }
}

void readMFC1() {

  if (MFC_O2.available()) {
    String x;
    long lts = millis();
    //Serial.print("O2_response: ");
    while (millis() - lts < 100) {
      while (MFC_O2.available()) {
        char c = (char)MFC_O2.read();
        x += c;
      }
    }
    //Serial.println(x);
    O2flow = ResponseToFlow(x);
    //Serial.println(O2flow, 3);


  }
}
void readMFC2() {

  if (MFC_N2.available()) {
    String x;
    long lts = millis();
    //Serial.print("N2_response: ");
    while (millis() - lts < 100) {
      while (MFC_N2.available()) {
        char c = (char)MFC_N2.read();
        x += c;

      }
    }
    //Serial.println(x);
    N2flow = ResponseToFlow(x);
    //Serial.println(N2flow, 3);

  }
}


double ResponseToFlow(String res) {
  res.trim();
  res.remove(0, res.length() - 4);
  long int t = hexToDec(res) * 10.0;
  return t / 32000.0;
}


int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}
