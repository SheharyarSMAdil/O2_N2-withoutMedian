void handleSerial() {
  if (Serial.available()) {
    String raw = Serial.readStringUntil('\n');
    raw.trim();

    String first_char = raw.substring(0, 1);
    String last_char = raw.substring(raw.length() - 1, raw.length());

    //Serial.println(first_char +" "+ last_char);
    if (first_char == R_DELIMETER & last_char == R_DELIMETER) {
      String s = raw.substring(1, raw.length() - 1);
      Serial.print("request: "); Serial.println(s);

      int  command_1 ;
      String command_2 = "", data = "";
      if (s.indexOf(M_DELIMETER) > -1)
      {
        command_1 = s.substring(0, s.indexOf(M_DELIMETER)).toInt();
        s.remove(0, s.indexOf(M_DELIMETER) + 1);
        if (s.indexOf(M_DELIMETER) > -1) {
          command_2 = s.substring(0, s.indexOf(M_DELIMETER));
          s.remove(0, s.indexOf(M_DELIMETER) + 1);
          data = s;
        } else {
          command_2 = s;
        }
      } else {
        command_1 = s.toInt();;
      }
      command_2.toUpperCase();
      lcd.clear();
      lcd.print(command_1);
      lcd.setCursor(0, 1);
      lcd.print(command_2);
      delay(2000);
      switch (command_1)
      {
        //////////////////////////////////CASE 100 (CONNECTION)/////////////////
        case 99:
          {
            if (command_2.indexOf(R_HANDSHAKE) > -1)
            {
              //resp = "@connect:@";
              // Serial.println("@connect:@");
              WriteS(99, W_CONNECT, _blank);
              softConnect = true;
              delay(500);
              String type = uart_analyser ? "UART" : "PM11E";
              WriteS(100, W_ANALYSER_TYPE , type);
            }
            break;
          }
        case 100:
          {
            if (command_2 == "O2") {
              MFC_O2.println(getRequestString(data.toFloat()));
            } else if (command_2 == "N2")  {
              MFC_N2.println(getRequestString(data.toFloat()));
            } else if (command_2 == R_SET_O2)  {
              forced_o2 = false;
              itt_count = 0;
              float to_set = data.toFloat();
              target_o2 = to_set;
              float O2_flow = to_set / 100.0 * TOTAL_FLOW - (mfc_error / 100.0 * TOTAL_FLOW);
              float N2_flow = (100 - to_set) / 100.0 * TOTAL_FLOW + (mfc_error / 100.0 * TOTAL_FLOW);

              
              N2_flow > 10 ? 10 : N2_flow;
              O2_flow > 10 ? 10 : O2_flow; 
  
              Serial.println(N2_flow);
              Serial.println(O2_flow);
              MFC_N2.println(getRequestString(N2_flow));
              MFC_O2.println(getRequestString(O2_flow));

              set_target = true;
              lt2 = millis();

            } else if (command_2 == R_SET_0)  {
              if (uart_analyser) {
                O2_Serial.print(zero_str);
              } else {
                set_PM11e_zero();
              }
            }  else if (command_2 == R_SET_100)  {

              if (uart_analyser) {
                O2_Serial.print(span_str);
              } else {
                set_PM11e_span();
              }
            } else if (command_2 == R_OFF_ALL)  {
              MFC_N2.println(getRequestString(FLOW_OFF));
              MFC_O2.println(getRequestString(FLOW_OFF));
              set_target = false;
            } else if (command_2 == R_PASS_O2)  {
              Serial.println("passing o2");
              MFC_O2.println(getRequestString(TOTAL_FLOW > 10 ? 10 : TOTAL_FLOW));
              MFC_N2.println(getRequestString(FLOW_OFF));
            } else if (command_2 == R_PASS_N2)  {
              Serial.println("passing n2");
              MFC_O2.println(getRequestString(FLOW_OFF));
              MFC_N2.println(getRequestString(TOTAL_FLOW > 10 ? 10 : TOTAL_FLOW));
            }
            else if (command_2 == R_SET_TOTALFLOW)  {
              TOTAL_FLOW = data.toFloat();
             // eeW(T_FLOW,TOTAL_FLOW);
              EEPROM.put(E_TOTAL_FLOW, TOTAL_FLOW);
            }
            else if (command_2 == R_RESET)  {
              O2_Serial.print(reset_str);
            } else {
              Serial.println("wrong command");
            }
            break;
          }
      }
    }
  }

}

String getRequestString(float flow) {
  uint16_t x = round(flow / 10.0 * 32000);
  //Serial.print("----------"); Serial.println(x);

  String val = String(x, HEX);
  Serial.println(val);
  int val_length = val.length();
  while (val_length < 4) {
    val = "0" + val;
    val_length = val.length();
  }
  val.toUpperCase();
  //Serial.println(int_str + val + "\r\n");
  return int_str + val + "\r\n";

}

String complement_flow(float flow) {
  uint16_t x = round((1 - flow) / 10.0 * 32000);
  String val = String(x, HEX);
  int val_length = val.length();
  while (val_length < 4) {
    val = "0" + val;
    val_length = val.length();
  }
  val.toUpperCase();
  return int_str + val + "\r\n";
}


void WriteS(int _case, String _command_1, String _command_2) {
  String R_Command = "";
  if (_command_1 != _blank)
  {
    R_Command =  W_DELIMETER + String(_case) + M_DELIMETER + _command_1 + M_DELIMETER + W_DELIMETER;
  }

  if (_command_2 != _blank)
  {
    R_Command = W_DELIMETER + String(_case) + M_DELIMETER + _command_1 + M_DELIMETER + _command_2 + M_DELIMETER + W_DELIMETER;
  }
  Serial.println(R_Command);
}
