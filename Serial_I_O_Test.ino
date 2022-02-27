#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

enum FSM_STATES {     //Byte parser states
  FSM_MAGIC_BYTE,
  FSM_CONTROLLER_ID,
  FSM_PACKET_ID,
  FSM_BYTE_LOW,
  FSM_BYTE_HIGH,
  FSM_CRC_CHECK,
};

enum SERIAL_REQUEST {     //Request loop states
  VOLTAGE,
  CURRENT,
  RPM,
  ESC_TEMP,
  MOTOR_TEMP,
  PWR_REQ,
  PWR_OUT,
  WARNINGS,
  ERRORS,
};

uint32_t time_for_the_next_loop = millis();     //Request loop delay
uint32_t time_for_the_next_message = millis();

uint32_t time_for_the_next_flash = millis();      //Warning flasher

void Request_Loop () {      //Request Loop

  static SERIAL_REQUEST requestState = VOLTAGE;

  uint8_t query1[] = {254, 00, 1, 1};
  uint8_t query2[] = {254, 00, 2, 2};
  uint8_t query3[] = {254, 00, 3, 3};
  uint8_t query4[] = {254, 00, 4, 4};
  uint8_t query5[] = {254, 00, 5, 5};
  uint8_t query6[] = {254, 00, 8, 8};
  uint8_t query7[] = {254, 00, 9, 9};
  uint8_t query8[] = {254, 00, 10, 10};
  uint8_t query9[] = {254, 00, 11, 11};

  uint32_t time_now = millis();

  if (time_now > time_for_the_next_loop) {
    if (time_now > time_for_the_next_message) {
      switch (requestState) {
        case VOLTAGE:
          Serial.write(query1, 4);
          requestState = CURRENT;
          break;

        case CURRENT:
          Serial.write(query2, 4);
          requestState = RPM;
          break;

        case RPM:
          Serial.write(query3, 4);
          requestState = ESC_TEMP;
          break;

        case ESC_TEMP:
          Serial.write(query4, 4);
          requestState = MOTOR_TEMP;
          break;

        case MOTOR_TEMP:
          Serial.write(query5, 4);
          requestState = PWR_REQ;
          break;

        case PWR_REQ:
          Serial.write(query6, 4);
          requestState = PWR_OUT;
          break;

        case PWR_OUT:
          Serial.write(query7, 4);
          requestState = WARNINGS;
          break;

        case WARNINGS:
          Serial.write(query8, 4);
          requestState = ERRORS;
          break;

        case ERRORS:
          Serial.write(query9, 4);
          requestState = VOLTAGE;
          time_for_the_next_loop += 500;
          break;
      }
      time_for_the_next_message += 50;
    }
  }
}

void Process_Input (uint8_t c) {      // Response parser

  static uint8_t controllerId;      //Bytes received
  static uint8_t packetId;
  static uint8_t msgByteHigh;
  static uint8_t msgByteLow;
  static uint8_t crcValReceived;

  static FSM_STATES state = FSM_MAGIC_BYTE;

  switch (state) {
    case FSM_MAGIC_BYTE:
      if (c == 0xfe) {
        state = FSM_CONTROLLER_ID;
      }
      break;

    case FSM_CONTROLLER_ID:
      if (c == 0x00) {
        state = FSM_PACKET_ID;
      } else {
        state = FSM_MAGIC_BYTE;
      }
      break;

    case FSM_PACKET_ID:
      if (c <= 0x0b) {
        packetId = c;
        state = FSM_BYTE_LOW;
      } else {
        state = FSM_MAGIC_BYTE;
      }
      break;

    case FSM_BYTE_LOW:
      msgByteLow = c;
      state = FSM_BYTE_HIGH;
      break;

    case FSM_BYTE_HIGH:
      msgByteHigh = c;
      state = FSM_CRC_CHECK;
      break;

    case FSM_CRC_CHECK:
      crcValReceived = c;
      state = FSM_MAGIC_BYTE;
      CRC_Check (controllerId, packetId, msgByteHigh, msgByteLow, crcValReceived);     //Run CRC checker
      break;
  }
}

void CRC_Check (uint8_t controllerId, uint8_t packetId, uint8_t msgByteHigh, uint8_t msgByteLow, uint8_t crcValReceived) {     //CRC checker
  uint8_t  crcValCalculated = (((controllerId ^ packetId) ^ msgByteHigh) ^ msgByteLow);

  if (crcValReceived == crcValCalculated) {
    uint16_t DataVal;
    DataVal = (msgByteLow) + (msgByteHigh * 256);
    Display_Data (packetId, DataVal);      //Run data display loop

  } else {      //CRC failed
    int DataCorrupted;
    DataCorrupted ++;
    if (DataCorrupted < 100) {
      lcd.setCursor(16, 3);
      lcd.print ("   ");
      lcd.setCursor(16, 3);
      lcd.print(DataCorrupted);
    }
  }
}
void Display_Data (uint8_t packetId, uint16_t DataVal) {     //Display data
  switch (packetId) {
    case 0x01:
      float voltVal = (DataVal / 10.0f);
      lcd.setCursor(2, 1);
      lcd.print("        ");
      lcd.setCursor(3, 1);
      lcd.print(voltVal);
      break;

    case 0x02:
      float currentVal = (DataVal / 10.0f);
      lcd.setCursor(13, 1);
      lcd.print("     ");
      lcd.setCursor(14, 1);
      lcd.print(currentVal);
      break;

    case 0x03:
      Serial.print("[0x03]: packetId: ");
      Serial.println(packetId);
      break;

    case 0x04:
      Serial.print("[0x04]: packetId: ");
      Serial.println(packetId);
      break;

    case 0x05:
      Serial.print("[0x05]: packetId: ");
      Serial.println(packetId);
      break;

    case 0x08:
      Serial.print("[0x08]: packetId: ");
      Serial.println(packetId);
      break;

    case 0x0a:
      Serial.print("[0x0a]: packetId: ");
      Serial.println(packetId);
      break;

    default:
      Serial.print("[DEFAULT]: packetId: ");
      Serial.println(packetId);
      break;
  }
}
/*void Display_Data (uint8_t packetId, uint16_t DataVal) {     //Display data

  switch (packetId) {
    case 0x01:       //Print Volts - only gets this far
      float voltVal = (DataVal / 10.0f);
      lcd.setCursor(2, 1);
      lcd.print("        ");
      lcd.setCursor(3, 1);
      lcd.print(voltVal);
      break;

    case 0x02:       //Print Current
      float currentVal = (DataVal / 10.0f);
      lcd.setCursor(13, 1);
      lcd.print("     ");
      lcd.setCursor(14, 1);
      lcd.print(currentVal);
      break;

    case 0x03:       //Print RPM
      uint16_t rpmValue;
      rpmValue = (DataVal * 10);
      lcd.setCursor(4, 0);
      lcd.print("     ");
      lcd.setCursor(5, 0);
      lcd.print(rpmValue);
      break;

    case 0x04:       //Print ESC temp
      lcd.setCursor(14, 2);
      lcd.print("   ");
      lcd.setCursor(14, 2);
      lcd.print(DataVal);
      break;

    case 0x05:      //Print Motor temp
      lcd.setCursor(6, 2);
      lcd.print("   ");
      lcd.setCursor(6, 2);
      lcd.print(DataVal);
      break;

    case 0x08:      //Print Power
      const int min = 0, max = 1023;
      const int span = max - min, hundred = 100.0;
      uint16_t PwrReqVal;
      uint16_t kWInput;
      PwrReqVal = hundred * (DataVal - min) / span;
      kWInput = ((voltVal * currentVal) / 1000);
      lcd.setCursor(11, 0);
      lcd.print("   ");
      lcd.setCursor(11, 0);
      lcd.print(kWInput);
      lcd.setCursor(16, 0);
      lcd.print("   ");
      lcd.setCursor(16, 0);
      lcd.print(pwrReqVal);
      break;

      /*case 0x0a:     Add flash function later

        //Run Warning function
        WarningCode = (msgByteLow + msgByteHigh)
        Warning_parser(WarningCode);
        break;

        case 0x0b:

        //Run Error function
        ErrorCode = (msgByteLow + msgByteHigh)
        Error_parser(ErrorCode);
        break;
        }
        }*/

void Warning_Parser(uint16_t WarningCode)  {        //Warning_code parser

  uint32_t flash_time_now = millis();

  switch (WarningCode) {
    case 0x0000:      //No warnings
      lcd.setCursor (0, 3);
      lcd.print("                   ");
      break;

    case 0x0001:
      lcd.setCursor (0, 3);
      lcd.print("LOW VOLTAGE!");
      if (flash_time_now > time_for_the_next_flash) {
        lcd.setCursor (0, 1);
        lcd.print("  ");
        lcd.setCursor (0, 3);
        lcd.print("                 ");
      } else {
        lcd.setCursor (0, 1);
        lcd.print("V:");
      }
      time_for_the_next_flash += 500;
      break;

    case 0x0002:
      lcd.setCursor (0, 3);
      lcd.print("HIGH CURRENT!");
      if (flash_time_now > time_for_the_next_flash) {
        lcd.setCursor (11, 1);
        lcd.print("  ");
        lcd.setCursor (0, 3);
        lcd.print("                 ");
      } else {
        lcd.setCursor (11, 1);
        lcd.print("A:");
      }
      time_for_the_next_flash += 500;
      break;

    case 0x0004:
      lcd.setCursor (0, 3);
      lcd.print("ESC TEMP!");
      if (flash_time_now > time_for_the_next_flash) {
        lcd.setCursor (11, 2);
        lcd.print("    ");
        lcd.setCursor (0, 3);
        lcd.print("                ");
      } else {
        lcd.setCursor (11, 2);
        lcd.print("ESC:");
      }
      time_for_the_next_flash += 500;
      break;

    case 0x0010:
      lcd.setCursor (0, 3);
      lcd.print("MOTOR TEMP!");
      if (flash_time_now > time_for_the_next_flash) {
        lcd.setCursor (0, 2);
        lcd.print("      ");
        lcd.setCursor (0, 3);
        lcd.print("               ");
      } else {
        lcd.setCursor (0, 2);
        lcd.print("Motor:");
      }
      time_for_the_next_flash += 500;
      break;
  }
}

void Error_Parser(uint16_t ErrorCode) {        //Error_code parser

  switch (ErrorCode) {
    case 0x0000:      //No errors
      lcd.setCursor (0, 3);
      lcd.print("               ");

    case 0x0001:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("SIGNAL LOST!");
      break;

    case 0x0002:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("NEUTRAL?");
      break;

    case 0x0008:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("MEMORY FAIL!");
      break;

    case 0x0020:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("HAll SENSOR!");
      break;

    case 0x0040:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("12V BUS!");
      break;

    case 0x0080:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("HALL RESET");
      break;

    case 0x0200:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("MOTOR TEMP!");
      break;
  }
}

void setup() {

  lcd.init();
  lcd.backlight();
  Serial.begin(2400);

  lcd.setCursor(0, 0);      // Set the cursor on the X column and Y row.
  lcd.print ("RPM:---");
  lcd.setCursor(0, 1);
  lcd.print ("V:---");
  lcd.setCursor(0, 2);
  lcd.print ("Motor:---");
  lcd.setCursor(9, 2);
  lcd.print ("C");
  lcd.setCursor(11, 0);
  lcd.print("--kW");
  lcd.setCursor(16, 0);
  lcd.print("---%");
  lcd.setCursor(11, 1);
  lcd.print ("A:---");
  lcd.setCursor(11, 2);
  lcd.print ("ESC:---");
  lcd.setCursor(18, 2);
  lcd.print ("C");
}

void loop () {         //Main loop

  //Request_Loop ();

  if (Serial.available() > 0) {
    uint8_t c = Serial.read ();
    Process_Input (c);
  }
}
