#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

enum FSM_STATES {     //Byte parser
  FSM_MAGIC_BYTE,
  FSM_CONTROLLER_ID,
  FSM_PACKET_ID,
  FSM_BYTE_LOW,
  FSM_BYTE_HIGH,
  FSM_CRC_CHECK,
};

uint8_t controller_Id;      //Bytes received
uint8_t packet_Id;
uint8_t msgByte_high;
uint8_t msgByte_low;
uint8_t crcVal_received;

enum SERIAL_REQUEST {     //Display data
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

uint16_t blinkPeriod = 500;     // Warning delay
unsigned long time_now = 0;

uint8_t query1[] = {254, 00, 1, 1};     //Send request arrays
uint8_t query2[] = {254, 00, 2, 2};
uint8_t query3[] = {254, 00, 3, 3};
uint8_t query4[] = {254, 00, 4, 4};
uint8_t query5[] = {254, 00, 5, 5};
uint8_t query6[] = {254, 00, 8, 8};
uint8_t query7[] = {254, 00, 9, 9};
uint8_t query8[] = {254, 00, 10, 10};
uint8_t query9[] = {254, 00, 11, 11};

void CRC_check (uint8_t controller_Id, uint8_t packet_Id, uint8_t msgByte_high, uint8_t msgByte_low, uint8_t crcVal_received) {     //CRC checker

  uint8_t  crcVal_calculated = (((controller_Id ^ packet_Id) ^ msgByte_high) ^ msgByte_low);

  if (crcVal_received == crcVal_calculated) {
    uint16_t DataVal;
    DataVal = (msgByte_low) + (msgByte_high * 256);
    DisplayData(packet_Id, DataVal);      //Run data display loop

  } else {      //CRC failed
    int Data_Corrupted;
    Data_Corrupted ++;

    if (Data_Corrupted < 100) {
      lcd.setCursor(16, 3);
      lcd.print ("   ");
      lcd.setCursor(16, 3);
      lcd.print(Data_Corrupted);
    }
  }
}

void Warning_parser(uint16_t Warning_code)  {        //Warning_code parser

  uint32_t time_for_the_next_loop = millis();
  uint32_t time_for_the_next_message = millis();

  switch (Warning_code) {

    case 0x0001:
      time_now = millis();

      if (time_now > time_for_the_next_loop) {

        lcd.setCursor (0, 1);
        lcd.print("      ");
      }
      time_for_the_next_loop += 100;

      if (time_now > time_for_the_next_loop) {

        lcd.setCursor (0, 1);
        lcd.print("Volts:");
      }
      time_for_the_next_loop += 100;

      lcd.setCursor (0, 1);
      lcd.print("Volts:");
      break;

    case 0x0002:
      time_now = millis();

      if (time_now > time_for_the_next_loop) {


        lcd.setCursor (10, 0);
        lcd.print("     ");
      }
      time_for_the_next_loop += 100;

      if (time_now > time_for_the_next_loop) {

        lcd.setCursor (10, 0);
        lcd.print("Amps:");
      }
      time_for_the_next_loop += 100;

      lcd.setCursor (10, 0);
      lcd.print("Amps:");
      break;

    case 0x0004:
      time_now = millis();

      if (time_now > time_for_the_next_loop) {
        lcd.setCursor (10, 2);
        lcd.print("      ");
      }
      time_for_the_next_loop += 100;
      if (time_now > time_for_the_next_loop) {

        lcd.setCursor (0, 2);
        lcd.print("ESC:");
      }
      time_for_the_next_loop += 100;
      lcd.setCursor (10, 2);
      lcd.print("ESC:");
      break;

    case 0x0010:
      time_now = millis();

      if (time_now > time_for_the_next_loop) {

        lcd.setCursor (0, 2);
        lcd.print("      ");
      }
      time_for_the_next_loop += 100;
      if (time_now > time_for_the_next_loop) {

        lcd.setCursor (0, 2);
        lcd.print("Motor:");
      }
      time_for_the_next_loop += 100;

      lcd.setCursor (0, 2);
      lcd.print("Motor:");
  }
}

void Error_parser(uint16_t Error_code) {        //Error_code parser

  switch (Error_code) {
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
  }
}

void DisplayData (uint8_t packet_Id, uint16_t DataVal) {     //Display data

  switch (packet_Id) {
    case 0x01:       //Print Volts - only gets this far
      float voltVal = (DataVal / 10);
      lcd.setCursor(2, 1);
      lcd.print("        ");
      lcd.setCursor(3, 1);
      lcd.print(voltVal);
      break;

    case 0x02:       //Print Current
      float currentVal = (DataVal / 10);
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

    case 0x08:      //Print Power Requested
      const int min = 0, max = 1023;
      const int span = max - min, hundred = 100.0;
      uint16_t PwrReq_Val;
      PwrReq_Val = hundred * (DataVal - min) / span;
      lcd.setCursor(17, 1);
      lcd.print("   ");
      lcd.setCursor(17, 1);
      lcd.print(PwrReq_Val);
      break;

      /*case 0x0a:     Add flash function later

        //Run Warning function
        Warning_code = (msgByte_low + msgByte_high)
        Warning_parser(Warning_code);
        break;

        case 0x0b:

        //Run Error function
        Error_code = (msgByte_low + msgByte_high)
        Error_parser(Error_code);
        break;  */
  }
}

void ProcessInput (uint8_t c) {      // Response parser

  static FSM_STATES state = FSM_MAGIC_BYTE;

  switch (state) {
    case FSM_MAGIC_BYTE:       // Check if the first byte is the magic byte
      if (c == 0xfe) {
        state = FSM_CONTROLLER_ID;
      }
      break;

    case FSM_CONTROLLER_ID:     // Check if the second byte is a valid controller
      if (c == 0x00) {
        state = FSM_PACKET_ID;
      } else {
        state = FSM_MAGIC_BYTE;      // It isn't, return to beginning
      }
      break;

    case FSM_PACKET_ID:      // Check if the third byte is a valid packet ID
      if (c <= 0x0b) {
        packet_Id = c;
        state = FSM_BYTE_LOW;
      } else {
        state = FSM_MAGIC_BYTE;     // It isn't, return to beginning
      }
      break;

    case FSM_BYTE_LOW:
      msgByte_low = c;      // Store the fourth byte and proceed to the fifth
      state = FSM_BYTE_HIGH;
      break;

    case FSM_BYTE_HIGH:
      msgByte_high = c;     // Store the fifth byte and proceed to the CRC check
      state = FSM_CRC_CHECK;
      break;

    case FSM_CRC_CHECK:
      crcVal_received = c;
      CRC_check (controller_Id, packet_Id, msgByte_high, msgByte_low, crcVal_received);     //Run CRC checker
      state = FSM_MAGIC_BYTE;
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
  lcd.setCursor(11, 0);
  lcd.print ("Req%:---");
  lcd.setCursor(11, 1);
  lcd.print ("A:---");
  lcd.setCursor(11, 2);
  lcd.print ("ESC:---");
}

void loop () {         //Main loop

  //Request loop goes here

  if (Serial.available() > 0) {
    uint8_t c = Serial.read ();
    ProcessInput (c);
  }
}
