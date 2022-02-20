#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

enum FSM_STATES {     //Byte parser
  FSM_MAGIC_BYTE,
  FSM_CONTROLLER_ID,
  FSM_PACKET_ID,
  FSM_BYTE_LOW,
  FSM_BYTE_HIGH,
  FSM_CRC_CHECK,
};
FSM_STATES state = FSM_MAGIC_BYTE;

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

enum DATA_FLAGS {
  NO_INFORMATION,
  DATA_GOOD,
};
DATA_FLAGS dataCorruptionFlag = NO_INFORMATION;

int DATA_CORRUPTED;
byte magicByte;
byte controllerID;
byte packetId;
byte msgByte_low;
byte msgByte_high;
byte crcVal_received;

static uint32_t time_for_the_next_message = millis();     // Request loop delay
static uint32_t time_for_the_next_loop = millis();

int period = 500;     // Warning delay
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

char value[3];

void setup() {

  lcd.init();
  lcd.backlight();
  Serial.begin(2400);

  lcd.setCursor(0, 0);      // Set the cursor on the X column and Y row.
  lcd.print ("RPM:-");
  lcd.setCursor(0, 1);
  lcd.print ("Volts:-");
  lcd.setCursor(0, 2);
  lcd.print ("Motor:-");
  lcd.setCursor(10, 0);
  lcd.print ("Amps:-");
  lcd.setCursor(10, 1);
  lcd.print ("PwrReq:-");
  lcd.setCursor(10, 2);
  lcd.print ("ESC:-");
  lcd.setCursor(0, 3);
  lcd.print ("NO DATA");
}

void Warning_parser(uint8_t Warning_code = (msgByte_low))  {        //Warning_code values are good

  switch (Warning_code) {

    case 0x01:
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

    case 0x02:
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

    case 0x04:
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

    case 0x10:
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

void Error_parser(uint8_t Error_code = (msgByte_low)) {        //Error_code vales are good

  switch (Error_code) {

    case 0x02:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("NEUTRAL?");
      break;

    case 0x08:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("MEMORY FAIL!");
      break;

    case 0x20:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("HAll SENSOR!");
      break;

    case 0x40:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("12V POWER!");
      break;

    case 0x80:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("HALL RESET");
      break;

    case 0x00:
      lcd.setCursor (0, 3);
      lcd.print("               ");
      lcd.setCursor (0, 3);
      lcd.print("MOTOR TEMP!");
  }
}

void DisplayData (uint8_t  DataVal = (msgByte_low) + (msgByte_high * 256)) {     //Send data to display

  switch (packetId) {
    case 0x01:

      //Print Volts
      lcd.setCursor(6, 1);
      lcd.print("    ");
      lcd.setCursor(7, 1);
      lcd.print(DataVal);
      break;

    case 0x02:

      //Print Current
      lcd.setCursor(15, 0);
      lcd.print("   ");
      lcd.setCursor(15, 0);
      lcd.print(DataVal);
      break;

    case 0x03:

      //Print RPM
      int RPM;
      RPM = (DataVal * 10);
      lcd.setCursor(4, 0);
      lcd.print("   ");
      lcd.setCursor(4, 0);
      lcd.print(RPM);
      break;

    case 0x04:

      //Print ESC temp
      lcd.setCursor(14, 2);
      lcd.print("   ");
      lcd.setCursor(14, 2);
      lcd.print(DataVal);
      break;

    case 0x05:

      //Print Motor temp
      lcd.setCursor(6, 2);
      lcd.print("   ");
      lcd.setCursor(6, 2);
      lcd.print(DataVal);
      break;

    case 0x08:

      //Print Power Requested
      const int min = 0, max = 1023;
      const int span = max - min, hundred = 100.0;
      int PwrReq_Val;
      PwrReq_Val = hundred * (DataVal - min) / span;
      lcd.setCursor(17, 1);
      lcd.print("   ");
      lcd.setCursor(17, 1);
      lcd.print(PwrReq_Val);
      break;

      /*case 0x0a:     Add flash function later

        //Run Warning function
        lcd.setCursor (0, 3);
        lcd.print("MOTOR TEMP!");
        Warning_parser();
        break;

        case 0x0b:

        //Run Error function
        Error_parser();*/
  }
}

void CRC_check (byte c) {     //CRC check

  byte crcVal_calculated = (((controllerID ^ packetId) ^ msgByte_high) ^ msgByte_low);

  if (crcVal_received == crcVal_calculated) {

    DisplayData(c);      //Run data display loop

  } else {      //CRC failed

    DATA_CORRUPTED ++;

    if (DATA_CORRUPTED < 100) {
      lcd.setCursor(16, 3);
      lcd.print ("   ");
      lcd.setCursor(16, 3);
      lcd.print(DATA_CORRUPTED);
    }
  }
}

void processInput (byte c) {      // Response parser

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
        packetId = c;
        Serial.print ("Next byte: ");
      Serial.print (packetId, HEX);
      Serial.print (" ");
        state = FSM_BYTE_LOW;
      }

      else {
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
      CRC_check(c);     //Run CRC checker
  }
}

void loop () {         //Main loop

  //Request loop goes here

  lcd.setCursor(0, 3);
  lcd.print ("NO DATA");

  while (Serial.available() > 0) {   //Check to see if anything is available in the serial receive buffer

    byte c = Serial.read ();

    lcd.setCursor(0, 3);      //Clear display warning field
    lcd.print("           ");
    processInput (c);
  }
}
