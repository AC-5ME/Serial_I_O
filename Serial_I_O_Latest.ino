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


byte c;
int DATA_CORRUPTED;
byte magicByte;
byte controllerID;
byte packetId;
byte msgByte_low;
byte msgByte_high;
byte crcVal_received;
byte crcVal_calculated;

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


void Warning_parser(uint8_t Warning_code = (msgByte_low))  {        //Warning parser function is working

  switch (Warning_code) {

    case 0x01:

      time_now = millis();

      while (millis() < time_now + period) {

        lcd.setCursor (0, 1);
        lcd.print("      ");
      }

      while (millis() < time_now + period) {

        lcd.setCursor (0, 1);
        lcd.print("Volts:");
      }

      lcd.setCursor (0, 1);
      lcd.print("Volts:");
      break;

    case 0x02:

      time_now = millis();

      while (millis() < time_now + period) {

        lcd.setCursor (10, 0);
        lcd.print("     ");
      }

      while (millis() < time_now + period) {

        lcd.setCursor (10, 0);
        lcd.print("Amps:");
      }

      lcd.setCursor (10, 0);
      lcd.print("Amps:");
      break;

    case 0x04:

      time_now = millis();

      while (millis() < time_now + period) {

        lcd.setCursor (10, 2);
        lcd.print("      ");
      }

      while (millis() < time_now + period) {

        lcd.setCursor (0, 2);
        lcd.print("ESC:");
      }

      lcd.setCursor (10, 2);
      lcd.print("ESC:");
      break;

    case 0x10:
      time_now = millis();

      while (millis() < time_now + period) {

        lcd.setCursor (0, 2);
        lcd.print("      ");
      }

      while (millis() < time_now + period) {

        lcd.setCursor (0, 2);
        lcd.print("Motor:");
      }

      lcd.setCursor (0, 2);
      lcd.print("Motor:");
  }
}

void Error_parser(uint8_t Error_code = (msgByte_low)) {        //Error parser function is working

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

void DisplayData() {     //Send data to diplay

  uint8_t  DataVal = (msgByte_low) + (msgByte_high * 256);

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

    case 0x0a:

      //Run Warning function
      lcd.setCursor (0, 3);
      lcd.print("MOTOR TEMP!");
      Warning_parser();
      break;

    case 0x0b:

      //Run Error function
      Error_parser();
  }
}

void CRC_check () {

  crcVal_calculated = (((controllerID ^ packetId) ^ msgByte_high) ^ msgByte_low);

  if (crcVal_received == crcVal_calculated) {
    dataCorruptionFlag = DATA_GOOD;
    DisplayData();      //Run data display loop

  } else {      //CRC failed

    dataCorruptionFlag = DATA_CORRUPTED;
    DATA_CORRUPTED ++;

    if (DATA_CORRUPTED < 100) {
      lcd.setCursor(16, 3);
      lcd.print ("   ");
      lcd.setCursor(16, 3);
      lcd.print(DATA_CORRUPTED);
    }
  }
}

void processInput ()        // Response parser
  {
        
   switch (state) {           

    case FSM_MAGIC_BYTE:
      // Check if the first byte is the magic byte

      if (c == 0xfe) {      // It is! Go on to the next state
        
        state = FSM_CONTROLLER_ID;
       
      }
      break;
    
 case FSM_CONTROLLER_ID:
        // Check if the second byte is a valid controller
  
        if (c == 0x00) {
          // It is! Go on to the third byte.
Serial.print ("Next byte: ");
    Serial.print (c, HEX);
    Serial.print (" ");
         
          state = FSM_PACKET_ID;

        } else {

          // It isn't, return to beginning
          state = FSM_MAGIC_BYTE;
        }
        break;
        
    case FSM_PACKET_ID:
      // Check if the third byte is a valid packet ID
 
      if (c <= 0x0b) {
        packetId = c;
        state = FSM_BYTE_LOW;
        
      }

      else {

        // It isn't, return to beginning
        state = FSM_MAGIC_BYTE;
      }
      break;

    case FSM_BYTE_LOW:
      // Store the fourth byte and proceed to the fifth

      msgByte_low = c;
      state = FSM_BYTE_HIGH;
      c = Serial.read();

    case FSM_BYTE_HIGH:
      // Store the fifth byte and proceed to the CRC check

      msgByte_high = c;
      state = FSM_CRC_CHECK;
      c = Serial.read();

    case FSM_CRC_CHECK:

      crcVal_received = c;
      void CRC_check();     //Run CRC checker
  }
 }

  
void loop() {         //Main loop
  

  //Request loop goes here

  lcd.setCursor(0, 3);
  lcd.print ("NO DATA");

  //Check to see if anything is available in the serial receive buffer
  while (Serial.available() > 0)
  {
    c = Serial.read ();
    lcd.setCursor(0, 3);      //Clear display warning field
    lcd.print("           ");
    processInput ();
  }
}



 
