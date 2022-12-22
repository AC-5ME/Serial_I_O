#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip

File SD_File;      //SD card logger

RTC_DS1307 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

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

bool isVoltageWarning = false;      //Warning flashers
bool isCurrentWarning = false;
bool isESCWarning = false;
bool isMotorTempWarning = false;
static bool isNoWarnings = false;
static bool isNoErrors = false;
static uint32_t flashTimeout;
static uint32_t time_for_the_next_flash = millis();

void Request_Loop();      //Function declarations
void Process_Input(uint8_t c);
void CRC_Check(uint8_t controllerId, uint8_t packetId, uint8_t msgByteHigh, uint8_t msgByteLow, uint8_t crcValReceived);
void Display_Data(uint8_t packetId, uint8_t msgByteHigh, uint8_t msgByteLow, uint16_t DataVal);
void Warning_Parser(uint16_t WarningCode);
void Error_Parser(uint16_t ErrorCode);
void Voltage_Flasher();
void Current_Flasher();
void ESC_Flasher();
void Motor_Flasher();

void Request_Loop() {      //Request Loop
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
        case VOLTAGE: {
            Serial1.write(query1, 4);
            requestState = CURRENT;
          } break;

        case CURRENT: {
            Serial1.write(query2, 4);
            requestState = RPM;
          } break;

        case RPM: {
            Serial1.write(query3, 4);
            requestState = ESC_TEMP;
          } break;

        case ESC_TEMP: {
            Serial1.write(query4, 4);
            requestState = MOTOR_TEMP;
          } break;

        case MOTOR_TEMP: {
            Serial1.write(query5, 4);
            requestState = PWR_REQ;
          } break;

        case PWR_REQ: {
            Serial1.write(query6, 4);
            requestState = PWR_OUT;
          } break;

        case PWR_OUT: {
            Serial1.write(query7, 4);
            requestState = WARNINGS;
          } break;

        case WARNINGS: {
            Serial1.write(query8, 4);
            requestState = ERRORS;
          } break;

        case ERRORS: {
            Serial1.write(query9, 4);
            requestState = VOLTAGE;
            time_for_the_next_loop += 100;
          } break;
      }
      time_for_the_next_message += 100;
    }
  }
}

void Process_Input(uint8_t c) {      // Response parser
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
      } break;

    case FSM_CONTROLLER_ID:
      if (c == 0x00) {
        state = FSM_PACKET_ID;
      } else {
        state = FSM_MAGIC_BYTE;
      } break;

    case FSM_PACKET_ID:
      if (c <= 0x0b) {
        packetId = c;
        state = FSM_BYTE_LOW;
      } else {
        state = FSM_MAGIC_BYTE;
      }  break;

    case FSM_BYTE_LOW: {
        msgByteLow = c;
        state = FSM_BYTE_HIGH;
      } break;

    case FSM_BYTE_HIGH: {
        msgByteHigh = c;
        state = FSM_CRC_CHECK;
      } break;

    case FSM_CRC_CHECK: {
        crcValReceived = c;
        state = FSM_MAGIC_BYTE;
        CRC_Check (controllerId, packetId, msgByteHigh, msgByteLow, crcValReceived);     //Run CRC checker
      } break;
  }
}

void CRC_Check (uint8_t controllerId, uint8_t packetId, uint8_t msgByteHigh, uint8_t msgByteLow, uint8_t crcValReceived) {     //CRC checker
  uint8_t  crcValCalculated = (((controllerId ^ packetId) ^ msgByteHigh) ^ msgByteLow);

  static uint8_t DataCorrupted;

  if (crcValReceived == crcValCalculated) {
    uint16_t DataVal = (msgByteLow) + (msgByteHigh * 256);
    Display_Data (packetId, msgByteHigh, msgByteLow, DataVal);      //Data display function

  } else {      //CRC failed
    DataCorrupted ++;
    if (DataCorrupted < 100) {
      lcd.setCursor(18, 3);
      lcd.print ("  ");
      lcd.setCursor(18, 3);
      lcd.print(DataCorrupted);

      SD_File = SD.open("ESC.log", FILE_WRITE);
      SD_File.print("CRC FAIL: ");
      SD_File.print(DataCorrupted);
      SD_File.println(", ");
      SD_File.close();
    }
  }
}

void Display_Data (uint8_t packetId, uint8_t msgByteHigh, uint8_t msgByteLow, uint16_t DataVal) {     //Display data
  const int min = 128;    //Power request % conversion
  const int max = 230;
  const int span = max - min;

  float alpha = 0.3;     //Current smoothing factor - 0.5 does little, lower is smoother

  static int voltVal;
  static int currentVal;
  static int filterCurrentVal;
  static int kWInput;

  uint16_t WarningCode = (msgByteHigh) << 8 | msgByteLow;
  uint16_t ErrorCode = (msgByteHigh) << 8 | msgByteLow;

  uint32_t timeStamp = millis();      //SD card

  switch (packetId) {
    case 0x01: {       //Print Volts
        voltVal = (DataVal / 10.0f);
        lcd.setCursor(2, 1);
        lcd.print("        ");
        lcd.setCursor(3, 1);
        lcd.print(voltVal);

        SD_File = SD.open("ESC.log", FILE_WRITE);
        SD_File.print("ms: ");
        SD_File.print(timeStamp);
        SD_File.print(", ");
        SD_File.print("V: ");
        SD_File.print(voltVal);
        SD_File.print(", ");
        SD_File.close();

        if (voltVal > 75 && voltVal < 90 ) {      //Low volt warning
          WarningCode = 0001;
          Warning_Parser(WarningCode);
        }
      } break;

    case 0x02: {      //Print Current
        currentVal = (DataVal / 10.0f);
        filterCurrentVal = alpha * (currentVal) + (1 - alpha) * filterCurrentVal;
        lcd.setCursor(13, 1);
        lcd.print("     ");
        lcd.setCursor(14, 1);
        lcd.print(filterCurrentVal);

        SD_File = SD.open("ESC.log", FILE_WRITE);
        SD_File.print("A: ");
        SD_File.print(filterCurrentVal);
        SD_File.print(", ");
        SD_File.close();
      } break;

    case 0x03: {       //Print RPM
        uint16_t rpmVal = (DataVal * 10);
        lcd.setCursor(4, 0);
        lcd.print("     ");
        lcd.setCursor(5, 0);
        lcd.print(rpmVal);

        SD_File = SD.open("ESC.log", FILE_WRITE);
        SD_File.print("RPM: ");
        SD_File.print(rpmVal);
        SD_File.print(", ");
        SD_File.close();
      } break;

    case 0x04: {       //Print ESC temp
        lcd.setCursor(15, 2);
        lcd.print("   ");
        lcd.setCursor(15, 2);
        lcd.print(DataVal);

        SD_File = SD.open("ESC.log", FILE_WRITE);
        SD_File.print("ESC(C): ");
        SD_File.print(DataVal);
        SD_File.print(", ");
        SD_File.close();
      } break;

    case 0x05: {      //Print Motor temp
        lcd.setCursor(6, 2);
        lcd.print("   ");
        lcd.setCursor(6, 2);
        lcd.print(DataVal);

        SD_File = SD.open("ESC.log", FILE_WRITE);
        SD_File.print("MOTOR(C): ");
        SD_File.print(DataVal);
        SD_File.print(", ");
        SD_File.close();
      } break;

    case 0x08: {      //Print Power requested
        if (DataVal >= 128 && DataVal <= 230) {
          int pwrReqVal = ((DataVal - min) / (float)span) * 100;
          lcd.setCursor(16, 0);
          lcd.print("   ");
          lcd.setCursor(16, 0);
          lcd.print(pwrReqVal);

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.print("%PWR: ");
          SD_File.print(pwrReqVal);
          SD_File.println(", ");
          SD_File.close();
        }
        else {
          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.print("%PWR: ---");
          SD_File.println(", ");
          SD_File.close();
        }
      } break;

    case 0x0a: {      //Warning function
        Warning_Parser(WarningCode);
      } break;

    case 0x0b: {       //Error function
        Error_Parser(ErrorCode);
      } break;
  }

  kWInput = ((voltVal * filterCurrentVal) / 100);
  lcd.setCursor(11, 0);
  lcd.print("  ");
  lcd.setCursor(11, 0);
  lcd.print(kWInput);
}

void Warning_Parser(uint16_t WarningCode)  {        //Warning_code parser
  switch (WarningCode) {
    case 0x0000: {      //No warnings
        isNoWarnings = true;
        if (isNoWarnings && isNoErrors == true) {
          lcd.setCursor (0, 3);
          lcd.print("                  ");
        } break;
      }
    case 0x0001: {
        isVoltageWarning = true;
        flashTimeout = millis() + 3000;
      } break;

    case 0x0002: {
        if (millis() > 6000) {      //Wait to stabilize
          isCurrentWarning = true;
          flashTimeout = millis() + 3000;
        }
      } break;

    case 0x0004: {
        isESCWarning = true;
        flashTimeout = millis() + 3000;
      } break;

    case 0x0010: {
        isMotorTempWarning = true;
        flashTimeout = millis() + 3000;
      } break;
  }
}

void Error_Parser(uint16_t ErrorCode) {        //Error_code parser
  switch (ErrorCode) {
    case 0x0000: {     //No errors
        isNoErrors = true;
        if (isNoWarnings && isNoErrors == true) {
          lcd.setCursor (0, 3);
          lcd.print("                  ");
        } break;

      case 0x0001: {
          lcd.setCursor (0, 3);
          lcd.print("SIGNAL LOST!      ");

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.println("SIGNAL LOST!,");
          SD_File.close();
        } break;

      case 0x0002: {
          lcd.setCursor (0, 3);
          lcd.print("NEUTRAL?          ");

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.println("NEUTRAL?,");
          SD_File.close();
        } break;

      case 0x0008: {
          lcd.setCursor (0, 3);
          lcd.print("MEMORY FAIL!     ");

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.println("MEMORY FAIL!,");
          SD_File.close();
        } break;

      case 0x0020: {
          lcd.setCursor (0, 3);
          lcd.print("HAll SENSOR!      ");

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.println("HALL SENSOR!,");
          SD_File.close();
        } break;

      case 0x0040: {
          lcd.setCursor (0, 3);
          lcd.print("12V BUS!          ");

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.println("12V BUS!,");
          SD_File.close();
        } break;

      case 0x0080: {
          lcd.setCursor (0, 3);
          lcd.print("HALL RESET!       ");

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.println("HALL RESET!,");
          SD_File.close();
        } break;

      case 0x0200: {
          lcd.setCursor (0, 3);
          lcd.print("MOTOR TEMP SENSOR!");

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.println("MOTOR TEMP SENSOR!,");
          SD_File.close();
        } break;
      }
  }
}

void Voltage_Flasher() {
  uint32_t now = millis();

  if (isVoltageWarning == true) {       // Check if the motor temperature error flag is set

    if (now > flashTimeout) {
      isVoltageWarning = false;     //Turn off flasher
      lcd.setCursor(0, 1);
      lcd.print ("V:");

    } else {

      static bool isFlashOn = false;      // Set a boolean to toggle the flash

      if (now > time_for_the_next_flash) {            // Check if enough time has gone by to need to rewrite the LDC
        time_for_the_next_flash += 500;

        if (isFlashOn == true) {        // Toggle as appropriate
          lcd.setCursor (0, 1);
          lcd.print("V:");
          lcd.setCursor (0, 3);
          lcd.print("LOW VOLTAGE!      ");
          isFlashOn = false;

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.println("LOW VOLTAGE!,");
          SD_File.close();

        } else {
          lcd.setCursor (0, 1);
          lcd.print("  ");
          lcd.setCursor (0, 3);
          lcd.print("                  ");
          isFlashOn = true;
        }
      }
    }
  }
}

void Current_Flasher() {
  uint32_t now = millis();

  if (isCurrentWarning == true) {

    if (now > flashTimeout) {
      isCurrentWarning = false;
      lcd.setCursor(11, 1);
      lcd.print ("A:");

    } else {
      static bool isFlashOn = false;

      if (now > time_for_the_next_flash) {
        time_for_the_next_flash += 500;

        if (isFlashOn == true) {
          lcd.setCursor(11, 1);
          lcd.print ("A:");
          lcd.setCursor (0, 3);
          lcd.print("HIGH CURRENT!     ");
          isFlashOn = false;

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.println("HIGH CURRENT!,");
          SD_File.close();

        } else {
          lcd.setCursor (11, 1);
          lcd.print("  ");
          lcd.setCursor (0, 3);
          lcd.print("                  ");
          isFlashOn = true;
        }
      }
    }
  }
}

void ESC_Flasher() {
  uint32_t now = millis();

  if (isESCWarning == true) {

    if (now > flashTimeout) {
      isESCWarning = false;
      lcd.setCursor(11, 2);
      lcd.print ("ESC:");

    } else {
      static bool isFlashOn = false;

      if (now > time_for_the_next_flash) {
        time_for_the_next_flash += 500;

        if (isFlashOn == true) {
          lcd.setCursor(11, 2);
          lcd.print ("ESC:");
          lcd.setCursor (0, 3);
          lcd.print("ESC TEMP!         ");
          isFlashOn = false;

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.println("ESC TEMP!,");
          SD_File.close();

        } else {
          lcd.setCursor (11, 2);
          lcd.print("    ");
          lcd.setCursor (0, 3);
          lcd.print("                  ");
          isFlashOn = true;
        }
      }
    }
  }
}

void Motor_Flasher() {
  uint32_t now = millis();

  if (isMotorTempWarning == true) {

    if (now > flashTimeout) {
      isMotorTempWarning = false;
      lcd.setCursor(0, 2);
      lcd.print ("Motor:");

    } else {
      static bool isFlashOn = false;

      if (now > time_for_the_next_flash) {
        time_for_the_next_flash += 500;

        if (isFlashOn == true) {
          lcd.setCursor(0, 2);
          lcd.print ("Motor:");
          lcd.setCursor (0, 3);
          lcd.print("MOTOR TEMP!       ");
          isFlashOn = false;

          SD_File = SD.open("ESC.log", FILE_WRITE);
          SD_File.println("MOTOR TEMP!,");
          SD_File.close();

        } else {
          lcd.setCursor (0, 2);
          lcd.print("      ");
          lcd.setCursor (0, 3);
          lcd.print("                  ");
          isFlashOn = true;
        }
      }
    }
  }
}

void setup() {
  lcd.begin(20, 4);

  Serial1.begin(2400);

  lcd.setCursor(0, 0);      // Set the cursor on the X column and Y row.
  lcd.print ("RPM: ----");
  lcd.setCursor(0, 1);
  lcd.print ("V: ---");
  lcd.setCursor(0, 2);
  lcd.print ("Motor:---");
  lcd.setCursor(9, 2);
  lcd.print ("C");
  lcd.setCursor(11, 0);
  lcd.print("- kW");
  lcd.setCursor(16, 0);
  lcd.print("---%");
  lcd.setCursor(11, 1);
  lcd.print ("A: ---");
  lcd.setCursor(11, 2);
  lcd.print ("ESC:---");
  lcd.setCursor(18, 2);
  lcd.print ("C");
  lcd.setCursor (0, 3);
  lcd.print("CHECK THROTTLE    ");

  if (! rtc.begin()) {
    lcd.setCursor (0, 3);
    lcd.print("RTC FAIL!         ");
  }

  if (!SD.begin(2)) {      //chipSelect pin - 2 Every, 10 Uno
    lcd.setCursor (0, 3);
    lcd.print("SD FAIL!          ");
  } else {
    DateTime now = rtc.now();

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.println();
    SD_File.print(now.year(), DEC);
    SD_File.print('/');
    SD_File.print(now.month(), DEC);
    SD_File.print('/');
    SD_File.print(now.day(), DEC);
    SD_File.print(" (");
    SD_File.print(daysOfTheWeek[now.dayOfTheWeek()]);
    SD_File.print(") ");
    SD_File.print(now.hour(), DEC);
    SD_File.print(':');
    SD_File.print(now.minute(), DEC);
    SD_File.print(':');
    SD_File.print(now.second(), DEC);
    SD_File.println();
    SD_File.close();
  }
}

void loop () {         //Main loop
  if (millis() > 2000) {
    Request_Loop ();
    Voltage_Flasher();
    Current_Flasher();
    ESC_Flasher();
    Motor_Flasher();

    if (Serial1.available() > 0) {
      uint8_t c = Serial1.read ();
      Process_Input (c);
    }
  }
}
