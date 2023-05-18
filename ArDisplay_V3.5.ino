#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <buffer.h>
#include <crc.h>
#include <datatypes.h>
#include <VescUart.h>
#include <LCD_I2C.h>

VescUart UART;      //Declarations
File SD_File;
RTC_DS1307 rtc;
LCD_I2C lcd(0x27, 20, 4);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};     //SD logger

bool isVoltageWarning = false;      //Warning flashers
bool isCurrentWarning = false;
bool isESCTempWarning = false;
bool isMotorTempWarning = false;
static bool isFaults = false;

static uint32_t flashTimeout;
static uint32_t time_for_the_next_flash = millis();

int erpmRx = 0;
int voltDisplay = 0;
int ampDisplay = 0;
float WhDisplay = 0.0;
float WhchargedDisplay = 0.0;
int escTempDisplay = 0;
int motorTempDisplay = 0;
float dutyCycleDisplay = 0.0;
int faultCode = 0;

uint32_t next_request = millis();     //Request loop delay

void Get_Data() {
  if (UART.getVescValues()) {
    erpmRx = (UART.data.rpm);
    voltDisplay = (UART.data.inpVoltage);
    ampDisplay = (UART.data.avgInputCurrent);
    WhDisplay = (UART.data.wattHours);
    WhchargedDisplay = (UART.data.wattHoursCharged);
    escTempDisplay = (UART.data.tempMosfet);
    motorTempDisplay = (UART.data.tempMotor);
    dutyCycleDisplay = (UART.data.dutyCycleNow);
    faultCode = (UART.data.error);

    Serial.print ("dutyCycleDisplay: ");
    Serial.println (dutyCycleDisplay);
  } else {

    lcd.setCursor (0, 3);
    lcd.print("NO DATA!            ");
  }
}

void Display_Data(int erpmRx, int voltDisplay, int ampDisplay, float WhDisplay, int escTempDisplay, int motorTempDisplay, int dutyCycleDisplay, int faultCode) {
  uint32_t timeStamp = millis();      //SD card

  int rpmDisplay = (erpmRx / 10);

  lcd.setCursor(4, 0);      //Print RPM
  lcd.print("     ");
  lcd.setCursor(5, 0);
  lcd.print(rpmDisplay);

  SD_File = SD.open("ESC.log", FILE_WRITE);
  SD_File.println();
  SD_File.print("ms: ");
  SD_File.print(timeStamp);
  SD_File.print(" RPM: ");
  SD_File.print(rpmDisplay);
  SD_File.print(", ");
  SD_File.close();

  lcd.setCursor(2, 1);     //Print input volts
  lcd.print("        ");
  lcd.setCursor(3, 1);
  lcd.print(voltDisplay);
  /*
    if (voltDisplay > 0 && voltDisplay < 90 ) {      //Low volt warning   ***Debug  disable***
      isVoltageWarning = true;
      flashTimeout = millis() + 3000;
    }
  */
  SD_File = SD.open("ESC.log", FILE_WRITE);
  SD_File.print("V: ");
  SD_File.print(voltDisplay);
  SD_File.print(", ");
  SD_File.close();

  lcd.setCursor(13, 1);     //Print current
  lcd.print("     ");
  lcd.setCursor(14, 1);
  lcd.print(ampDisplay);

  if (ampDisplay > 350) {      //High current warning
    isCurrentWarning = true;
    flashTimeout = millis() + 3000;
  }

  SD_File = SD.open("ESC.log", FILE_WRITE);
  SD_File.print("A: ");
  SD_File.print(ampDisplay);
  SD_File.print(", ");

  SD_File.print("Wh: ");
  SD_File.print(WhDisplay);
  SD_File.print(", ");

  SD_File.print("WhCh: ");
  SD_File.print(WhchargedDisplay);
  SD_File.print(", ");
  SD_File.close();

  lcd.setCursor(15, 2);      //Print ESC temp
  lcd.print("   ");
  lcd.setCursor(15, 2);
  lcd.print(escTempDisplay);

  if (escTempDisplay > 80) {      //ESC temp warning
    isESCTempWarning = true;
    flashTimeout = millis() + 3000;
  }

  SD_File = SD.open("ESC.log", FILE_WRITE);
  SD_File.print("ESC: ");
  SD_File.print(escTempDisplay);
  SD_File.print(", ");
  SD_File.close();

  lcd.setCursor(6, 2);
  lcd.print("   ");
  lcd.setCursor(6, 2);
  lcd.print(motorTempDisplay);

  if (motorTempDisplay > 110) {      //Motor temp warning
    isMotorTempWarning = true;
    flashTimeout = millis() + 3000;
  }

  SD_File = SD.open("ESC.log", FILE_WRITE);
  SD_File.print("MOTOR: ");
  SD_File.print(motorTempDisplay);
  SD_File.print(", ");
  SD_File.close();

  float throttlePos = (dutyCycleDisplay);
  //char dutyBuffer[5];
  //sprintf(dutyBuffer, "%u%%", dutyCycleDisplay);

  lcd.setCursor(16, 0);      //Print throttle %
  lcd.print("    ");
  lcd.setCursor(16, 0);
  // lcd.print(dutyBuffer);

  lcd.print(dutyCycleDisplay);

  SD_File = SD.open("ESC.log", FILE_WRITE);
  SD_File.print("THROTTLE: ");
  // SD_File.print(dutyBuffer);
  SD_File.print(", ");
  SD_File.close();
}


void Voltage_Flasher() {
  uint32_t now = millis();

  if (isVoltageWarning == true) {       //Motor temperature error flag set?

    isFaults = true;      //Set for Fault_Checker()

    if (now > flashTimeout) {
      isVoltageWarning = false;     //Turn off flasher
      lcd.setCursor(0, 1);
      lcd.print ("V:");

    } else {

      static bool isFlashOn = false;      //Toggle the flash

      if (now > time_for_the_next_flash) {            //Time to rewrite the LDC?
        time_for_the_next_flash += 1000;

        if (isFlashOn == true) {        //Toggle as appropriate
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

    isFaults = true;      //Set for Fault_Checker()

    if (now > flashTimeout) {
      isCurrentWarning = false;
      lcd.setCursor(11, 1);
      lcd.print ("A:");

    } else {
      static bool isFlashOn = false;

      if (now > time_for_the_next_flash) {
        time_for_the_next_flash += 1000;

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

  if (isESCTempWarning == true) {

    isFaults = true;      //Set for Fault_Checker()

    if (now > flashTimeout) {
      isESCTempWarning = false;
      lcd.setCursor(11, 2);
      lcd.print ("ESC:");
    } else {

      static bool isFlashOn = false;

      if (now > time_for_the_next_flash) {
        time_for_the_next_flash += 1000;

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
        time_for_the_next_flash += 1000;

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

void Fault_Flasher() {
  uint32_t now = millis();

  if (isFaults == true) {

    if (now > flashTimeout) {
      isFaults = false;
      lcd.setCursor(0, 3);
      lcd.print ("                    ");
    } else {

      static bool isFlashOn = false;

      if (now > time_for_the_next_flash) {
        time_for_the_next_flash += 1000;

        if (isFlashOn == true) {
          lcd.setCursor (0, 3);
          lcd.print("ESC Fault!          ");
          isFlashOn = false;
        } else {

          lcd.setCursor (0, 3);
          lcd.print("                  ");
          isFlashOn = true;
        }
      }
    }
  }
}

void Fault_Checker() {
  if (faultCode = 0) {     //Check for fault code
    isFaults = false;      //Set for Fault_Checker()
   // lcd.setCursor (0, 3);
   // lcd.print("                    ");
  } else {
    Serial.print ("Fault!");

    isFaults = true;
    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.println("ESC Fault!,");
    SD_File.close();
  }
}

void setup() {
  delay (500);      //Stabilization delay

  Serial.begin(9600);     //Debug
  Serial1.begin(115200);

  UART.setSerialPort(&Serial1);       //Set UART port
  while (!Serial1) {
    ;
  }

  lcd.begin();
  lcd.backlight();

  lcd.setCursor(0, 0);      //X column, Y row
  lcd.print ("RPM: ----");
  lcd.setCursor(0, 1);
  lcd.print ("V: ---");
  lcd.setCursor(0, 2);
  lcd.print ("Motor:---");
  lcd.setCursor(16, 0);
  lcd.print("---%");
  lcd.setCursor(11, 1);
  lcd.print ("A: ---");
  lcd.setCursor(11, 2);
  lcd.print ("ESC:---");
  // lcd.setCursor (0, 3);
  // lcd.print("CHECK THROTTLE    ");

  if (! rtc.begin()) {
    lcd.setCursor (0, 3);
    lcd.print("RTC FAIL!         ");
    delay(2000);
  }

  if (!SD.begin(10)) {      //chipSelect pin - 10 M4 board
    lcd.setCursor (0, 3);
    lcd.print ("SD FAIL!          ");
    delay(2000);
  } else {

    DateTime now = rtc.now();

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.println();
    SD_File.print("--------------------");
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
    SD_File.print("--------------------");
    SD_File.println();
    SD_File.close();
  }
}

void loop () {
  uint32_t time_now = millis();

  if (time_now > next_request) {
    Get_Data();
    Display_Data (erpmRx, voltDisplay, ampDisplay, WhDisplay, escTempDisplay, motorTempDisplay, dutyCycleDisplay, faultCode);
    next_request += 500;
  }

  Voltage_Flasher();
  Current_Flasher();
  ESC_Flasher();
  Motor_Flasher();
  Fault_Flasher();
  Fault_Checker();
}
