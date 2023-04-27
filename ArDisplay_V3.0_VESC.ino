#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <VescUart.h>
VescUart UART;

hd44780_I2Cexp lcd; //Declare lcd object: auto locate & auto config expander chip

File SD_File;      //SD card declaration

RTC_DS1307 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};     //SD logger

bool isVoltageWarning = false;      //Warning flashers
bool isCurrentWarning = false;
bool isESCTempWarning = false;
bool isMotorTempWarning = false;
static bool isNoWarnings = false;
static bool isNoErrors = false;
static uint32_t flashTimeout;
static uint32_t time_for_the_next_flash = millis();

void Display_Data() {
  uint32_t timeStamp = millis();      //SD card

  if (UART.getVescValues()) {     //Acquire data from VESC

    lcd.setCursor(4, 0);      //Print RPM
    lcd.print("     ");
    lcd.setCursor(5, 0);
    lcd.print(UART.data.rpm);

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.println();
    SD_File.print("ms: ");
    SD_File.print(timeStamp);
    SD_File.print("RPM: ");
    SD_File.print(UART.data.rpm);
    SD_File.print(", ");
    SD_File.close();

    lcd.setCursor(2, 1);     //Print input volts
    lcd.print("        ");
    lcd.setCursor(3, 1);
    lcd.print(UART.data.inpVoltage);

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("V: ");
    SD_File.print(UART.data.inpVoltage);
    SD_File.print(", ");
    SD_File.close();

    lcd.setCursor(13, 1);     //Print current
    lcd.print("     ");
    lcd.setCursor(14, 1);
    lcd.print(UART.data.avgInputCurrent);

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("A: ");
    SD_File.print(UART.data.avgInputCurrent);
    SD_File.print(", ");
    SD_File.close();

    SD_File.print("Ah: ");
    SD_File.print(UART.data.ampHours);
    SD_File.print(", ");
    SD_File.close();

    lcd.setCursor(15, 2);      //Print ESC temp
    lcd.print("   ");
    lcd.setCursor(15, 2);
    lcd.print(UART.data.tempMosfet);

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("ESC: ");
    SD_File.print(UART.data.tempMosfet);
    SD_File.print(", ");
    SD_File.close();

    lcd.setCursor(6, 2);      //Print motor temp
    lcd.print("   ");
    lcd.setCursor(6, 2);
    lcd.print(UART.data.tempMotor);

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("MOTOR: ");
    SD_File.print(UART.data.tempMotor);
    SD_File.print(", ");
    SD_File.close();

    lcd.setCursor(16, 0);      //Print throttle %
    lcd.print("    ");
    lcd.setCursor(16, 0);
    lcd.print(UART.data.dutyCycleNow);

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("THROTTLE: ");
    SD_File.print(UART.data.dutyCycleNow);
    SD_File.print(", ");
    SD_File.close();
  }
  else
  {
    lcd.setCursor (0, 3);
    lcd.print("                    ");
    lcd.print("NO DATA!");
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
        time_for_the_next_flash += 1000;

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

void setup() {
  lcd.begin(20, 4);

  Serial.begin(9600);     //Debug

  Serial1.begin(19200);
  while (!Serial) {
    ;
  }
  UART.setSerialPort(&Serial1);       //Define which port to use as UART

  lcd.setCursor(0, 0);      //Set cursor on the X column and Y row.
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

void loop () {         //Main loop
  Display_Data ();
  Voltage_Flasher();
  Current_Flasher();
  ESC_Flasher();
  Motor_Flasher();
}