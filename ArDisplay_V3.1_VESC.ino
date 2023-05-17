#include <buffer.h>
#include <crc.h>
#include <datatypes.h>
#include <VescUart.h>
#include <LCD_I2C.h>
VescUart UART;

LCD_I2C lcd(0x27, 20, 4);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};     //SD logger

bool isVoltageWarning = false;      //Warning flashers
bool isCurrentWarning = false;
bool isESCTempWarning = false;
bool isMotorTempWarning = false;
static bool isNoWarnings = false;
static bool isNoErrors = false;
static uint32_t flashTimeout;
static uint32_t time_for_the_next_flash = millis();

int rpmDisplay = 0;
int voltDisplay = 0;
int ampDisplay = 0;
float WhDisplay = 0.0;
float WhchargedDisplay = 0.0;
int escTempDisplay = 0;
int motorTempDisplay = 0;
int dutyCycleDisplay = 0;

uint32_t next_request = millis();     //Request loop delay

void Get_Data() {
  if (UART.getVescValues()) {       //Acquire data from VESC
    rpmDisplay = (UART.data.rpm);
    voltDisplay = (UART.data.inpVoltage);
    ampDisplay = (UART.data.avgInputCurrent);
    WhDisplay = (UART.data.wattHours);
    WhchargedDisplay = (UART.data.wattHoursCharged);
    escTempDisplay = (UART.data.tempMosfet);
    motorTempDisplay = (UART.data.tempMotor);
    dutyCycleDisplay = (UART.data.dutyCycleNow);
  }
  else
  {
    lcd.setCursor (0, 3);
    lcd.print("NO DATA!            ");
  }
}

void Display_Data(int rpmDisplay, int voltDisplay, int ampDisplay, float WhDisplay, int escTempDisplay, int motorTempDisplay, int dutyCycleDisplay) {
  uint32_t timeStamp = millis();      //SD card

  lcd.setCursor(4, 0);      //Print RPM
  lcd.print("     ");
  lcd.setCursor(5, 0);
  lcd.print(rpmDisplay);
  /*
    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.println();
    SD_File.print("ms: ");
    SD_File.print(timeStamp);
    SD_File.print(" RPM: ");
    SD_File.print(rpmDisplay);
    SD_File.print(", ");
    SD_File.close();
  */
  lcd.setCursor(2, 1);     //Print input volts
  lcd.print("        ");
  lcd.setCursor(3, 1);
  lcd.print(voltDisplay);

  if (voltDisplay > 0 && voltDisplay < 90 ) {      //Low volt warning
    isVoltageWarning = true;
    flashTimeout = millis() + 3000;
  }
  /*
    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("V: ");
    SD_File.print(voltDisplay);
    SD_File.print(", ");
    SD_File.close();
  */
  lcd.setCursor(13, 1);     //Print current
  lcd.print("     ");
  lcd.setCursor(14, 1);
  lcd.print(ampDisplay);

  if (ampDisplay > 350) {      //High current warning
    isCurrentWarning = true;
    flashTimeout = millis() + 3000;
  }
  /*
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
  */
  //char escTempBuffer[5];
  //sprintf(escTempBuffer, "%uC", escTempDisplay);

  lcd.setCursor(15, 2);      //Print ESC temp
  lcd.print("   ");
  lcd.setCursor(15, 2);
  //lcd.print(escTempBuffer);
  lcd.print(escTempDisplay);

  if (escTempDisplay > 80) {      //ESC temp warning
    isESCTempWarning = true;
    flashTimeout = millis() + 3000;
  }
  /*
    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("ESC: ");
    SD_File.print(escTempBuffer);
    SD_File.print(", ");
    SD_File.close();
  */
  //char motorTempBuffer[5];
  //sprintf(motorTempBuffer, "%uC", motorTempDisplay);

  lcd.setCursor(6, 2);
  lcd.print("   ");
  lcd.setCursor(6, 2);
  //lcd.print(motorTempBuffer);
  lcd.print(motorTempDisplay);

  if (motorTempDisplay > 110) {      //Motor temp warning
    isMotorTempWarning = true;
    flashTimeout = millis() + 3000;
  }
  /*
    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("MOTOR: ");
    SD_File.print(motorTempBuffer);
    SD_File.print(", ");
    SD_File.close();
  */
  //char throttlePosBuffer[5];
  //sprintf(throttlePosBuffer, "%u%%", dutyCycleDisplay);

  lcd.setCursor(16, 0);      //Print throttle %
  lcd.print("    ");
  lcd.setCursor(16, 0);
  //lcd.print(throttlePosBuffer);
  lcd.print(dutyCycleDisplay);

  /*
    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("THROTTLE: ");
    SD_File.print(throttlePosBuffer);
    SD_File.print(", ");
    SD_File.close();*/
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
          /*
                    SD_File = SD.open("ESC.log", FILE_WRITE);
                    SD_File.println("LOW VOLTAGE!,");
                    SD_File.close();
          */
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
          /*
                    SD_File = SD.open("ESC.log", FILE_WRITE);
                    SD_File.println("HIGH CURRENT!,");
                    SD_File.close();
          */
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
          /*
                    SD_File = SD.open("ESC.log", FILE_WRITE);
                    SD_File.println("ESC TEMP!,");
                    SD_File.close();
          */
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
          /*
                    SD_File = SD.open("ESC.log", FILE_WRITE);
                    SD_File.println("MOTOR TEMP!,");
                    SD_File.close();
          */
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
  delay (500);      //Stabilization delay

  Serial1.begin(115200);

  UART.setSerialPort(&Serial1);       //Define which port to use as UART
  // while (!Serial1) { ; }

  lcd.begin();
  lcd.backlight();

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
}

void loop () {         //Main loop

  uint32_t time_now = millis();

  if (time_now > next_request) {
    Get_Data();
    next_request += 500;
  }

  Display_Data (rpmDisplay, voltDisplay, ampDisplay, WhDisplay, escTempDisplay, motorTempDisplay, dutyCycleDisplay);
  // Voltage_Flasher();
  Current_Flasher();
  ESC_Flasher();
  Motor_Flasher();
}
