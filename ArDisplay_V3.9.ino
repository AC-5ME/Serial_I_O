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
RTC_PCF8523 rtc;
LCD_I2C lcd(0x27, 20, 4);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};     //SD logger

bool isVoltageWarning = false;      //Warning flashers
bool isCurrentWarning = false;
bool isESCTempWarning = false;
bool isMotorTempWarning = false;
bool isEscFault = false;
bool isDataFault = false;
bool isFaults = false;

int faultTime = 0;     //Fault time-out

int erpmRx = 0;
int voltDisplay = 0;
int ampDisplay = 0;
float WhDisplay = 0.0;
float WhchargedDisplay = 0.0;
int escTempDisplay = 0;
int motorTempDisplay = 0;
//float dutyCycleDisplay = 0.0;
int faultCode = 0;

uint32_t next_request = 0;     //Request loop delay

void Print_Screen() {     //Refresh screen
  lcd.setCursor(0, 2);
  lcd.print ("Motor:");
}

void Get_Data() {
  if (UART.getVescValues()) {
    erpmRx = (UART.data.rpm);
    voltDisplay = (UART.data.inpVoltage);
    ampDisplay = (UART.data.avgInputCurrent);
    WhDisplay = (UART.data.wattHours);
    WhchargedDisplay = (UART.data.wattHoursCharged);
    escTempDisplay = (UART.data.tempMosfet);
    motorTempDisplay = (UART.data.tempMotor);
    // dutyCycleDisplay = (UART.data.dutyCycleNow);
    faultCode = (UART.data.error);

    isDataFault = false;      //Good Data
    isFaults = false;
  } else {

    isDataFault = true;
    isFaults = true;
    faultTime = millis();
  }
}

void Display_Data(int erpmRx, int voltDisplay, int ampDisplay, float WhDisplay, int escTempDisplay, int motorTempDisplay, int faultCode) {
  uint32_t timeStamp = millis();      //SD card

  int rpmDisplay = (erpmRx / 10);

  int unsignedAmpDisplay = abs(ampDisplay);     //Remove negative sign

  const int min = 0;    //Throttle position % conversion
  const int max = 350;      //Max ESC current input
  const int span = max - min;

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

  lcd.setCursor(3, 1);     //Print input volts
  lcd.print("   ");
  lcd.setCursor(3, 1);
  lcd.print(voltDisplay);

  if (voltDisplay < 90) {      //Low volt warning (28S X 3V2)
    isVoltageWarning = true;
    isFaults = true;
    faultTime = millis();
  } else {

    isVoltageWarning = false;
    isFaults = false;
  }

  SD_File = SD.open("ESC.log", FILE_WRITE);
  SD_File.print("V: ");
  SD_File.print(voltDisplay);
  SD_File.print(", ");
  SD_File.close();


  lcd.setCursor(14, 1);     //Print current
  lcd.print("   ");
  lcd.setCursor(14, 1);
  lcd.print(unsignedAmpDisplay);

  if (unsignedAmpDisplay > 350) {      //High current warning
    isCurrentWarning = true;
    isFaults = true;
    faultTime = millis();
  } else {

    isCurrentWarning = false;
    isFaults = false;
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
    isFaults = true;
    faultTime = millis();
  } else {

    isESCTempWarning = false;
    isFaults = false;
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
    isFaults = true;
    faultTime = millis();
  } else {

    isMotorTempWarning = false;
    isFaults = false;
  }

  SD_File = SD.open("ESC.log", FILE_WRITE);
  SD_File.print("MOTOR: ");
  SD_File.print(motorTempDisplay);
  SD_File.print(", ");
  SD_File.close();

  int throttleDisplay = ((unsignedAmpDisplay - min) / (float)span) * 100;

  char throttleBuffer[5];
  sprintf(throttleBuffer, "%u%%", throttleDisplay);

  lcd.setCursor(16, 0);      //Print throttle % of max current (350)
  lcd.print("    ");
  lcd.setCursor(16, 0);
  lcd.print(throttleBuffer);

  SD_File = SD.open("ESC.log", FILE_WRITE);
  SD_File.print("THROTTLE: ");
  SD_File.print(throttleBuffer);
  SD_File.print(", ");
  SD_File.close();

  if (faultCode > 0) {      //ESC fault warning
    isEscFault = true;
    isFaults = true;
    faultTime = millis();
  } else {

    isEscFault = false;
    isFaults = false;
  }
}

void Voltage_Flasher() {
  if (isVoltageWarning == true) {

    lcd.noBacklight();      //Flash LCD

    lcd.setCursor (0, 3);
    lcd.print("LOW VOLTAGE!        ");

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.println("LOW VOLTAGE!, ");
    SD_File.close();

    lcd.backlight();
  }
}

void Current_Flasher() {
  if (isCurrentWarning == true) {

    lcd.noBacklight();

    lcd.setCursor (0, 3);
    lcd.print("HIGH CURRENT!       ");

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.println("HIGH CURRENT!, ");
    SD_File.close();

    lcd.backlight();
  }
}

void ESC_Flasher() {
  if (isESCTempWarning == true) {

    lcd.noBacklight();

    lcd.setCursor (0, 3);
    lcd.print("ESC TEMP!           ");

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.println("ESC TEMP!, ");
    SD_File.close();

    lcd.backlight();
  }
}

void Motor_Flasher() {
  if (isMotorTempWarning == true) {

    lcd.noBacklight();

    lcd.setCursor (0, 3);
    lcd.print("MOTOR TEMP!         ");

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("MOTOR TEMP!, ");
    SD_File.close();

    lcd.backlight();
  }
}

void ESC_Fault_Flasher() {
  if (isEscFault == true) {

    lcd.noBacklight();

    lcd.setCursor (0, 3);
    lcd.print("ESC FAULT!         ");

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("ESC FAULT: ");
    SD_File.print(faultCode);
    SD_File.println(", ");
    SD_File.close();

    lcd.backlight();
  }
}

void No_Data_Flasher() {
  if (isDataFault == true) {

    lcd.noBacklight();

    lcd.setCursor (0, 3);
    lcd.print("NO DATA!            ");

    SD_File = SD.open("ESC.log", FILE_WRITE);
    SD_File.print("NO DATA!, ");
    SD_File.close();

    lcd.backlight();
  }
}

void setup() {
  delay (200);      //Stabilization delay

  //Serial.begin(9600);     //Debug
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
    lcd.print("RTC FAIL!           ");
    delay(2000);
    lcd.setCursor (0, 3);
    lcd.print("                    ");
  }

  if (!SD.begin(10)) {      //chipSelect pin - 10 M4 board
    lcd.setCursor (0, 3);
    lcd.print("SD FAIL!            ");
    delay(2000);
    lcd.setCursor (0, 3);
    lcd.print("                    ");
  }

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

void loop () {
  uint32_t time_now = millis();

  if (time_now > next_request) {
    Print_Screen();     //Refresh screen
    Get_Data();
    Display_Data (erpmRx, voltDisplay, ampDisplay, WhDisplay, escTempDisplay, motorTempDisplay, faultCode);
    next_request += 500;
  }

  Voltage_Flasher();
  Current_Flasher();
  ESC_Flasher();
  Motor_Flasher();
  ESC_Fault_Flasher();
  No_Data_Flasher();

  if (isFaults == false) {      //Clear message time-out
    unsigned long currentTime = millis();

    if (currentTime - faultTime >= 2000) {

      lcd.setCursor (0, 3);
      lcd.print("                    ");
    }
  }
}
