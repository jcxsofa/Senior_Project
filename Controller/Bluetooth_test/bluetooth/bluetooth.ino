#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2,3);
 
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack

void setup() {
  // activate LCD module
  lcd.begin (20,4); // for 16 x 2 LCD module
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home (); // set cursor to 0,0
  BTSerial.begin(9600);  // HC-05 default speed
  
}

void loop() {
  char rx_byte;
  if(BTSerial.available()){

    rx_byte = BTSerial.read();
    lcd.print(rx_byte);

    BTSerial.write(rx_byte);
    
  }

}
