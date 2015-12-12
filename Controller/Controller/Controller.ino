#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
 
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack
 
void setup()
{
  // activate LCD module
  lcd.begin (20,4); // for 16 x 2 LCD module
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
   lcd.home (); // set cursor to 0,0
  lcd.print("Joystick 1 X: ");
  lcd.setCursor(1,0);
  lcd.print("Joystick 1 Y: ");
  lcd.setCursor(2,0);
  lcd.print("Joystick 2 X: ");
  lcd.setCursor(3,0);
  lcd.print("Joystick 2 Y: "); // 16
}
 
void loop()
{
 int j1x, j1y, j2x, j2y;
  j1x = analogRead();
  j1y = analogRead();
  j2x = analogRead();
  j2y = analogRead();
}
