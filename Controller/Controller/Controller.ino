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
  lcd.setCursor(0,1);
  lcd.print("Joystick 1 Y: ");
  lcd.setCursor(0,2);
  lcd.print("Joystick 2 X: ");
  lcd.setCursor(0,3);
  lcd.print("Joystick 2 Y: "); // 16
}
 
void loop()
{
 int j1x, j1y, j2x, j2y;
  j1x = analogRead(3);
  j1y = analogRead(2);
  j2x = analogRead(0);
  j2y = analogRead(1);

  lcd.setCursor(14,0);
  lcd.print("    ");
  lcd.setCursor(14,0);
  lcd.print(j1x);
  lcd.setCursor(14,1);
  lcd.print("    ");
  lcd.setCursor(14,1);
  lcd.print(j1y);
  lcd.setCursor(14,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print(j2x);
  lcd.setCursor(14,3);
  lcd.print("    ");
  lcd.setCursor(14,3);
  lcd.print(j2y);  
}
