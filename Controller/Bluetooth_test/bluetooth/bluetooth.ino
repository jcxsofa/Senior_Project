#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2,3);
 
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack

byte data[4];

void setup() {
  // activate LCD module
  lcd.begin (20,4); // for 16 x 2 LCD module
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home (); // set cursor to 0,0
  BTSerial.begin(57600);  // HC-05 default speed
  char counter1;
}

void loop() {
 long int* number;
  if(BTSerial.available()){
    //counter = 0
    data[0] = BTSerial.read();
    
    for (int i = 1; i < 2; i++) {
    while(!BTSerial.available());
    data[i] = BTSerial.read();
    }

    //number = (long int*)&data[0];

   for (int i = 1; i >= 0; i--) {
      lcd.print(data[i], DEC);
      //lcd.print("-");
    }
    lcd.print(" ");
    
    lcd.print(*(int*)&data[0], DEC);

    //BTSerial.write(rx_byte);
    
  }

}
