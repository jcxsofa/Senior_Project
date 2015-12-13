#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2,3);
 
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack

byte data[5];

void setup() {
  // activate LCD module
  lcd.begin (20,4); // for 16 x 2 LCD module
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home (); // set cursor to 0,0
  lcd.print("M1 current: "); // 12 characters
  BTSerial.begin(57600);  // HC-05 default speed
  
}

void loop() {
 long int* number;

  // wait for data to be available
  if(BTSerial.available()){
    
    // read in first byte
    data[0] = BTSerial.read();

    // for the rest of the expected bytes
    for (int i = 1; i < 4; i++) {

    // wait for serial data to be available
    while(!BTSerial.available());

    // read new data in
    data[i] = BTSerial.read();
    }

    //number = (long int*)&data[0];

   //for (int i = 1; i >= 0; i--) {
   //   lcd.print(data[i], DEC);
      //lcd.print("-");
   // }
    lcd.setCursor(12, 0);
    lcd.print("          ");
    lcd.setCursor(12, 0);
    lcd.print(*((float*)&data[0]), 5);

    //BTSerial.write(rx_byte);
    
  }

}
