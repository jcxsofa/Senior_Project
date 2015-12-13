#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

struct Remote_Data {
  
  // motor 1 data to transmit
  float M1_Current;
  float M1_BEMF_Speed;
  float M1_Encoder_Speed;
  float M1_Error;
  float M1_Desired_Speed;
  
  // motor 2 data to transmit
  float M2_Current;
  float M2_BEMF_Speed;
  float M2_Encoder_Speed;
  float M2_Error;
  float M2_Desired_Speed;
  
  // motor 3 data to transmit
  float M3_Current;
  float M3_BEMF_Speed;
  float M3_Encoder_Speed;
  float M3_Error;
  float M3_Desired_Speed;
  
  // motor 4 data to transmit
  float M4_Current;
  float M4_BEMF_Speed;
  float M4_Encoder_Speed;
  float M4_Error;
  float M4_Desired_Speed;
  
  // power stuff
  float Battery_Voltage;
  float Total_Current;
  
};

SoftwareSerial BTSerial(2,3);
 
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack

byte data_buffer[88];

void fill_data(void);
void display_speed(void);

struct Remote_Data data;
int j1x, j1y, j2x, j2y;
int stat = 0;

void setup() {
  // activate LCD module
  lcd.begin (20,4); // for 16 x 2 LCD module
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  BTSerial.begin(115200);  // HC-05 default speed
  // empty serial buffer in case in the middle of a transmission
  //while(BTSerial.available()) BTSerial.read();
  
}

void loop() {

    // get joystik values 
    j1x = analogRead(2); // thresholds 20,440,650,1010
    j1y = analogRead(3); // 20,421,623,1010
    j2x = analogRead(1);// 20,400,600,1010
    j2y = analogRead(0); // mode switch
    
    // get new data
    fill_data();
    // updated stuff
    display_speed();  
  }
}

// write nested for loop to place bytes correctly, along list, but reverse bytes in variable

void display_speed(void) {
if (stat == 0) {
  // motor 1 display
  lcd.setCursor(0,0);
  lcd.print("1 SPD "); // 6 characters
  lcd.setCursor(12,0);
  lcd.print("ER "); // 4

  // motor 2 display
  lcd.setCursor(0,1);
  lcd.print("2 SPD "); // 6 characters
  lcd.setCursor(12,1);
  lcd.print("ER "); // 4

  // motor 3 display
  lcd.setCursor(0,2);
  lcd.print("3 SPD "); // 6 characters
  lcd.setCursor(12,2);
  lcd.print("ER "); // 4

  // motor 4 display
  lcd.setCursor(0,3);
  lcd.print("4 SPD "); // 6 characters
  lcd.setCursor(12,3);
  lcd.print("ER "); // 4
  stat = 1;
}
    // display motor 1 speed
    lcd.setCursor(6, 0);
    lcd.print("    ");
    lcd.setCursor(6, 0);
    lcd.print(data.M1_Desired_Speed, 2);
    //print error
    lcd.setCursor(15, 0);
    lcd.print("    ");
    lcd.setCursor(15, 0);
    lcd.print(data.M1_Current, 2);

    // display motor 2 speed
    lcd.setCursor(6, 1);
    lcd.print("    ");
    lcd.setCursor(6, 1);
    lcd.print(data.M2_Desired_Speed, 2);
    //print error
    lcd.setCursor(15, 1);
    lcd.print("    ");
    lcd.setCursor(15, 1);
    lcd.print(data.M2_Current, 2);

    // display motor 3 speed
    lcd.setCursor(6, 2);
    lcd.print("    ");
    lcd.setCursor(6, 2);
    lcd.print(data.M3_Desired_Speed, 2);
    //print error
    lcd.setCursor(15, 2);
    lcd.print("    ");
    lcd.setCursor(15, 2);
    lcd.print(data.M3_Current, 2);

    // display motor 4 speed
    lcd.setCursor(6, 3);
    lcd.print("    ");
    lcd.setCursor(6, 3);
    lcd.print(data.M4_Desired_Speed, 2);
    //print error
    lcd.setCursor(15, 3);
    lcd.print("    ");
    lcd.setCursor(15, 3);
    lcd.print(data.M4_Current, 2);

    


  
  
}


void fill_data(void) {
    
  // recieve words in normal order
    for (int j = 0; j < 80; j = j + 4) {
      // fill words in reverse order of bytes transmitted
      for (int i = j; i < j+4; i++){
        // read in current byte and wait for next one
        int count = 0;
        while(!BTSerial.available()){
          if (count++ > 5000) return;
        }
        data_buffer[i] = BTSerial.read();
        //Serial.println(data_buffer[i]);
      }
    }

    // insert motor 1 data
    data.M1_Current = *((float*)&data_buffer[0]);
    data.M1_BEMF_Speed = *((float*)&data_buffer[4]);
    data.M1_Encoder_Speed = *((float*)&data_buffer[8]);
    data.M1_Error = *((float*)&data_buffer[12]);
    data.M1_Desired_Speed = *((float*)&data_buffer[16]);
    
    // insert motor 2 data
    data.M2_Current = *((float*)&data_buffer[20]);
    data.M2_BEMF_Speed =*((float*)&data_buffer[24]);
    data.M2_Encoder_Speed = *((float*)&data_buffer[28]);
    data.M2_Error = *((float*)&data_buffer[32]);
    data.M2_Desired_Speed = *((float*)&data_buffer[36]);
    
    // insert motor 3 data
    data.M3_Current = *((float*)&data_buffer[40]);
    data.M3_BEMF_Speed = *((float*)&data_buffer[44]);
    data.M3_Encoder_Speed = *((float*)&data_buffer[48]);
    data.M3_Error = *((float*)&data_buffer[52]);
    data.M3_Desired_Speed = *((float*)&data_buffer[56]);
    
    // insert motor 4 data
    data.M4_Current = *((float*)&data_buffer[60]);
    data.M4_BEMF_Speed = *((float*)&data_buffer[64]);
    data.M4_Encoder_Speed = *((float*)&data_buffer[68]);
    data.M4_Error = *((float*)&data_buffer[72]);
    data.M4_Desired_Speed = *((float*)&data_buffer[76]);


}

