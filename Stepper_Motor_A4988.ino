#include <LiquidCrystal.h>
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

#define potentiometer  A0  //10k Variable Resistor
#define bt_F A1 // Clockwise Button
#define bt_S A2 // Stop Button
#define bt_B A3 // Anticlockwise Button

#define dirPin  8  //8Pin  of Arduino--Direction of stepper motor driver 
#define stepPin 9  //9Pin  of Arduino--Step of stepper motor driver 
#define enPin   10 //10Pin of Arduino--Enabled of stepper motor driver  

int read_ADC;
int Speed_LCD, Speed_Delay;
int Mode=1, flag=0;

void setup() { // put your setup code here, to run once
  
pinMode(potentiometer, INPUT); // declare potentiometer as input 

pinMode(bt_F, INPUT_PULLUP); // declare bt_F as input
pinMode(bt_S, INPUT_PULLUP); // declare bt_S as input
pinMode(bt_B, INPUT_PULLUP); // declare bt_B as input

pinMode(dirPin,  OUTPUT); // declare as output for Direction of stepper motor driver 
pinMode(stepPin, OUTPUT); // declare as output for Step of stepper motor driver 
pinMode(enPin,   OUTPUT); // declare as output for Enabled of stepper motor driver 
  
lcd.begin(16,2);  
lcd.setCursor(0,0);
lcd.print(" WELCOME To  My ");
lcd.setCursor(0,1);
lcd.print("YouTube  Channel");
delay(2000); // Waiting for a while
lcd.clear();
}

void loop() { 

read_ADC = analogRead(potentiometer); // read analogue to digital value 0 to 1023 
Speed_Delay = map(read_ADC, 0, 1023, 5000, 10); //value map for Microstep resolution Delay
Speed_LCD = map(read_ADC, 0, 1023, 0, 100); //value map to Display on the LCD

lcd.setCursor(0,0);
lcd.print("   Speed: ");
lcd.print(Speed_LCD); 
lcd.print("%  ");

if(digitalRead (bt_F) == 0){Mode = 2; digitalWrite(enPin, LOW);} //For Clockwise

if(digitalRead (bt_S) == 0){ //For Stop
if(flag==0){flag=1;
 if(Mode>1)Mode=1; 
      else{Mode=!Mode; 
      if(Mode==0)digitalWrite(enPin, HIGH);
            else digitalWrite(enPin, LOW);
      }
delay(100);
 }
}else{flag=0;} 

if(digitalRead (bt_B) == 0){Mode = 3; digitalWrite(enPin, LOW);} //For Anticlockwise

lcd.setCursor(0,1);

     if(Mode==0)lcd.print("      Free      ");
else if(Mode==1)lcd.print("      Stop      ");
else if(Mode==2)lcd.print("    Clockwise   ");
else if(Mode==3)lcd.print("  Anticlockwise ");

if(Speed_LCD>0 && Mode>1){ 
   
     if(Mode==2)digitalWrite(dirPin, LOW);// Stepper motor rotates CW (Clockwise)
           else digitalWrite(dirPin, HIGH);// Stepper motor rotates CCW (Anticlockwise)

digitalWrite(stepPin, HIGH);
delayMicroseconds(Speed_Delay);
digitalWrite(stepPin, LOW);
delayMicroseconds(Speed_Delay);
}  

}
