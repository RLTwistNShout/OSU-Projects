// Arduino NES Controller Test Software.
// Feel free to use this for whatever purpose you like.
 
// Bradsprojects - This code came about by an intelligent designer (me). It did not happen by random chance.
// Our universe and everything in it also came about by an intelligent designer (God). It did not happen by random chance.


/*
 * DO THIS: 
 * WRITE CODE THAT CAN TAKE IN A VOLTAGE USING THE ADC.
 *  THIS VOLTAGE WILL COME FROM A POTENTIOMETER PHYSICALLY
 *  COUPLED TO A DC MOTOR SHAFT. BY FEEDING IN 5V FROM THE
 *  ARDUINO AND READING THE DROP IN POTENTIAL ACROSS THE 
 *  POT, WE CAN EXTRAPOLATE POSITION. 
 *  
 * NEXT:
 *  WRITE CODE THAT WILL DECODE THE VOLTAGE DROP AND RETURN
 *  THE POSITION OF THE MOTOR SHAFT.
 * 
 * NEXT:
 *  WRITE CODE THAT CAN TURN A DC MOTOR FORWARD OR REVERSE
 *  DEPENDENT UPON THE OUTCOME OF SOME MATH
 * 
 * NEXT:
 *  WRITE CODE THAT CAN USE A VALUE (STORED IN "motorPos")
 *  AS A SETPOINT TO DRIVE THE MOTOR SHAFT IN EITHER THE 
 *  REVERSE OR FORWARD DIRECTION. (PROBABLY JUST FIND THE
 *  DIFFERENCE AND USE SOME SORT OF THRESHOLD TO MAKE
 *  THE MOTOR TURN (F or RV) OR STAY.
 *  
 *  **NOTE: A MUCH MORE EFFICIENT AND ACCURATE WAY TO DO
 *  THIS WOULD BE TO CONSTRUCT AN OPTICAL SESNSOR. THIS
 *  IMPROVEMENT WOULD ALSO GIVE US THE ADDED BENEFIT OF
 *  360 DEGREE ROTATION
 */


#include "LCD.h"
#include "LiquidCrystal_I2C.h"

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);

int currentOutput = 0;
int motorPos = 0;

byte NESData = 9;       // this is the pin that the Data connection is connected to
byte NESLatch = 11;         // this is the pin that the Latch (otherwise known as strobe) connection is connected to
byte NESClock = 12;         // this is the pin that the Clock connection is connected to
byte NESButtonData;         // This is where we will store the received data that comes from the NES Control Pad
 
void setup() {              // let's get a few things setup before we get into the main code
    Serial.begin(9600);         // serial data will be sent at 9600bps
    pinMode(NESLatch, OUTPUT);  // Latch connection is an output
    pinMode(NESClock, OUTPUT);  // Clock connection is an output
    pinMode(NESData, INPUT);    // Data connection is an Input (because we need to receive the data from the control pad)

    lcd.begin(20,4); 
    lcd.setBacklightPin(3,POSITIVE);
    lcd.setBacklight(HIGH);
    lcd.setCursor(3,0);
    lcd.clear();
    lcd.setCursor(3,0);
    lcd.print("Arduino + NES");
    lcd.setCursor(2,1);
    lcd.print("Servo Controller"); 
    lcd.setCursor(3,2);
    lcd.print("Servo Position:");
    
}
 
void loop() {                       // we will run this code in a constant loop until power is removed
    GetNESControllerData();         // This calls the function to grab the NES control pad data and it will store it in 'NESButtonData'
    convertNESControllerData(); 
    printPosition();
}
 
void GetNESControllerData(){            // this is where it all happens as far as grabbing the NES control pad data
    digitalWrite(NESLatch, HIGH);       // we need to send a clock pulse to the latch (strobe connection)...
    digitalWrite(NESLatch, LOW);        // this will cause the status of all eight buttons to get saved within the 4021 chip in the NES control pad.
    for(int x=0; x<=7; x++){         // Now we need to transmit the eight bits of data serially from the NES control pad to the Arduino
        bitWrite(NESButtonData,x,digitalRead(NESData)); // one by one, we will read from the NESData line and store each bit in the NESButtonData variable.
        digitalWrite(NESClock, HIGH);                   // once each bit is saved, we send a clock pulse to the NES clock connection...
        digitalWrite(NESClock, LOW);                    // this will now shift all bits in the 4021 chip in the NES control pad, so we can read the next bit.
    }
}

  // Convert the NES Data to position
void convertNESControllerData(){
  if((NESButtonData == 0b11101111) && currentOutput < 150)
        currentOutput++;
  if((NESButtonData == 0b11011111) && currentOutput > 0)
        currentOutput= currentOutput - 1;

}

//Finish this funcion below...
void printPosition(){

  // Scale the counter value
    motorPos = currentOutput *1.8;
    
  // print the position    
    lcd.setCursor(5,3);
    lcd.print(motorPos);
    lcd.print(" degrees");
  
  // Clean up the unwanted zeros
  if(motorPos < 100){
    lcd.setCursor(18,3);
    lcd.print(" ");
  }
  if(motorPos < 10){
    lcd.setCursor(17,3);
    lcd.print(" ");
  }

}



// this code allows us to specify the number of binary digits we want (which is always 8) 
// The code has been made available from: http://www.phanderson.com/arduino/arduino_display.html
void print_binary(int v, int num_places){
    int mask = 0, n;
 
    for (n=1; n<=num_places; n++){
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask;  // truncate v to specified number of places
    while(num_places){
        if (v & (0x0001 << num_places-1)){
            Serial.print("1");
        }
        else{
            Serial.print("0");
        }
    num_places--;
    }
}
