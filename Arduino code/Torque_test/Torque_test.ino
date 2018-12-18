/*
J.Teda 21/04/2013

Exsample of how to run Dynamixel in SERVO mode - tell servo what angle to move to.

Robotis e-Manual ( http://support.robotis.com )

*/

#include <Dynamixel_Serial.h>       // Library needed to control Dynamixal servo

#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)

unsigned int pos_desired = 1024;
unsigned int start_pos = 0;
unsigned int diff_pos = 0;

/* //for constant velocity
unsigned int end_pos = 0;
unsigned int t_start = 0;
unsigned int t_end = 0;
unsigned int t_diff = 0;
float vel = 0;
float vel_desired = 0.2;              //steps pr. ms (200 steps pr. sec)
float vel_diff = 0;
*/

float k_p = 0.1171875;

int value = 0;

void setup(){
    delay(1000);                                                            // Give time for Dynamixel to start on power-up
    Serial.begin(57600);
    Dynamixel.begin(SERVO_SET_Baudrate);                                    // We now need to set Ardiuno to the new Baudrate speed
    Dynamixel.setDirectionPin(SERVO_ControlPin);                            // Optional. Set direction control pin
    Dynamixel.enableTorque(2,true);

    Dynamixel.setHoldingTorque(3,true);
    
    delay(1000);
    
    start_pos = Dynamixel.readPosition(2);
    while (start_pos > 4100 ) start_pos = Dynamixel.readPosition(2); //read again if error
  
    Serial.println(start_pos);

    while(start_pos<pos_desired){
        diff_pos = pos_desired-start_pos;
      
        value = diff_pos*k_p;
        
        Dynamixel.torque(2,false,value);  
        
        delay(1);
         
        start_pos = Dynamixel.readPosition(2);
        while (start_pos > 4100 ) start_pos = Dynamixel.readPosition(2); //read again if error
    }
  
        Dynamixel.enableTorque(2,false);
        delay(1000);
        Dynamixel.setHoldingTorque(2,false);

/* //for constant velocity
   
   start_pos = Dynamixel.readPosition(2);
   while (start_pos > 4100 ) start_pos = Dynamixel.readPosition(2); //read again if error
  
   t_start = millis();
   value = 40; //initial torque

   Serial.println(start_pos);

   while(start_pos<pos_desired){
       Dynamixel.torque(2,false,value);  
       delay(1);
    
       end_pos = Dynamixel.readPosition(2);
       while (end_pos > 4100) end_pos = Dynamixel.readPosition(2); //read again if error
    
       t_end = millis();
       t_diff = t_end-t_start;
       t_start = t_end;
    
       diff_pos = end_pos - start_pos;
       start_pos = end_pos;
    
       vel = (float)diff_pos/(float)t_diff;
       vel_diff = vel_desired-vel;
    
       if(vel_diff>0){       // give more torque
          value = value*1.05;
       }
    
       else{ // give less torque
          value = value*0.95;
       }
   }

        Dynamixel.enableTorque(2,false);
        delay(10);
        Dynamixel.setHoldingTorque(2,true);
        delay(1000);
        Dynamixel.setHoldingTorque(2,false);
*/
}


void loop(){

}

