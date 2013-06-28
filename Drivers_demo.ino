/*
    Pololu A4988 driver example

  Use the Serial Monitor to manipulate
  a stepper motor using the A4988 driver.
  
  - Drive 1 revolution (r);
  - Invert the direction (i);
  - Change the mode (1,2,4,8 or 9);
  - Drive 100 steps (if set to QUARTER mode - for example -, will drive real 25 steps - 100 quarters of step);
  - Drive 200 steps with a frequency of 333 Hz (equivalent to motor of 200 steps running at 100 rpm on FULL step mode);
  - Enable the driver (e);
  - Disable the driver (u);
  - Put the driver to sleep (s);
  - Wake the driver up (w);
  - Change the drive mode (k,m or n).
  
*/

//*********************************************************************
//********* ADD ENABLE, RESET and SLEEP capabilities
//********* change README.txt

#include "Drivers.h"


char c;
A4988 stepper(4,2);

//-------------------------------------------------------------------------------------------------

void setup(){
  Serial.begin(9600);
  stepper.setMSpins(5,6,7);
//  stepper.setENABLEpin();
//  stepper.setRSTpin();
//  stepper.setSLEEPpin();
  Serial.println("--- RoboCore Driver Library demo ---");
}

//-------------------------------------------------------------------------------------------------

void loop(){
  if(Serial.available()){
    c = Serial.read();
    switch(c){
      case 'r': // 1 revolution
        stepper.DriveRevolutions(1);
        break;
      
      case 'i': // invert direction
        stepper.InvertDirection();
        break;
      
      case '1': // FULL step
        stepper.setMode(A4988_FULL_STEP);
        break;
      
      case '2': // HALF step
        stepper.setMode(A4988_HALF_STEP);
        break;
      
      case '4': // QUARTER step
        stepper.setMode(A4988_QUARTER_STEP);
        break;
      
      case '8': // EIGHTH step
        stepper.setMode(A4988_EIGHTH_STEP);
        break;
      
      case '9': // SIXTEENTH step
        stepper.setMode(A4988_SIXTEENTH_STEP);
        break;
      
      case 'd': // drive
        stepper.Drive(100);
        break;
      
      case 'f': // drive with 333 Hz
        stepper.Drive(200, 333);
        break;
      
      case 'e': // enable
        stepper.Enable();
        break;
      
      case 'u': // disable
        stepper.Disable();
        break;
      
      case 's': // sleep
        stepper.Sleep();
        break;
      
      case 'w': // wake up
        stepper.WakeUp();
        break;
      
      case 'k': // mode ENABLE
        stepper.setDriveMode(A4988_DRIVE_MODE_ENABLE);
        break;
      
      case 'm': // mode SLEEP
        stepper.setDriveMode(A4988_DRIVE_MODE_SLEEP);
        break;
      
      case 'n': // mode NONE
        stepper.setDriveMode(A4988_DRIVE_MODE_NONE);
        break;
    }

    Serial.print("Command: ");
    Serial.print(c);
    Serial.print(" (");
    Serial.print(stepper.getMode());
    Serial.println('-');
    Serial.print(stepper.getDriveMode());
    Serial.println(')');
  }
}




