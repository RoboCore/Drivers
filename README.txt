
	RoboCore Drivers Library
		(v1.0 - 25/04/2013)

************** STILL BEING DEVELOPED
************************************

  Driver classes for Arduino
    (tested with Arduino 1.0.1)

  Released under the Beerware license
  Written by François
  
  Supports:
    - Pololu A4988 driver
  
  NOTES:
    # the Pololu A4988 driver can be
      automatically disabled/put to
      sleep when motor is not running
      and enabled/waken up when driving
      (setDriveMode() method)


***********************
A4988

    A4988(void);
    A4988(int pinDIR, int pinSTEP);
    A4988(int pinDIR, int pinSTEP, int pinMS1, int pinMS2, int pinMS3);
    
    void Drive(long steps);
    void Drive(long steps, unsigned int frequency);
    void DriveRevolutions(int revolutions);
    int getDir(void);
    int getMode(void);
    int getSpeed(void);
    unsigned int getSteps(void);
    boolean Initialize(int pinDIR, int pinSTEP);
    boolean Initialize(int pinDIR, int pinSTEP, int pinMS1, int pinMS2, int pinMS3);
    void InvertDirection(void);
    boolean ModePinsSet(void);
    void setDir(int dir);
    int setMode(byte mode);
    int setMSpins(int pinMS1, int pinMS2, int pinMS3);
    void setSpeedRPM(int rpm);
    void setSteps(unsigned int steps);





	





