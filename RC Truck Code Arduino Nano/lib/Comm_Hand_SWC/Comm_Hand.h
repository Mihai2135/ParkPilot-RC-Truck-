/// @file Comm_Hand.h

#ifndef Comm_Hand_h
#define Comm_Hand_h


#define forwardBit 128      ///< "10000000" 
#define backwardBit 64      ///< "01000000"
#define leftBit 32          ///< "00100000"
#define rightBit 16         ///< "00010000"
#define lightonBit 4        ///< "00000100"
#define lightautoBit 8      ///< "00001000"
#define drivingmodeBit 2    ///< "00000010"
#define handbrakeBit 1      ///< "00000001"
#define MAXCOUNTER 10       ///< The max value of he tick counter


#define BACKWARD  0  ///< Constant that tells the I/O Handler the rotation direction of the DC motor
#define FREERUNNING  1  ///< Constant that tells the I/O Handler to operate the DC motor in free running mode
#define FORWARD  2  ///< Constant that tells the I/O Handler the rotation direction of the DC motor  

#define TURN_RIGHT -1   ///< Constant that tells to steer right
#define TURN_LEFT 1     ///< Constant that tells to steer left
#define FACE_FORWARD 0  ///< Constant that tells to face forward

#define HANDBRAKE_ON 1  ///< Value when the Handbrake is on
#define HANDBRAKE_OFF 0 ///< Value when the Handbrake is off

#define PARKPILOT_ON true  ///<Tells if the Car should try to park automatically
#define PARKPILOT_OFF false ///<Tells if the driving mode is manual

#define AUTOLIGHT true      ///<Tells if the headlight are in automatic mode
#define MANUALLIGHT false   ///<Tells if the headlight are in manual mode
#define ONLIGHT true        ///<Tells if the headlight needs to be turned on manually
#define OFFLIGHT false      ///<Tells if the headlight needs to be turned off manually

void initCommHandler();
void Comm_Handler();

#endif