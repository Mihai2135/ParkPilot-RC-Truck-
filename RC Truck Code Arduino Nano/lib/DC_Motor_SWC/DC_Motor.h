/// @file DC_Motor.h
#define MAXPWM (uint8_t) 255  ///< Constant for the maximum PWM command value
#define ZERO_TORQUE (uint8_t) 0  ///< Constant for the  ParkPilot mode, it is used to determine the DC motor turning direction

#define BACKWARD_ACC (uint8_t) 0  ///< Constant, if the backward button is pressed on the web server
#define NO_ACC (uint8_t) 1  ///< Constant, if: backward, forward buttons are not pressed on the web server 
#define FORWARD_ACC (uint8_t) 2  ///< Constant, if the forward button is pressed on the web server 

#define BACKWARD (uint8_t) 0  ///< Constant that tells the I/O Handler the rotation direction of the DC motor
#define FREERUNNING (uint8_t) 1  ///< Constant that tells the I/O Handler to operate the DC motor in free running mode
#define FORWARD (uint8_t) 2  ///< Constant that tells the I/O Handler the rotation direction of the DC motor  
#define HANDBRAKE (uint8_t) 3  ///< Constant that tells the I/O Handler to use forced-motor break 

#define EMERGENCY_ON true  ///< Turn on constant for the emergency mode
#define HANDBRAKE_ON true  ///< Turn on constant for the handbrake
#define PARKPILOT_ON true  ///< Turn on constant for the park pilot mode

#define PWMSCALINGFACTOR 2.55  ///< Scaling constant used to calculate the DC motor PWM in park pilot mode

void SetDCMotorParameters ();