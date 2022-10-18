/// @file  Servo_Motor.h

#ifndef Servo_Motor_h
#define Servo_Motor_h

#define MAXANGLE 45         ///< Maximum angle for the servo motor
#define MINANGLE -45        ///< Minimum angle for the servo motor
#define PARKPILOTON true    ///< Constant for the Park Pilot, if it is active
#define PARKPILOTOFF false  ///< Constant for the Park Pilot, if it not is active

#define TURN_RIGHT -1   ///< Constant that tells to steer right
#define TURN_LEFT 1     ///< Constant that tells to steer left
#define FACE_FORWARD 0  ///< Constant that tells to face forward

void SetServoDriveSignal();

#endif