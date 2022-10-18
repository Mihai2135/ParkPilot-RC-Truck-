/// @file Park_Pilot.h

#ifndef Park_Pilot_h
#define Park_Pilot_h


/*All the stages, what we use*/
#define FACE_FORWARD_BEGIN 0        ///< Stage 0 Facing forward
#define GO_FORWARD_STARIGHT 1       ///< Stage 1 Going forward
#define TURN_RIGHT1 2               ///< Stage 2 Turning right
#define GO_FORWARD_RIGHT 3          ///< Stage 3 Go forward and right
#define TURN_LEFT1 4                 ///< Stage 4 Turning left
#define GO_BACKWARD_LEFT 5          ///< Stage 5 Going backward and left
#define TURN_RIGHT2 6               ///< Stage 6 Turning Right
#define GO_BACKWARD_RIGHT 7         ///< Stage 7 Going Backward and right
#define FACE_FORWARD1 8             ///< Stage 8 Facing the tires straight forward
#define GO_FORWARD_STRAIGHT2 9      ///< Stage 9 Going forward straight 
#define FACE_FORWARD_END 10         ///< Stage 10 Final stage



/*Ultrasonic sensor indexes*/
#define FRONT_LEFT 3        ///< The index of the Front Left Ultrasonic Sensor
#define FRONT_CENTER 0      ///< The index of the Front Center Ultrasonic Sensor
#define FRONT_RIGHT 4       ///< The index of the Front Right Ultrasonic Sensor
#define REAR_LEFT 5         ///< The index of the Rear Left Ultrasonic Sensor
#define REAR_CENTER 1       ///< The index of the Rear Center Ultrasonic Sensor
#define REAR_RIGHT 2        ///< The index of the Rear Right Ultrasonic Sensor


#define PARKPILOT true      ///< Constant for the ParkPilot if it is active

#define COUNTER_VALUE_1_SEC 100     ///< Counter to wait 1 sec to steer  

#define FRONT_LEFT_MIN_S1 10        ///< Constant helping value for the front left Ultrasonic sensor in stage 1
#define REAR_LEFT_MIN_S3 15         ///< Constant helping value for the rear left Ultrasonic sensor in stage 3
#define REAR_LEFT_MIN_S5 45         ///< Constant helping value for the rear left Ultrasonic sensor in stage 5
#define REAR_CENTER_MIN_S5 54       ///< Constant helping value for the rear center Ultrasonic sensor in stage 5
#define FRONT_CENTER_MIN_S7 40      ///< Constant helping value for the front center Ultrasonic sensor in stage 7

#define EMARGENCY_ACCURED true      ///< Constant for the Emergency mode if it is Accured

#define TURN_RIGHT -45              ///< Maximum angle of the tire to the right
#define TURN_LEFT 45                ///< Maximum angle of the tire to the left
#define GO_FORWARD 1                ///< Constant for the torque when the car should go forward
#define GO_FORWARD_MORE_TORQUE 10   ///< 10 times more torque
#define GO_BACKWARD -1              ///< Constant for the torque when the car should go backward
#define STOP 0                      ///< Constant for the torque when the car should stop
#define FACE_FORWARD 0              ///< Constant for the servo angle, when the tires should face forward

void Park_Pilot();

#endif

