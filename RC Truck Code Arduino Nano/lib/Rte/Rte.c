/**
 * @page RTE
 *  **Author:** Molnar Andrei, Varga Rajmond, Gergely Imre Norbert, Szabo Zoltan
 * 
 *  **Date:** 20.07.2022
 *  ## Module's Role
 * In this module are declared the Rte variables
 */

#include <Arduino.h>
#include <Rte.h>

bool LowBeamMode;   /* Variable for the low beam mode */
bool LowBeamStateManual;   /* Variable for the manual low beam state */
int LightLevel;  /* Variable for the light level */
bool LowBeamState;  /* Variable for the low beam state */


uint8_t FlashLightState; /* 0 both off 1 both on 2 left on, 3 right on*/
int TickCounter; /*Counter for the flash */



bool DrivingMode; /* TRUE -> Park Pilot  FALSE -> Manual  */
uint8_t stage = 0;  /*Stage counter of the Park Pilot*/
int UltrasonicSensors[6];   /*Array to store the data of the Ultrasonuc sensors*/



int TurningDirection = 0; /* -1 -> LEFT    0 -> Straight    1 -> RIGHT */
int SteeringAngle = 0; /* Calculated Angle by the Park Pilot SWC */
int LastTurningDirection; /* The last turning direction */



uint8_t DCMotorPower = 0; /* from 0 to 255, PWM signal */
short int DCMotorTorque; /* variable in range of (-100; 100), in case of negative number the motor rotates backward, the abs|DMT| means, we use the DC motor with DMT % power. */
uint8_t DCMotorDirection; /*0 if backward rotation, 1 free,  2 if forward rotation, 3 if handbrake */
uint8_t AccelerationDirection = 1; /* 0 if backward, 1 if no input, 2 if forward */
bool Handbrake; /* 0 if OFF, 1 if ON */
bool Emergency; /* 0 if NOT, 1 in case of an emergency */



bool CommunicationEmergency; /* 0 if NOT, 1 in case of a communication emergency */
int CommCounter;    /*Counter for the communication handler timeout*/
