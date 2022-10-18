/// @file Rte.h

#ifndef Rte_h
#define Rte_h

#include<Arduino.h>

extern bool LowBeamMode;   ///< Variable for the low beam mode,  1 -> AUTO  0 -> MANUAL 
extern bool LowBeamStateManual;   ///< Variable for the manual low beam state     1 -> ON    0-> OFF
extern int LightLevel;  ///< Variable for the light level 
extern bool LowBeamState;  ///< Variable for the low beam state 


extern uint8_t FlashLightState; ///< Variable for the Flash -1 for LEFT, 0 for OFF, 1 for RIGHT, 2 for BOTH(Emergency) 
extern int TickCounter; ///< Counter for the flash 



extern bool DrivingMode; ///< Variable for the Driving mode TRUE -> Park Pilot  FALSE -> Manual  
extern uint8_t stage;  ///< Stage counter of the Park Pilot 
extern int UltrasonicSensors[6];   ///< Array to store the data of the Ultrasonuc sensors



extern int TurningDirection; ///< Variable for the turning directon -1 -> LEFT    0 -> Straight    1 -> RIGHT 
extern int SteeringAngle; ///< Calculated Angle by the Park Pilot SWC 
extern int LastTurningDirection; ///< The last turning direction of the Servo



extern uint8_t DCMotorPower; ///< from 0 to 255, PWM signal 
extern short int DCMotorTorque; ///< variable in range of (-100; 100), in case of negative number the motor rotates backward, the abs|DMT| means, we use the DC motor with DMT % power. 
extern uint8_t DCMotorDirection; ///<0 if backward rotation, 1 free,  2 if forward rotation, 3 if handbrake 
extern uint8_t AccelerationDirection; ///< 0 if backward, 1 if no input, 2 if forward 
extern bool Handbrake; ///< Variable for the Handbrake 0 if OFF, 1 if ON 
extern bool Emergency; ///< Variable for the Emergency 0 if NOT, 1 in case of an emergency 
 


extern bool CommunicationEmergency; ///< Variable which says if the commHandler doesn't get data 0 if NOT, 1 in case of a communication emergency
extern int CommCounter; ///< Counter for the communication handler timeout



#define Rte_Call_ReadLowBeamMode() LowBeamMode  ///< Read function of the LowBeamMode
#define Rte_Call_ReadLowBeamStateManual() LowBeamStateManual    ///< Read function of the LowBeamStateManual
#define Rte_Call_ReadLightLevel() LightLevel    ///< Read function of the LightLevel
#define Rte_Call_ReadLowBeamState() LowBeamState    ///< Read function of the LowBeamState



#define Rte_Call_ReadFlashLightState() FlashLightState  ///< Read function of the FlashLightState
#define Rte_Call_ReadTickCounter() TickCounter  ///< Read function of the TickCounter



#define Rte_Call_ReadDrivingMode() DrivingMode  ///< Read function of the DrivingMode
#define Rte_Call_ReadStage() stage  ///< Read function of the stage
#define Rte_Call_ReadUltrasonicSensors(n) UltrasonicSensors[n]  ///< Read function of the UltrasonicSensors


#define Rte_Call_ReadTurningDirection() TurningDirection    ///< Read function of the TurningDirection
#define Rte_Call_ReadSteeringAngle() SteeringAngle  ///< Read function of the SteeringAngle
#define Rte_Call_ReadLastTurningDirection() LastTurningDirection    ///< Read function of the LastTurningDirection


#define Rte_Call_ReadDCMotorTorque() DCMotorTorque  ///< Read function of the DCMotorTorque
#define Rte_Call_ReadDCMotorPower() DCMotorPower    ///< Read function of the DCMotorPower
#define Rte_Call_ReadDCMotorDirection() DCMotorDirection    ///< Read function of the DCMotorDirection
#define Rte_Call_ReadAccelerationDirection() AccelerationDirection  ///< Read function of the AccelerationDirection
#define Rte_Call_ReadHandbrake() Handbrake  ///< Read function of the Handbrake
#define Rte_Call_ReadEmergency() Emergency  ///< Read function of the Emergency


#define Rte_Call_ReadCommunicationEmergency() CommunicationEmergency    ///< Read function of the CommunicationEmergency
#define Rte_Call_ReadCommCounter() CommCounter  ///< Read function of the CommCounter



#define Rte_Call_WriteLowBeamMode(LBM) LowBeamMode = LBM    ///< Write function of the LowBeamMode
#define Rte_Call_WriteLowBeamStateManual(LBSM) LowBeamStateManual = LBSM    ///< Write function of the LowBeamStateManual
#define Rte_Call_WriteLightLevel(LL) LightLevel = LL    ///< Write function of the LightLevel
#define Rte_Call_WriteLowBeamState(LBS) LowBeamState = LBS  ///< Write function of the LowBeamState


#define Rte_Call_WriteFlashLightState(FLS) FlashLightState = FLS    ///< Write function of the FlashLightState
#define Rte_Call_WriteTickCounter(TC) TickCounter = TC  ///< Write function of the TickCounter



#define Rte_Call_WriteDrivingMode(DM) DrivingMode = DM  ///< Write function of the DrivingMode
#define Rte_Call_WriteStage(S) stage = S    ///< Write function of the stage
#define Rte_Call_WriteUltrasonicSensors(n,US) UltrasonicSensors[n]=US   ///< Write function of the UltrasonicSensors



#define Rte_Call_WriteTurningDirection(LeftOrRight) TurningDirection = LeftOrRight  ///< Write function of the TurningDirection
#define Rte_Call_WriteSteeringAngle(degree) SteeringAngle = degree  ///< Write function of the SteeringAngle
#define Rte_Call_WriteLastTurningDirection(LeftOrRight) LastTurningDirection = LeftOrRight  ///< Write function of the LastTurningDirection
#define Rte_Call_WriteDCMotorPower(DMP) DCMotorPower = DMP  ///< Write function of the DCMotorPower



#define Rte_Call_WriteDCMotorDirection(DMD) DCMotorDirection = DMD  ///< Write function of the DCMotorDirection
#define Rte_Call_WriteAccelerationDirection(AD) AccelerationDirection = AD  ///< Write function of the AccelerationDirection
#define Rte_Call_WriteHandbrake(HB) Handbrake = HB  ///< Write function of the Handbrake
#define Rte_Call_WriteEmergency(E) Emergency = E    ///< Write function of the Emergency
#define Rte_Call_WriteDCMotorTorque(DMT) DCMotorTorque = DMT    ///< Write function of the DCMotorTorque



#define Rte_Call_WriteCommunicationEmergency(CE) CommunicationEmergency = CE    ///< Write function of the CommunicationEmergency
#define Rte_Call_WriteCommCounter(CC) CommCounter=CC    ///< Write function of the CommCounter


#endif