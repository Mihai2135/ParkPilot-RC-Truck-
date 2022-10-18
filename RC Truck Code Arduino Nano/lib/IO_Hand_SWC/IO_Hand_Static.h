#ifndef IO_Hand_Static_h
#define IO_Hand_Static_h

#ifndef PRIVATE_FUNC
#define PRIVATE_FUNC static
#endif
#ifndef PRIVATE_DATA
#define PRIVATE_DATA static
#endif

#include <Arduino.h>
/*

PRIVATE_DATA uint8_t interruptFlagDCMotor_ui8;
PRIVATE_DATA uint8_t interruptFlagServo_ui8;
PRIVATE_DATA int lastLightLevel_i;

PRIVATE_DATA uint8_t UltrasonicInterruptFlag;
PRIVATE_DATA uint8_t UltrasonicMeasuredTime;
PRIVATE_DATA uint8_t UltrasonicIndex;
PRIVATE_DATA uint8_t UltrasonicTriggerPins[ ULTRASONIC_MAX_INDEX + 1 ];
PRIVATE_DATA uint8_t UltrasonicDistances [ ULTRASONIC_MAX_INDEX+1 ];
PRIVATE_DATA uint8_t UnfilteredDistance;
*/

PRIVATE_FUNC void ConfigureTimer1();
PRIVATE_FUNC void ConfigureTimer2();
PRIVATE_FUNC void SetupServoAndDCMotor();
PRIVATE_FUNC void SendCommandToDCMotorUnscaled( uint8_t DCMotorPower_ui8, uint8_t DCMotorDirection_ui8 );
PRIVATE_FUNC void SendCommandToDCMotorScaled( uint8_t DCMotorPower_ui8, uint8_t DCMotorDirection_ui8 );
PRIVATE_FUNC void SendCommandToServoMotor( int ServoAngle );
PRIVATE_FUNC void SetupLowBeam();
PRIVATE_FUNC void SendCommandToLowBeam( bool LowBeamState_b);
PRIVATE_FUNC void SetupFleshLights();
PRIVATE_FUNC void SendCommandToFleshLights( uint8_t FleshLightState );
PRIVATE_FUNC void SetupLightSensor();
PRIVATE_FUNC int ReadAndFilterDataFromLightSensor();
PRIVATE_FUNC void InitializeTimer7ms();
PRIVATE_FUNC void SetupUltrasonicSensor();
PRIVATE_FUNC void GiveOutTriggerAndStartTimers (uint8_t TriggerPin );
PRIVATE_FUNC void SelectUltrasonicMux( uint8_t SelectIndex );
PRIVATE_FUNC uint8_t RotateIndex ( uint8_t Index );
PRIVATE_FUNC void ReadUltrasoicSensorsWithRotation();
void IOHandlerSetup();
void IOHandler10msTasks();
void IOHandler20msTasks();

ISR (TIMER1_COMPA_vect);
ISR (TIMER1_COMPB_vect);
ISR ( TIMER1_OVF_vect );
ISR( TIMER2_COMPA_vect );
ISR( INT0_vect );

#endif