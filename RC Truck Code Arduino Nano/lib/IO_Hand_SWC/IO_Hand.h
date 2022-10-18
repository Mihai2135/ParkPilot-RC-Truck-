/// @file IO_Hand.h

#ifndef IO_Hand_h
#define IO_Hand_h

#ifndef PRIVATE_FUNC
#define PRIVATE_FUNC static
#endif
#ifndef PRIVATE_DATA
#define PRIVATE_DATA static
#endif


/*The used pins*/

#define SERVO_PIN 3 ///< Defines which pin of the arduino is connected to the servo motor.

#define DCMOTOR_PWM_PIN 5 ///< Defines which pin of the arduino goes to the DC motor enable.
#define DCMOTOR_FORWARD_PIN 4 ///< Defines which pin of the arduino goes to the DC motor forward pin.
#define DCMOTOR_BACKWARD_PIN 7  ///< Defines which pin of the arduino goes to the DC motor backward pin.

#define LOWBEAM_PIN 13 ///< Defines which pin of the arduino is connected to the low beam.

#define LIGHT_SENSOR_PIN A6 ///< Defines which pin of the arduino is connected to the light sensor.

#define TURN_LEFT_SIGNAL_PIN 6 ///< Defines which pin of the arduino is connected to the left fleshing lights.
#define TURN_RIGHT_SIGNAL_PIN 12 ///< Defines which pin of the arduino is connected to the right fleshing lights.

/*The configuration of the DC Motor driving PWM signal*/
#define MAX_DCMOTOR_POWER 255.0 ///< Defines the maximum value of the incoming data what contains the desired DC motor power. When the incoming data reaches this level, that means the DC motor should run with the maximum allowed power.
#define MIN_DCMOTOR_DUTY_CYCLE 0.25 ///< Defines the minimum duty cycle of the DC motor driving PWM signal, with what the DC motor can move the car.
#define MAX_DCMOTOR_DUTY_CYCLE 0.40 ///< Defines the maximum duty cycle of the DC motor driving PWM signal, with what the DC motor is allowed to run.
#define DEFAULT_DCMOTOR_DUTY_CYCLE 0.001 ///< Defines the duty cycle of the DC motor driving PWM signal when there was no other command given.


/*The direction of the DC motor*/
#define BACKWARD ( uint8_t ) 0 ///< Defines the backward command for the dc motor.
#define FREERUN ( uint8_t ) 1 ///< Defines the freerun command for the dc motor.
#define FORWARD ( uint8_t ) 2 ///< Defines the forward command for the dc motor.
#define HANDBRAKE ( uint8_t ) 3 ///< Defines the handbreak command for the dc motor.

/*Servo configuration*/
#define NEUTRAL_ANGLE 95 ///< Defines the angle of the servo motor with what the car is facing forward.
#define MIN_ANGLE -45 ///< Defines the maximum angle with what we can tun the wheels left, compared to the NEUTRAL_ANGLE.
#define MAX_ANGLE 45 ///< Defines the maximum angle with what we can tun the wheels roght, compared to the NEUTRAL_ANGLE.
#define DEFAULT_SERVO_DUTY_CYCLE 0.07555 ///< Defines the duty cycle of the Servo motor driving PWM signal when there was no other command given.

/*Low beam config*/
#define LOWBEAM_ON true ///< Defines beam on state.
#define LOWBEAM_OFF false ///< Defines beam off state.
#define DEFAULT_LOWBEM_STATE LOWBEAM_OFF ///< Defines the state of the low beam when there was no other command given.

/*Light sensor config*/
#define LIGHT_SENSOR_SMOOTHING_FACTOR 0.5 ///< Defines the smoothing factor of the low pass filter, for the measured light level filtering. It is between 0 and 1. If it's 1, we have no filtering.
#define DEFAULT_LIGHT_LEVEL 0 ///< Defines the starter value of the light level.
#define MIN_LIGHT_LEVEL_FROM_SENSOR 0 ///< Defines the minimum value what we accept from the light sensor.
#define MAX_LIGHT_LEVEL_FROM_SENSOR 1000 ///< Defines the maximum value what we accept from the light sensor.

/*Flash light config*/
#define FLESH_LIGHTS_OFF 0 ///< Defines the flesh lights of state.
#define FLESH_LIGHTS_ON 1 ///< Defines the both flesh lights on state.
#define FLESH_LIGHT_LEFT_ON 2 ///< Defines the left flesh lights on state.
#define FLESH_LIGHT_RIGHT_ON 3 ///< Defines the right flesh lights on state.

/*Timer1 config:*/
/*The TOP value for the timer modules*/
#define TOP_TIMER1 (uint16_t)(20000) ///< Defines the TOP value of the timer1 (16 bit timer) to generate the desired PWM period .

/*The interrupt flegs for  the timer 1 (For the DC and Servo motor)*/
#define LAST_INTERRUPT_OVERFLOW 1 ///< Defines the value of an interrupt flag to indicate that the last interrupt came from overflow.
#define LAST_INTERRUPT_FROM_COMPARE 0 ///< Defines the value of an interrupt flag to indicate that the last interrupt came from compare.


/*EVERYTHING FOR THE ULTRASONIC AND TIMER 2*/


/*Ultrasonic pins*/
#define ULTRASONIC0_TRIGGER_PIN A0 ///< Defines which pin is the front center ultrasonic sensor trigger pin (FRONT_CENTER).
#define ULTRASONIC1_TRIGGER_PIN A1 ///< Defines which pin is the front left ultrasonic sensor trigger pin (FRONT_LEFT).
#define ULTRASONIC2_TRIGGER_PIN A2 ///< Defines which pin is the front right ultrasonic sensor trigger pin (FRONT_RIGHT).
#define ULTRASONIC3_TRIGGER_PIN A3 ///< Defines which pin is the front center ultrasonic sensor trigger pin (REAR_CENTER).
#define ULTRASONIC4_TRIGGER_PIN A4 ///< Defines which pin is the front left ultrasonic sensor trigger pin (REAR_LEFT).
#define ULTRASONIC5_TRIGGER_PIN A5 ///< Defines which pin is the front right ultrasonic sensor trigger pin (REAR_RIGHT).

#define ULTRASONIC1_ECHO_PIN 2 ///< Defines which pin is the ultrasonic sensor echo pin (it is the output of the MUX).

#define MUX_SEL0_PIN 8 ///< Defines which pin is the MUX select 0.
#define MUX_SEL1_PIN 9 ///< Defines which pin is the MUX select 1.
#define MUX_SEL2_PIN 10 ///< Defines which pin is the MUX select 2.
#define MUX_SEL3_PIN 11 ///< Defines which pin is the MUX select 3.

/*Ultrasonic sensor indexes*/

enum UltrasonicIndexes { ///< An enum to contain the Ultrasonic Sensor indexes.
    FRONT_CENTER = 0, ///< Front center Ultrasonic Sensor index.
    REAR_CENTER = 1, ///< Rear center Ultrasonic Sensor index.
    REAR_RIGHT = 2, ///< Rear right Ultrasonic Sensor index.
    FRONT_LEFT = 3, ///< Front left Ultrasonic Sensor index.
    FRONT_RIGHT = 4, ///< Front right Ultrasonic Sensor index.
    REAR_LEFT = 5 ///< Rear center Ultrasonic Sensor index.
};

/*Ultrasonic sensor index configuration*/
#define ULTRASONIC_FIRST_TO_READ FRONT_CENTER ///< Defines which ultrasonic sensor to read first.
#define ULTRASONIC_MIN_INDEX 0 ///< Defines the minimum ultrasonic sensor index.
#define ULTRASONIC_MAX_INDEX 5 ///< Defines the maximum ultrasonic sensor index.
#define ULTRASONIC_START_INDEX 6 ///< Defines the starter ultrasonic sensor index, this indicates this is the first read.
#define ULTRASONIC_INDEX_INCREMENTATION 1 ///< Defines the incrementation of the ultrasonic sensor index.

#define SPEED_OF_SOUND 0.0343 ///< It is the speed of sound (km/s).

#define MAX_DISTANCE_CM 121 ///< Defines the maximum distance we want to measure.
#define DEFAULT_DISTANCE MAX_DISTANCE_CM ///< Defines the starter distance

/*Timer 2 config*/
#define TOP_TIMER2_FOR_7MS 110 ///< defines the TOP of the timer2 to generate a signal with 7ms period (with the used configuration)

#define ULTRASONIC_SMOOTHING_FACTOR 0.7  ///< Defines the smoothing factor of the low pass filter, for the measured distance filtering. It is between 0 and 1. If it's 1, we have no filtering.

/*The interrupt flggs so I can know if I am waiting for falling or rising edge on the echo pin*/
#define ULTRASONIC_INTERRAPT_NOT_ACCURED 0 ///< Defines the value of a flag what indicates that there was no rising edge interrupt before
#define ULTRASONIC_INTERRAPT_ACCURED 1 ///< Defines the value of a flag what indicates that there was a rising edge interrupt before

/*The necesarry includes*/
#include <Arduino.h>
#include "Rte.h"

/*The public functions*/
void IOHandlerSetup();      /*Sets up the IO communication, and outpust the default values to the modules*/
void IOHandler10msTasks();  /*The 10 ms operations should be called from here: DC and Servo motor*/
void IOHandler20msTasks();  /*The 20 ms operations should be called from here: Light sensor, Low beam and Flesh lights*/

#endif