/**
 * @page IO_Hand
 * **Author: ** Varga Rajmond
 * 
 * **Date: ** 08.08.2022
 * ## Module's Role
 *  This module makes the connection between the software components (through the RTE layer) and the hardware.
*/

#include "IO_Hand.h"
#include "IO_Hand_Static.h"

/**
 * An interrupt flag.
 * 
 * It is a private variable for the DC Motor PWM generation.
 * It indicates if the last interrupt came from compare or overflow so we know if we are in rising or falling edge.
*/
PRIVATE_DATA uint8_t interruptFlagDCMotor_ui8 = LAST_INTERRUPT_FROM_COMPARE;

/**
 * An interrupt flag.
 * 
 * It is a private variable for the Servo Motor PWM generation.
 * It indicates if the last interrupt came from compare or overflow so we know if we are in rising or falling edge.
*/
PRIVATE_DATA uint8_t interruptFlagServo_ui8 = LAST_INTERRUPT_FROM_COMPARE;
 
/**
 * This variable contains the light level from the last iteration.
 * 
 * It is a private variable for the measured light level filtering.
*/
PRIVATE_DATA int lastLightLevel_i = DEFAULT_LIGHT_LEVEL;


/**
 * Configures the Timer1 (16 bit timer).
 * 
 * Sets the internal timer1 of the CPU in phase correct PWM mode.
 * Configures the PWM frequency to the desired value and enables timer interrupts. 
 * @param[in] TOP_Timer1_A
 * @param[in] TOP_Timer1_B
 * @param[out] Timer1_Configuration 
*/ 
PRIVATE_FUNC void ConfigureTimer1(){

  /*Configure timer1 (16 bit)*/
  TCCR1A = 0; /* reset Timer Control Register A*/
  TCCR1B = 0; /* reset Timer Control Register B*/

  /*Configure TCCR2A*/
  /*Deconnect the OC2A output */
  TCCR1A &= ~( 1 << COM1A1 );
  TCCR1A &= ~( 1 << COM1A0 );

  /*Connect the OC2B output */
  TCCR1A &= ~( 1 << COM1B1 );
  TCCR1A &= ~( 1 << COM1B0 );

  /*First two bit to select PWM, phase correct mode*/
  TCCR1A |= ( 1 << WGM11 );
  TCCR1A &= ~( 1 << WGM10 );

  /*Configure TCCR2B*/
  /*Last bit to select PWM, phase correct mode*/
  TCCR1B &= ~( 1 << WGM12 );
  TCCR1B |= ( 1 << WGM13 );

  /*Configure 1024 prescale*/
  TCCR1B &= ~( 1 << CS12 );
  TCCR1B |= ( 1 << CS11 );
  TCCR1B &= ~( 1 << CS10 );

  /*Configure the frequency and the duty cycle */
  OCR1A = ( uint16_t ) ( TOP_TIMER1 * DEFAULT_DCMOTOR_DUTY_CYCLE );
  OCR1B = ( uint16_t ) ( TOP_TIMER1 * DEFAULT_SERVO_DUTY_CYCLE );
  ICR1 = ( uint16_t ) ( TOP_TIMER1 );

  /*Reset timer counter value to 0*/
  TCNT1 = 0;

  /*Enable interrupts */
  TIMSK1 = 0;
  TIMSK1 |= ( 1 << OCIE1A );
  TIMSK1 |= ( 1 << OCIE1B );
  TIMSK1 |= ( 1 << TOIE1 );
}

/**
 * Configures the Timer2 (8 bit timer).
 * 
 * Sets the internal timer1 of the CPU in phase correct PWM mode.
 * Configures the PWM frequency to the desired value and enables timer interrupts.
 * It isn't used in the final version! 
 * @param[out] Timer2_Configuration
*/
PRIVATE_FUNC void ConfigureTimer2(){

    /*Configure timer2 (8 bit)*/
    TCCR2A = 0; /* reset Timer Control Register A*/
    TCCR2B = 0; /* reset Timer Control Register B*/

    /*Configure TCCR2A*/
    /*Deconnect the OC2A output */
    TCCR2A &= ~( 1 << COM2A1 );
    TCCR2A &= ~( 1 << COM2A0 );

    /*Deconnect the OC2B output */
    TCCR2A &= ~( 1 << COM2B1 );
    TCCR2A &= ~( 1 << COM2B0 );

    /*First two bit to select PWM, phase correct mode*/
    TCCR2A &= ~( 1 << WGM21 );
    TCCR2A |= ( 1 << WGM20 );

    /*Configure TCCR2B*/
    /*Last bit to select PWM, phase correct mode*/
    TCCR2B |= ( 1 << WGM22 );
    
    /*Configure 1024 prescale*/
    TCCR2B |= ( 1 << CS22 );
    TCCR2B |= ( 1 << CS21 );
    TCCR2B |= ( 1 << CS20 );
    
    /*Configure the frequency and the duty cycle */
    //OCR2A = ( uint8_t ) ( TOP_CALC );
    //OCR2B = ( uint8_t ) ( TOP_CALC * duty_cycle );

    /*Reset timer counter value to 0*/
    TCNT2 = 0;

    /*Disable interrupts */
    TIMSK2 = 0;
    TIMSK2 &= ~ ( 1 << OCIE2A );
    TIMSK2 &= ~ ( 1 << OCIE2B );
    TIMSK2 &= ~ ( 1 << TOIE2 );
}

/**
 * Configure and initialize pins for DC and servo motors, and the timer used by them.
 * It configures and inicializes the Timer1, 16 bit timer.
 * @param[in] DCMOTOR_FORWARD_PIN
 * @param[in] DCMOTOR_BACKWARD_PIN
 * @param[in] SERVO_PIN
 * @param[out] DC_and_Servo_Configuration
*/

PRIVATE_FUNC void SetupServoAndDCMotor (){
  /*Set DCMotor pin to output mode and inicialize*/
  pinMode( DCMOTOR_FORWARD_PIN, OUTPUT );
  pinMode( DCMOTOR_BACKWARD_PIN, OUTPUT );

  digitalWrite ( DCMOTOR_FORWARD_PIN, LOW );
  digitalWrite ( DCMOTOR_BACKWARD_PIN, LOW );

  /*Set Servo pin to output mode*/
  pinMode( SERVO_PIN, OUTPUT );

  /*Configure timer1 (16 bit)*/
  ConfigureTimer1();
}

/**
 * It sends command to the DC motor (unscaled).
 * 
 * As an input, it takes the desired motor power.
 * It configures Timer1 and configures the motor direction control output pins.
 * It's unscaled so just limits the incoming control isgnal if it is above the max value or below the min value.
 * It isn't used in the final version! 
 * @param[in] DCMotorPower_ui8 A variable what contains the desired DC motor power
 * @param[in] DCMotorDirection_ui8 A variable what contains the desired DC motor direction
 * @param[out] DC_Motor_Directional_Pins_Configurations
 * @param[out] Timer1_Configuration_For_Desired_DC_Motor_Driving_PWM
*/
PRIVATE_FUNC void SendCommandToDCMotorUnscaled( uint8_t DCMotorPower_ui8, uint8_t DCMotorDirection_ui8 ){
  /*A variable to calculate the duty cycle*/
  float DCMotorDutyCycle_f = DEFAULT_DCMOTOR_DUTY_CYCLE; 

  /*A temporal variable for the calculation, and the initial calculation*/
  float DCMotorDutyCycleTemp_f =  ((float)( DCMotorPower_ui8 )) / (( float ) MAX_DCMOTOR_POWER);
  
  /*Check if we are in a safe range*/
  if ( MIN_DCMOTOR_DUTY_CYCLE >  DCMotorDutyCycleTemp_f){ /*Check if we are above the minimal value*/
    /*If we are below the minimal value disable interrupts and put low to the output*/
    
    /*Disable pwm generation by disabling the interupt what generates it*/
    TIMSK1 &= ~ ( 1 << OCIE1A );

    /*Put low to output to disable the mototr*/
    digitalWrite ( DCMOTOR_PWM_PIN, LOW );

    /*We don't have to reconfigure the timer*/
    return; 
  }else{
    /*Check if we are below the maximal value*/
    if ( MAX_DCMOTOR_DUTY_CYCLE <  DCMotorDutyCycleTemp_f){
      /*If we are, we will ise the maximal allowed duty cycle*/
      DCMotorDutyCycleTemp_f = MAX_DCMOTOR_DUTY_CYCLE; 
    }else{
      /*DO nothing*/
    }
  }

  /*Refresh the DCMotor duty cycle*/
  DCMotorDutyCycle_f = DCMotorDutyCycleTemp_f;

  /*Check which mode are we in*/
  if ( HANDBRAKE == DCMotorDirection_ui8 ){ 
    /*If we are in handbreak mode*/
    digitalWrite ( DCMOTOR_FORWARD_PIN, LOW );
    digitalWrite ( DCMOTOR_BACKWARD_PIN, LOW );
  }else{
    if( FORWARD == DCMotorDirection_ui8){
      /*If we are in move forward mode*/
      digitalWrite ( DCMOTOR_FORWARD_PIN, HIGH );   //////////////   FORDITVA MUKODIK JAVITSD KIIII
      digitalWrite ( DCMOTOR_BACKWARD_PIN, LOW );
    }else{
      if (BACKWARD == DCMotorDirection_ui8 ){
        /*If we are in move backward mode*/
        digitalWrite ( DCMOTOR_FORWARD_PIN, LOW );
        digitalWrite ( DCMOTOR_BACKWARD_PIN, HIGH );
      } else {
        /*We are in freerun*/
      }
    }
  }
  
  /*Refresh the duty cycle for the DCMotor in the timer*/
  OCR1A = ( uint16_t ) ( ( ( float ) TOP_TIMER1 ) * DCMotorDutyCycle_f );
  
  /*Enable interrupt for PWM generation*/
  TIMSK1 |= ( 1 << OCIE1A );
}

/**
 * It sends command to the DC motor (scaled).
 * 
 * As an input, it takes the desired motor power.
 * It configures Timer1 and configures the motor direction control output pins.
 * It scales the incoming 0 to 255 value between min dutycycle and max dutycicle.
 * 255 will mean the MAX_DCMOTOR_DUTY_CYCLE and not a duty cycle with a value of 1.
 * 1 will mean a value close to the MIN_DCMOTOR_DUTY_CYCLE.
 * If the power is 0 then we gave 0 to the output, if it is 1, we give the minimal output what runs the car.
 * @param[in] DCMotorPower_ui8 A variable what contains the desired DC motor power
 * @param[in] DCMotorDirection_ui8 A variable what contains the desired DC motor direction
 * @param[out] DC_Motor_Directional_Pins_Configurations
 * @param[out] Timer1_Configuration_For_Desired_DC_Motor_Driving_PWM
*/ 
PRIVATE_FUNC void SendCommandToDCMotorScaled( uint8_t DCMotorPower_ui8, uint8_t DCMotorDirection_ui8 ){
  /*A variable to calculate the duty cycle*/
  float DCMotorDutyCycle_f = DEFAULT_DCMOTOR_DUTY_CYCLE; 

  /*A temporal variable for the calculation, and the initial calculation*/
  float DCMotorDutyCycleTemp_f =  DEFAULT_DCMOTOR_DUTY_CYCLE;
  
  /*Check which mode are we in*/
  if ( HANDBRAKE == DCMotorDirection_ui8 ){
    /*If we are in handbreak mode*/
    digitalWrite ( DCMOTOR_FORWARD_PIN, LOW );
    digitalWrite ( DCMOTOR_BACKWARD_PIN, LOW );
  }else{
    if( FORWARD == DCMotorDirection_ui8){
      /*If we are in move forward mode*/
      digitalWrite ( DCMOTOR_FORWARD_PIN, HIGH );   //////////////   FORDITVA MUKODIK JAVITSD KIIII
      digitalWrite ( DCMOTOR_BACKWARD_PIN, LOW );
    }else{
      if (BACKWARD == DCMotorDirection_ui8 ){
        /*If we are in move backward mode*/
        digitalWrite ( DCMOTOR_FORWARD_PIN, LOW );
        digitalWrite ( DCMOTOR_BACKWARD_PIN, HIGH );
      } else {
        /*We are in freerun*/
      }
    }

  }

  /*Check if the desired power is 0*/
  if ( 0 ==  DCMotorPower_ui8){ 
    /*If the desired power is 0*/
    /*Disable pwm generation by disabling the interupt what generates it*/
    TIMSK1 &= ~ ( 1 << OCIE1A );

    /*Put low to output to disable the mototr*/
    digitalWrite ( DCMOTOR_PWM_PIN, LOW );

    /*We don't have to reconfigure the timer*/

  } else {
    /*IF the desired power is above 0*/
    /*Calculate the duty cycle*/
    DCMotorDutyCycleTemp_f =  ((float)( DCMotorPower_ui8 )) / (( float ) MAX_DCMOTOR_POWER);
    /*Scale the duty cycle between the min and max value*/
    DCMotorDutyCycle_f = ( MAX_DCMOTOR_DUTY_CYCLE - MIN_DCMOTOR_DUTY_CYCLE ) * DCMotorDutyCycleTemp_f + MIN_DCMOTOR_DUTY_CYCLE;

    /*Refresh the duty cycle for the DCMotor in the timer*/
    OCR1A = ( uint16_t ) ( ( ( float ) TOP_TIMER1 ) * DCMotorDutyCycle_f );
    
    /*Enable interrupt for PWM generation*/
    TIMSK1 |= ( 1 << OCIE1A );
  }

}

/**
 * It sends command to the servo motor by configuring the driving PWM signal.
 * 
 * It takes the desired angle as an input.
 * To control the servo, it reconfigures Timer1 what generates the Servo driving PWM signal.
 * @param[in] ServoAngle A variable what contains the desired Servo motor angle
 * @param[out] Timer1_Configuration_For_Desired_DC_Motor_Driving_PWM
*/ 
PRIVATE_FUNC void SendCommandToServoMotor( int ServoAngle ){
    float servo_duty_cycle = DEFAULT_SERVO_DUTY_CYCLE; /*By default, for the car to face forward*/
    int AbsoluteAngle=NEUTRAL_ANGLE;/*It is between 0 and 180 degree*/

    /*Saftey check*/
    if ( ( MIN_ANGLE <= ServoAngle ) && ( MAX_ANGLE >= ServoAngle ) ){
        AbsoluteAngle = ServoAngle + NEUTRAL_ANGLE;
    }

    /*Calculation of the duty cycle*/
    servo_duty_cycle = ( ( ( float ) AbsoluteAngle * 0.05 ) / ( 180.0 ) ) + 0.05;

    /*reconfugure the internal timers duty cycle (this is the compare registers value)*/
    OCR1B = ( uint16_t ) ( TOP_TIMER1 * servo_duty_cycle );
}

/**
 * It configures the pin what is connected to the low beam to output mode and initialize it.
 * 
 * @param[in] LOWBEAM_PIN
 * @param[in] DEFAULT_LOWBEM_STATE
 * @param[out] Low_Beam_Pin_Configuration_And_Initialization
*/
PRIVATE_FUNC void SetupLowBeam(){
  /*Set LowBeam pin to output mode and inicialize*/
  pinMode ( LOWBEAM_PIN, OUTPUT );
  /*Inicializing of the low beam*/
  if ( DEFAULT_LOWBEM_STATE == LOWBEAM_OFF ){
    digitalWrite ( LOWBEAM_PIN, LOW );
  } else {
    digitalWrite ( LOWBEAM_PIN, HIGH );
  }
  
}

/**
 * The function sends command to the low beam hardware by onfiguring the output pins.
 * 
 * As an input it takes the desired low beam state.
 * @param[in] LOWBEAM_PIN
 * @param[in] LowBeamState_b A variable what stores the desired Low beam state.
 * @param[out] Low_Beam_Pin_Configuration
*/
PRIVATE_FUNC void SendCommandToLowBeam( bool LowBeamState_b){
  /*Check if we want the low beam to be turned on or off*/
  if ( LOWBEAM_ON == LowBeamState_b ){
    /*Turn the low beam on*/
    digitalWrite ( LOWBEAM_PIN, HIGH );
  } else {
    /*Turn the low beam off*/
    digitalWrite ( LOWBEAM_PIN, LOW );
  }
}

/**
 * Configure and initialize pins for the flash lights.
 * 
 * @param[in] TURN_LEFT_SIGNAL_PIN
 * @param[in] TURN_RIGHT_SIGNAL_PIN
 * @param[out] Flash_Light_Pin_Configuration_And_Initialization
*/
PRIVATE_FUNC void SetupFleshLights(){
  /*Set the pins to output mode*/
  pinMode( TURN_LEFT_SIGNAL_PIN, OUTPUT );
  pinMode( TURN_RIGHT_SIGNAL_PIN, OUTPUT );
  /*Inicialize the outputs*/
  digitalWrite( TURN_LEFT_SIGNAL_PIN, LOW );
  digitalWrite( TURN_RIGHT_SIGNAL_PIN, LOW );
}

/**
 * The functions sends command to the flash lights.
 * 
 * As an input takes the desired flash light states and outputs it to the hardware.
 * @param[in] FleshLightState A variable to store the desired flash light state
 * @param[in] TURN_LEFT_SIGNAL_PIN
 * @param[in] TURN_RIGHT_SIGNAL_PIN
 * @param[out] Flash_Light_Pin_Configuration
*/
PRIVATE_FUNC void SendCommandToFleshLights( uint8_t FleshLightState ){
  /*Check what state are we in.*/
  if ( FLESH_LIGHTS_ON == FleshLightState ){
    /*Turn both lights on*/
    digitalWrite( TURN_LEFT_SIGNAL_PIN, HIGH );
    digitalWrite( TURN_RIGHT_SIGNAL_PIN, HIGH );
  } else {
    if ( FLESH_LIGHT_LEFT_ON == FleshLightState ) {
      /*Turn the left light on*/
      digitalWrite( TURN_LEFT_SIGNAL_PIN, HIGH );
      digitalWrite( TURN_RIGHT_SIGNAL_PIN, LOW );
    } else {
      if( FLESH_LIGHT_RIGHT_ON == FleshLightState ) {
        /*Turn the right light on*/
        digitalWrite( TURN_LEFT_SIGNAL_PIN, LOW );
        digitalWrite( TURN_RIGHT_SIGNAL_PIN, HIGH );
      } else {
        /*Turn all the fleshlights off*/
        digitalWrite( TURN_LEFT_SIGNAL_PIN, LOW );
        digitalWrite( TURN_RIGHT_SIGNAL_PIN, LOW );
      }
    }
  }
}

/**
 * The functione initialize the pin what is connected to the light sensor.
 * @param[in] LIGHT_SENSOR_PIN
 * @param[out] Light_Sensor_Pin_Configuration_And_Initialization
*/ 
PRIVATE_FUNC void SetupLightSensor(){
  /*Set the pin to output mode*/
  pinMode( LIGHT_SENSOR_PIN, INPUT );
}

/**
 * The function reads and filters data from the light sensor.
 * @param[in] LIGHT_SENSOR_PIN 
 * @param[in] LIGHT_SENSOR_SMOOTHING_FACTOR
 * @param[in] Light_Level_From_The_Sensor
 * @param[out] lastLightLevel_i A variable tos store the measured and filtered light level
*/
PRIVATE_FUNC int ReadAndFilterDataFromLightSensor(){
  /*Read and filter the data*/
  lastLightLevel_i = LIGHT_SENSOR_SMOOTHING_FACTOR * analogRead(LIGHT_SENSOR_PIN) + ( 1 - LIGHT_SENSOR_SMOOTHING_FACTOR ) * lastLightLevel_i;

  /*Safety check, if we are in a safe range*/
  if( MIN_LIGHT_LEVEL_FROM_SENSOR > lastLightLevel_i ){
    /*If the light level is unreasonably low*/
    lastLightLevel_i = MIN_LIGHT_LEVEL_FROM_SENSOR;
  } else {
    if( MAX_LIGHT_LEVEL_FROM_SENSOR < lastLightLevel_i ){
      /*If the light level is unreasonably high*/
      lastLightLevel_i = MAX_LIGHT_LEVEL_FROM_SENSOR;
    }else{
      /*Do nothing, everything is fine*/
    }
  }
  /*return the value*/
  return lastLightLevel_i;
}


/*Ultrasonic sensor variables/


/*Interrupt flags*/
PRIVATE_DATA uint8_t UltrasonicInterruptFlag = ULTRASONIC_INTERRAPT_NOT_ACCURED; ///< An interrupt flag to check if this is the first echo interrupt (rising edge).

/*Variable to store the value of the timer when saved*/
PRIVATE_DATA uint8_t UltrasonicMeasuredTime = TOP_TIMER2_FOR_7MS; ///< A variable to store the value of the counter for calculating time.

/*The index what rotets the sensors*/
PRIVATE_DATA uint8_t UltrasonicIndex = ULTRASONIC_START_INDEX; ///< A variable to store the index of the last ultrasonic sensor what was read.

/*An array to store the ultrasonic trugger pins*/
PRIVATE_DATA uint8_t UltrasonicTriggerPins[ ULTRASONIC_MAX_INDEX + 1 ]; ///< An array to store the ultrasonic trigger pins.

/*An array to store Distances*/
PRIVATE_DATA uint8_t UltrasonicDistances [ ULTRASONIC_MAX_INDEX+1 ] ; ////< An array to store Distances.

/*A variable for unfiltered temporal distance*/
PRIVATE_DATA uint8_t UnfilteredDistance = MAX_DISTANCE_CM; ///< A variable to store the unfiltered temporal distance


/*Ultrasonic sensor functions*/

/**
 * The function initializes Timer2 (8 bit timer). 
 * 
 * Configure it to generate a signal with 7 ms period so we are able to measure time up to 7 ms.
 * @param[in] TOP_TIMER2_FOR_7MS
 * @param[out] Timer2_Configuration
*/ 
PRIVATE_FUNC void InitializeTimer7ms(){
  /*Configure timer2 (8 bit)*/
  TCCR2A = 0; /* reset Timer Control Register A*/
  TCCR2B = 0; /* reset Timer Control Register B*/

  /*Configure TCCR2A*/
  /*Deconnect the OC2A output */
  TCCR2A &= ~( 1 << COM2A1 );
  TCCR2A &= ~( 1 << COM2A0 );

  /*Deconnect the OC2B output */
  TCCR2A &= ~( 1 << COM2B1 );
  TCCR2A &= ~( 1 << COM2B0 );

  /*First two bit to select mode*/
  TCCR2A |= ( 1 << WGM21 );
  TCCR2A |= ( 1 << WGM20 );

  /*Configure TCCR2B*/
  /*Last bit to select mode*/
  TCCR2B |= ( 1 << WGM22 );
  
  /*Configure prescale*/
  TCCR2B |= ( 1 << CS22 );
  TCCR2B |= ( 1 << CS21 );
  TCCR2B |= ( 1 << CS20 );
  
  /*Configure the frequency and the duty cycle */
  OCR2A = ( uint8_t ) ( TOP_TIMER2_FOR_7MS );
  //OCR2B = ( uint8_t ) ( TOP_CALC * duty_cycle );

  /*Reset timer counter value to 0*/
  TCNT2 = 0;

  /*Disable interrupts */
  
  TIMSK2 &= ~ ( 1 << OCIE2A );
  TIMSK2 &= ~ ( 1 << OCIE2B );
  TIMSK2 &= ~ ( 1 << TOIE2 );
}

/**
 * The function makes setup for the multiplexer and the ultrasonic sensors.
 * 
 * It configures the pins connected to the multiplexer and the ultrasonic sensors and initialize them.
 * Configures what wil be the order of reading the 6 ultrasonic sensors and initializes the distances with an initial value.
 * It allso initialize the value of the ultrasonic index.
 * @param[in] MUX_SEL0_PIN
 * @param[in] MUX_SEL1_PIN
 * @param[in] MUX_SEL2_PIN
 * @param[in] MUX_SEL3_PIN
 * @param[in] ULTRASONIC0_TRIGGER_PIN
 * @param[in] ULTRASONIC1_TRIGGER_PIN
 * @param[in] ULTRASONIC2_TRIGGER_PIN
 * @param[in] ULTRASONIC3_TRIGGER_PIN
 * @param[in] ULTRASONIC4_TRIGGER_PIN
 * @param[in] ULTRASONIC5_TRIGGER_PIN
 * @param[in] ULTRASONIC1_ECHO_PIN
 * @param[out] Timer2_Configuration
*/
PRIVATE_FUNC void SetupUltrasonicSensor(){
  /*Configure pins*/
  pinMode(MUX_SEL0_PIN, OUTPUT);
  pinMode(MUX_SEL1_PIN, OUTPUT);
  pinMode(MUX_SEL2_PIN, OUTPUT);
  pinMode(MUX_SEL3_PIN, OUTPUT);

  pinMode(ULTRASONIC0_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC1_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC2_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC3_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC4_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC5_TRIGGER_PIN, OUTPUT);

  pinMode(ULTRASONIC1_ECHO_PIN, INPUT);

  /*Configure which pin goes to which sensor*/
  UltrasonicTriggerPins[FRONT_CENTER] = ULTRASONIC0_TRIGGER_PIN;
  UltrasonicTriggerPins[FRONT_LEFT] = ULTRASONIC1_TRIGGER_PIN;
  UltrasonicTriggerPins[FRONT_RIGHT] = ULTRASONIC2_TRIGGER_PIN;
  UltrasonicTriggerPins[REAR_CENTER] = ULTRASONIC3_TRIGGER_PIN;
  UltrasonicTriggerPins[REAR_LEFT] = ULTRASONIC4_TRIGGER_PIN;
  UltrasonicTriggerPins[REAR_RIGHT] = ULTRASONIC5_TRIGGER_PIN;

  /*SetupDistances*/
  UltrasonicDistances[ FRONT_CENTER ] = DEFAULT_DISTANCE;
  UltrasonicDistances[ FRONT_LEFT ] = DEFAULT_DISTANCE;
  UltrasonicDistances[ FRONT_RIGHT ] = DEFAULT_DISTANCE;
  UltrasonicDistances[ REAR_CENTER ] = DEFAULT_DISTANCE;
  UltrasonicDistances[ REAR_LEFT ] = DEFAULT_DISTANCE;
  UltrasonicDistances[ REAR_RIGHT ] = DEFAULT_DISTANCE;

  /*This is where we start*/
  UltrasonicIndex = ULTRASONIC_START_INDEX;

  /*Setup the timer2*/
  InitializeTimer7ms();
}

/**
 * The function give out the trigger signal to the trigger pin and starts the timer for timeout measurements.
 * 
 * Takes the trigger pin as an input and gives out the trigger on it.
 * It allso starts a 7 ms timer (timer2 8 bit) what will call an iterrupt if we didn't get a respons in time.
 * @param[in] TriggerPin A variable what stores the pin name of the sensor we want to give the trigger.
 * @param[out] Trigger_To_The_Correspondong_Sensor
 * @param[out] Timer2_Configuration
 * @param[out] Interrupt_Configuration
*/ 
PRIVATE_FUNC void GiveOutTriggerAndStartTimers (uint8_t TriggerPin ){
  /*We will have to wait for the start of the echo and not for the end of it when the trigger finishes*/
  UltrasonicInterruptFlag = ULTRASONIC_INTERRAPT_NOT_ACCURED;

  /*Give out the trigger signal*/
  digitalWrite(TriggerPin,HIGH);
  delayMicroseconds(12);
  digitalWrite(TriggerPin,LOW);



  /*Cofigure the timer*/
  TCNT2 = 0;  /*Set the timer value to 0*/
  TIMSK2 |= ( 1 << OCIE2A ); /*Enable interrupt*/

  /*Configure external interrupt to rising edge*/
  EICRA |= ( 1 << ISC00 );
  EICRA |= ( 1 << ISC01 );

  /*Enable external interrupts on INT0*/
  SREG |= ( 1 << SREG_I );
  EIMSK |= ( 1 << INT0 );

}

/*we will configure the select bits of the muxindex*/
/**
 * The function configures the multiplexer what connects the Ultrasonic sensor echo pins to the Arduino.
 * 
 * Takes the indext of the sensor what we want to connect to as an input and configures the mux select pins in the necesarry way.
 * @param[in] SelectIndex A variable what stores which sensor we want to connect to with the multiplexer.
 * @param[in] MUX_SEL0_PIN
 * @param[in] MUX_SEL1_PIN
 * @param[in] MUX_SEL2_PIN
 * @param[in] MUX_SEL3_PIN
 * @param[out] Multiplexer_Configuration
*/
PRIVATE_FUNC void SelectUltrasonicMux( uint8_t SelectIndex ){
  switch (SelectIndex)
  {
  case FRONT_LEFT:
    digitalWrite ( MUX_SEL0_PIN, LOW );
    digitalWrite ( MUX_SEL1_PIN, HIGH );
    digitalWrite ( MUX_SEL2_PIN, HIGH );
    digitalWrite ( MUX_SEL3_PIN, HIGH );
    break;
  case FRONT_CENTER:
    digitalWrite ( MUX_SEL0_PIN, HIGH );
    digitalWrite ( MUX_SEL1_PIN, HIGH );
    digitalWrite ( MUX_SEL2_PIN, HIGH );
    digitalWrite ( MUX_SEL3_PIN, HIGH );
    break;
  case FRONT_RIGHT:
    digitalWrite ( MUX_SEL0_PIN, HIGH );
    digitalWrite ( MUX_SEL1_PIN, LOW );
    digitalWrite ( MUX_SEL2_PIN, HIGH );
    digitalWrite ( MUX_SEL3_PIN, HIGH );
    break;
  case REAR_LEFT:
    digitalWrite ( MUX_SEL0_PIN, HIGH );
    digitalWrite ( MUX_SEL1_PIN, HIGH );
    digitalWrite ( MUX_SEL2_PIN, LOW );
    digitalWrite ( MUX_SEL3_PIN, HIGH );
    break;
  case REAR_CENTER:
    digitalWrite ( MUX_SEL0_PIN, LOW );
    digitalWrite ( MUX_SEL1_PIN, LOW );
    digitalWrite ( MUX_SEL2_PIN, HIGH );
    digitalWrite ( MUX_SEL3_PIN, HIGH );
    break;
  case REAR_RIGHT:
    digitalWrite ( MUX_SEL0_PIN, LOW );
    digitalWrite ( MUX_SEL1_PIN, HIGH );
    digitalWrite ( MUX_SEL2_PIN, LOW );
    digitalWrite ( MUX_SEL3_PIN, HIGH );
    break;

  default:
    /*Do nothing*/
    break;
  }
} 

/**
 * The functions rotates the index what indicates which Ultrasonic sensor comes next or was before. 
 * 
 * @param[in] Index A variable what stores the current index.
 * @param[out] Next_Index
*/
PRIVATE_FUNC uint8_t RotateIndex ( uint8_t Index ){
  if ( ULTRASONIC_START_INDEX == Index ) {
    /*If we have to start the rotation*/
    return ULTRASONIC_FIRST_TO_READ;
  } else {
    if ( ULTRASONIC_MAX_INDEX == Index) {
      /*if we reached the max, go bac to the min index*/
      return ULTRASONIC_MIN_INDEX;
    }else{
      /*Increment the index*/
      return Index + ULTRASONIC_INDEX_INCREMENTATION;
    }
  }

  /*We should never get here*/
  return ULTRASONIC_START_INDEX;
}

/**
 * The function coordonates the ultrasonic sensor readings.
 * 
 * It commands to give out the the trigger signal and start timer, rotates the index, reads the last measured echo length and calculates the distance from it.
 * It allso commands the configuration of the multiplexer.
 * 
 * @param[in] UltrasonicIndex A variable what stores the current ultrasonic sensor index.
 * @param[in] ULTRASONIC_START_INDEX
 * @param[in] UltrasonicTriggerPins An array what stores all the ultrasonic trigger pins in order.
 * @param[out] Trigger_Signal_To_The_Ultrasonic_Sensors
 * @param[out] Configure_Timer2_To_Measure_Echo_Or_Timeout
 * @param[out] Mux_Setup
*/
PRIVATE_FUNC void ReadUltrasoicSensorsWithRotation(){
  if ( ULTRASONIC_START_INDEX == UltrasonicIndex ) {
    /*Rotate the index*/
    UltrasonicIndex = RotateIndex( UltrasonicIndex );

    /*Configure mux*/
    SelectUltrasonicMux( UltrasonicIndex );

    /*Start the sensor reding process*/
    GiveOutTriggerAndStartTimers ( UltrasonicTriggerPins [ UltrasonicIndex ] );

  } else {
    /*Precess last data*/
    UnfilteredDistance = ( MAX_DISTANCE_CM * UltrasonicMeasuredTime ) / TOP_TIMER2_FOR_7MS;
    UltrasonicDistances[ UltrasonicIndex ] = ULTRASONIC_SMOOTHING_FACTOR * UnfilteredDistance + ( 1 - ULTRASONIC_SMOOTHING_FACTOR ) * UltrasonicDistances[ UltrasonicIndex ];

    /*Write data to RTE*/
    Rte_Call_WriteUltrasonicSensors ( UltrasonicIndex, UltrasonicDistances[ UltrasonicIndex ] );

    /*Rotate index*/
    UltrasonicIndex = RotateIndex( UltrasonicIndex );

    /*Configure MUX*/
    SelectUltrasonicMux( UltrasonicIndex );

    /*Start the sensor reding process*/
    GiveOutTriggerAndStartTimers ( UltrasonicTriggerPins [ UltrasonicIndex ] );
  }
}


/**
 * This public function will call all the necesarry setup functions. 
 * @param[in] All_The_Setup_Functions
 * @param[out] Sets_Up_All_The_Hardware_Connected_Parts
*/
void IOHandlerSetup(){
    /*DC motor and Servo motor*/
    SetupServoAndDCMotor ();

    /*Low beam*/
    SetupLowBeam();

    /*Fleshlight*/
    SetupFleshLights();

    /*Light sensor*/
    SetupLightSensor();

    /*Ultrasonic sensor*/
    SetupUltrasonicSensor();
}

/**
 * This public function will call all the functions which has to run in every 10 ms. 
 * @param[in] All_The_Functions_What_Should_Run_In_Every_10ms
 * @param[out] Runs_all_The_Functions_What_Should_Run_In_Every_10ms
*/
void IOHandler10msTasks(){
  /*Send data to DC motor*/
  SendCommandToDCMotorScaled( Rte_Call_ReadDCMotorPower (), Rte_Call_ReadDCMotorDirection () );

  /*Send data to Servo motor*/
  SendCommandToServoMotor( Rte_Call_ReadSteeringAngle () );

  /*Read Ultrasonic sensors*/
  ReadUltrasoicSensorsWithRotation(); /*The RTE write function is called in this function*/
};

/**
 * This public function will call all the functions which has to run in every 20 ms. 
 * @param[in] All_The_Functions_What_Should_Run_In_Every_20ms
 * @param[out] Runs_all_The_Functions_What_Should_Run_In_Every_20ms
*/
void IOHandler20msTasks(){
  /*Low beam*/
  SendCommandToLowBeam(Rte_Call_ReadLowBeamState());
  
  /*Fleshlight*/
  SendCommandToFleshLights(Rte_Call_ReadFlashLightState());

  /*Light sensor*/
  Rte_Call_WriteLightLevel(ReadAndFilterDataFromLightSensor());
};


/*Interrupt functions*/


/*Interrupt for DCMotor PWM*/
/**
 * This function is the compare A interrupt function of the Timer1, it is used by the DC motor.
 * The function will take care of inverting the pin which goes to the DC motor enable to generate PWM signal.
 * It follows if we have to turn the output to low or hight based on the counting direction of the timer (Timer1 16 bit timer). 
*/
ISR (TIMER1_COMPA_vect){
  if ( LAST_INTERRUPT_OVERFLOW == interruptFlagDCMotor_ui8 ){
    /*It is the first compare interrupt, set the output to low and set the flag to 0*/
    digitalWrite( DCMOTOR_PWM_PIN, LOW );
    interruptFlagDCMotor_ui8 = LAST_INTERRUPT_FROM_COMPARE;
  } else {
    /*It is the second compare interrupt, set the output to high*/
    digitalWrite( DCMOTOR_PWM_PIN, HIGH );
  }
}

/*Interrupt for Servo motor PWM*/
/**
 * This function is the compare B interrupt function of the Timer1, it is used by the servo motor.
 * The function will take care of inverting the pin which goes to the servo motor to generate PWM signal.
 * It follows if we have to turn the output to low or hight based on the counting direction of the timer (Timer1 16 bit timer). 
*/
ISR (TIMER1_COMPB_vect){
  if ( LAST_INTERRUPT_OVERFLOW == interruptFlagServo_ui8 ){
    /*It is the first compare interrupt, set the output to low and set the flag to 0*/
    digitalWrite( SERVO_PIN, LOW );
    interruptFlagServo_ui8 = LAST_INTERRUPT_FROM_COMPARE;
  } else {
    /*It is the second compare interrupt, set the output to high*/
    digitalWrite( SERVO_PIN, HIGH );
  }
}

/**
 * This function is the overflow interrupt function of Timer1.
 * This function sets a flag if we reached 0 with the counting, so we know if we will count up or down while the next compare interrupt function comes.
 * It is used for the DC and Servo motor PWM signal generation.
*/
ISR ( TIMER1_OVF_vect ){
  /*Set the flags to 1 whan an overflow happened*/
  interruptFlagDCMotor_ui8 = LAST_INTERRUPT_OVERFLOW;
  interruptFlagServo_ui8 = LAST_INTERRUPT_OVERFLOW;
}

/*Iterrupt rutine for timer 2 comparator A for the ultrasonic sensors*/
/**
 * This function is the compare A interrupt function of the Timer2, it is used by the ultrasonic sensor measurements.
 * It gets called if we didn't get a rising edge of the echo in 7 ms, or the echo took longer than 7 ms.
 * If we didn't get an echo, it is an errorr and we won't refresh the measured time but disable the interrupts.
 * If the echo didn't end, it took too long, we set the measured time to the max value and we disable the interrupts.
*/
ISR( TIMER2_COMPA_vect ){
  /*Check if we are waiting for the start of the echo or the end of it*/
  if( ULTRASONIC_INTERRAPT_NOT_ACCURED == UltrasonicInterruptFlag ){
    /*We are waiting for the echo - > there was an errorr, don't refresh the measured time just disable interrupts*/
    /*Disable external interrupts*/
    
    SREG &= ~ ( 1 << SREG_I );
    EIMSK &= ~ ( 1 << INT0 );
    
    /*Disable timer interrupts*/
    TIMSK2 &= ~ ( 1 << OCIE2A );
  } else {
    /*We are waiting for the end of the echo, the measured time is too long, set it to the max value*/
    /*Set the measured time to the max +1*/
    UltrasonicMeasuredTime = TOP_TIMER2_FOR_7MS+1;
    /*Disable external interrupts*/
    SREG &= ~ ( 1 << SREG_I );
    EIMSK &= ~ ( 1 << INT0 );

    /*Disable timer interrupts*/
    TIMSK2 &= ~ ( 1 << OCIE2A );
  }

}

/**
 * This function is the interrupt rutine of the external interrupt 0.
 * It is used by the Ultrasonic sensor measurements.
 * It keeps track if we are waiting for rising or falling edge.
 * If we got a rising edge, that is the start of the echo and we have to configure to wait for falling edge and set up the timer.
 * If it was a falling edge, we save the value of the timer and disable the interrupts.
 * It allso handles the flag what indicats if we are waiting for falling or rising edge.
*/ 
ISR( INT0_vect ){
  /*Check if this is the rising edge echo interrupt*/
  if ( ULTRASONIC_INTERRAPT_NOT_ACCURED == UltrasonicInterruptFlag ){
    /*This is the start of the echo, the rising edge*/
    /*Config timer*/
    TCNT2 = 0;  /*Set the timer value to 0*/
    TIMSK2 |= ( 1 << OCIE2A ); /*Enable interrupt*/
  
    /*Config interrupts*/
    /*Configure external interrupt to falling edge*/
    EICRA &= ~ ( 1 << ISC00 );
    EICRA |= ( 1 << ISC01 );
    /*Enable external interrupts*/
    SREG |= ( 1 << SREG_I );
    EIMSK |= ( 1 << INT0 );

    /*Tell that this was the first  interrupt*/
    UltrasonicInterruptFlag = ULTRASONIC_INTERRAPT_ACCURED;
  }else{
    /*This is the end of the echo, the falling edge*/
    /*Save the time*/
    UltrasonicMeasuredTime = TCNT2;
    
    /*Disable timer interrupts*/
    TIMSK2 &= ~ ( 1 << OCIE2A );

    /*Tell that this was the second  interrupt and next time we will have to wait for the rising edge*/
    UltrasonicInterruptFlag = ULTRASONIC_INTERRAPT_NOT_ACCURED;

    /*Disable external interrupts*/
    SREG &= ~ ( 1 << SREG_I );
    EIMSK &= ~ ( 1 << INT0 );
  }
  
}