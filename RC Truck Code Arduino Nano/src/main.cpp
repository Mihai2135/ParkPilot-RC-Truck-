//#define IO_DEBUG
#include <TaskScheduler.h>

#include <Comm_Hand.h>
#include <DC_Motor.h>
#include <Flasher.h>
#include <IO_Hand.h>
#include <Light_Sensor.h>
#include <Low_Beam.h>
#include <Park_Pilot.h>
#include <Servo_Motor.h>
#include <Emergency.h>

// Scheduler
Scheduler ts;

/*Task periods*/
#define PERIOD1 10
#define PERIOD2 20

/*The Callback function for the 10 ms task*/
void Tasks10ms(){
  IOHandler10msTasks();
  EmergencyRequest();
  Park_Pilot();
  SetServoDriveSignal();
  SetDCMotorParameters();
}

/*The Callback function for the 20 ms task*/
void Tasks20ms(){
  
  IOHandler20msTasks();
  Comm_Handler();
  flasher();
  lowbeam();
}

/*The 10 and the 20 ms task*/
Task Task10ms ( PERIOD1 * TASK_MILLISECOND, TASK_FOREVER , &Tasks10ms, &ts, true );
Task Task20ms ( PERIOD1* TASK_MILLISECOND, TASK_FOREVER , &Tasks20ms, &ts, true );

void setup() {
  /*Initialize the ESP communication*/
  initCommHandler();
  /*Initialize the IO communication communication*/
  IOHandlerSetup();

}

void loop() {

  /*Start the task scheduler*/
  ts.execute();
  
}
