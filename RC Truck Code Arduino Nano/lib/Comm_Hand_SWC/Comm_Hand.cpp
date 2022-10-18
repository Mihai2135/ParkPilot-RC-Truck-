/**
 * @page CommHandler
 *  **Author:** Szabo Zoltan
 * 
 *  **Date:** 01.08.2022
 *  ## Module's Role
 * This module transmits the data from the ESP to the Rte.
 */

#include <Comm_Hand.h>
#include <Rte.h>
#include <Arduino.h>


using namespace std;

/**
 * Gets one bite and splits into bits.
 * 
 * Then gives the necesary data to the Rte.
 * 
 * If it doesn't get data it increments a counter.
 * 
 * If the counter reaches the max value, the Emergency SWC will give Emergency Mode.
 * @param[in] inputByte                 The incoming Byte from the ESP.
 * @param[out] AccelerationDirection    Tells if we need to accelerate or decelerate
 * @param[out] TurningDirection         Tells which side we need to steer
 * @param[out] Handbrake                Tells if the handbrake is on/off
 * @param[out] DrivingMode              Tells if we are in Manual/Park Pilot
 * @param[out] LowBeamMode              Tells if the light mode is Auto/Manual
 * @param[out] LowBeamStateManual       If we are in Manual Light mode, it tells if the headlights are ON/OFF
 */

void initCommHandler()
{   
    Serial.begin(115200);
    pinMode ( 0 , INPUT );  
    Rte_Call_WriteCommCounter(0);
}

void Comm_Handler()
{
    if( Serial.available() > 0 )            /*Looks if is there incoming data*/
    {
        Rte_Call_WriteCommCounter(0);       /*Reset the counter*/
        int inputByte = Serial.read();      /*Reads the Byte*/
        Serial.flush();
        if ( ( (inputByte) & (forwardBit) ) == forwardBit )     /*If forward button is pressed*/
        {
            Rte_Call_WriteAccelerationDirection(FORWARD);      /*Gives forward Acceleration*/       
        }
        else 
        {
            if ( ( (inputByte) & (backwardBit) ) == backwardBit )   /*If Backward button is pressed*/
            {
                Rte_Call_WriteAccelerationDirection(BACKWARD);  /*Gives backward Acceleration*/
            }
            else 
            {
                Rte_Call_WriteAccelerationDirection(FREERUNNING);   /*If the forward and backward buttons are not pressed gives freerunning mode*/
            }
        }
        

        if ( ( (inputByte) & (leftBit) ) == leftBit )       /*If the Left button is pressed*/
        {
            Rte_Call_WriteTurningDirection(TURN_LEFT);      /*Tells to steer left*/
        }
        else 
        {
            if ( ( (inputByte) & (rightBit) ) == rightBit ) /*If the Right button is pressed*/
            {
                Rte_Call_WriteTurningDirection(TURN_RIGHT);       /*Tells to steer Right*/    
            }
            else 
            {
                Rte_Call_WriteTurningDirection(FACE_FORWARD);   /*If the Left and Right buttons are not pressed it tells to face forward*/
            }
        }


        if ( ( (inputByte) & (handbrakeBit) ) == handbrakeBit )      /*If the handbrake button is pressed*/
        {
            Rte_Call_WriteHandbrake(HANDBRAKE_ON);      /*Turns on the handbrake*/
        }
        else
        {
            Rte_Call_WriteHandbrake(HANDBRAKE_OFF);      /*Turns off the handbrake*/
        }



        if( ( (inputByte) & (drivingmodeBit) ) == drivingmodeBit )  /*If the parkpilot button is pressed*/
        {
            Rte_Call_WriteDrivingMode(PARKPILOT_ON);        /*Activates the automatic parking system*/
        }
        else 
        {
            Rte_Call_WriteDrivingMode(PARKPILOT_OFF);   /*Deactivates the automatic parking system*/
        }


        if( ( (inputByte) & (lightautoBit) ) == lightautoBit )  /*If the auto light button is pressed*/
        {
            Rte_Call_WriteLowBeamMode(AUTOLIGHT);       /*Tells that we are in Automatic Light mode*/
        }
        else
        {   
            //Rte_Call_WriteLowBeamMode(true);
            
            Rte_Call_WriteLowBeamMode(MANUALLIGHT);         /*Tells that we are in Manual Light mode*/
            if (  ( (inputByte) & (lightonBit) ) == lightonBit )    /*If we are not in auto light mode it looks if the manual light button is ON/OFF*/
            {
                Rte_Call_WriteLowBeamStateManual(ONLIGHT);  /*Tells that the headlights needs to be turned on manually*/
            }
            else 
            {
                Rte_Call_WriteLowBeamStateManual(OFFLIGHT);    /*Tells that the headlights needs to be turned of manually*/
            }
            
         }
        
    }
    else 
    {
        if( Rte_Call_ReadCommCounter() <= MAXCOUNTER )   /*If the comm handler doesn't get data and the counter doesnt reached the maximum value*/
        {
           Rte_Call_WriteCommCounter(Rte_Call_ReadCommCounter()+1); /*We increment the counter*/
        }
        else 
        {
            /*DO NOTHING*/
        }
    }
}