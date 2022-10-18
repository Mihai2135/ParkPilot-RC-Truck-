/**
 * @page Emergency
 *  **Author:** Szabo Zoltan
 * 
 *  **Date:** 02.08.2022
 *  ## Module's Role
 * This module decides if we are in emergency mode or not.
 */

#include <Rte.h>
#include <Emergency.h>

/**
 * It looks if there is something too close to one of the six ultrasonic sensors
 * 
 * If there is, it gives Emergency mode
 * 
 * It looks if the Comm Handler gets data from the ESP
 * 
 * If it doesn't get data for the Maxcounter number of ticks it gives Emergency mode
 * @param[in] UltrasonicSensors         The distances from the 6 ultrasonic sensors to the closest object
 * @param[in] CommCounter               The Number of ticks the Comm Handler doesn't get data
 * @param[out] EmergencyMode            Tells if we are in Emergency or not
 */

void EmergencyRequest()
{
    /*Check if we are in close range emergency*/
    for(int i=0; i<6; ++i)
    {
        if ( Rte_Call_ReadUltrasonicSensors(i) < CLOSEDISTANCE )
        {
            Rte_Call_WriteEmergency(EMERGENCY); /*Turns on the Emergency mode*/
            break;
            //return;
        }
        else
        {
            Rte_Call_WriteEmergency(NOEMERGENCY);   /*Turns off the Emergency mode*/
        }
    }
    /*Check if we are in communication emergency, communication timeout*/
    if ( Rte_Call_ReadCommCounter() >= MAXCOUNTER )
    {
        Rte_Call_WriteCommunicationEmergency(EMERGENCY);    /*Turns on the Emergency mode*/
        return;
    }
    else
    {
        Rte_Call_WriteCommunicationEmergency(NOEMERGENCY);  /*Turns off the Emergency mode*/
    }
    
    
}