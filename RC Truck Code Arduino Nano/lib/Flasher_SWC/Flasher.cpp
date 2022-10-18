/**
 *  @page Flasher
 * **Author:** Molnar Andrei
 * 
 * **Date:** 01.08.2022
 * ## Module's Role
 * This module implements the flashing lights into the RC Car.
*/

#include <Flasher.h>   
#include <Rte.h>

/**
     * Reads actual car status, if in case of emergency, turns on the hazard lights. 
     * 
     * If not in emergency mode, it will blink the flashers based on the steering directions.
     * 
     * The flashers are blinking for a specific time (320ms ON and 420ms OFF). 
     * 
     * The time is converted in ticks (1 tick = 20ms).
     * @param[in] Emergency            Emergency Mode variable.
     * @param[in] SteeringAngle        Steerging angle variable.
     * @param[in] TickCounter          Tick counter variable.
     * @param[out] Flashing_Mode       Turns on the flashers.
     * @param[out] FlashLightState     Set the time for blinking.
    */
void flasher()
{
    int Flashing_Mode;

    /* If in emergency mode, turns on hazard lights */
    if( ( ( Rte_Call_ReadEmergency() == REQUESTED ) && (true == Rte_Call_ReadDrivingMode()) ) || ( Rte_Call_ReadCommunicationEmergency() == REQUESTED ) )  
    {
        /* Turns on the lights needed for Emergency Mode */
        Flashing_Mode = BOTH; 
    }

    /* Otherwise it will work based on the steering angle */
    else 
    {
        if( true == Rte_Call_ReadDrivingMode() )
        {
            Flashing_Mode = LEFT;
        }
        else
        {
            /* If there is no request, flashers will be turned off */
            Flashing_Mode = NEUTRAL;  

            /* If the steering direction is to the left, blink left flasher */
            if( Rte_Call_ReadSteeringAngle() > NEUTRAL )   
            {
            /* Turns on left flasher */
            Flashing_Mode = LEFT;   
            }
            /* If the steering direction is to the right, blink right flasher */
            if( Rte_Call_ReadSteeringAngle() < NEUTRAL ) 
            {
            /* Turns on right flasher */
            Flashing_Mode = RIGHT;
            }
        }
        if( Rte_Call_ReadStage() == FACE_FORWARD_END )
        {
            Flashing_Mode = NEUTRAL;
        }
    }

    /* Sets the time for the flashers to blink */
    if( Rte_Call_ReadTickCounter() < 32)  /* Turns on flasher for 320ms */
    {
        switch ( Flashing_Mode )
        {
        case BOTH:
            {
                Rte_Call_WriteFlashLightState(BOTH);
                break;
            }
        case LEFT:
            {
                Rte_Call_WriteFlashLightState(LEFT);
                break;
            }
        case RIGHT:
            {
                Rte_Call_WriteFlashLightState(RIGHT);
                break;
            }
        default:
            {
                Rte_Call_WriteFlashLightState(NEUTRAL);
                break;
            }
        }
    }
    else 
    {
        /* When the flasher reaches a cycle of 740ms, it resets the tick counter */
        if( Rte_Call_ReadTickCounter() < 72 )      
        {
            /* Turns off blinker */
            Rte_Call_WriteFlashLightState(NEUTRAL); 
        }
        else 
        {
            /* This resets the counter */
            Rte_Call_WriteTickCounter(0);    
        }
    }
    /* Incremented by 1 so it doesn't stuck between 0 and 37 ticks */
    Rte_Call_WriteTickCounter(Rte_Call_ReadTickCounter()+1);  
}