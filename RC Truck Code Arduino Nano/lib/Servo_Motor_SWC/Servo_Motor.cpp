/**
 * @page ServoMotor
 *  **Author:** Szabo Zoltan
 * 
 *  **Date:** 25.07.2022
 *  ## Module's Role
 * This module gets the turning direction and gives the angle for the Servo between -45 and +45 degree.
 */

#include <stdbool.h>
#include <Rte.h>
#include <Servo_Motor.h>

/**
 * It looks if the park pilot is on, then it transmits the calculated angle from the park pilot to the I/O Handler threw the Rte
 * 
 * If the Perk Pilot is off, then it looks for the value of the turning direction.
 * 
 * If it is 0 then it should give 0 for the steering angle.
 * 
 * If the turning direction is -1 it decrements the angle
 * 
 * If the turning direction is +1 it oncrements the angle
 * 
 * At the end there is a safety check, not to increment the angle to more then the Max or Minimum angle.
 * @param[in] Drivingmode       It tells if the Park Pilot is on/off
 * @param[in] SteeringAngle     The calculated angle by the Park Pilot SWC
 * @param[in] TurningDirection  The value which decides if we need to increment or decrement the angle
 * @param[out] degree           The Angle given for the I/O handler
 */

static float incrementation = 1; /* initializing the incrementation */

void SetServoDriveSignal()
{
    int degree = Rte_Call_ReadSteeringAngle(); /* reading the current angle of the Servo */

    if (PARKPILOTON == Rte_Call_ReadDrivingMode())  /*If the Park Pilot is ON*/
    {
        int SteeringAngle1 = Rte_Call_ReadSteeringAngle();  /*Gets the calculated angle*/
        if (SteeringAngle1 > MAXANGLE)      /*Safety check*/
        {
            SteeringAngle1 = MAXANGLE;
        }
        else
        {
            if (SteeringAngle1 < MINANGLE)     /*Safety check*/
            {
                SteeringAngle1 = MINANGLE;
            }
            else
            {
                /* do nothing */
            }
        }

        /* degree = NEUTRALANGLE + SteeringAngle1;
        int percentage = degree * 100 / 180;
        Rte_Call_WriteServoDutyCicle( ServoPeriod1 * percentage / 100 );
        Rte_Call_WriteServoPWMPeriod( ServoPeriod1 ); */
        Rte_Call_WriteSteeringAngle(SteeringAngle1);        /*Giving the angle for the I/O handler*/
    }
    else                                            /*If the Park Pilot is not active*/
    {
        int CurrentTurningDirection = Rte_Call_ReadTurningDirection();  /*Reads the Turning direction*/
        if (CurrentTurningDirection == FACE_FORWARD)  /*If we don't need to steer we will increment or decrement the angle to 0*/
        {
            if (degree < 0)
            {
                degree ++;      
            }
            else
            {
                if (degree > 0)
                {
                    degree --;
                }
                else
                {
                    /*Do nothing*/
                }
            }
        }
        else
        {
            degree += ( CurrentTurningDirection * incrementation);          /*If the turning directoin is not 0 then we increment or decrement the angle with the help of the turning direction*/
        }                                                                      /*If it is -1 we will decrement the degree because of the multiplication, then if it's +1 we will increment*/

        if (degree < (MINANGLE))           /*Safety check*/
        {
            degree = MINANGLE;
        }
        else
        {
            if (degree > (MAXANGLE))            /*Safety check*/
            {
                degree = MAXANGLE;
            }
            else
            {
                /* do nothing */
            }
        }
        /* int percentage = degree * 100 / 180;
        if ( percentage >= 100 )
            percentage = 100;
        Rte_Call_WriteServoDutyCicle( ServoPeriod1 * percentage / 100 );
        Rte_Call_WriteServoPWMPeriod( ServoPeriod1 ); */
        /*Rte_Call_WriteLastTurningDirection(Rte_Call_ReadTurningDirection());*/
        Rte_Call_WriteSteeringAngle(degree);                                            /*Giving the angle for the I/O handler*/
    }
}