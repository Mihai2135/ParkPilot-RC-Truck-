/// @file DC_Motor.cpp
#include <Arduino.h>
#include <Rte.h>
#include <DC_Motor.h>


static uint8_t LastDirection = FREERUNNING;

void SetDCMotorParameters () 
{
    uint8_t dcMotorPWM = Rte_Call_ReadDCMotorPower(); ///< Initializes the variable with the current DCMotorPower
    
    /// This is the emergency mode for the dc motor
    if ( ( (PARKPILOT_ON == Rte_Call_ReadDrivingMode())&&(EMERGENCY_ON == Rte_Call_ReadEmergency()) )||( HANDBRAKE_ON == Rte_Call_ReadHandbrake() ) || ( ( (EMERGENCY_ON == Rte_Call_ReadCommunicationEmergency())  )  ) ) /* If emergency mode is on or someone pulled the handbrake */
    { 
        Rte_Call_WriteDCMotorPower( MAXPWM ); ///< In the case of forced motor brake the PWM that goes to the dc motor must be the maximum PWM
        Rte_Call_WriteDCMotorDirection( HANDBRAKE ); ///< If in emergency mode the car must stop as soon as possible
        LastDirection = HANDBRAKE;

    }

    /// This is the normal operating mode for the dc motor
    else 
    {
        /// This is the Park Pilot mode for the dc motor
        if ( PARKPILOT_ON == Rte_Call_ReadDrivingMode() )
        { 
            dcMotorPWM = ( uint8_t ) abs( ( PWMSCALINGFACTOR * Rte_Call_ReadDCMotorTorque() ) ); ///< Scaling the input from the park pilot swc

            if ( Rte_Call_ReadDCMotorTorque() < ZERO_TORQUE )
            {
                Rte_Call_WriteDCMotorDirection( BACKWARD ); ///< If the Park Pilot wants to go backward, the the dc motor direction is set to backward running mode
            }
            else if ( Rte_Call_ReadDCMotorTorque() > ZERO_TORQUE )
            {
                Rte_Call_WriteDCMotorDirection( FORWARD ); ///< If the Park Pilot wants to go forward, the the dc motor direction is set to forward running mode
            }
            else 
            {
                Rte_Call_WriteDCMotorDirection( HANDBRAKE ); ///< If there is neither forward or backward command, then the dc motor will be set to hand brake mode, for faster stopping time sake
                dcMotorPWM = MAXPWM; ///< In the case of forced motor brake the PWM that goes to the dc motor must be the maximum PWM
                LastDirection = HANDBRAKE;
            }
        }

        /// This is the Manual mode for the dc motor
        else
        { 
            /// This is the forward going mode when in manual mode
            if ( FORWARD_ACC == Rte_Call_ReadAccelerationDirection() )
            {
                if ( ( BACKWARD_ACC == LastDirection ) || ( HANDBRAKE == LastDirection ) ) 
                {
                    dcMotorPWM = 0; ///< If the last direction was backward or handbrake, then the cdMotorPWM should be set to 0 before incrementing
                } 
                else 
                {
                    if ( MAXPWM != dcMotorPWM )
                    {
                        ++dcMotorPWM; ///< If the last direction was forward then increment the dcMotorPWM to rotate faster
                    }
                }
                LastDirection = FORWARD; ///< Changing the variable that stores the information of the previous rotation direction of the dc motor
                Rte_Call_WriteDCMotorDirection( FORWARD ); ///< 
            }

            /// This is the backward going mode when in manual mode
            else if ( BACKWARD_ACC == Rte_Call_ReadAccelerationDirection() )
            { 
                if ( ( FORWARD_ACC == LastDirection ) || ( HANDBRAKE == LastDirection ) ) 
                {
                    dcMotorPWM = 0; ///< If the last direction was forward or handbrake, then the cdMotorPWM should be set to 0 before incrementing
                } else 
                {
                    if ( MAXPWM != dcMotorPWM )
                    {
                        ++dcMotorPWM; ///< If the last direction was backward then increment the dcMotorPWM to rotate faster
                    }
                }
                LastDirection = BACKWARD; ///< Changing the variable that stores the information of the previous rotation direction of the dc motor
                Rte_Call_WriteDCMotorDirection( BACKWARD );
            }
            else 
            {
                dcMotorPWM = 0; ///< When no button is pressed, the PWM that contorls the dc motor will be set to 0 to operate in freerunning mode
                Rte_Call_WriteDCMotorDirection( FREERUNNING ); 
                LastDirection = FREERUNNING;
            }

            if (dcMotorPWM > MAXPWM)
            {
                dcMotorPWM = MAXPWM; ///< If the calculated PWM signal exceeds the maximum allowable PWM, then the variable will be set to the maximum PWM
            }
            
        }
        Rte_Call_WriteDCMotorPower( dcMotorPWM ); ///< Update the dc motor PWM
    }
}