/**
 * @page LowBeam
 * **Author:** Molnar Andrei
 *
 * **Date:** 20.07.2022
 * ## Module's Role
 * This module implements the low beam lights into the RC Car.
 */
#include <Low_Beam.h>
#include <Rte.h>
#include <Arduino.h>

/**
 * This implements the low beam lights on the RC Car.
 *
 * It has two modes: auto and anual.
 *
 * If in auto mode, the low beam will be turned on based on the darkness outside.
 * It works with a light sensor module.
 *
 * If in manual mode, the low beam can be turned ON or OFF by a button.
 * @param[in] LightLevel            Light level temporary variable.
 * @param[in] LowBeamMode           Low beam mode variable.
 * @param[in] LowBeamStateManual    Low beam state variable set manually.
 * @param[out] localLightLevel      Light level variable.
 * @param[out] LowBeamState         Low beam state variable.
 */
void lowbeam()
{
  int localLightLevel = Rte_Call_ReadLightLevel(); /* Assign value of light sensor to a temporary variable */
  if (Rte_Call_ReadCommunicationEmergency() == REQUESTED)
  {
    Rte_Call_WriteLowBeamState(HEADLIGHTON);
  }
  else
  {
    /* This is the automatic mode for the low beam lights */
    if (MODAUTO == Rte_Call_ReadLowBeamMode())
    {
      if (MAXIMUMTHRESHOLD < localLightLevel)
      {
        Rte_Call_WriteLowBeamState(HEADLIGHTON); /* In the abscence of light, the low beam is turned on */
      }
      else if (MINIMUMTHRESHOLD > localLightLevel)
      {
        Rte_Call_WriteLowBeamState(HEADLIGHTOFF); /* In the presence of light the low beam is turned off */
      }
    }

    /* This is the manual mode for the low beam lights */
    else if (MODMANUAL == Rte_Call_ReadLowBeamMode())
    {
      if (LOWBEAMON == Rte_Call_ReadLowBeamStateManual())
      { /* If the low beam button is pressed, the lights will turn on */
        Rte_Call_WriteLowBeamState(HEADLIGHTON);
      }
      else if (LOWBEAMOFF == Rte_Call_ReadLowBeamStateManual())
      { /* If the low beam button is pressed again, the lights will turn off */
        Rte_Call_WriteLowBeamState(HEADLIGHTOFF);
      }
    }
  }
}