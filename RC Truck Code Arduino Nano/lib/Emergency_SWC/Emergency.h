/// @file Emergency.h

#ifndef Emergency_h
#define Emergency_h

#define CLOSEDISTANCE 3     ///< If something is closer than this value to one of the Ultrasonic Sensors it gives Emergency
#define EMERGENCY true      ///< Tells if we are in emergency mode
#define NOEMERGENCY false   ///< Tells if we are not in emergency mode
#define MAXCOUNTER 10       ///< If the Comm Handler handler doesn't get data for this number of ticks it gives Emergency mode ON
void EmergencyRequest();

#endif