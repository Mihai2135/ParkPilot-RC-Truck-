/**
 * @page ParkPilot
 *  **Author:** Szabo Zoltan, Varga Rajmond
 * 
 *  **Date:** 05.08.2022
 *  ## Module's Role
 * This module will control the car to park automatically in a defined enviroment.
 */

#include <Rte.h>
#include <Park_Pilot.h>




//static uint8_t stage = 0;
static int Counter = 0;

static void FaceForwardBegin();
static void GoForwardStraight();
static void Turn_Right1();
static void GoForwardRight();
static void Turn_Left();
static void GoBackwardLeft();
static void Turn_Right2();
static void GoBackwardRight();
static void GoForwardStraight2();
static void FaceForwardEnd();
static void FaceForward1();

/**
 * First it looks if Emergency mode is Accured
 * 
 * If it is, it goes in the final stage and stops.
 * 
 * If it is not, the Park Pilot system has 10 stages. 
 * 
 * It will try to go threw them and park correctly.
 * @param[in] Emergency
 * @param[in] DrivingMode
 * @param[in] UltrasonicSensors
 * @param[out] DCmotorTorque
 * @param[out] SteeringAngle
 */

void Park_Pilot()
{   
    /*if an emergency acccured stop the parking*/
    if(EMARGENCY_ACCURED == Rte_Call_ReadEmergency()) {         /*Emergency check*/
        Rte_Call_WriteStage(FACE_FORWARD_END);                  /*Set the final stage*/
    }
    if( PARKPILOT == Rte_Call_ReadDrivingMode() )           /*Check if the Park Pilot is active*/
    {
        switch (Rte_Call_ReadStage())                  /*Looking in which stage it is*/
        {
        case FACE_FORWARD_BEGIN:
            FaceForwardBegin();         
            break;
        
        case GO_FORWARD_STARIGHT:
            GoForwardStraight();
            break;
        case TURN_RIGHT1:
            Turn_Right1();
            break;
        case GO_FORWARD_RIGHT:
            GoForwardRight();
            break;
        case TURN_LEFT1:
            Turn_Left();
            break;
        case GO_BACKWARD_LEFT:
            GoBackwardLeft();
            break;
        case TURN_RIGHT2:
            Turn_Right2();
            break;
        case GO_BACKWARD_RIGHT:
            GoBackwardRight();
            break;
        case FACE_FORWARD1:
            FaceForward1();
            break;
        case GO_FORWARD_STRAIGHT2:
            GoForwardStraight2();
            break;
        case FACE_FORWARD_END:
            FaceForwardEnd();
            break;
        default:
            break;
        }
    }
    else
    {
        Rte_Call_WriteStage(FACE_FORWARD_BEGIN);
    }


}

static void FaceForwardBegin(){                         /*Stage 0*/
    
    if ( COUNTER_VALUE_1_SEC >= Counter )           /*Wait one second to steer the wheels to straight forward*/
    {  
        Rte_Call_WriteSteeringAngle(FACE_FORWARD);  /*Face forward*/
        ++Counter;
    }
    else
    {
        Rte_Call_WriteSteeringAngle(FACE_FORWARD);
        Counter = 0;
        Rte_Call_WriteStage(Rte_Call_ReadStage()+1);    /*Going to the next stage*/
    }

}

static void GoForwardStraight()         /*Stage 1*/
{
    if( FRONT_LEFT_MIN_S1 > Rte_Call_ReadUltrasonicSensors(FRONT_LEFT) )    /*Going straight while the Front left ultrasonic sensor*/
    {                                                                       /*senses the front edge of the parking slot, then stops*/
        Rte_Call_WriteSteeringAngle(FACE_FORWARD);
        Rte_Call_WriteDCMotorTorque(STOP);
        Counter = 0;
        Rte_Call_WriteStage(Rte_Call_ReadStage()+1);            /*Going to the next stage*/
    }
    else 
    {   
        Rte_Call_WriteSteeringAngle(FACE_FORWARD);
        Rte_Call_WriteDCMotorTorque(GO_FORWARD);
    }
}

static void Turn_Right1()           /*Stage 2*/
{
    if ( COUNTER_VALUE_1_SEC >= Counter )               /*Wait 1 second to steer the wheel*/
    {  
        Rte_Call_WriteSteeringAngle(TURN_RIGHT);        /*Steering right*/
        ++Counter;
    }
    else
    {
        Rte_Call_WriteSteeringAngle(TURN_RIGHT);
        Counter = 0;
        Rte_Call_WriteStage(Rte_Call_ReadStage()+1);            /*Going to the next stage*/
    }
}

static void GoForwardRight()                    /*Stage 3*/
{
    if( REAR_LEFT_MIN_S3 < Rte_Call_ReadUltrasonicSensors(REAR_LEFT) )      /*Going forward and right until the rear left sensor*/
    {                                                                       /*senses the front edge of the parking slot, then stops*/
        Rte_Call_WriteSteeringAngle(TURN_RIGHT);
        Rte_Call_WriteDCMotorTorque(GO_FORWARD_MORE_TORQUE);
        
    }
    else 
    {   
        Rte_Call_WriteSteeringAngle(TURN_RIGHT);
        Rte_Call_WriteDCMotorTorque(STOP);
        Counter = 0;
        Rte_Call_WriteStage(Rte_Call_ReadStage()+1);            /*Going to the next stage*/
    }
}

static void Turn_Left()     /*stage 4*/
{
    if ( COUNTER_VALUE_1_SEC >= Counter )               /*Wait 1 second to steer the wheel*/
    {  
        Rte_Call_WriteSteeringAngle(TURN_LEFT);         /*steering left*/
        Rte_Call_WriteDCMotorTorque(STOP);
        ++Counter;
    }
    else
    {
        Rte_Call_WriteSteeringAngle(TURN_LEFT);
        Rte_Call_WriteDCMotorTorque(STOP);
        Counter = 0;
        Rte_Call_WriteStage(Rte_Call_ReadStage()+1);            /*Going to the next stage*/
    }
}

static void GoBackwardLeft()            /*stage 5*/
{
    if( ( REAR_LEFT_MIN_S5 > Rte_Call_ReadUltrasonicSensors(REAR_LEFT) ) && ( REAR_CENTER_MIN_S5 > Rte_Call_ReadUltrasonicSensors(REAR_CENTER) ) )
    {
        Rte_Call_WriteSteeringAngle(TURN_LEFT);
        Rte_Call_WriteDCMotorTorque(STOP);
        Counter = 0;
        Rte_Call_WriteStage(Rte_Call_ReadStage()+1);            /*Going to the next stage*/
    }
    else 
    {   
        Rte_Call_WriteSteeringAngle(TURN_LEFT);                 /*Going backward and left until the rear left and the rear center*/
        Rte_Call_WriteDCMotorTorque(GO_BACKWARD);               /*ultrasonic sensors will be under a defined treshold.*/
    }
}


static void Turn_Right2()  /*stage 6*/
{
    if ( COUNTER_VALUE_1_SEC >= Counter )               /*Wait 1 second to steer the wheel*/
    {  
        Rte_Call_WriteSteeringAngle(TURN_RIGHT);        /*Turning the wheels to right*/
        ++Counter;
    }
    else
    {
        Rte_Call_WriteSteeringAngle(TURN_RIGHT);
        Counter = 0;
        Rte_Call_WriteStage(Rte_Call_ReadStage()+1);            /*Going to the next stage*/
    }
}


static void GoBackwardRight()           /*Stage 7*/
{
    if((Rte_Call_ReadUltrasonicSensors(REAR_LEFT) >=  (Rte_Call_ReadUltrasonicSensors(FRONT_LEFT)-1)) && (Rte_Call_ReadUltrasonicSensors(FRONT_CENTER)<FRONT_CENTER_MIN_S7))
    {
        Rte_Call_WriteSteeringAngle(FACE_FORWARD);
        Rte_Call_WriteDCMotorTorque(STOP);
        Counter = 0;
        Rte_Call_WriteStage(Rte_Call_ReadStage()+1);            /*Going to the next stage*/
    }
    else 
    {   
        Rte_Call_WriteSteeringAngle(TURN_RIGHT);        /*Going backward and right until the two left side*/
        Rte_Call_WriteDCMotorTorque(GO_BACKWARD);       /*and the front center sensors will get decent data*/
    }
}


static void FaceForward1()      /*Stage 8*/
{
    
    if ( COUNTER_VALUE_1_SEC >= Counter )           /*Wait 1 second to steer the wheel*/
    {  
        Rte_Call_WriteSteeringAngle(FACE_FORWARD);  /*Steering the wheel to face forward*/
        Rte_Call_WriteDCMotorTorque(STOP);
        ++Counter;
    }
    else
    {
        Rte_Call_WriteSteeringAngle(FACE_FORWARD);
        Rte_Call_WriteDCMotorTorque(STOP);
        Counter = 0;
        if (Rte_Call_ReadUltrasonicSensors(FRONT_CENTER) > 60){     /*There were cases when the car not parked properly deep,*/
            Rte_Call_WriteStage(FACE_FORWARD_END);                  /*then the front center doesnt sensed the front edge of the parking slot*/
        }                                                           /*and the car just ran away. That's why we need to check this.*/
        else
        {
            Rte_Call_WriteStage(Rte_Call_ReadStage()+1);            /*Going to the next stage*/
        }
    }
}

static void GoForwardStraight2()  /*Stage 9*/
{
    if( Rte_Call_ReadUltrasonicSensors(FRONT_CENTER) <= Rte_Call_ReadUltrasonicSensors(REAR_CENTER) )       /*Moving the car to the middle of the parking slot*/
    {
        Rte_Call_WriteStage(Rte_Call_ReadStage()+1);            /*Going to the next stage*/
    }
    else
    {
        Rte_Call_WriteSteeringAngle(FACE_FORWARD);
        Rte_Call_WriteDCMotorTorque(GO_FORWARD);
    }
}

static void FaceForwardEnd()        /*Stage 10*/
{
    Rte_Call_WriteSteeringAngle(FACE_FORWARD);      /*Final stage, stopping the car, and set the wheels to face forward*/
    Rte_Call_WriteDCMotorTorque(0);
}


    
