#define CONSTANTS_H
#define CONSTANTS_H

/* ******************** STRATEGY ***************************** */

#define RIGHT        0                      
#define LEFT         1
#define REGULAR_TURN 0
#define BLIND_TURN   1
#define STAR_SEEK    0
#define WAIT_ENEMY    1

/* **********************  TURN TIMES ****************************** */
//those are time constants for the blind turn movement

#define BACKWARD_TURN_TIME           400

#define RIGHT_ROTATE_TIME            400
#define LEFT_ROTATE_TIME             400

/* ********************* MOTORS **************************** */

//Revert motor (1 or -1)
#define RIGHT_MOTOR_INVERTER   -1
#define LEFT_MOTOR_INVERTER    -1

//Those Constants Set the stop points to each motor
#define STOP_PWM_LEFT        132  // 134 for usb.
#define STOP_PWM_RIGHT         132  //  134 for usb.

#define MOTOR_MAX_PWM 255
#define RIGHT_MOTOR_RISE_PWM_RATE        (MOTOR_MAX_PWM - STOP_PWM_RIGHT) / 100.0
#define RIGHT_MOTOR_DOWN_PWM_RATE        STOP_PWM_RIGHT / 100.0
#define LEFT_MOTOR_RISE_PWM_RATE        (MOTOR_MAX_PWM - STOP_PWM_LEFT) / 100.0
#define LEFT_MOTOR_DOWN_PWM_RATE        STOP_PWM_LEFT / 100.0

/* ****************** ROBOT CONSTANTS ******************************* */

#define NUMBER_OF_SENSORS             5
#define MAXIMUM_NOT_SEEING_TIMES 200

/* ******************* TIMING CONSTANTS ****************************** */

//Timing Constants
#define INITIAL_DELAY                        4800        // Delay before start runing
#define MIN_SENSOR_READ_TIME        1000        // Minimum time beetwen sensor reads in uS
#define DT                                                 5                // Code Runs every DT mS  (for a fixed dt on PID controll)
#define TIME_BLIND    1000

/* ******************* SENSORS CONSTANTS ****************************** */

#define LEFT_EDGE_SENSOR_THRESHOLD  70 //we need to get experimental values in the dojo         
#define RIGHT_EDGE_SENSOR_THRESHOLD 70
#define ALPHA   1


/* ********************** PRINT CONTROL *************************** */

#define BAUD_RATE                    9600

#define ANALISE_SWITCHES_PRINT        0
#define CONTROL_MOTORS_PRINT          0
#define FOLLOW_FUNCTION_PRINT         0
#define READ_OPPONENT_SENSORS_PRINT   0
#define READ_EDGE_SENSORS_PRINT       0
#define IREMOTE_PRINT                 0
#define BUTTON_PRINT                  0   
#define SERIAL_BEGIN                 (ANALISE_SWITCHES_PRINT + CONTROL_MOTORS_PRINT + FOLLOW_FUNCTION_PRINT + READ_OPPONENT_SENSORS_PRINT + READ_EDGE_SENSORS_PRINT + IREMOTE_PRINT+BUTTON_PRINT) 
/**************************** PINS ******************************/        
// PIN 6, A1, A2 NOT USED!!!!
# define  DIAGONAL_LEFT_SENSOR RB0_bit
# define  LEFT_SENSOR RB1_bit
# define CENTRAL_SENSOR RB2_bit
# define RIGHT_SENSOR RB3_bit
# define DIAGONAL_RIGHT_SENSOR  RB4_bit
# define BORDER_SENSOR_LEFT  RB5_bit
# define BORDER_SENSOR_RIGHT RB6_bit
# define POWER_BUTTON RB7_bit
# define REMOTE_CONTROL_PIN RA0_bit
# define LEFT_MOTOR RA1_bit
# define RIGHT_MOTOR RA2_bit
# define BATTERY_SENSOR_PIN RA3_bit
# define SWITCH_1_PIN RA4_bit
# define SWITCH_2_PIN RC0_bit
# define SWITCH_3_PIN RC1_bit
# define  LED_DEBUG RC2_bit

/*
enum ROBOT_PINS
{
     DIAGONAL_LEFT_SENSOR,
     LEFT_SENSOR,
     CENTRAL_SENSOR,
     RIGHT_SENSOR,
     DIAGONAL_RIGHT_SENSOR,
     BORDER_SENSOR_LEFT ,
     BORDER_SENSOR_RIGHT,
     POWER_BUTTON ,
     REMOTE_CONTROL_PIN,
     LEFT_MOTOR,
     RIGHT_MOTOR ,
     BATTERY_SENSOR_PIN ,
     SWITCH_1_PIN,
     SWITCH_2_PIN ,
     SWITCH_3_PIN,
     LED_DEBUG,
};
    */
/********************** STRATEGY VARIABLES **************************/
int strategy[3]; // 0 = initial side , 1 = blind turn, 2 = not used
                 //initial side stores the side the robot should turn
 unsigned long timeTurn = 0;
 unsigned long initialTimeTurn = 0;
/****************** RETURN FLAGS **************************/

enum RETURN
{
        READINGS_UNCHANGED,
        READINGS_DONE,
        DIDNT_READ, 
        LEFT_EDGE_DETECTED, 
        RIGHT_EDGE_DETECTED,
        BOTH_EDGE_DETECTED, 
        INSIDE_ARENA, 
        KEEP_GOING, 
        STAR, 
        FOLLOW, 
        RAMPANDO_GERAL, 
        QUALQUER_MERDA,
        ENEMY_FOUND,
        ENEMY_NOT_FOUND,
        OK
};

/****************** REMOTE CONTROL **************************/
//IRrecv irrecv(REMOTE_CONTROL_PIN);
//decode_results results;

/******************* SENSOR READING *************************/

int opponentSensorsReadings[NUMBER_OF_SENSORS];
int sensorsOn = 0;
int timesNotSeeing = 0;


# define DIAGONAL_LEFT_SENSOR
# define LEFT_SENSOR
# define CENTRAL_SENSOR
# define RIGHT_SENSOR
# define DIAGONAL_RIGHT_SENSOR


int rightEdge;    //bool rightEdge
int leftEdge;      //bool leftEdge

int leftEdgeSensorValue = 1024;
int rightEdgeSensorValue = 1024;

/*************** PID Controll *******************/
float dt;
float kp, ki, kd;
float integral, derivative, proportional;
float error = 0;
float lastError = 0;

static const float SENSOR_WEIGHTS[] = {-2.3, -1, 0, 1, 2.3};

unsigned long int lastRun;
/*************** STAR *******************/

//unsigned long int starRotationTime;

/***************** FUNCTIONS *****************/  

void   analiseSwitches();
int blindTurn[];
void  controlMotors(float leftPower, float rightPower);
void  checkStopMotor();
int follow();
float rotate();
void  initialDelay(unsigned int deadline);
void   initialTurn();
void  initRobot();
void  justGo();
void  printEdgeSensors();
void printEdgeLuminosityValues();
void  printPID();
void  printOpponentSensors();
void printSwitches();
int readEdgeSensors();
int  readOpponentSensors();
void  serialEvent();
void  setPidConstants();
int   star();
void  startTimer();
void  stopRobot();
void testTimer(float initialTime, float testTime);
int verifyEdgeSensors();
void  waitStartButton();
void catandoInimigoBolado();