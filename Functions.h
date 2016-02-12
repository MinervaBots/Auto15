#include "Constants.h"




/********** SETUP **********/
void initRobot()
{
  //set pin modes
  //R        76543210
   TRISB = 0b11111111;  // all RB ports are set as input
   TRISA = 0b00000;     // all RA ports are set as output
   TRISC = 0b00000000;
   UART1_Init(9600);
   Delay_ms(100);
   ADC_Init();
   



 
/*
  pinMode(LEFT_SENSOR,INPUT);
  pinMode(CENTRAL_SENSOR,INPUT);
  pinMode(RIGHT_SENSOR,INPUT);
  pinMode(DIAGONAL_RIGHT_SENSOR,INPUT);

  pinMode(BORDER_SENSOR_LEFT,INPUT);
  pinMode(BORDER_SENSOR_RIGHT,INPUT);

  pinMode(POWER_BUTTON,INPUT);
  pinMode(REMOTE_CONTROL_PIN,INPUT);
  
  pinMode(RIGHT_MOTOR,OUTPUT);
  pinMode(LEFT_MOTOR,OUTPUT);

  pinMode(BATTERY_SENSOR_PIN, INPUT);

  pinMode(SWITCH_1_PIN, INPUT);
  pinMode(SWITCH_2_PIN, INPUT);
  pinMode(SWITCH_3_PIN, INPUT);
  
  pinMode(LED_DEBUG, OUTPUT);
  */
  
  // Ensure when the arduino is on, it will provide +-2,5V to stop motors
  analogWrite(RIGHT_MOTOR, STOP_PWM_RIGHT);
  analogWrite(LEFT_MOTOR, STOP_PWM_LEFT);

  //irrecv.enableIRIn();
  
}

/********** BEGINNING **********/
void initialDelay (unsigned int deadline){
  int timeFlag = false;
  unsigned long int initialTime = millis();
  LED_DEBUG= 1;
  
  while (!timeFlag)
  {
    readOpponentSensors();  
    
    if (millis() - initialTime >= deadline)
      timeFlag = true;
  }
  LED_DEBUG = 0;
}


void analiseSwitches()
{
  
  if (SWITCH_1_PIN == 0)
  {
    strategy[0] = RIGHT;
    //error = -1;
  }
  else
  {
    strategy[0] = LEFT; // button pressed down = LEFT
    //error = 1;
  }
  
  if (SWITCH_2_PIN== 0)
  {
    strategy[1] = REGULAR_TURN;
  }
  else
  {
    strategy[1] = BLIND_TURN; 
  }  

  if (SWITCH_3_PIN == 0)
  {
    strategy[2] = STAR_SEEK;
  }
  else
  {
    strategy[2] = WAIT_ENEMY;
  }
  
  #if (ANALISE_SWITCHES_PRINT)

      printSwitches();

      if(strategy[0] == RIGHT)
        UART1_Write_text( "RIGHT" );
      else
        UART1_Write_text( "LEFT" );
      if(strategy[1] == REGULAR_TURN)
        UART1_Write_text( "REGULAR TURN" );
      else
        UART1_Write_text( "BLIND TURN" );
      if(strategy[2] == REGULAR_TURN)
        UART1_Write_text( "STAR SEEK" );
      else
        UART1_Write_text( "SOMETHING" );
  #endif
     
}

void printSwitches()
{
    UART1_Write_Text("Sw1: ");
    UART1_Write(digitalRead(SWITCH_1_PIN));
    UART1_Write_Text("\t");
    UART1_Write_Text("Sw2: ");
    UART1_Write(digitalRead(SWITCH_2_PIN));
    UART1_Write_Text("\t");
    UART1_Write_Text("Sw3: ");
    UART1_Write(digitalRead(SWITCH_3_PIN));
    UART1_Write_Text("\t");
    UART1_Write_Text(13);
}

void waitStartButton()
{
  #if (IREMOTE_PRINT)
    UART1_Write_Text( "Remote control code to start: 0xFF02FD" );
  #endif
    
  while(1)
  {
    if (POWER_BUTTON)
    {
      #if (BUTTON_PRINT)
        UART1_Write_Text( "Button pressed, robot starting..." );
        UART1_Write(13);
      #endif
      break;
    }
    if (irrecv.decode(&results)) 
    {
      #if (IREMOTE_PRINT)  
        UART1_Write_Text(results.value, HEX);
      #endif  
      
      if (results.value == 0xFF02FD)
      {
        irrecv.resume(); // Receive the next value
        break;  
      }
      irrecv.resume(); // Receive the next value
    }
  }
}

int blindTurn()
{
  if(strategy[0] == RIGHT)
  {
   controlMotors(30,100); //35, 100    
   delay(670);
   
   while( !(opponentSensorsReadings[1] || opponentSensorsReadings[2] || opponentSensorsReadings[3] ) )
   {
    controlMotors(-100, 100);
    readOpponentSensors();
    checkStopMotor();
   }
   
  }
  if(strategy[0] == LEFT)
  {
   controlMotors(100, 50);
   delay(650);
   
   while( !(opponentSensorsReadings[1] || opponentSensorsReadings[2] || opponentSensorsReadings[3] ) )
   {
    controlMotors(100, -100);
    readOpponentSensors();
    checkStopMotor();
   }   
  }
}

int seekEnemy(int LastSide)
{
  
 while(!sensorsOn){
   timeTurn = 0;
   initialTimeTurn = 0;  
   if (LastSide == RIGHT)
   {
    
    initialTimeTurn= millis();
    readOpponentSensors(); 
     while(1)
     {
      timeTurn= millis(); 
      readOpponentSensors();
      controlMotors(-100, 100);
      if(sensorsOn){
        return ENEMY_FOUND;
      }
      if((timeTurn - initialTimeTurn) > BACKWARD_TURN_TIME){
        break;
      }
     }
     
     initialTimeTurn= millis(); 
     while(1)
     {
      timeTurn= millis(); 
      readOpponentSensors();
      controlMotors(100, -100);
      if(sensorsOn){
        return ENEMY_FOUND;
      }
      if((timeTurn - initialTimeTurn) > BACKWARD_TURN_TIME){
        break;
      }
     }
   }
  
   if (LastSide == LEFT) 
   {
    
    initialTimeTurn= millis();
    readOpponentSensors(); 
     while(1)
     {
      timeTurn= millis(); 
      readOpponentSensors();
      controlMotors(100, -100);
      if(sensorsOn){
        return ENEMY_FOUND;
      }
      if((timeTurn - initialTimeTurn) > BACKWARD_TURN_TIME){
        break;
      }
     }
     
     initialTimeTurn= millis(); 
     while(1)
     {
      timeTurn= millis(); 
      readOpponentSensors();
      controlMotors(-100, 100);
      if(sensorsOn){
        return ENEMY_FOUND;
      }
      if((timeTurn - initialTimeTurn) > BACKWARD_TURN_TIME){
        break;
      }
     }
   }
 }
 return ENEMY_FOUND;   
}
void initialTurn()
{
  timeTurn = 0;
  initialTimeTurn = 0;
  
  if(strategy[1])
    blindTurn();
  else
  {
   if (strategy[0] == RIGHT )
    controlMotors(-100, 100);
   else
    controlMotors(100, -100);
   /*if (strategy[0] == RIGHT)
   {
    
    initialTimeTurn = millis();
    readOpponentSensors(); 
     while(1)
     {
      timeTurn= millis(); 
      readOpponentSensors();
      controlMotors(-100, 100);
      if(sensorsOn){
        break;
      }
      if((timeTurn - initialTimeTurn) > BACKWARD_TURN_TIME){
        break;
      }
     }
     
     initialTimeTurn= millis(); 
     while(1)
     {
      timeTurn= millis(); 
      readOpponentSensors();
      controlMotors(100, -100);
      if(sensorsOn){
        break;
      }
      if((timeTurn - initialTimeTurn) > BACKWARD_TURN_TIME){
        break;
      }
     }
   }
  
   if (strategy[0] == LEFT) 
   {
    initialTimeTurn= millis();
    readOpponentSensors(); 
     while(1)
     {
      timeTurn= millis(); 
      readOpponentSensors();
      controlMotors(100, -100);
      if(sensorsOn){
        break;
      }
      if((timeTurn - initialTimeTurn) > BACKWARD_TURN_TIME){
        break;
      }
     }
     
     initialTimeTurn= millis(); 
     while(1)
     {
      timeTurn= millis(); 
      readOpponentSensors();
      controlMotors(-100, 100);
      if(sensorsOn){
        break;
      }
      if((timeTurn - initialTimeTurn) > BACKWARD_TURN_TIME){
        break;
      }
     }
   }
   */
  }
}

void testTimer(float initialTime, float testTime)
{
    if(millis() - initialTime > testTime)
  {
    while(1)
    {
      LED_DEBUG = 1 ;
      controlMotors(0,0);
    }
  }
}
/********** MOTORS **********/
void controlMotors(double leftPower, double rightPower)
{
  leftPower = leftPower * LEFT_MOTOR_INVERTER;
  rightPower = rightPower* RIGHT_MOTOR_INVERTER;

  if (leftPower > 100) leftPower = 100;
  if (leftPower < -100) leftPower = -100;

  if (rightPower > 100) rightPower = 100;
  if (rightPower < -100) rightPower = -100;
  
  if (leftPower >= 0)
    analogWrite(LEFT_MOTOR, (STOP_PWM_LEFT + (LEFT_MOTOR_RISE_PWM_RATE * leftPower)));  
  else
    analogWrite(LEFT_MOTOR, (STOP_PWM_LEFT + (LEFT_MOTOR_DOWN_PWM_RATE * leftPower)));  
  
  if (rightPower > 0)
    analogWrite(RIGHT_MOTOR, STOP_PWM_RIGHT + RIGHT_MOTOR_RISE_PWM_RATE * rightPower);
  else
    analogWrite(RIGHT_MOTOR, STOP_PWM_RIGHT + RIGHT_MOTOR_DOWN_PWM_RATE * rightPower);  
  
  
  
  #if (CONTROL_MOTORS_PRINT)
    UART1_Write();
    UART1_Write_text( "Left PWM: " );

    if(leftPower > 0)
      UART1_Write( STOP_PWM_LEFT + LEFT_MOTOR_RISE_PWM_RATE * leftPower);
    else
      UART1_Write( STOP_PWM_LEFT + LEFT_MOTOR_DOWN_PWM_RATE * leftPower);

    UART1_Write_text( "\tRight PWM: ");

    if(rightPower > 0)
      UART1_Write( STOP_PWM_RIGHT + RIGHT_MOTOR_RISE_PWM_RATE * rightPower);
    else
      UART1_Write( STOP_PWM_RIGHT + RIGHT_MOTOR_DOWN_PWM_RATE * rightPower);

  #endif
  
}

int star()
{
  /*
  long int starRotationTime = millis();
  int edgeControl;
  readOpponentSensors();
  sensorsOn = 0;
  if( sensorsOn != 0 )
    return ENEMY_FOUND;
  
  //Serial.println("star");
  edgeControl = verifyEdgeSensors();
  
  if(edgeControl != INSIDE_ARENA)
    {
     //Serial.println("borda encontrada");
     starRotationTime = millis();
     
     do{
      Serial.print("curva");
        readOpponentSensors();
        if( sensorsOn != 0 )
          return ENEMY_FOUND;
     }while(millis() - starRotationTime < STAR_ANGLE_TIME_CONTROL);
    }

  controlMotors(100,100);
  return STAR;
  */
}

void checkStopMotor()
{ 
  if(POWER_BUTTON == 1)
  {
    while(1)
    {
      controlMotors( 0, 0);
      digitalWrite(LED_DEBUG, HIGH);
      delay(500);
      digitalWrite(LED_DEBUG, LOW);
      delay(500);      
      irrecv.resume();
      UART1_Write_Text("DEBUG: MOTORS STOPPED!");
      UART1_Write(13);
      
    }
  }

  if (irrecv.decode(&results)) 
  {
    #if (IREMOTE_PRINT)  
      Serial.println(results.value, HEX);
    #endif  
    
    if (results.value == 0xFF4AB5)
    {
      irrecv.resume(); // Receive the next value
      while(1)
      {
        controlMotors( 0, 0);
        LED_DEBUG = 1;
        Delay_ms(500);
        LED_DEBUG = 0;
        Delay_ms(500);
        irrecv.resume();
        UART1_Write_Text("DEBUG: MOTORS STOPPED!");
        UART1_Write(13);
      }  
    }
    irrecv.resume(); // Receive the next value
  }
}

/********** PID **********/

float PIDControlerRUN(double error){

    if((millis() - lastRun) > 2)
    {
      /*proportional = kp * error;
      return proportional + integral + derivative;
    }*/

    integral += ki * error * 0.002;
    derivative = kd * (error - lastError) / 0.002;
    lastError = error;
    proportional = kp * error;
    lastRun = millis();
    return (proportional + integral + derivative);
    }
   return (proportional + integral + derivative);
}

void setConstants( float proportional, float integrative, float derivative)
{
  kp = proportional;
  ki = integrative;
  kd = derivative;
}

float rotate()
{        
  float power = 0;
  sensorsOn = 0;
          
  readOpponentSensors();

  if( sensorsOn == 0 )
    {
      int starControl = 0;
    
      #if(FOLLOW_FUNCTION_PRINT)
          
        UART1_Write_Text( "Nenhum sensor esta vendo" );
        UART1_Write(13);
      
      #endif
    }
  else
  {
    error = 0;

    //aply weights to the sensors read
    for(int i = 0; i < NUMBER_OF_SENSORS; i++)
    {
      error += SENSOR_WEIGHTS[i] * opponentSensorsReadings[i];
    }
  
    error = error/sensorsOn;
    power = PIDControlerRUN(error);

    #if(FOLLOW_FUNCTION_PRINT)

      UART1_Write_Text( "Algum sensor viu:\t" );
      UART1_Write_Text("Error: ");
      UART1_Write(error);
      UART1_Write_Text("\t");
      UART1_Write_Text("Power: ");
      UART1_Write(power);
      UART1_Write_Text("\t");
      UART1_Write_Text("sensorsOn : ");
      UART1_Write(sensorsOn);
      UART1_Write(13);
     
    #endif

//saturate power

    if (error > 0)
     {
      controlMotors(abs(power), - abs(power));
     }
    else if (error < 0)
     {
      controlMotors(- abs(power),abs(power)); 
     }
    else if (error == 0)
     {
      controlMotors(0,0);
      integral = 0;
     }
  
  }
  return error;        
}

int follow()
{        
  float power = 0;
  sensorsOn = 0;
          
  readOpponentSensors();

  if( sensorsOn == 0 )
  {
    /*
     if (error > 0)
      controlMotors( 100, -100 );
     if (error < 0)
      controlMotors( -100, 100 );
     if (error == 0)
      controlMotors( 100, 100 ); */
      
      #if(FOLLOW_FUNCTION_PRINT)
          
        UART1_Write_Text( "Nenhum sensor esta vendo" );
      
      #endif
        // turn(70 * error / abs(error) );
      return ENEMY_NOT_FOUND;
    }
  else
  {
    timesNotSeeing = 0;
    //lastTimeSeen = millis();
    error = 0;

    //aply weights to the sensors read
    for(int i = 0; i < NUMBER_OF_SENSORS; i++)
    {
     error += SENSOR_WEIGHTS[i] * opponentSensorsReadings[i];
    }
  
    error = error/sensorsOn;
    power = PIDControlerRUN(error);

    #if(FOLLOW_FUNCTION_PRINT)

      UART1_Write_Text( "Algum sensor viu:\t" );
      UART1_Write_Text("Error: ");
      UART1_Write(error);
      UART1_Write_Text("\t");
      UART1_Write_Text("Power: ");
      UART1_Write(power);
      UART1_Write(13);
    #endif

//saturate power

    if (error > 0)
     {
      controlMotors(100, 100 - abs(power));
     }
    else if (error < 0)
     {
      controlMotors(100 - abs(power),100); 
     }
    else if (error == 0)
     {
      controlMotors(100,100);
      integral = 0;
     }
  
  }
  return ENEMY_FOUND;        
}

/********** SENSORS **********/
int readOpponentSensors()
{
     sensorsOn = 0;
    //Read Sensors
    for(int i =0; i < NUMBER_OF_SENSORS; i++)
    {
      opponentSensorsReadings[i] = digitalRead(SENSORS_MAP[i]);
      sensorsOn += opponentSensorsReadings[i];
    }
    //if reading nothing keep last reading and return 
    // if(!(readings[0] || readings[1] || readings[2] || readings[3] || readings[4]))
    //   return READINGS_UNCHANGED;


    //LOW PASS FITLTER => H(Z) = Z/(Z- (1-a))
    //Y[n] = a * X[n] + (1-a) * Y[N-1]
    #if (READ_OPPONENT_SENSORS_PRINT)
      printOpponentSensors();
    #endif
  return READINGS_DONE;
}

void printOpponentSensors()
{  
  UART1_Write(13);
  UART1_Write(opponentSensorsReadings[0]);
  UART1_Write_text("\t");
  UART1_Write(opponentSensorsReadings[1]);
  UART1_Write_text("\t");
  UART1_Write(opponentSensorsReadings[2]);
  UART1_Write_text("\t");
  UART1_Write(opponentSensorsReadings[3]);
  UART1_Write_text("\t");
  UART1_Write(opponentSensorsReadings[4]);
  UART1_Write(13);
}

// Y[n] = a * X[n] + (1-a) * Y[N-1]
RETURN readEdgeSensors()
{
  rightEdgeSensorValue = ALPHA * ADC_Read(BORDER_SENSOR_RIGHT) + (1-ALPHA) * rightEdgeSensorValue;
  leftEdgeSensorValue = ALPHA * ADC_Read(BORDER_SENSOR_LEFT) + (1-ALPHA) * leftEdgeSensorValue;
  
  if(leftEdgeSensorValue < LEFT_EDGE_SENSOR_THRESHOLD)
    leftEdge = true;
  else
    leftEdge = false;

  if(rightEdgeSensorValue < RIGHT_EDGE_SENSOR_THRESHOLD)
    rightEdge = true;
  else
    rightEdge = false;

  #if (READ_EDGE_SENSORS_PRINT)
    printEdgeSensors();
  #endif

  return READINGS_DONE;
}

int verifyEdgeSensors()
{
  float rotationTime;
  readEdgeSensors();

  if((rightEdge == false)&&(leftEdge == false))
     return INSIDE_ARENA;

  if(rightEdge)
   {
    controlMotors(0 ,0);
    Delay_ms(100);
    controlMotors(-100, -100);
    Delay_ms(BACKWARD_TURN_TIME);
    
    rotationTime = millis();
    do
    {
        readOpponentSensors();
        if( sensorsOn != 0 )
        {
          return ENEMY_FOUND;
        }
        controlMotors(-100,100);  
    }
    while(millis() - rotationTime < RIGHT_ROTATE_TIME);
     
    return RIGHT_EDGE_DETECTED;
   }

  if(leftEdge)
   { 
    controlMotors(0 ,0);
    Delay_ms(100);
    controlMotors(-100, -100);
    Delay_ms(BACKWARD_TURN_TIME);
    
    rotationTime = millis();
    do{
        readOpponentSensors();
        if(sensorsOn != 0)
        {
          return ENEMY_FOUND;
        }  
        
        controlMotors(100,-100);
        
     }while(millis() - rotationTime < LEFT_ROTATE_TIME);
     
    return LEFT_EDGE_DETECTED;
   }
}

void printEdgeSensors()
{
  UART1_Write(13);
  UART1_Write_text("Left Edge :");
  UART1_Write(leftEdge);
  UART1_Write_text("\t");
  UART1_Write_text("Right Edge :");
  UART1_Write(rightEdge);
  UART1_Write(13);
}

void printEdgeLuminosityValues()
{
  UART1_Write_text(13);
  UART1_Write_text("Left Edge Analog :");
  UART1_Write(analogRead(BORDER_SENSOR_LEFT));
  UART1_Write_text("\t");
  UART1_Write_text("Right Edge Analog:");
  UART1_Write(analogRead(BORDER_SENSOR_RIGHT));
  UART1_Write_text(13);
}

/*void catandoInimigoBolado()
{
  if ( strategy[2] == STAR_SEEK )
  {
    controlMotors( 80, 80 );
    verifyEdgeSensors();
    follow();
    
  }
}
/*
void initialRotation()
{
  int direcao = 1;
  while(1){
    rotationTime = millis();
    do
    {
      readOpponentSensors();
      if( sensorsOn != 0 )
      {
        return ENEMY_FOUND;
      }
      controlMotors(-100 * direcao,100 * direcao);  
    }
    while(millis() - rotationTime < RIGHT_ROTATE_TIME);
    direcao *= -1;
  }  
}
*/
*/