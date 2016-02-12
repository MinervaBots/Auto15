#include "Functions.h"
#include "Constants.h"

unsigned long timer = 0;
unsigned long blindTime = 0;

void main()
{
  initRobot();
  controlMotors(0,0);
  setConstants(40,2,0); // 40 0.1 30  //40 2 0
  waitStartButton();
  analiseSwitches();
  initialDelay(INITIAL_DELAY);
  initialTurn();
  timer = millis();
}

while (1)
{

  follow();
  verifyEdgeSensors();
  checkStopMotor();

}