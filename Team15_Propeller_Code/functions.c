#include "simpletools.h"                      // Include simple tools
#include "servo.h"                            // Include servo header
#include "sensors.h"                          // sensors header for Ultrasonics


//=============================================================================================
// PIN CONNECTIONS
//=============================================================================================
#define LEFT  4
#define RIGHT  6
#define CENTER 5
#define LEFT_SERVO 17
#define RIGHT_SERVO 16
#define LEFTMOST 7
#define RIGHTMOST 8


//=============================================================================================
// OPERATING VARIABLES
//=============================================================================================
#define MOTOR_KP 10.0
static volatile int currentIntersection = 0;
static volatile int numOfObjs = 0;
static volatile int intLCD = 0;
static volatile int frontLCD = 0;
static volatile int objAtB5 = 0;
static volatile int IntNumOfWidgetLocationInLaneA = 0;
static volatile int IntNumOfMachineLocationInLaneB = 0;
static volatile int distLCD = 0;
static volatile int distance = 0;
static volatile int skipFlag = 0;


/*
------------------------Hardware Functions----------------------
----------------------------------------------------------------
----------------------------------------------------------------
----------------------------------------------------------------
*/


//=============================================================================================
// float lineSensors(long *readings)
// ----- Function that reads the values from each of the IR sensors and saves them to a 
// ----- long array. This array is passed as a pointer.
//=============================================================================================
void lineSensors(long *readings){
    
    set_direction(CENTER, 1);
    set_output(CENTER, 1);
    pause(1);
    set_direction(CENTER,0);
    readings[1] = rc_time(CENTER,1);
    //print("centerSen = %d\n", readings[1]);
    
    
    set_direction(RIGHT, 1);
    set_output(RIGHT, 1);
    pause(1);
    set_direction(RIGHT,0);
    readings[2] = rc_time(RIGHT,1);
    //print("rightSen = %d\n", readings[2]);


    set_direction(LEFT, 1);
    set_output(LEFT, 1);
    pause(1);
    set_direction(LEFT,0);
    readings[0] = rc_time(LEFT,1);
   // print("leftSen = %d\n", readings[0]);
    pause(5); 

}


//=============================================================================================
// void drive(float LEFTPower, float RIGHTPower)
// ----- Function that moves the continuous servos depending on the power values supplied. 
// ----- 1500 represenents stop, however the values close to 1500 are too slow therefore
// ----- the power values begin incrementing with a starting value of 1500+-25. 
//=============================================================================================
void drive(float LEFTPower, float RIGHTPower){

  //1700 (1600)is maximum in one direction and 1300 (1400) is maximum in other direction
  //1500 is stop

  servo_set(LEFT_SERVO,(1525+(LEFTPower*2.5)));   //LEFT
  servo_set(RIGHT_SERVO,(1475-(RIGHTPower*2.5)));  // RIGHT

}


//=============================================================================================
// void checkFrontObj()
// ----- Function that checks if there is an object in front of the robot within 10 cm
// ----- and returns 1 if there is
//=============================================================================================
int checkFrontObj(){
  
  if(getUSReadingCenter()<35){
    return 1;
  }  else{
    return 0;
  }      
  
} 

//=============================================================================================
// void checkFLeftObj()
// ----- Function that checks if there is an object in left of the robot within 10 cm
// ----- and returns 1 if there is
//=============================================================================================
int checkLeftObj(){
  
  if(getUSReadingLeft()<10){
    return 1;
  }  else{
    return 0;
  }      
  
} 

//=============================================================================================
// void stopMotors()
// ----- Function that stops the continuous servos. 1500 represenents stop in the servo_set
// ----- function
//=============================================================================================
void stopMotors() {
  servo_set(LEFT_SERVO,1500);
  servo_set(RIGHT_SERVO,1500);
}


//=============================================================================================
// void checkForIntersect()
// ----- Function that uses the farthest LEFT and farthest RIGHT sensor on the sensor array
// ----- to check if an intersection has been reached
//=============================================================================================
int checkForIntersect(){
    long leftInt, rightInt;

    set_direction(LEFTMOST, 1);
    set_output(LEFTMOST, 1);
    pause(1);
    set_direction(LEFTMOST,0);
    leftInt = rc_time(LEFTMOST,1);

    set_direction(RIGHTMOST, 1);
    set_output(RIGHTMOST, 1);
    pause(1);
    set_direction(RIGHTMOST,0);
    rightInt = rc_time(RIGHTMOST,1);
    pause(5);

    if(leftInt > 2500 && rightInt > 2500){
      intLCD = 1;//variable for triggering the LCD to display intersection detected
      return 1;
    }else{
      return 0;
    }
}


//=============================================================================================
// void lineFollow(float Kp, int objectInFront)
// ----- This function controls the robot to follow a line until an intersection is detected.
//=============================================================================================
void lineFollow(float Kp, int objectInFront ){

  //Variables local to this function
  long leftSen;
  long midSen;
  long rightSen;
  long sensorValues[3];
  float stndMtrPwr = 10.0;
  float PosError = 0;
  float one;
  float senPos = 2.50;
  int breakCondition;

  //Checking the break condition once before the loop begins
  // if(objectInFront){  
  
  //   breakCondition = checkForIntersect() || checkFrontObj();

  // }else{
  //   breakCondition = checkForIntersect();
  // }    
  breakCondition = checkForIntersect();

  //---------------------------Line Follwoing Loop----------------------------------

    while(!breakCondition){
      //Reading sensor values and applying the power values to the motor

    //-----------------------Reading Sensors Values---------------------------------
      lineSensors(sensorValues);
      leftSen = sensorValues[0];
      //print("leftSen = %0.1f\n", sensorValues[0]);
      midSen = sensorValues[1];
      //print("midSen = %0.1f\n", midSen);
      rightSen = sensorValues[2];
      //print("rightSen = %0.1f\n", rightSen);
      
      
      //Updating Motor Power to lower speed in lanes A and B
      if (((currentIntersection <= 15)&&(currentIntersection>10)) || ((currentIntersection > 5) && (currentIntersection < 10)) || (skipFlag == 1)){
        stndMtrPwr = 6.0;
      }
      else{
        stndMtrPwr = 20.0;
      }

      drive(stndMtrPwr+Kp*PosError,stndMtrPwr-Kp*PosError);

      
    //-----------------------Computing Error --------------------------------

      if(midSen > 2500){
        //If the middle sensor is on the line then the robot needs to just continue straight
        senPos = 2.50;
        //2.5

      }else if (rightSen > leftSen){
        one = rightSen/2500.0;
        senPos = senPos + 2.5*one;
        //RIGHT Sensor is on the line.
        //Robot needs to turn RIGHT

        //5.0
        //Increase LEFT power
        //DECREASE RIGHT power
        
        
      } else if(rightSen < leftSen){
        one = leftSen/2500.0;
        senPos = senPos - 2.5*one;
        //LEFT Sensor is on the line.
        //Robot needs to turn LEFT

        //0
        //Increase RIGHT power
        //Decrease LEFT Power
        
      }
      
      PosError =  senPos - 2.50;
      senPos = 2.5;
      //From -2.50 to +2.5
      //From LEFT side to RIGHT Side

      /*Error is 2.500 if RIGHT side is on line
      Needs to turn RIGHT. 

      Error is -2.500 if LEFT side is on line
      Needs to turn LEFT. 
    
      */
    
    if(objectInFront){ 
      while(getUSReadingCenter()<6){
        stopMotors();
      }
    } 
    //-----------------------Break Conditions--------------------------------

    // if(objectInFront){  
    //   breakCondition = checkForIntersect() || checkFrontObj();
    // }else{
    //   breakCondition = checkForIntersect();
    // }  
    breakCondition = checkForIntersect();
    //When breakCondition is 1, the loop will break and the linefollow function ends
    }

  //Once loop ends, an intersection has been detected. Therefore stop motors
  //stopMotors();
}

//=============================================================================================
// void turnLeft()
// ----- This function turns the robot left until the middle line sensor detects black
//=============================================================================================
void turnLeft(){
  long readings[3];
  if(numOfObjs !=2){
    servo_set(LEFT_SERVO,(1450));   //LEFT
    servo_set(RIGHT_SERVO,(1450));  // RIGHT
    pause(500);

    do{
      servo_set(LEFT_SERVO,(1450));   //LEFT
      servo_set(RIGHT_SERVO,(1450));  // RIGHT
      
      lineSensors(readings);
    } while(readings[1]<2200);



    stopMotors();

  }
}
  
//=============================================================================================
// void turnRight()
// ----- This function turns the robot right until the middle line sensor detects black
//=============================================================================================
void turnRight(){
  long readings[3];
  if (numOfObjs !=2){
    servo_set(LEFT_SERVO,(1550));   //LEFT
    servo_set(RIGHT_SERVO,(1550));  // RIGHT
    pause(500);
    
    do{
      servo_set(LEFT_SERVO,(1550));   //LEFT
      servo_set(RIGHT_SERVO,(1550));  // RIGHT
      
      lineSensors(readings);
    } while(readings[1]<2200);


    stopMotors();
  }
}

//=============================================================================================
// void turn180()
// ----- This function turns the robot right until the right line sensor detects black
//=============================================================================================
void turn180(){
  long readings[3];
  if (numOfObjs !=2){
    servo_set(LEFT_SERVO,(1550));   //LEFT
    servo_set(RIGHT_SERVO,(1550));  // RIGHT
    pause(500);
    
    do{
      servo_set(LEFT_SERVO,(1550));   //LEFT
      servo_set(RIGHT_SERVO,(1550));  // RIGHT
      
      lineSensors(readings);
    } while(readings[2]<2200);


    stopMotors();
  }
}

//=============================================================================================
// void BotForward()
// ----- This function moves the bot forward so the wheels are over the line
//=============================================================================================
void BotForward(){
  if(numOfObjs!=2){
    drive(20,20);
    pause(300);
   // stopMotors();
  }
}

//=============================================================================================
// void multLineFollow(int n, int sign)
// ----- This function calls the linefollow function multiple times. This is for when the robot
// ----- is on the A or B lanes. n is the number of times to line follow and sign is whether to
// ----- add or subtract to the current intersection counter while driving
//=============================================================================================
void multLineFollow(int n, int sign){

  for (int i= 0; i<n; i++){

    lineFollow(MOTOR_KP,0);         // repeat line follow "n" times
    if (i != n-1){
      drive(20,20);
      pause(300);
    }
    currentIntersection = currentIntersection + sign;
  }
}


/*
------------------------State Functions-------------------------
----------------------------------------------------------------
----------------------------------------------------------------
----------------------------------------------------------------
*/


//=============================================================================================
// Getters and Setters
// ----- These functions allow other cogs to access the variables that are local to the main Cog
//=============================================================================================

//------Intersection Counter
void setIntersect(int intersection){
  currentIntersection = intersection;
}

int getIntersect(){
  return currentIntersection;
}

int getDistance(){
  return distance;
} 

//------Object Detected Counter
void addObjCount(){
  numOfObjs += 1;
}

//------Intersection LCD update variable
void setIntLCD(int disp){
  intLCD = disp;
}

int getIntLCD(){
  return intLCD;
} 

//------Front Obstacle LCD update variable
void setfrontLCD(int disp){
  frontLCD = disp;
}

int getFrontLCD(){
  return frontLCD;
} 

//------Flag for if Object is at B5
int isObjatB5(){
  return objAtB5;
} 

//------Intersection LCD update variable
void setDistLCD(int disp){
  distLCD = disp;
}

int getDistLCD(){
  return distLCD;
} 



//=============================================================================================
// void getToStart()
// ----- This function drives the robot until i1. Following the line the whole time
//=============================================================================================
void getToStart(){
  
  lineFollow(MOTOR_KP,1);//located in functions.c
  //!
  // //i0
  // drive(20,20);
  // pause(300);
  // //print("START");
  // lineFollow(MOTOR_KP,0);
  //!
  currentIntersection = 1; 
  //i1 reached  
  BotForward();
    
}  


//=============================================================================================
// int driveToi5()
// ----- This function drives the robot until i5.
// ----- Checks for obstacles at each intersection and skips it if needed
//=============================================================================================
int driveToi5(){
    
  while(currentIntersection<5){

    //The robot drives until an intersection and then checks if there is an obstacle in front.
    //If there is then it will turn accordingly. Else is will line follow to the next intersection
    if(checkFrontObj()){
      
      frontLCD = 1; //variable for triggering the LCD to display obstacle detected
      skipOneIntersection();  
    }
    else{
      lineFollow(MOTOR_KP,1);
      BotForward();
    }
    currentIntersection = currentIntersection +1;
  }  
  
  return currentIntersection;
  
} 


//=============================================================================================
// void skipOneIntersection()
// ----- This function turns the robot left ( facing towards lane B), goes to B lane, turns right
// ----- does one length along lane B, turns right and comes back to the center lane 
//=============================================================================================
void skipOneIntersection(){
  turnLeft();
  lineFollow(MOTOR_KP,1);
  BotForward();
  turnRight();
  skipFlag = 1;
  lineFollow(MOTOR_KP,1);
  skipFlag = 0;
  BotForward();
  turnRight();
  lineFollow(MOTOR_KP, 1);
  BotForward();
  turnLeft();
}

//=============================================================================================
// void goToA5()
// ----- This function turns the robot right ( facing towards lane A), line follows to lane A, 
// ----- and turns right
//=============================================================================================
void goToA5(){
  turnRight();
  lineFollow(MOTOR_KP,1);
  BotForward();
  turnRight();
  currentIntersection = currentIntersection +1; // 6 --> A5
}

//=============================================================================================
// void checkForWidgetsinLaneA()
// ----- This function checks for obstacles along lane A 
// ----- while stopping whenever an obstacle in introduced along the way
//=============================================================================================
void checkForWidgetsinLaneA(){

 while(currentIntersection<10){ // 10 --> A1

    if(checkLeftObj()){
      
      frontLCD = 1; //variable for triggering the LCD to display obstacle detected
      stopMotors();
      pause(2000);
      BotForward();
      addObjCount();
      IntNumOfWidgetLocationInLaneA = currentIntersection;
    }

    lineFollow(MOTOR_KP,1);
    currentIntersection = currentIntersection +1;
    BotForward();


  }

  if (currentIntersection == 10){
    if(checkLeftObj()){
      
      frontLCD = 1; //variable for triggering the LCD to display obstacle detected
      stopMotors();
      pause(2000);
      addObjCount();
      IntNumOfWidgetLocationInLaneA = currentIntersection;
    }
  }
  
  return currentIntersection;

}


//=============================================================================================
// void gotoB1()
// ----- This function drives the robot along to B1
// ----- starting from A1 with wheels along A1
//=============================================================================================
void goToB1(){
  turnRight();
  lineFollow(MOTOR_KP,1);
  BotForward();
  lineFollow(MOTOR_KP,1);
  BotForward();
  turnRight();
  BotForward();
  currentIntersection = currentIntersection +1; // 11 --> B1
}

//=============================================================================================
// void checkForMachinesinLaneB()
// ----- This function checks for obstacles along lane B
// ----- while stopping whenever an obstacle in introduced along the way
//=============================================================================================
void checkForMachinesinLaneB(){

 while(currentIntersection<15 && numOfObjs != 2){ // 15 --> B5

    if(checkLeftObj()){
      
      frontLCD = 1; //variable for triggering the LCD to display obstacle detected
      stopMotors();
      pause(2000);
      addObjCount();
      IntNumOfMachineLocationInLaneB = currentIntersection;
    }

    if (numOfObjs != 2){
      BotForward();
      lineFollow(MOTOR_KP,1);
      currentIntersection = currentIntersection +1;
      BotForward();
    }

  }

  if (currentIntersection == 15){
    if(checkLeftObj()){
      
      frontLCD = 1; //variable for triggering the LCD to display obstacle detected
      stopMotors();
      pause(2000);
      addObjCount();
      IntNumOfMachineLocationInLaneB = currentIntersection;
    }
  }
  
  return currentIntersection;

}

void distanceCalc(){
  distLCD = 1;
  int numOfIntersections = IntNumOfMachineLocationInLaneB - IntNumOfWidgetLocationInLaneA;
  distance = (numOfIntersections + 1) * 40;

}
