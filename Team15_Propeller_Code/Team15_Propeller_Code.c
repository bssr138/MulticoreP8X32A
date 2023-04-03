/*
TEAM15 Propeller Project Code
*/

//LIBRARIES
#include "simpletools.h"                      // Include simple tools

//SELF MADE FILES
#include "functions.h"                        // Header file for functions created
#include "sensors.h"                          // Header file for sensor functions created
#include "outputs.h" 


unsigned int stack[128];
unsigned int stack2[128];
unsigned int stack3[65];


int main(){

  // Cog 1
  //------------------------------------------------------------------------
  cogstart(&ultraSonic, NULL, stack, sizeof(stack));//Located in sensors.c
  
  // Cog 2
  //------------------------------------------------------------------------
  cogstart(&LCD, NULL, stack2, sizeof(stack2));//Located in outputs.c
  
  // Cog 3
  //------------------------------------------------------------------------
  // cogstart(&objectIndication, NULL, stack3, sizeof(stack3));//Located in outputs.c


  // Cog 0 
  //------------------------------------------------------------------------

  getToStart();
  // i1;

  driveToi5();
  goToA5();
  checkForWidgetsinLaneA();
  goToB1();
  checkForMachinesinLaneB();
  stopMotors();
  distanceCalc();

} 

