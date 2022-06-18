/*
  Simple_collision_avoider.ino - Usage of the libraries Example
  Using the Mona_ESP library in C style.
  Created by Bart Garcia, December 2020.
  bart.garcia.nathan@gmail.com
  Released into the public domain.
*/
//Include the Mona_ESP library
#include <Wire.h>
#include "Mona_ESP_lib.h"


//Variables
bool IR_values[5] = {false, false, false, false, false};
//Threshold value used to determine a detection on the IR sensors.
//Reduce the value for a earlier detection, increase it if there
//false detections.
int threshold = 40;
//State Machine Variable
// 0 -move forward , 1 - forward obstacle , 2 - right proximity , 3 - left proximity
int state, old_state, determine;


void setup()
{
  //Initialize the MonaV2 robot
  Mona_ESP_init();
  //Initialize variables
  state=0;
  old_state=0;
  randomSeed(analogRead(0));
}


void loop(){

    //--------------Motors------------------------
  //Set motors movement based on the state machine value.
  if(state == 1){
    // Start moving Forward
    Motors_forward(200);
  }
  if(state == 2){
    //Spin to the left
    Motors_spin_left(1000);
  }
   if(state == 3){
    //Spin to the left
    Motors_spin_right(1000);
  }
   if(state == 4){
    //Spin to the right
    Motors_backward(150);
  }
  if(state == 5){
    Motors_stop();
    delay(500);
    state = random(2, 4);
  }

  Serial.begin(115200);
  Serial.println("Reading data from infra-red sensors");
  //--------------IR sensors------------------------
  //Decide future state:
  //Read IR values to determine maze walls
 
  IR_values[0] = Detect_object(1,threshold);
  IR_values[1] = Detect_object(2,threshold);
  IR_values[2] = Detect_object(3,threshold);
  IR_values[3] = Detect_object(4,threshold);
  IR_values[4] = Detect_object(5,threshold);
 
/*
  Serial.println(Get_IR(1));
  Serial.println(Get_IR(2));
  Serial.println(Get_IR(3));
  Serial.println(Get_IR(4));
  Serial.println(Get_IR(5));
  delay(500);
*/

  //Flag counts how many IR sensors read above threshold
  determine = (int) IR_values[0] + (int) IR_values[1] + (int) IR_values[2] + (int) IR_values[3] + (int) IR_values[4];

  switch(determine){
    case 0:
      state = 1;
      break;
    case 3:
      state = random(2, 4);
      break;
    case 2:
      state = random(2, 4);
      break;  
    case 1:
      if(!Detect_object(1, 100)|| !Detect_object(5, 100)){
        state = random(2, 4);
      }
      else{
        state = 5;
      }
      break;
    case 5:
      state = 4;
      state = random(2, 4);
      delay(800);
      break;
    default:  
      state = 1;
      break;
  }  
}