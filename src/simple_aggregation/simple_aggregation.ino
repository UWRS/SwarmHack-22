//Include the Mona_ESP library
#include <Wire.h>
#include "Mona_ESP_lib.h"


//Variables
int IR_values[5] = {0, 0, 0, 0, 0};
int IR_raw[5] = {0, 0, 0, 0, 0};



//value used to determine a detection on the IR sensors.
//Reduce the value for a earlier detection, increase it if there are
//false detections.



int th_stop = 35;
int th_robot = 150;
int th_wall = 245;


float p = 100;
float center = 2.8;
int base_spd = 200;
int SPD_COEFF = 0.4;
int MOT_TRIM = -10; // -: left, +: right

int wait_time = 3000;

//If robot detected, stop for full 10s
//Otherwise if wall detected, deduct appropriate seconds (see code below for 'wait_time')
int STOP_BASE = 10000;  // ms

// y = mx + c
// where m:STOP_SENSE
float STOP_SENSE = (float) STOP_BASE / (float) (th_wall - th_robot);


int TURN_TIME_BASE = 800;




//State Machine Variable
// 0 - random walk
// 1 - stop and scan
// 2 - spin around
int state, old_state;

void setup()
{
  //Initialize the MonaV2 robot
  Mona_ESP_init();
  //Initialize variables
  state=0;
  old_state=0;
  Set_LED(1, 255, 0, 0);
  Set_LED(2, 255, 0, 0);
  Serial.begin(9600);
}


int wait_start_time = millis();
int turn_start_time = millis();
int turn_time = 500;

float ir_sum = 0;
float raw_sum = 0;

int st2_dir = 0;

int trials = 3;
float ir_pos = 0, norm = 0;


void loop(){
  int cur_time = millis();

  Disable_IR(1);
  Enable_IR(2);
  Disable_IR(3);
  Enable_IR(4);
  Disable_IR(5);

  delay(5);


  int max_ir_val = -1, max_sensor = 2;  // Default to use the middle sensor
  for (int i=0; i<5; i+=2) {  // 0 2 4
    IR_values[i] = Get_IR(i+1);
    if (IR_values[i] >= max_ir_val) {
      max_ir_val = IR_values[i];
      max_sensor = i;
    }
  }


  //The following three placed right, left and centre are to be solely used for detection of IR emitted by other robots
  //See edge case diagram explained under media dir
  Disable_IR(1);
  Disable_IR(3);
  Disable_IR(5);
  delay(5);
  IR_raw[0] = Read_IR(1);
  IR_raw[2] = Read_IR(3);
  IR_raw[4] = Read_IR(5);


  ir_pos = 0;
  norm = 0;
  ir_sum = 0;
  raw_sum = 0;
  for (int i=0; i<5; i+=2) {  // 0 2 4
    ir_sum += IR_values[i];
    raw_sum += IR_raw[i];
    
    ir_pos += (i+1) * IR_values[i];
    norm += IR_values[i];
  }
  ir_pos /= norm;
  ir_sum /= 5;
  raw_sum /= 5;


  int ir_stop = IR_values[max_sensor];
  float ir_test = IR_raw[max_sensor];


  // WAITING

  Serial.print("State["+String(state)+"]: max_sensor = "+String(max_sensor)+" , ir_stop = " + String(ir_stop) + " , ir_test = " + String(ir_test) + " ");

  //wait_time = STOP_BASE - (ir_test - th_robot) * STOP_SENSE;

  //Serial.print(" | p_wait_time = " + String(wait_time));


  if (state == 0) {
    if (ir_stop >= th_stop) {

      wait_start_time = cur_time;
      wait_time = STOP_BASE - (ir_test - th_robot) * STOP_SENSE;
      
      Serial.print(" | Start Waiting for " + String(wait_time));

      
      state = 1;
    }
  } else if (state == 1) {
    if (cur_time > wait_start_time + wait_time) {
      state = 2;
    }
  } else if (state == 2) {
    turn_start_time = cur_time;
    turn_time = random(TURN_TIME_BASE, TURN_TIME_BASE * 1.5);

    if (max_sensor > 2) {
      st2_dir = 0;
    } else {
      st2_dir = 1;
    }

    state = 3;
  } else if (state == 3) {
    if (ir_test <= th_robot) {
      wait_start_time = cur_time;
      wait_time = STOP_BASE - (ir_test - th_robot) * STOP_SENSE;
      
      Serial.print(" | Turn wait for " + String(wait_time));
      state = 1;
    } else if (ir_stop < th_stop && abs(ir_pos - center) < 1.0) {
      if (cur_time > turn_start_time + turn_time) {
        state = 0;
      }
    }
  }
  
  delay(5);


  //--------------Motors------------------------
  //Set motors movement based on the state machine value.
  //
  //Random Walk
  if(state == 0){
    Set_LED(1, 255, 100, 0);
    Set_LED(2, 255, 100, 0);
    
    //Slow down when approaching obstacle
    //i.e speed is function of IR readings
    int forward_spd = base_spd - ir_sum * SPD_COEFF;
    int diff = (ir_pos - center) * p;

    diff = random(0, 30);
    

    diff += MOT_TRIM;

    int left = max(0, min(base_spd + diff, 255));
    int right = max(0, min(base_spd - diff, 255));
        
    Left_mot_forward(left);
    Right_mot_forward(right);
  }

  //Stop and scan
  if(state == 1){
    Set_LED(1, 0, 255, 0);
    Set_LED(2, 0, 255, 0);
    
    // Wait
    Left_mot_stop();
    Right_mot_stop();
  }

  //Spin around
  if (state == 3) {
    Set_LED(1, 0, 0, 255);
    Set_LED(2, 0, 0, 255);
    
    if (st2_dir) {
      Motors_spin_left(base_spd);
    } else {
      Motors_spin_right(base_spd);
    }
  }


  Serial.println();
}