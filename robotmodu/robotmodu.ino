
#include "IRLibAll.h"

IRrecvPCI FrontReceiver(2);
 
int LeftSensor = A0;
int RightSensor = A1;
int FrontSensor = A2;


//Create a decoder object 
IRdecode myDecoder;   


unsigned int sensorValues[8];
void SetTunings(double Kp, double Ki, double Kd);
int motor_left[] = {9, 6};
int motor_right[] = {3, 5};
int motorSpeed = 255;
int SampleTime = 50; 
int _speed = 255;
//decode_results results;

int PWM_LEFT = 0;
int PWM_RIGHT = 0;

int RIGHT        = 0;
int MIDDLE_RIGHT = 1;
int MIDDLE_LEFT  = 2;
int LEFT         = 3;


unsigned long lastTime;

double speedRatio = 1;
double Input, Output, Setpoint = 3.5;
double ITerm, lastInput;
double kp, ki, kd;
double outMin, outMax;

bool inAuto = false;
bool printSensorValues = false;

#define MANUAL 0
#define AUTOMATIC 1

void drive_forward(int _speed){
  analogWrite(motor_left[0], _speed);
  analogWrite(motor_left[1], 0);
  
  analogWrite(motor_right[0], _speed);
  analogWrite(motor_right[1], 0);
}

void drive_stop(){
  analogWrite(motor_left[0], 0);
  analogWrite(motor_left[1], 0);
  
  analogWrite(motor_right[0], 0);
  analogWrite(motor_right[1], 0);
}


void drive_backward(int _speed){
  analogWrite(motor_left[0], 0);
  analogWrite(motor_left[1], _speed );
  
  analogWrite(motor_right[0], 0);
  analogWrite(motor_right[1], _speed+45);
    
}

void turn_left(int _speed){
  analogWrite(motor_left[0], LOW);
  analogWrite(motor_left[1], LOW);

  analogWrite(motor_right[0], _speed);
  analogWrite(motor_right[1], LOW);
}

void turn_hardleft(int _speed){
  analogWrite(motor_left[0], LOW);
  analogWrite(motor_left[1], _speed);
  
  analogWrite(motor_right[0], _speed +45);
  analogWrite(motor_right[1], LOW);
}


void turn_right(int _speed){
  analogWrite(motor_left[0], _speed);
  analogWrite(motor_left[1], LOW);
  
  analogWrite(motor_right[0], LOW);
  analogWrite(motor_right[1], LOW);
}

void turn_hardright(int _speed){
  analogWrite(motor_left[0], _speed);
  analogWrite(motor_left[1], LOW);
  
  analogWrite(motor_right[0], LOW);
  analogWrite(motor_right[1], _speed + 45);
}



void motor_stop(){
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], LOW);
  
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], LOW);
  delay(25);
}
void clamp(int * value, int MIN, int MAX){
  *value = *value > MAX ? MAX : (*value < MIN ? MIN : *value);  
}



void powerMotors(){
  if(!inAuto) return;
  analogWrite(motor_left[0], int(PWM_LEFT * speedRatio));
  analogWrite(motor_right[0], int(PWM_RIGHT * speedRatio));
  analogWrite(motor_right[1], 0); 
  analogWrite(motor_left[1], 0);
}

void computeSpeeds(){
   if(Output<0) // Turn right
  {
    PWM_LEFT = motorSpeed;
    PWM_RIGHT = motorSpeed - abs(int(Output));
  }else // Turn left
  {
    PWM_LEFT = motorSpeed - int(Output);
    PWM_RIGHT = motorSpeed;
  }
  clamp(&PWM_LEFT, 0, 255);
  clamp(&PWM_RIGHT, 0, 255);
}


void PID()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      Input = 0; 
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm> outMax) ITerm= outMax;
      else if(ITerm< outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm - kd * dInput;
      if(Output> outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
      
      
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
    
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}
 
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    { 
        Initialize();
    }
    inAuto = newAuto;
}
 
void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}


void setup() {


  FrontReceiver.enableIRIn(); // Start the receiver
  
  
  
      Serial.println(F("Ready to receive IR signals"));
  for(int i = 0; i < 2; i++){
    pinMode(motor_left[i], OUTPUT);
    pinMode(motor_right[i], OUTPUT);
  }

  pinMode(A0,INPUT);
  
  pinMode(A1,INPUT);

  
  pinMode(A2,INPUT);
//  irrecv.enableIRIn(); 
  Serial.begin(9600);
  printSensorValues = false;
  
  SetTunings(72,.4,4000);
  SetMode(AUTOMATIC);
  SetOutputLimits(-255,255);


   
  
}

int LeftLight( ) {

  if(analogRead(LeftSensor) == 0) return 1;

  return 0;
  
}

int FrontLight( ) {

  if(analogRead(FrontSensor)== 0) return 1;

  return 0;
  
}

int RightLight( ) {

  if(analogRead(RightSensor)== 0) return 1;

  return 0;
  
}

int isDrivingFwd ;
void dreapta(int _speed){
  analogWrite(motor_right[0], 0);
  analogWrite(motor_right[1], _speed+45);
}
void loop() {


  //drive_backward(150);

  



  if (FrontReceiver.getResults()) {
        myDecoder.decode(); //Decode it

        uint32_t val;

        
        val = myDecoder.dumpValueOnly();
        
        Serial.print("front value:");
        Serial.println(val);

        if(val == 55 || val == 144)  {
          drive_backward(100);
          Serial.println("Go FORWARD");
          isDrivingFwd = 0;
          
        }

        
        else {
          isDrivingFwd = 1;
          drive_stop();
        }
        FrontReceiver.enableIRIn();      //Restart receiver
      }

    if(LeftLight() ) {


      if(  isDrivingFwd ) {
         
            turn_hardleft(150);
            Serial.println("Turning left");
      
      }
      Serial.print( analogRead(LeftSensor) ) ;
      Serial.println(" ON LEFT~~" ) ;
      
    }
    if( RightLight() ) { 

       if(  isDrivingFwd ) {
           turn_hardright(150);
           Serial.println("Turning right");
       }
      Serial.print(analogRead(RightSensor));
      Serial.println(" ON Right");
    }

    

    delay(50);

    
 /* if(inAuto){
    PID();
    computeSpeeds();
    powerMotors();
  }*/
  
}



