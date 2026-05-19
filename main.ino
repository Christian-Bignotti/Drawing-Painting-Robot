#include <math.h>
#include "StepperMotor.h"

#define pi 3.141592

#define Sol 2

StepperMotor LeftMotor(13, 12, 10);
StepperMotor RightMotor(4, 5, 7);


// State Machine
enum State {Uncalibrated, Ready, Moving, Paint, Quit};
State state = Uncalibrated;

// Functions

float dis(float p1[], float p2[])
{ 
    float p[] = {p1[0]-p2[0], p1[1]-p2[1]};
    float d = sqrt(pow(p[0],2)+pow(p[1],2));

    return d;
}

float steps_calc(float dlength_per_step, float Len_Lf, float Len_Rf, float Len_L, float Len_R)
{
   
    float LS = 2; // Lipschitz constant
 
    float dLen_L = abs(Len_Lf-Len_L); // total length change of L cable
    float dLen_R = abs(Len_Rf-Len_R); // total length change of R cable

    float steps_L = (dLen_L/dlength_per_step);// amount of steps needed for the left 
    float steps_R = (dLen_R/dlength_per_step); // amount of steps needed for the right
    return ceil(LS*(max(steps_L, steps_R))); // ceil is somewhat unecessary if LS is an int
}


// Global Variables

  int H = 80; 
  int W = 74; 
  int r = 5; 
  int micro_step = 6400;
  float perim = 2*pi*r; // perimeter
  float dlength_per_step = perim/micro_step; 

  float aL[] = {0, H}; 
  float aR[] = {W, H}; 
  float xi;
  float yi;
  float gon_i[2]; // initial gondola position
  float gon_f[2];
  float gon[2];
  float i = 0;

  float Len_L;
  float Len_R; 

  int steps = 1; // at least one step to make sure i != steps on the first iteration, i tells us how many steps we have taken while moving to a point
  int steps_L;
  int steps_R;

  float xf;
  float yf;

void len2coord(float Len_L, float Len_R){

    float a = (pow(Len_L, 2)+(pow(W, 2))-(pow(Len_R, 2)))/(2*W);
    float h = sqrt(pow(Len_L, 2)-pow(a, 2));

    float y = H-h;
    float x = a;
    xi = x;
    yi = y;
}


void move2(float xf, float yf){       
  
  if (i == steps || steps == 0)
  {
    gon[0] = xf; 
    gon[1] = yf;
    i = steps;
    return;
  }

  i = i+1;

  int delay = 80;  // Delay for stepper motor step
  steps_L = round(Len_L/dlength_per_step);
  steps_R = round(Len_R/dlength_per_step);
  float dx = (gon_f[0]-gon_i[0])*i/steps;
  float dy = (gon_f[1]-gon_i[1])*i/steps;
  float next_gon[] = {gon_i[0]+dx, gon_i[1]+dy};
  float nextLen_L = dis(next_gon, aL);
  float nextLen_R = dis(next_gon, aR);
  int nextsteps_L = round(nextLen_L/dlength_per_step);
  int nextsteps_R = round(nextLen_R/dlength_per_step);
  int dsteps_L = nextsteps_L-steps_L;
  int dsteps_R = nextsteps_R-steps_R;

  gon[0] = next_gon[0];
  gon[1] = next_gon[1];
  steps_L = nextsteps_L;
  steps_R = nextsteps_R;
          
  if (dsteps_R >= 1)
  { 
      RightMotor.step(true, delay);
      Len_R = Len_R + dlength_per_step;

  }
  else if (dsteps_R <= -1)
  {
      RightMotor.step(false, delay);
      Len_R = Len_R - dlength_per_step;
  }

  if (dsteps_L >= 1){ 
      LeftMotor.step(true, delay);
      Len_L = Len_L + dlength_per_step;
  }
  else if (dsteps_L <= -1)
  {
      LeftMotor.step(false, delay);
      Len_L = Len_L - dlength_per_step;
  }
}


void setup() 
{

  pinMode(Sol, OUTPUT);
  digitalWrite(Sol, LOW);

  LeftMotor.begin();
  RightMotor.begin();

  Serial.begin(9600);
}

void loop() 
{
  char in[64];
  char cmd = '\0'; 
  char cmd2;
  float Len_Li;
  float Len_Ri;
  char buf[64];
  int x = 0; 
  int y = 0; 

  if (Serial.available())
  {
    int len = Serial.readBytesUntil('\n', buf, 64);
    buf[len] = '\0';
    int parsed = sscanf(buf, "%s %d %d", &in, &x, &y);

    if (parsed == 3 || tolower(in[0]) == 'q')
    {
      cmd = tolower(in[0]);
      cmd2 = tolower(in[1]);
  
      if (cmd=='c')
      {
        Len_Li = x;
        Len_Ri = y;
          if(cmd2 == 'p'){
            xi = x;
            yi = y;
          }
         
      }
      else
      {
        xf = x;
        yf = y;
      }
     
    }
    else
    {
      Serial.println("Wrong input; 'c Len_L Len_R' or 'p xi yi' to set a starting point, 'm xf yf' to move to a point, and 'q' to quit and reset");
    }
  }

  
  switch (state){
    case Uncalibrated:
      LeftMotor.disable();
      RightMotor.disable();
      if(cmd == 'c'){

          if (cmd2 != 'p'){
            cmd2 = '\0';
            Serial.println("Initial lenghts set to (Left, Right): ");
            Serial.println(Len_Li);
            Serial.println(Len_Ri);
            len2coord(Len_Li, Len_Ri);
          }
          gon_i[0] = xi;
          gon_i[1] = yi;

          gon_i[0] = xi;
          gon_i[1] = yi;
          Serial.println("Starting position is:");
          Serial.println(gon_i[0]);
          Serial.println(gon_i[1]);
          
          state = Ready;
          break;        
      }else{break;}


    case Ready:
     
      LeftMotor.enable();
      RightMotor.enable();
      if (cmd == 'q'){
        state = Quit;
        break;
      }
      
      else if(cmd == 'm'){
        Serial.println("Moving to: ");
        Serial.println(xf);
        Serial.println(yf);
        

        // new
       
        gon_f[0] = xf; // final gondola position, where we want to go to
        gon_f[1] = yf; 
        gon[0] = gon_i[0]; // gon is the current position of the gondola
        gon[1] = gon_i[1];

        Len_L = dis(gon_i, aL); // length of left cable
        Len_R = dis(gon_i, aR); // length of right cable
        float Len_Lf = dis(gon_f, aL); // final length of left cable
        float Len_Rf = dis(gon_f, aR); // final length of right cable
      
        steps = steps_calc(dlength_per_step, Len_Lf, Len_Rf, Len_L, Len_R); 
        state = Moving;
        if (cmd2 == 'p')
          {
            digitalWrite(Sol, HIGH);
          }
        i = 0;
        break;
          
      }
      else
      {
        break;
      }

    case Moving:
      if (cmd == 'q'){
        state = Quit;
        break;
      }
      
      else if(i == steps){ 

          if (cmd2 == 'p'){
            cmd2 = '\0';
            digitalWrite(Sol, LOW);
            Serial.println("Done painting and moving:");
            state = Ready;
          }
          else{
            
            state = Ready;
            Serial.println("Done moving");
            }
          
          i = 0;
          
          Serial.println(gon[0]);
          Serial.println(gon[1]);
          gon_i[0] = xf;
          gon_i[1] = yf;
          i = 0;
          steps = 1;
          break;
          
          
      }
      else
      {
        
        move2(xf, yf);
        cmd = '\0'; 
        
        break;}

    case Paint:

     if (cmd == 'q'){
        state = Quit;
        break;
  
      }
      else
      {
        delay(500);
        digitalWrite(Sol, HIGH);
        delay(1500);
        digitalWrite(Sol, LOW);
        Serial.println("R");
        state = Ready;
        break;}
 

    case Quit:
      Serial.println("Force quit, resetting to uncalibrated state");
      
      state = Uncalibrated;
      break;
    }
  }  