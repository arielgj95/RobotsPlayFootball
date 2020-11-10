//motor1

#define encoderA 32
#define encoderB 33
#define motorPin1 3 //correct
//#define en3 24    //invert?
//#define en4 25
#define en1 24
#define en2 25


//motor2

#define encoderC 30
#define encoderD 31
#define motorPin2 2 //correct
#define en3 22
#define en4 23


//motor3

//#define encoderE 53
//#define encoderF 52
#define encoderE 53
#define encoderF 52
#define motorPin3 4
//#define en7 44    //43,42
//#define en8 45
#define en5 40
#define en6 41


//motor4

//#define encoderG 50
//#define encoderH 51
#define encoderG 51
#define encoderH 50
#define motorPin4 5
//#define en5 42 //invert?  45,44
//#define en6 43
#define en7 46
#define en8 47



#define LOOPTIME 100  
double last_error[]={0.0, 0.0, 0.0, 0.0};    //init
int motorPin[]={3,2,4,5};
//int motorPin_ordered[]={3,2,4,5};
int PWM_val[]={0,0,0,0};
double speed_req[] = {0.0, 0.0, 0.0, 0.0};
double speed_act[] = {0.0, 0.0, 0.0, 0.0};
int counter[]={0,0,0,0};
int countAnt[]={0,0,0,0};
String sw="";
String motor="";
double Kp =   16.0;                                // PID proportional control Gain
double Kd =    4.0;                                // PID Derivitave control gain
double vel = 0.0;
unsigned long t_start;
//double actvel=0;
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing


//FOR MOTOR 1

int stateA = 0;
int stateB = 0;
double vel_1 = 0;
double speed_req_1 = 0.0;                            // speed (Set Point)
int PWM_val_1 = 0; 
double speed_act_1 = 0.0;                              // speed (actual value)



//FOR MOTOR 2

int stateC = 0;
int stateD = 0;
double vel_2 = 0;
double speed_req_2 = 0.0;                            // speed (Set Point)
int PWM_val_2 = 0;
double speed_act_2 = 0.0;                              // speed (actual value) 

//FOR MOTOR 3

int stateE = 0;
int stateF = 0;
double vel_3 = 0;
double speed_req_3 = 0.0;                            // speed (Set Point)
int PWM_val_3 = 0;
double speed_act_3 = 0.0;                              // speed (actual value) 

//FOR MOTOR 4

int stateG = 0;
int stateH = 0;
double vel_4 = 0;
double speed_req_4 = 0.0;                            // speed (Set Point)
int PWM_val_4 = 0;
double speed_act_4 = 0.0;                              // speed (actual value) 


int f=0;


void setup() {
  Serial.begin(115200);           // set up Serial library at 115200 bps
  while (!Serial)                 //while seriale stream is not started, don't do anything
  
  pinMode(motorPin[0], OUTPUT);                                  //MOTOR1
  pinMode(en1, OUTPUT);  //en and encoder are pins
  pinMode(en2, OUTPUT);
  pinMode (encoderA, INPUT);
  pinMode (encoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA), checkA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), checkB, CHANGE);
  digitalWrite(en1, LOW);
  digitalWrite(en2, HIGH);


  pinMode(motorPin[1], OUTPUT);                                  //MOTOR2
  pinMode(en3, OUTPUT); 
  pinMode(en4, OUTPUT);
  pinMode (encoderC, INPUT);
  pinMode (encoderD, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderC), checkC, CHANGE); //(interrupt,ISR,mode); CHANGE is used to trigger the interrupt whenever the pin changes value
  attachInterrupt(digitalPinToInterrupt(encoderD), checkD, CHANGE);
  digitalWrite(en3, LOW);
  digitalWrite(en4, HIGH);

 
  pinMode(motorPin[2], OUTPUT);                                 //MOTOR3
  pinMode(en5, OUTPUT);
  pinMode(en6, OUTPUT);
  pinMode (encoderE, INPUT);
  pinMode (encoderF, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderE), checkE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderF), checkF, CHANGE);
  digitalWrite(en5, LOW);
  digitalWrite(en6, HIGH);


  
  pinMode(motorPin[3], OUTPUT);                                 //MOTOR4
  pinMode(en7, OUTPUT);
  pinMode(en8, OUTPUT);
  pinMode (encoderG, INPUT);
  pinMode (encoderH, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderG), checkG, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderH), checkH, CHANGE);
  digitalWrite(en7, LOW);
  digitalWrite(en8, HIGH);

  t_start = millis();  //return number of milliseconds passed from the start of arduino

  Serial.print("<Arduino is ready>"); //this activate arduino_four_motors
  Serial.flush(); //Waits for the transmission of outgoing serial data to complete
}

void loop() {
  
  String serialResponse =""; //init of serialResponse
     
  if (Serial.available()) {  //if there is something available in the serial port
    serialResponse  = Serial.readString();  //read it and put the string serialResponse
    motor = serialResponse.substring(1,7);  //the string is like: motor1 -0.5 motor2 -0.5 motor3 -0.5 motor4 -0.5
    sw = serialResponse.substring(8,9); // 7,8 means that you take the characters from the next character after 7 and the final one will be 8: in this case I will take 8
    vel = serialResponse.substring(9,12).toDouble(); //take and convert in a double number

    if (motor == "motor1")
    {
      if(sw =="+"){
        //go forward
        digitalWrite(en1, LOW);
        digitalWrite(en2, HIGH);
        }
      else if (sw =="-"){
        //go back
        digitalWrite(en1, HIGH);
        digitalWrite(en2, LOW);
        }
    }
    speed_req[0] = vel; //put the first velocity in the first element of speed_req
    
    motor = serialResponse.substring(13,19);
    sw = serialResponse.substring(20,21);
    vel = serialResponse.substring(21,24).toDouble();
   
    if (motor == "motor2")
    {
      if(sw =="+"){
        //go forward
        digitalWrite(en3, LOW);
        digitalWrite(en4, HIGH);
        }
      else if (sw =="-"){
        //go back
        digitalWrite(en3, HIGH);
        digitalWrite(en4, LOW);
        }
    }
    speed_req[1]=vel;

    
    motor = serialResponse.substring(25,31);
    sw = serialResponse.substring(32,33);
    vel = serialResponse.substring(33,36).toDouble();

    if (motor == "motor3")
    {
      if(sw =="+"){
        //go forward
        digitalWrite(en5, LOW);
        digitalWrite(en6, HIGH);
        }
      else if (sw =="-"){
        //go back
        digitalWrite(en5, HIGH);
        digitalWrite(en6, LOW);
        }
    }
    speed_req[2] = vel;

    
    motor = serialResponse.substring(37,43);
    sw = serialResponse.substring(44,45);
    vel = serialResponse.substring(45,48).toDouble();
    
    if (motor == "motor4")
    {
      if(sw =="+"){
        //go forward
        digitalWrite(en7, LOW);
        digitalWrite(en8, HIGH);
        }
      else if (sw =="-"){
        //go back
        digitalWrite(en7, HIGH);
        digitalWrite(en8, LOW);
        }
    }
    speed_req[3] = vel;  //spped_req is ordered from motor 1 to motor 4
  }

   if((millis()-lastMilli) >= LOOPTIME)   { //LOOPTIME=100,lastmilli=0 at the start. This is done every 100ms and it does not depend on messages received! So I update the PWM every 100ms and print 
    lastMilli = millis();                   //every 500ms. If a new message is not received, try to mantain the correct PWM by checking it every 100ms
    for (int i = 0; i <= 3; i++) {    //countAnt and counter=0,0,0,0 initially; counter is updated by encoder changes
       speed_act[i] = (double)(((double)(counter[i] - countAnt[i])*(1000.0/(double)LOOPTIME))/(double)(320.0)); //compute the actual velocity (RPM) by checking encoder values
       countAnt[i]=counter[i];
       PWM_val[i]= updatePid(PWM_val[i], speed_req[i], speed_act[i], i);   //update pwm value passing lastPWM, adding the pwm to reach targed and applying Kd and Kp corrections
       analogWrite(motorPin[i], PWM_val[i]);
       printMotorInfo(i,speed_act[i],PWM_val[i]); 
   }  
  }
     
}


void printMotorInfo(int i, double speed_act, double PWM_val)  { // display data
      
   if((millis()-lastMilliPrint) >= 1500)  
   {
     if(i==0)
       Serial.print("<");
     Serial.print("RPM");     //print RPM and PWM each time >= 500 ms
     Serial.print(i);    
     Serial.print(":"); 
     Serial.print(speed_act);         
     Serial.print("  PWM:");  
     Serial.print(PWM_val);
     Serial.print("     ");   
     if(i==3){
       Serial.print(" > \n");
       lastMilliPrint = millis();
     }
   }          
}



int updatePid(int command, double targetValue, double currentValue, int i)   {   //command is PWM_val[i], target is speed_req (passed by us in message), currentValue is speed_act computed before
double pidTerm = 0.0;                                                            // PID correction
double error=0.0;                                                               
 error = (double) (fabs(targetValue) - fabs(currentValue)); 
 pidTerm = (Kp * error) + (Kd * (error - last_error[i]));                          
 last_error[i] = error;
 return constrain(command + int(pidTerm), 0, 255); //https://www.arduino.cc/reference/en/language/functions/math/constrain/
}

void checkA() {
  stateB = digitalRead(encoderB);
  stateA = digitalRead(encoderA);
  if (stateA != stateB) {
    counter[0]++;
  }
  else {
    counter[0]--;
  }
}

void checkB() {
  stateB = digitalRead(encoderB);
  stateA = digitalRead(encoderA);
  if (stateA == stateB) {
    counter[0]++;
  }
  else {
    counter[0]--;
  }
}


void checkC() {
  stateD = digitalRead(encoderD);
  stateC = digitalRead(encoderC);
  if (stateC != stateD) {
    counter[1]++;
  }
  else {
    counter[1]--;
  }
}

void checkD() {
  stateD = digitalRead(encoderD);
  stateC = digitalRead(encoderC);
  if (stateC == stateD) {
    counter[1]++;
  }
  else {
    counter[1]--;
  }
}

void checkE(){
  stateE = digitalRead(encoderE);
  stateF = digitalRead(encoderF);
  if (stateE != stateF) {
    counter[2]++;
  }
  else {
    counter[2]--;
  }
}

void checkF() {
  stateF = digitalRead(encoderF);
  stateE = digitalRead(encoderE);
  if (stateE == stateF) {
    counter[2]++;
  }
  else {
    counter[2]--;
  }
}


void checkG(){
  stateG = digitalRead(encoderG);
  stateH = digitalRead(encoderH);
  if (stateG != stateH) {
    counter[3]++;
  }
  else {
    counter[3]--;
  }
}

void checkH() {
  stateH = digitalRead(encoderH);
  stateG = digitalRead(encoderG);
  if (stateG == stateH) {
    counter[3]++;
  }
  else {
    counter[3]--;
  } 
}
