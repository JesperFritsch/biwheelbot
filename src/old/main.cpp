#include <Arduino_LSM9DS1.h>



#define enA 10
#define inA1 11
#define inA2 12
#define encodeA1 5
#define encodeA2 6
#define enB 9
#define inB1 7
#define inB2 8
#define encodeB1 3
#define encodeB2 4
#define SPF 20
#define POSF 10
#define IPF 4
#define RATE_FAC 10
#define MAX_PWM 255
#define MAX_SETPOINT 45
#define MAX_ANGLE 75 //70
#define MAX_SPEED 12
#define MAX_TURN 150
#define MAX_INPUT 100
#define MAX_TURNSPEED 25
#define MAX_TURN_CUT 0.35

float a_x, a_y, a_z, g_x, g_y, g_z; // variables for accelerometer raw data
float x_angle = 0;

float bkp = 20; //6
float bki = 0;
float bkd = 65; //17

float skp = 5; //3.5
float skd = 0; //0
float ski = 0; //0

float pkp = 0.00003; //0.00005
float pkd = 0.17; //0.85
float pki = 0.001;

float tukp1 = 6.2; // 4
float tukd1 = 1.5; // 0.03
float tuki1 = 0.0;
float tukp2 = 4;
float tukd2 = 8;

float tkp = 1.7; //1.2
float tkd = 1.5; //0
float tki = 0.0005;


float set_point = 0;
int desiredSpeed = 0;
float turnspeed = 0;

bool reset = 0;
bool balance = 0;
volatile long int motorA = 0;
volatile long int motorB = 0;
int speedA, speedB;
long int posA, posB, ABrelation, wanted_posA = 0, wanted_posB = 0;
int moveA = 0, moveB = 0;
int16_t pwmA = 0, pwmB = 0;
int16_t balancePWM = 0;
float wantedSpeed;
float travelSpeed;
float botspeed;
long int wantedRelAB = 0;
int speedABRel = 0;

char tmp_str[200]; // temporay variable used in convert function
unsigned long timer = 0;
unsigned long timestamp = 0, milli;
double looptime = 0;

void encodeA();
void encodeB();

void setup() {
  Serial.begin(115200);
//   Serial1.begin(230400);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  pinMode(encodeA1, INPUT_PULLUP);
  pinMode(encodeA2, INPUT_PULLUP);
  pinMode(encodeB1, INPUT_PULLUP);
  pinMode(encodeB2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encodeA1), encodeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encodeB1), encodeB, CHANGE);

}

void encodeA(){
  if (digitalRead(encodeA2) == digitalRead(encodeA1)){
    motorA++;
  }
  else{
    motorA--;
  }
}

void encodeB(){
  if (digitalRead(encodeB2) != digitalRead(encodeB1)){
    motorB++;
  }
  else{
    motorB--;
  }
}

void get_angle(float g_x, float a_y, float* x_angle, double looptime){
  static double y_acc, x_gyro;
  y_acc = a_y * 90;
  x_gyro = -g_x;

  *x_angle = (0.97)*(*x_angle + (x_gyro * (looptime / 1000))) + (0.03)*(y_acc);
}

void receive_vals(int* drive, float* turn){
  static int8_t buff[2];
  static int8_t pointsT[IPF], pointsD[IPF];
  static float Dout;
  static float Tout;
  static unsigned long failsafeTimer = 0;
  bool go_filter = 0;
  static int i = 0;
  if(Serial1.available()){
    int8_t in = Serial1.read();
    if(in == '\n' && i >= 2){
      i = 0;
    }
    else{
      buff[i] = in;
      i++;
    }
    if(i == 2){
      go_filter = 1;
    }
  }
  if(millis() > failsafeTimer + 200){
    *drive = 0;
    *turn = 0;
  }
  if(go_filter){
    failsafeTimer = millis();
    Dout = buff[0];
    Tout = buff[1];
    for(uint8_t i = IPF - 1; i > 0; i--){
      pointsT[i] = pointsT[i-1];
      pointsD[i] = pointsD[i-1];
      Tout += pointsT[i];
      Dout += pointsD[i];
    }
    pointsD[0] = buff[0];
    pointsT[0] = buff[1];
    Tout /= IPF;
    Dout /= IPF;
    *drive = MAX_SPEED * (Dout / MAX_INPUT);
    *turn = Tout / MAX_INPUT;
  }
}

void safe_start(){
  static bool angleReady = 0;
  if(x_angle < 5 && x_angle > -5 && angleReady){
    balance = 1;
  }
  else if(x_angle < -MAX_ANGLE || x_angle > MAX_ANGLE){
    angleReady = 1;
    reset = 1;
    motorA = 0;
    motorB = 0;
    balance = 0;
    wanted_posA = 0;
    wanted_posB = 0;
    wantedRelAB = 0;
    pwmA = 0;
    pwmB = 0;
  }
}

void stand_still(){
  static bool stopped = 0;
  static bool upright = 0;
  static bool once = 0;
  if(desiredSpeed != 0){
    stopped = 0;
    upright = 0;
    once = 1;
  }
  else{
    if(botspeed <= 1 && botspeed >= -1){
      stopped = 1;
    }
    if(x_angle <= 7 && x_angle >= -7){
      upright = 1;
    }
  }
  if(once && stopped && upright){
    once = 0;
    wanted_posA = motorA;
    wanted_posB = motorB;
  }

}

void get_speedandpos(){
  static long int lastA, lastB;
  static int last_ABrel = 0;
  speedA = motorA - lastA;
  speedB = motorB - lastB;
  botspeed = (speedA + speedB) / 2;
  lastA = motorA;
  lastB = motorB;
  posA = motorA;
  posB = motorB;
  ABrelation = motorA - motorB;
  speedABRel = ABrelation - last_ABrel;
  last_ABrel = ABrelation;

}

int16_t speed_to_pwm(float wanted, float actual){
  static bool is_neg = 0;
  static float error, error_rate = 0, last_error = 0, cum_error = 0;
  error = wanted - actual;
  error_rate = error - last_error;
  cum_error += error;
  int out = (error * skp) + (error_rate * skd) + (cum_error * ski);
  last_error = error;

  if(reset){
    cum_error = 0;
    reset = 0;
  }

  if(cum_error < -MAX_PWM){
    cum_error = -MAX_PWM;
  }
  else if(cum_error > MAX_PWM){
    cum_error = MAX_PWM;
  }

  if(out < -MAX_PWM){
    out = -MAX_PWM;
  }
  else if(out > MAX_PWM){
    out = MAX_PWM;
  }
  balancePWM = out;
  pwmA = out;
  pwmB = out;
}


void assign_turn(int (*turn)(float, float), float turnspeed, float desiredSpeed, float g_z){
  int turn_pwm = (*turn)(turnspeed, g_z);
  turn_pwm *= 1 - abs(MAX_TURN_CUT * (botspeed / MAX_SPEED));
  float speed_rate = desiredSpeed / MAX_SPEED;
  float balance_rate = x_angle / MAX_ANGLE;
  float used_rate = 0;
  if(abs(speed_rate) >= abs(balance_rate)){
    used_rate = speed_rate;
  }
  else{
    used_rate = balance_rate;
  }
  pwmA += (turn_pwm + (abs(turn_pwm) * used_rate));
  pwmB -= (turn_pwm - (abs(turn_pwm) * used_rate));
}

int turn(float turnspeed, float g_z){
  static float errorT, errorRateT, lastErrorT, cumError = 0;
  float wanted_turn = turnspeed * MAX_TURNSPEED;
  int Tout;
   if(turnspeed != 0){
    wantedRelAB = ABrelation;
    errorT = wanted_turn - speedABRel;
    errorRateT = errorT - lastErrorT;
    cumError += errorT;

    Tout = (errorT * tukp1) + (errorRateT * tukd1) + (cumError * tuki1);
    if(Tout > MAX_TURN){
      Tout = MAX_TURN;
    }
    else if(Tout < -MAX_TURN){
      Tout = -MAX_TURN;
    }
    sprintf(tmp_str, "wantedT: %3.2f, errorT: %3.2f, out: %4d, speed: %3d, TS: %.2f", wanted_turn, errorT, Tout, speedABRel, turnspeed);
  }
  else{
    cumError = 0;
    errorT = wantedRelAB - ABrelation;
    errorRateT = errorT - lastErrorT;
    Tout = (errorT * tukp2) + (errorRateT * tukd2);
    lastErrorT = errorT;

    if(Tout > MAX_TURN){
      Tout = MAX_TURN;
    }
    else if(Tout < -MAX_TURN){
      Tout = -MAX_TURN;
    }
  }
  return Tout;
}

float positionPD(){
  static int last_error = 0, error, error_rate, cum_error = 0;
  static int outs[POSF];
  static bool is_neg = 0;
  error = (wanted_posA + wanted_posB) - (posA + posB);
  error_rate = (error - last_error);
  cum_error += error;

  if(((posA + posB) < 0) != is_neg){
    cum_error = 0;
    is_neg = !is_neg;
  }

  if(cum_error < -500){
    cum_error = -500;
  }
  else if(cum_error > 500){
    cum_error = 500;
  }
  outs[0] = (error * abs(error)) * pkp + (error_rate * pkd) + (cum_error * pki);
  float out = 0;
  for(uint8_t i = POSF - 1; i > 0; i--){
    outs[i] = outs[i-1];
    out += outs[i];
  }
  out /= POSF;

  if(out < -MAX_SPEED){
    out = -MAX_SPEED;
  }
  else if(out > MAX_SPEED){
    out = MAX_SPEED;
  }
  last_error = error;
  travelSpeed = out;
}

int travelPD(){
  static float error, error_rate, last_error = 0, cum_error = 0;
  static bool is_neg = 0;
  error = travelSpeed - botspeed;
  error_rate = last_error - error;
  cum_error += error;
  if(!balance){
    cum_error = 0;
  }
  float out = (error * tkp) + (error_rate * tkd) + (cum_error * tki);
  if(out > MAX_SETPOINT){
    out = MAX_SETPOINT;
  }
  else if(out < -MAX_SETPOINT){
    out = -MAX_SETPOINT;
  }
  last_error = error;
  return out;
}

float setpoint_filter(float wanted_point){
  static float points[SPF];
  float out = wanted_point;
  for(uint8_t i = SPF - 1; i > 0; i--){
    points[i] = points[i-1];
    out += points[i];
  }
  points[0] = wanted_point;
  return out / (float)SPF;

}

int balancePID(float angle){
  static bool is_neg = 0;
  static float error, cumError, errorRate, lastError, lastRerror = 1;
  float out;
  error = angle - set_point;
  cumError += error;
  errorRate = (error - lastError);

  out = (error * bkp) + (cumError * bki) + (errorRate * bkd);
  lastError = error;

  if((angle < 0) != is_neg){
    cumError = 0;
    is_neg = !is_neg;
  }

  if(cumError > 100){
    cumError = 100;
  }
  else if(cumError < -100){
    cumError = -100;
  }
  balancePWM = out;
}


void loop() {
//   receive_vals(&desiredSpeed, &turnspeed);
  float measure_time;
  if(millis() > timer + 10){
    milli = millis();
    timer = milli;
    looptime = (double)(milli - timestamp);
    timestamp = milli;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(g_x, g_y, g_z);
    }
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(a_x, a_y, a_z);
    }
    safe_start();
    get_angle(g_x, a_y, &x_angle, looptime);
    get_speedandpos();
    stand_still();
    if(balance){
      balancePID(x_angle);
      pwmA = balancePWM;
      pwmB = balancePWM;
      assign_turn(turn, turnspeed, desiredSpeed, g_z);
    }
    if(desiredSpeed != 0){
      wanted_posA = motorA;
      wanted_posB = motorB;
      travelSpeed = desiredSpeed;
    }
    else{
      positionPD();
    }
    set_point = setpoint_filter(travelPD());


    if(pwmA > 0){
      digitalWrite(inA1, HIGH);
      digitalWrite(inA2, LOW);
      analogWrite(enA, pwmA);
    }
    else{
      digitalWrite(inA1, LOW);
      digitalWrite(inA2, HIGH);
      analogWrite(enA, -pwmA);
    }
    if(pwmB > 0){
      digitalWrite(inB1, HIGH);
      digitalWrite(inB2, LOW);
      analogWrite(enB, pwmB);
    }
    else{
      digitalWrite(inB1, LOW);
      digitalWrite(inB2, HIGH);
      analogWrite(enB, -pwmB);
    }
    measure_time = millis() - milli;
    sprintf(tmp_str, "looptime: %f, time: %f, pwmA: %d, pwmB: %d", looptime, measure_time, pwmA, pwmB);
    Serial.println(tmp_str);
  }

}