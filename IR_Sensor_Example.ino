#include <ECE3.h>
#define SENS_ERROR 100

uint16_t sensorValues[8];
uint16_t calibrationMin[] = { 773, 689, 821, 596, 576, 717, 721, 834 };
uint16_t calibrationMax[] = { 1727, 1733, 1679, 1037, 1682, 1688, 1666 };
int16_t sensorWeights[] = { -15, -12, -8, -4, 4, 8, 12, 15 };

int diffSum;
int16_t prevErrorValue = 0;
float kD = .105;  //Works up to spd=100
float kP = .015;
int PIDSum;
bool hasTurned = false;
bool hasLooped = false;

const int left_nslp_pin = 31;   // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin = 11;  // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin = 29;
const int right_dir_pin = 30;
const int left_pwm_pin = 40;
const int right_pwm_pin = 39;

const int LED_RF = 75;
int wheelSpd = 20;
int distance = 300;

int average()  //average pulse count
{
  int getL = getEncoderCount_left();
  int getR = getEncoderCount_right();
  //  Serial.print(getL);Serial.print("\t");Serial.println(getR);
  return ((getEncoderCount_left() + getEncoderCount_right()) / 2);
}


void setup() {
  ECE3_Init();

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, LOW);

  resetEncoderCount_left();
  resetEncoderCount_right();

  Serial.begin(9600);  // set the data rate in bits per second for serial data transmission
  delay(2000);
}


void loop() {
  // read raw sensor values
  ECE3_read_IR(sensorValues);
  int16_t errorValue = 0;
  int maxError = 0;
  bool isAllBlack;
  int reading = 0;
  int count1 = 0; 
  static int count2 = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] >= calibrationMin[i]) {
      //int reading = sensorValues[i]
      reading = (sensorValues[i] - calibrationMin[i]) * 1000 / (float)calibrationMax[i];
      errorValue += (sensorWeights[i] / 8) * reading;
    }
    if (reading > 600)
      count1 += 1;
  }

  if(count1 >= 7){
    count1 = 0;
    count2++;
}

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance

  int leftSpd = wheelSpd;
  int rightSpd = wheelSpd;
  //Serial.println(errorValue);
  //PID controller
  diffSum = (errorValue - prevErrorValue);
  PIDSum = (kP * errorValue) + (kD * diffSum);
  leftSpd -= PIDSum;
  rightSpd += PIDSum;  //look at signs

  analogWrite(left_pwm_pin, leftSpd);
  analogWrite(right_pwm_pin, rightSpd);

  resetEncoderCount_left();
  resetEncoderCount_right();
  //Serial.println(errorValue);

  prevErrorValue = errorValue;
 
  if (count2 >= 6 && !hasTurned) {
    count2 = 0;
    digitalWrite(LED_RF, HIGH);
    digitalWrite(left_dir_pin, HIGH);
    analogWrite(left_pwm_pin, 20);
    analogWrite(right_pwm_pin, 20);
    delay(4725);
    hasTurned = true;
    digitalWrite(left_dir_pin, LOW);
  }


  /*
if (hasTurned == false && atTurn == true)
  Do CCW 225 degree turn by making wheels go opposite directions
*/
}