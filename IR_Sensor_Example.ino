#include <ECE3.h>
#define SENS_ERROR 100

uint16_t sensorValues[8];
uint16_t calibrationMin[] = {773, 689, 821, 596, 576, 717, 721, 834};
uint16_t calibrationMax[] = {1727, 1733, 1679, 1037, 1682, 1688, 1666};
int16_t sensorWeights[] = {-15, -12, -8, -4, 4, 8, 12, 15};

int diffSum;
int16_t prevErrorValue = 0;
float kD = .105; // Works up to spd=100
float kP = .015;
int PIDSum;
int line_count = 0;
int past_read_count = 0;

bool hasTurned = false;
bool hasReachedTimedStart = false;
bool hasReachedTimedEnd = false;
bool hasReachedBiasing = false;

bool isOnBlack = false;

const int left_nslp_pin = 31;  // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin = 11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin = 29;
const int right_dir_pin = 30;
const int left_pwm_pin = 40;
const int right_pwm_pin = 39;

const int LED_RF = 75;
int wheelSpd = 20;

void do_turn_degree(int degree);
void turn_off_right_sensors();
void stop_car();

void setup()
{
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

    Serial.begin(9600); // set the data rate in bits per second for serial data
                        // transmission
    delay(2000);
}

void loop()
{
    // read raw sensor values
    ECE3_read_IR(sensorValues);
    int16_t errorValue = 0;
    int maxError = 0;
    int reading = 0;
    int max_read_count = 0;
    static int count2 = 0;

    for (int i = 0; i < 8; i++)
    {
        if (sensorValues[i] >= calibrationMin[i])
        {
            reading = (sensorValues[i] - calibrationMin[i]) * 1000 /
                      (float)calibrationMax[i];
            errorValue += (sensorWeights[i] / 8) * reading;
        }
        if (reading > 600)
            max_read_count += 1;
    }



    int leftSpd = wheelSpd;
    int rightSpd = wheelSpd;

    // PID controller
    diffSum = (errorValue - prevErrorValue);
    PIDSum = (kP * errorValue) + (kD * diffSum);
    leftSpd -= PIDSum;
    rightSpd += PIDSum;

    analogWrite(left_pwm_pin, leftSpd);
    analogWrite(right_pwm_pin, rightSpd);

        if (max_read_count > 6)
    {
        if (past_read_count > 6 && !isOnBlack) {
            isOnBlack = true;
            if (!hasTurned)
            {
                hasTurned = true; //turn fucks up on positions to the right
                digitalWrite(LED_RF, HIGH);
                do_turn_degree();
                digitalWrite(LED_RF, LOW);
            }
            else if (!hasReachedTimedStart) {
                hasReachedTimedStart = true;
            }
            else if (!hasReachedTimedEnd) {
                hasReachedTimedEnd = true;
                turn_off_right_sensors();
                digitalWrite(LED_RF, HIGH);
            }
            else if (hasTurned)
            {
                stop_car();
            }
        } else if (isOnBlack) {
            digitalWrite(LED_RF, HIGH);
        }
    } else {
        isOnBlack = false;
    }

    prevErrorValue = errorValue;
    past_read_count = max_read_count;
}

void stop_car() //doesnt work since 2 sensors are off
{
    digitalWrite(left_nslp_pin, LOW);
    digitalWrite(right_nslp_pin, LOW);

    while (true)
    {
        digitalWrite(LED_RF, LOW);
        delay(1000);
        digitalWrite(LED_RF, HIGH);
        delay(1000);
    }
}

void do_turn_degree()
{
    digitalWrite(LED_RF, HIGH);
    digitalWrite(left_dir_pin, HIGH);
    analogWrite(left_pwm_pin, 20);
    analogWrite(right_pwm_pin, 20);
    delay(4725);
    hasTurned = true;
    digitalWrite(left_dir_pin, LOW);
}

void turn_off_right_sensors()
{
    sensorWeights[0] = 0;
    sensorWeights[1] = 0;
    //sensorWeights[2] = 0; fixed issue