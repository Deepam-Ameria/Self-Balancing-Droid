#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

//Change the control pin values as per your connections with the L293D Motor Driver
const int EN1 = 3;  // Motor 1 speed control
const int IN1 = 4;  // Motor 1 direction
const int IN2 = 5;  /* Motor 1 direction (Requires 2 directional pins because L293D 
                    uses an H-Bridge configuration to control the flow of current)*/
const int EN2 = 6;  // Motor 2 speed control
const int IN3 = 7;  // Motor 2 direction
const int IN4 = 8;  // Motor 2 direction

const float angle_t = 0; //Target angle for steady-state
const float KP = 1.0;
const float KI = 1.0;
const float KD = 1.0;

float prev_error = 0;
float integral = 0;
unsigned long prev_time = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  //Initiallize object
  mpu.initialize();
  //Verify connection
  if (mpu.testConnection()) {
        Serial.println("MPU6050 connected successfully!");
    } else {
        Serial.println("MPU6050 connection failed.");
        while (1); // Stop if connection fails
    }

    // Initialize motor control pins
    pinMode(EN1, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(EN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    Steady(); //Stop the motors initially
 
}

void loop() {

  float angle = getAngle();
  float error = angle_t - angle;

  unsigned long current_time = millis(); //Milliseconds
  float dt = (current_time - prev_time)/1000.0; //Seconds

  //PID block
  integral += error*dt;
  float derivative = (error-prev_error)/dt;
  float pid = KP*error + KI*integral + KD*derivative;

  
  int motorSpeed = constrain(abs(pid), 0, 255); //Constraining to reasonable motor speed

  //Conditional motion of the droid
  if (pid>0){
    Backward(motorSpeed);
    }
    else if (pid<0){
      Forward(motorSpeed);
    }
    else {
      Steady();    
    }

  //Updating variable
    prev_error = error;
    prev_time = current_time;

    delay(20);

}

float getAngle(){
     // Read accelerometer and gyroscope data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Calculate pitch angle (simplified)
    float angle = atan2f(ax, az) * 180 / PI;
    return angle;
}

void Backward(int speed) //Function to rotate the wheel forward 
{
    motorState(HIGH, LOW, HIGH, LOW, speed);
    Serial.print("Moving backward..."); //Debugging statement 
}

void Forward(int speed) //Function to rotate the wheel Backward  
{
    motorState(LOW, HIGH, LOW, HIGH, speed);
    Serial.print("Moving forward..."); //Debugging statement
}

void Steady() //Function to stop both the wheels when steady state is achieved
{
    motorState(LOW, LOW, LOW, LOW, 0);
    Serial.print("Steady state..."); //Debugging statement
}

void motorState(int in1, int in2, int in3, int in4, int speed){
    analogWrite(EN1, speed);
    analogWrite(EN2, speed);
    digitalWrite(IN1, in1);
    digitalWrite(IN2, in2);
    digitalWrite(IN3, in3);
    digitalWrite(IN4, in4);
}
