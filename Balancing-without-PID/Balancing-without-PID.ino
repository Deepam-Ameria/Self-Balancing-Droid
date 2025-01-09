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

    // Set initial motor state (off)
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    
 
}

void loop() {
  //Function call
  float angle = getAngle();
  float error = abs(angle_t - angle);
  int motorSpeed = constrain((error*10), 0, 255);

  //Conditional motion of the droid
  if (error>0){
    Backward();
    }
    else if (error<0){
      Forward();
    }
    else {
      Steady();    
    }
     analogWrite(EN1, motorSpeed);
     analogWrite(EN2, motorSpeed); 

     delay(20);
}

float getAngle(){
     // Read accelerometer and gyroscope data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Calculate pitch angle (simplified)
    float angle = atan2(ax, az) * 180 / PI;
    return angle;
}

void Backward() //Function to rotate the wheel forward 
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.print("Moving backward..."); //Debugging statement 
}

void Forward() //Function to rotate the wheel Backward  
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.print("Moving forward..."); //Debugging statement
}

void Steady() //Function to stop both the wheels when steady state is achieved
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    Serial.print("Steady state..."); //Debugging statement
}
