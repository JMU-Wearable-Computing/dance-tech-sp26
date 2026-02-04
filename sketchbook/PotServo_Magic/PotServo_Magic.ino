/****************************************************
   SPARKFUN BLACK BOARD MULTI-IO TEMPLATE
   Inputs:
     - ADXL313 Accelerometer (Qwiic / I2C)
     - 10k Potentiometer
     - Ultrasonic Distance Sensor

   Outputs:
     - RGB LED
     - Buzzer
     - Servo Motor

   Edit pin numbers and logic as needed!

****************************************************/

#include <Wire.h>                 // I2C communication
#include <SparkFunADXL313.h>     // SparkFun library
#include <Servo.h>               // Servo control

/**************** PIN DEFINITIONS ****************/

// --- Potentiometer ---
const int POT_PIN = A0;

// --- Ultrasonic Sensor ---
const int TRIG_PIN = 6;
const int ECHO_PIN = 7;

// --- RGB LED (PWM pins) ---
const int RED_PIN   = 9;
const int GREEN_PIN = 10;
const int BLUE_PIN  = 11;

// --- Buzzer ---
const int BUZZER_PIN = 5;

// --- Servo ---
const int SERVO_PIN = 3;


/**************** OBJECTS ****************/

ADXL313 accel;     // Accelerometer object
Servo myServo;     // Servo object


/**************** USER SETTINGS ****************/

// Change these limits to fit your project
int distanceThreshold = 10;     // thresh hold for distence senser to be set off in cm
int potThreshold = 512;         // midpoint of analog range

// Servo positions
int servoMin = 0;
int servoMax = 180;


/**************** SETUP ****************/

void setup()
{
  Serial.begin(115200);

  Wire.begin();   // Start I2C bus for Qwiic devices

  // ----- Accelerometer -----
  if (accel.begin() == false)
  {
    Serial.println("ADXL313 not detected. Check Qwiic cable!");
    while (1); //Freeze
  }

  accel.standby(); // Must be in standby before changing settings.
  accel.setRange(ADXL313_RANGE_1_G);  //range of the accel (could be 05, 1, 2, 4)
  accel.measureModeOn(); // wakes up the sensor from standby and puts it into measurement mode

  // ----- Ultrasonic -----
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // ----- RGB LED -----
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // ----- Buzzer -----
  pinMode(BUZZER_PIN, OUTPUT);

  // ----- Servo -----
  myServo.attach(SERVO_PIN);

  Serial.println("System Ready!");
}


/**************** INPUT FUNCTIONS ****************/

// Reads X/Y/Z acceleration values
void readAccelerometer(float xyz [3])
{
  accel.readAccel();

  xyz[0] = accel.x;
  xyz[1] = accel.y;
  xyz[2] = accel.z;

}


// Reads analog pot (0–1023)
int readPotentiometer()
{
  int value = analogRead(POT_PIN);

  return value;
}


// Measures distance in centimeters
float readUltrasonic()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);

  float distance = duration * 0.034 / 2;

//  Serial.print("Distance: ");
//  Serial.print(distance);
//  Serial.println(" cm");

  return distance;
}


/**************** OUTPUT FUNCTIONS ****************/

// Control servo using potentiometer
void controlServo(int potValue)
{
  // Map potentiometer to servo angle
  int angle = map(potValue, 0, 1023, 0, 180);

  // Limit range if needed (change these!)
  angle = constrain(angle, 10, 170);

  // Move servo
  myServo.write(angle);

  // Debug output
  Serial.print("Servo Angle: ");
  Serial.println(angle);
}


// Sounds buzzer if object is close
void controlBuzzer(float distance)
{
  if (distance > 0 && distance < distanceThreshold)
  {
    tone(BUZZER_PIN, 2000);  // 2kHz tone
  }
  else
  {
    noTone(BUZZER_PIN);
  }
}


//Light of RGB Based on XYZ accel data
void controlRGBfromAccel(float ax, float ay, float az)
{
  // Convert from G's to usable PWM values
  // Adjust range if needed (±2G, ±8G, etc.)
  int redValue   = map(abs(ax), 0.0, 520.0, 0, 255);
  int greenValue = map(abs(ay), 0.0, 520.0, 0, 255);
  int blueValue  = map(abs(az), 0.0, 520.0, 0, 255);

  // Output to LED
  if((redValue > greenValue) && (redValue > blueValue)) {
    digitalWrite(RED_PIN, HIGH);   
  } 
  else {
    digitalWrite(RED_PIN, LOW);
  }
  if((greenValue > redValue) && (greenValue > blueValue)) {
    digitalWrite(GREEN_PIN, HIGH);  
  } 
  else {
    digitalWrite(GREEN_PIN, LOW);
  }
  if((blueValue > redValue) && (blueValue > greenValue)) {
    digitalWrite(BLUE_PIN, HIGH);
  }  
  else {
    digitalWrite(BLUE_PIN, LOW);
  }
}


/**************** MAIN LOOP ****************/

float xyz[3];

void loop()
{
  // ----- READ INPUTS -----
  // pass these variables to the functions you write to used the input data

  readAccelerometer(xyz); // will return an array of the x, y and z values
  float ax = xyz[0];  //just the x value as a float
  float ay = xyz[1];  //just the y value as a float
  float az = xyz[2];  //just the x value as a float
  int potValue = readPotentiometer(); // will return an int 0-1023 
  float distance = readUltrasonic(); // will return a int distence in cm

  // ----- CONTROL OUTPUTS -----
  //call the functions you want to happen on the board

  controlBuzzer(distance); //buzzer sounds when within 10cm of distance sensor
  controlServo(potValue); // potentiometer controls servo movement
  controlRGBfromAccel(ax, ay, az); // lights up RGB led based on accelerometer orientation

  delay(100);   // Adjust refresh rate
}

