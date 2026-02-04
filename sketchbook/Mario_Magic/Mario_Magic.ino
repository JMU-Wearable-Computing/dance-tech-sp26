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

// ========================================
// PLAY MARIO KART THEME ON BUZZER
// ========================================

void playMarioKartTheme()
{
  // Note frequencies (Hz)
  const int NOTE_E7  = 2637;
  const int NOTE_C7  = 2093;
  const int NOTE_G7  = 3136;
  const int NOTE_G6  = 1568;
  const int NOTE_E6  = 1319;
  const int NOTE_A6  = 1760;
  const int NOTE_B6  = 1976;
  const int NOTE_AS6 = 1865;
  const int NOTE_F7  = 2794;
  const int NOTE_D7  = 2349;
  const int NOTE_A7  = 3520;

  // Melody notes
  int melody[] = {
    NOTE_E7, NOTE_E7, 0,
    NOTE_E7, 0,
    NOTE_C7, NOTE_E7, 0,
    NOTE_G7, 0, 0, 0,
    NOTE_G6, 0, 0, 0,

    NOTE_C7, 0, 0,
    NOTE_G6, 0, 0,
    NOTE_E6, 0, 0,
    NOTE_A6, 0,
    NOTE_B6, 0,
    NOTE_AS6, NOTE_A6, 0,

    NOTE_G6, NOTE_E7, NOTE_G7,
    NOTE_A7
  };

  int noteDurations[] = {
    12,12,12,
    12,12,
    12,12,12,
    12,12,12,12,
    12,12,12,12,

    12,12,12,
    12,12,12,
    12,12,12,
    12,12,
    12,12,
    12,12,12,

    9,9,9,
    6
  };

  int notes = sizeof(melody) / sizeof(melody[0]);

  for (int i = 0; i < notes; i++)
  {
    int duration = 1000 / noteDurations[i];

    if (melody[i] > 0)
    {
      tone(BUZZER_PIN, melody[i], duration);
    }

    delay(duration * 1.3);
    noTone(BUZZER_PIN);
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

  if (potValue < 512)   // or button pressed, etc.
{
  playMarioKartTheme();
}

  delay(100);   // Adjust refresh rate
}

