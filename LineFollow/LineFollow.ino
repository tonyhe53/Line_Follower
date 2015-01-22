#include <QTRSensors.h>

// This example is designed for use with eight QTR-1RC sensors or the eight sensors of a
// QTR-8RC module.  These reflectance sensors should be connected to digital inputs 3 to 10.
// The QTR-8RC's emitter control pin (LEDON) can optionally be connected to digital pin 2, 
// or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to 
// QTR_NO_EMITTER_PIN.

// The setup phase of this example calibrates the sensor for ten seconds and turns on
// the LED built in to the Arduino on pin 13 while calibration is going on.
// During this phase, you should expose each reflectance sensor to the lightest and 
// darkest readings they will encounter.
// For example, if you are making a line follower, you should slide the sensors across the
// line during the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is.  Improper calibration will result in poor readings.
// If you want to skip the calibration phase, you can get the raw sensor readings
// (pulse times from 0 to 2500 us) by calling qtrrc.read(sensorValues) instead of
// qtrrc.readLine(sensorValues).

// The main loop of the example reads the calibrated sensor values and uses them to
// estimate the position of a line.  You can test this by taping a piece of 3/4" black
// electrical tape to a piece of white paper and sliding the sensor across it.  It
// prints the sensor values to the serial monitor as numbers from 0 (maximum reflectance) 
// to 1000 (minimum reflectance) followed by the estimated location of the line as a number
// from 0 to 5000.  1000 means the line is directly under sensor 1, 2000 means directly
// under sensor 2, etc.  0 means the line is directly under sensor 0 or was last seen by
// sensor 0 before being lost.  5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.


#define NUM_SENSORS   14     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   32     // emitter is controlled by digital pin 2
#define QTR_EMITTERS_ON 1  // Makes sure Emitters on reflectance sensor are set to on.
#define black_line 0        // 3rd input to readLine(); use if black line on white surface
#define white_line 1        // 3rd input to readLine(); use if white line on black surface

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {5, 6, 7, 8, 9, 10, A0, A1, A2, A3, A4, A5, 3, 4},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];
int lastError = 0;
double KP = 0.1;
double KD = 5;
double M1 = 10;
double M2 = 10;


void stayStraight(){
  unsigned int sensors[14];
  // get calibrated sensor values returned in the sensors array, along with the line position
  // position will range from 0 to 2000, with 1000 corresponding to the line over the middle 
  // sensor
  int position = qtrrc.readLine(sensors, QTR_EMITTERS_ON, black_line);
 
  // compute our "error" from the line position.  We will make it so that the error is zero when
  // the middle sensor is over the line, because this is our goal.  Error will range from
  // -1000 to +1000.  If we have sensor 0 on the left and sensor 2 on the right,  a reading of 
  // -1000 means that we see the line on the left and a reading of +1000 means we see the 
  // line on the right.
  int error = position - 7000;
 
  // set the motor speed based on proportional and derivative PID terms
  // KP is the a floating-point proportional constant (maybe start with a value around 0.1)
  // KD is the floating-point derivative constant (maybe start with a value around 5)
  // note that when doing PID, it's very important you get your signs right, or else the
  // control loop will be unstable
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
 
  // M1 and M2 are base motor speeds.  That is to say, they are the speeds the motors should
  // spin at if you are perfectly on the line with no error.  If your motors are well matched,
  // M1 and M2 will be equal.  When you start testing your PID loop, it might help to start with
  // small values for M1 and M2.  You can then increase the speed as you fine-tune your
  // PID constants KP and KD.
  int m1Speed = M1 + motorSpeed;
  int m2Speed = M2 - motorSpeed;
 
  // it might help to keep the speeds positive (this is optional)
  // note that you might want to add a similiar line to keep the speeds from exceeding
  // any maximum allowed value
  if (m1Speed < 0)
    m1Speed = 0;
  if (m2Speed < 0)
    m2Speed = 0;
    
    if (m2Speed > 300)
    m2Speed = 300;
     if (m1Speed > 300)
     m1Speed = 300;
 
  // set motor speeds using the two motor speed variables above
  Serial.print("LS: ");
  Serial.print(m1Speed);
  Serial.print("     RS: ");
  Serial.print(m2Speed);
  Serial.print("     ");
  Serial.print(position);
  Serial.println();
}


void setup()
{
  Serial.begin(9600);
  Serial.println("Calibrating");
  delay(500);
  //pinMode(13, OUTPUT);
  //digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate(QTR_EMITTERS_ON);       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  //digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  /*
  unsigned int cMinOn[14] = {568, 728, 496, 736, 896, 816, 696, 576, 656, 652, 492, 576, 900, 656};
  unsigned int cMaxOn[14] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
  qtrrc.calibratedMinimumOn = cMinOn;
  qtrrc.calibratedMaximumOn = cMaxOn;
  */
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  

  Serial.println();
  Serial.println("Calibration End");
  delay(1000);
}


void loop()
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, black_line);
  int pos, low = 1000;
  //qtrrc.read(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance (white) and
  // 1000 means minimum reflectance (black), followed by the line position
  Serial.print("position = ");
  Serial.print(position);
  Serial.println();
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
  // need logic here to see if we want to turn left, right, or stay straight
  // follow mode: 0 = idle, 1 = turn left, 2 = stay straight, 3 = turn right, 4 = backwards.
  //stayStraight();
  delay(500);
  // test comment for commit
}
