#include "QTRSensors.h"

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
int normFactor = 1000;
char incomingByte;
char output_string[100];


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

void serialEvent(){
  // read the incoming byte:
  incomingByte = Serial.read();
  // parse the input
  /*
  '1' = read sensor  
  */
  switch (incomingByte) {
    
    case '1':
    sprintf(output_string, "Sensors 0:%u 1:%u 2:%u 3:%u 4:%u 5:%u 6:%u 7:%u 8:%u 9:%u 10:%u 11:%u 12:%u 13:%u", sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3], sensorValues[4], sensorValues[5], sensorValues[6], sensorValues[7], sensorValues[8], sensorValues[9], sensorValues[10], sensorValues[11], sensorValues[12], sensorValues[13]);
    Serial.print(output_string);
    break;
    
    default: // did not match one of the cases
    Serial.println("Did not match a case");
    
  }

}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, black_line);

  
}
