#include "OneWire.h"
extern ApplicationWatchdog *wd;
// --------------------------------------------------------------------------------------------
// For Enclosure Compass Heading
#define CMPS11_ADDRESS 0x60   // Address of CMPS11 shifted right one bit for arduino wire library
#define ANGLE_16 2            // Register to read 16 bit angle from
unsigned int CompassHeading;  // To hold value returned from read compass function
int readCompassHeading(void); // Returns heading as int 0 to 359 degrees.
int readWindAngle(void);
void ReadWind(int, bool);

// --------------------------------------------------------------------------------------------
// Reading wind speed
volatile int countRotations = 0;
volatile unsigned long TimeAtWindStartCounting = 0;
void countOneSpin(void);

// --------------------------------------------------------------------------------------------
// Reading wind direction
int readWindDirection(bool);
byte mask = 0x3F; // data mask for bits coming from rotary encoder
#define ROTARYENCODER_ADDRESS 0x40

const float OFFSET_COMPASS_POINTER = 264.0f; // Physical difference in degrees between 0 compass heading and 0 value of pointer

// --------------------------------------------------------------------------------------------
// Watchdog checks.
extern retained int lastTryRead;
#define TRYREADCOMPASS 1 // copied from main ino file.
#define TRYREADANGLE 2
#define TRYREADALLWORK 10
// --------------------------------------------------------------------------------------------
// Data storage arrays
// 10 second samples
#define RawWindArraySize 60 // sample every 10 seconds, consolidate every 10 minutes
unsigned int arrayWindRaw[RawWindArraySize][4];
// Array columns
#define colElapsedTime 0 // mS
#define colPulseCount 1
#define colWindDirection 2 // degrees
#define colCompass 3       // degrees

int indexArrayWindRaw = 0;

// Consolidated wind for off boarding
#define ConsolidateWindArraySize 12 // send the last 2 hours
unsigned int arrayWindConsolidated[ConsolidateWindArraySize][4];

// Array columns
#define colTimeStamp 0        // Unix timestamp at end of 10 minute sample
#define colWindSpeed 1        // mph x 100
#define colWindGust 2         // mph x 100
#define colWindDirectionAvg 3 // degrees

int indexArrayWindConsoidated = 0;
void consolidateWindMeasurements(int);
float CalcWindDirection(int, int);

void readWindIntoArray(bool notSkipDirection)
{
  Log.info("Do Wind");
  // read the 3 sensors and store in array.
  ReadWind(indexArrayWindRaw, notSkipDirection);

  indexArrayWindRaw++;
  if (indexArrayWindRaw >= RawWindArraySize)
  {
    indexArrayWindRaw = 0;
    // consolidate the 10 minutes of wind samples into a reading.
    consolidateWindMeasurements(indexArrayWindConsoidated); // calc avg and max wind speed, avg direction

    indexArrayWindConsoidated++;
    if (indexArrayWindConsoidated >= ConsolidateWindArraySize)
      indexArrayWindConsoidated = 0;
  }
}

/*******************************************************************************************/
/*******************************************************************************************/
//*******************************************************************************************/
// WIND FUNCTIONS
// Pin Interrupt handler for hall effect switch being activated with one revolution of the cups.
void countOneSpin()
{
  // for each pulse, incurrment the pulseCount
  countRotations++;
}

String WindDataJson(unsigned int lastSend)
{
  String data = "{\"site\":\"PYC\",\"wind\":[ ";

  for (size_t i = 0; i < arraySize(arrayWindConsolidated); ++i)
  {

    if (arrayWindConsolidated[i][colTimeStamp] > lastSend)
    {
      data.concat(String::format("{\"Time\":%lu,\"Spd\":%3.1f,\"Gst\":%3.1f,\"Dir\":%u},",
                                 arrayWindConsolidated[i][colTimeStamp],
                                 static_cast<float>(arrayWindConsolidated[i][colWindSpeed]) / 100.0,
                                 static_cast<float>(arrayWindConsolidated[i][colWindGust]) / 100.0,
                                 arrayWindConsolidated[i][colWindDirectionAvg]));
    }

    Log.info("wind string lenght: %d", data.length());
    if (!(data.length() < 570))
    {
      // max is 622, 52 characters in a data line
      Log.info("wind SKIPPING");
      break;
    }
  }
  data.remove(data.length() - 1); // Trim off extra comma
  data.concat("]}");
  return data;
}

void consolidateWindMeasurements(int index)
{
  // Calculate the wind average and guest (max)
  float windSpeedCalc = 0.0;
  float windSpeedAvg = 0.0;
  float windSpeedSum = 0.0;
  float windSpeedGust = 0.0;

  float windDirectionSum = 0.0;
  float windDirectionCalc = 0.0;
  float windDirectionAvg = 0.0;
  float sampleSize = static_cast<float>(arraySize(arrayWindRaw));

  for (size_t i = 0; i < arraySize(arrayWindRaw); ++i)
  {
    // Speed PPS to MPH
    windSpeedCalc = (static_cast<float>(arrayWindRaw[i][colPulseCount]) * 1000.0 / static_cast<float>(arrayWindRaw[i][colElapsedTime])) * 3.1462;

    windSpeedSum = windSpeedSum + windSpeedCalc;

    if (windSpeedCalc > windSpeedGust)
      windSpeedGust = windSpeedCalc;

    // Direction
    // apply offset to wind direction pointer
    windDirectionCalc = CalcWindDirection(arrayWindRaw[i][colCompass], arrayWindRaw[i][colWindDirection]);

    windDirectionSum = windDirectionSum + windDirectionCalc + 360;
  }
  // Calcuate the 2 averages
  windSpeedAvg = windSpeedSum / sampleSize;
  windDirectionAvg = (windDirectionSum / sampleSize) - 360.0;

  Log.info("consolidate: samples:%3.1f, SpdAvg:%3.1f, DirAvg:%3.1f", sampleSize, windSpeedAvg, windDirectionAvg);

  // Shift and store into array
  arrayWindConsolidated[index][colTimeStamp] = Time.now();
  arrayWindConsolidated[index][colWindSpeed] = static_cast<unsigned int>(windSpeedAvg * 100.0);
  arrayWindConsolidated[index][colWindGust] = static_cast<unsigned int>(windSpeedGust * 100.0);
  arrayWindConsolidated[index][colWindDirectionAvg] = static_cast<unsigned int>(windDirectionAvg);
}

float CalcWindDirection(int compassDirection, int windAngle)
{
  int windDirectionAdjustment = 0;
  int windDirectionCalc = 0;

  // Direction
  // apply offset to wind direction pointer
  windDirectionAdjustment = windAngle - OFFSET_COMPASS_POINTER;
  if (windDirectionAdjustment < 0)
    windDirectionAdjustment = windDirectionAdjustment + 360;

  if (windDirectionAdjustment >= 360)
    windDirectionAdjustment = windDirectionAdjustment - 360;

  // calc direction
  windDirectionCalc = compassDirection + windDirectionAdjustment;

  if (windDirectionCalc >= 360)
    windDirectionCalc = windDirectionCalc - 360;
  //  Log.info("{\"Comp2\":%i,\"Ang2\":%i,\"CalcWind\":%i}",compassDirection,windAngle, windDirectionCalc);

  return static_cast<float>(windDirectionCalc);
}

/*******************************************************************************************/
// Read sensors every 10 seconds and store into raw wind array. Called by software timer.
void ReadWind(int index, bool notSkipDirection)
{
  int angle = OFFSET_COMPASS_POINTER;
  int CompassHeading = 0;

  if (notSkipDirection)
  {
    lastTryRead = TRYREADANGLE;
    angle = readWindAngle();
    lastTryRead = TRYREADCOMPASS;
    CompassHeading = readCompassHeading();
  }
  lastTryRead = TRYREADALLWORK;

  wd->checkin(); // resets the AWDT count

  unsigned int currentTime = millis();
  unsigned int elapsedTime = currentTime - TimeAtWindStartCounting;

  arrayWindRaw[index][colElapsedTime] = elapsedTime;
  arrayWindRaw[index][colPulseCount] = countRotations;

  arrayWindRaw[index][colWindDirection] = angle;
  arrayWindRaw[index][colCompass] = CompassHeading;

  TimeAtWindStartCounting = currentTime;
  countRotations = 0;

  // TESTING
  Log.info("{\"Elsp\":%u,\"Cnt\":%u,\"Comp\":%u,\"Ang\":%u,\"calcWind\":%3.0f}",
           arrayWindRaw[index][colElapsedTime],
           arrayWindRaw[index][colPulseCount],
           arrayWindRaw[index][colCompass],
           arrayWindRaw[index][colWindDirection],
           CalcWindDirection(CompassHeading, angle));
}

/*******************************************************************************************/

// Function to read the magnetic rotary encoder
int readWindAngle()
{

  // Request heading
  Wire.beginTransmission(ROTARYENCODER_ADDRESS); // transmit to slave device #64
  Wire.write(254);                               // request direction
  Wire.endTransmission();                        // stop transmitting

  // Read heading
  Wire.requestFrom(ROTARYENCODER_ADDRESS, 2); // request 2 bytes from slave device #64

  if (2 == Wire.available()) // slave may send less than requested
  {
    char c = Wire.read(); // receive a byte as character
    char d = Wire.read(); // receive a byte as character

    // calcuate heading for the sensor. Sensor has counter clockwise increase. Change to clockwise.
    int dr = (d << 6) + (c & mask);
    int angle = 360.0 - ((dr >> 4) * 0.3516);

    if (angle == 360)
      angle = 0;

    return angle;
  }
  else
    return -99;
}
/*******************************************************************************************/
// COMPASS FUNCTIONS

int readCompassHeading()
{
  //Read the compensated heading.
  unsigned int lCompassHeading;
  unsigned char high_byte, low_byte;

  Wire.beginTransmission(CMPS11_ADDRESS); //starts communication with CMPS11
  Wire.write(ANGLE_16);                   //Sends the register we wish to start reading from
  Wire.endTransmission();

  Wire.requestFrom(CMPS11_ADDRESS, 2);

  while (Wire.available() < 2)
    ; // Wait for all bytes to come back

  high_byte = Wire.read();
  low_byte = Wire.read();

  lCompassHeading = high_byte; // Calculate 16 bit angle
  lCompassHeading <<= 8;
  lCompassHeading += low_byte;

  return lCompassHeading / 10; //compass gives 0.1 degree resoluiton. Do not need.
}