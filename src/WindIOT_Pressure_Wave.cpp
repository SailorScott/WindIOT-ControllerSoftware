// Special thanks to Earl C. from PYC with picking out the best approach for estimating wave heights.
// Researching sensors, doing testing and selecting the BMP390.
// Developing the crossing approach.
// https://github.com/adafruit/Adafruit_BMP3XX
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf
// https://forums.adafruit.com/viewtopic.php?f=19&t=189932&p=920153&hilit=bmp390#p920153

#include "Adafruit_BMP3XX.h"
// --------------------------------------------------------------------------------------------
// Watchdog checks.
extern retained int lastTryRead;
#define TRYREADPRESSURE 6
#define TRYREADALLWORK 10

#define STIPFIRSTREADINGS 5
#define SEALEVELPRESSURE_HPA (1013.25)
// Data storage arrays
// Array Sizes
#define PressureWaveArraySize 6              // send the last 1 hours
#define WaveSetPressureSamplesArraySize 2700 // 9 minutes x 0.200 second sample rate
#define WaveHeightsArraySize 120             // Estimate for 10 minutes of waves at 5 second period.

#define colTimeStamp 0   // Unix timestamp
#define colAbsPress 1    // degrees Pa
#define colSigWaveHtFt 2 // Ft
#define colBiggestWave 3
#define colCountWave 4
#define colWavePeriod 5 // Seconds

unsigned int arrayPressureWave[PressureWaveArraySize][6]; // save summarized data for sending to server.
int indexarrayPressureWave = 0;

float arrayWaveSetPressureSamples[WaveSetPressureSamplesArraySize];
int indexArrayWaveSetPressureSamples = 0;

float arrayWaveHeightsSamples[WaveHeightsArraySize];
int indexArrayWaveHeightsSamples = 0;

bool WriteSinglePressureIntoArray(void);
float calcWaveHeight(float, float);
void SaveWave(float);
float calcSigWaveHeight();
float biggestWave();
void writeWaveStats(int, float, float, float, int);

Adafruit_BMP3XX bmp;

float readPress();

bool setupPressureSensor()
{
  bool foundI2C = false;

  Log.info("Start BMP config");

  foundI2C = bmp.begin_I2C();
  if (!foundI2C)
  { // hardware I2C mode, can pass in address & alt Wire

    Log.info("Could not find a valid BMP3 sensor, check wiring!");
  }
  else
  {

    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    Log.info("Done BMP config");
  }
  return foundI2C;
}
/*******************************************************************************************/
// When done collecting wave data every 0.2 seconds for 9 minutes, then analyze the data looking for
// average pressure, crossing points and then doing calcualtions for significat wave height.
void waveAnalysisAndStore(int countPressureFail)
{
  float avgPressure = 0.0;
  float sumPressure = 0.0;
  float SigWaveHtFt = 0.0;
  float tmpMinPressure = 1000000.0;
  float tmpMaxPressure = 0.0;
  float tmpWaveHeight = 0.0;

  indexArrayWaveHeightsSamples = 0;
  int i = 0;

  // Reset the wave height array.
  for (i = 0; i < WaveHeightsArraySize; i++)
    arrayWaveHeightsSamples[i] = 0.0;

  // Loop through current set of measurements and find the average.
  for (i = STIPFIRSTREADINGS; i < WaveSetPressureSamplesArraySize; i++)
  {
    sumPressure = sumPressure + arrayWaveSetPressureSamples[i];
  }

  avgPressure = sumPressure / static_cast<float>(WaveSetPressureSamplesArraySize - STIPFIRSTREADINGS);

  // Loop through second time and get the waves
  for (i = STIPFIRSTREADINGS; i < WaveSetPressureSamplesArraySize; i++)
  {
    // Look for local min and max
    if (arrayWaveSetPressureSamples[i] < tmpMinPressure)
      tmpMinPressure = arrayWaveSetPressureSamples[i];

    if (arrayWaveSetPressureSamples[i] > tmpMaxPressure)
      tmpMaxPressure = arrayWaveSetPressureSamples[i];

    // Check for crossing, save the difference and reset variables
    // loop through and find the local high and low when crossing average.
    if (arrayWaveSetPressureSamples[i - 1] <= avgPressure && arrayWaveSetPressureSamples[i] > avgPressure)
    {
      tmpWaveHeight = calcWaveHeight(tmpMinPressure, tmpMaxPressure); // Sets the wave height for last wave

      if (tmpWaveHeight > 0.44) // .44 from analysis of non-moving sensor noise, cuts 84% of false crossings.
      {
        SaveWave(tmpWaveHeight);
        //  Log.info("in a crossing at index:%d, wave ht:%.2f", i, tmpWaveHeight);
      }
      tmpMinPressure = 1000000.0;
      tmpMaxPressure = 0.0;
    }
  } // done looping through wave data

  // Calc the significant wave height
  SigWaveHtFt = calcSigWaveHeight();

  Log.info("avgPressure: %.2f, minPressure:%.2f, maxPressure:%.2f, SigWaveHtFt:%.2f, biggestWave:%.2f, countWaves:%d",
           avgPressure, tmpMinPressure, tmpMaxPressure, SigWaveHtFt, biggestWave(), indexArrayWaveHeightsSamples);
  // store in array.
  writeWaveStats(indexarrayPressureWave, avgPressure, SigWaveHtFt, biggestWave(), indexArrayWaveHeightsSamples);

  // next saved wave
  indexarrayPressureWave++;
  if (indexarrayPressureWave >= PressureWaveArraySize)
    indexarrayPressureWave = 0;
}

// Save the wave height into a sorted array.
void SaveWave(float waveHt)
{

  float tmpWaveHt = 0;

  if (indexArrayWaveHeightsSamples < WaveHeightsArraySize)
  {
    //  Log.info("SortWaveArray wave#%d", indexArrayWaveHeightsSamples);
    indexArrayWaveHeightsSamples = indexArrayWaveHeightsSamples + 1;

    // go through array and insert after first record which is just bigger.
    for (size_t i = 0; i < WaveHeightsArraySize; i++)
    {
      if (waveHt > arrayWaveHeightsSamples[i])
      {
        tmpWaveHt = arrayWaveHeightsSamples[i]; // all others that are smaller are pushed down the stack
        arrayWaveHeightsSamples[i] = waveHt;
        waveHt = tmpWaveHt;
      }
    }
  }
}

// Siginicant wave height is average of top 1/3 of the wave measurements.
float calcSigWaveHeight()
{
  float sumSigWavs = 0.0;
  int top3rd = 0;

  // calculate what is 1/3 of the array
  top3rd = round(static_cast<float>(indexArrayWaveHeightsSamples) / 3.0);
  Log.info("countWaves %d, top3rd %d", indexArrayWaveHeightsSamples, top3rd);
  // Loop through and sum
  for (int i = 0; i < top3rd; i++)
    sumSigWavs += arrayWaveHeightsSamples[i];

  // Average and report.
  return sumSigWavs / static_cast<float>(top3rd);
}

// get the biggest wave by looking at the top cell in sorted array
float biggestWave()
{
  return arrayWaveHeightsSamples[0];
}

/*******************************************************************************************/
// Write and read data from pressure data array.
void writeWaveStats(int index, float pressure, float SigWaveHtFt, float biggestWave, int countWave)
{
  Log.info("countWave:%d, unsignedx10:%u", countWave, static_cast<unsigned int>(countWave * 10));
  // Start filling in array. Stored in unsigned value so shift and offset temps.
  arrayPressureWave[index][colTimeStamp] = Time.now();
  arrayPressureWave[index][colAbsPress] = static_cast<unsigned int>((pressure)*100.0);
  arrayPressureWave[index][colSigWaveHtFt] = static_cast<unsigned int>((SigWaveHtFt)*100.0);
  arrayPressureWave[index][colBiggestWave] = static_cast<unsigned int>((biggestWave)*100.0);
  arrayPressureWave[index][colCountWave] = static_cast<unsigned int>(countWave * 10);
}

// Read the temperatures array and send back a formated string for off boarding.
String PressureWaveJson(unsigned int lastSend)
{
  String data = "{\"site\":\"PYC\",\"pressureWave\":[";

  for (size_t i = 0; i < arraySize(arrayPressureWave); ++i)
  {
    if (arrayPressureWave[i][colTimeStamp] > lastSend)
    {
      data.concat(String::format("{\"Time\":%lu,\"AbsPressPa\":%3.1f, \"SigWaveHtFt\":%3.1f, \"BiggestWaveFt\":%3.1f, \"CountWave\":%u},",
                                 arrayPressureWave[i][colTimeStamp],
                                 static_cast<float>(arrayPressureWave[i][colAbsPress] / 100.0), // Recover from unsigned value.
                                 static_cast<float>(arrayPressureWave[i][colSigWaveHtFt] / 100.0),
                                 static_cast<float>(arrayPressureWave[i][colBiggestWave] / 100.0),
                                 arrayPressureWave[i][colCountWave] / 10));
    }
    // if (!(data.length() < 558)) // max is 622, 111 characters in a data line
    //   break;

    Log.info("Pressure string lenght: %d", data.length());
    if (!(data.length() < 545))
    {
      // max is 622, 68 characters in a data line
      Log.info("Pressrure SKIPPING");
      break;
    }
  }

  if(data.length() > 32)
    data.remove(data.length() - 1); // Trim off extra comma

  data.concat("]}");
  return data;
}
/*******************************************************************************************/
// Write the wave pressure data into analysis array.
// when done return true so that the analysis can be done.
bool WriteSinglePressureIntoArray(void)
{

  arrayWaveSetPressureSamples[indexArrayWaveSetPressureSamples] = readPress();

  indexArrayWaveSetPressureSamples++;
  if (indexArrayWaveSetPressureSamples >= WaveSetPressureSamplesArraySize)
  {
    indexArrayWaveSetPressureSamples = 0;
    Log.info("Done reading all the wave samples");
    return true;
  }
  else
    return false;
}
// Read a BMP390 prssure & temp sensor
float readPress()
{
  // Read the pressure from Adafruit BMP390.
  // Output: pressure in Pa.
  //  https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/pressure-sensors-bmp390.html#documents
  // https://github.com/adafruit/Adafruit_BMP3XX

  // Read pressure and set flag if we crash here.
  lastTryRead = TRYREADPRESSURE;

  float pressure = 1.0;
  if (!bmp.performReading())
  {
    Log.info("Failed to perform reading :(");
    return 0.0;
  }

  pressure = bmp.pressure;
  // Log.info("Temp:%f, Pressure:%f", bmp.temperature, pressure);
  lastTryRead = TRYREADALLWORK;
  return pressure;
}

/**************************************************************************/
/*!
    @brief Calculates the wave height in feet

    pass in the wave's low and high pressure reading, calculat difference.
   Assuming a fixed sea level sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @return Altitude in meters
*/
/**************************************************************************/
float calcWaveHeight(float pressureLow, float pressureHigh)
{
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  float pressureLowhPa;
  float pressureHighhPa;

  if (pressureLow < pressureHigh)
  {
    pressureLowhPa = pressureLow / 100.0F;
    pressureHighhPa = pressureHigh / 100.0F;
  }
  else
  {
    pressureLowhPa = pressureHigh / 100.0F;
    pressureHighhPa = pressureLow / 100.0F;
  }

  // lower pressure at higher elevation
  float elevHigh = 44330.0 * (1.0 - pow(pressureLowhPa / SEALEVELPRESSURE_HPA, 0.1903)) * 3.2808399;
  float elevLow = 44330.0 * (1.0 - pow(pressureHighhPa / SEALEVELPRESSURE_HPA, 0.1903)) * 3.2808399;

  if ((elevHigh - elevLow) > 50.0)
    return 0.0;
  else
    return elevHigh - elevLow;
}
