#include "OneWire.h"
// --------------------------------------------------------------------------------------------
// Watchdog checks.
extern retained int lastTryRead;

#define TRYREADTEMPWATER 3 // copied from main ino file.
#define TRYREADTEMPAIR 4
#define TRYREADTEMPBOX 5
#define TRYREADALLWORK 10

// Data storage arrays
// Array Sizes
#define TemperaturesArraySize 6     // send the last 2 hours

#define colTimeStamp 0      // Unix timestamp
#define colTempWater 1      // degrees C x 10
#define colTempAir 2
#define colTempBox 3

unsigned int arrayTemperatures[TemperaturesArraySize][4];
void ReadTemperatures(int);
int indexArrayTemperatures = 0;

// --------------------------------------------------------------------------------------------
// Reading Temperatures
#define ONE_WIRE_BUS D4
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire ds = OneWire(ONE_WIRE_BUS);

// Temp addresses on one wire
byte waterThermometer[8] = {0x28, 0xB3, 0x9C, 0x75, 0x7, 0x0, 0x0, 0x85};
byte airThermometer[8] = {0x28, 0xE5, 0xB7, 0x75, 0x7, 0x0, 0x0, 0x91};
byte boxThermometer[8] = {0x28, 0x27, 0xB, 0x56, 0x7, 0x0, 0x0, 0xF6};

float readTemp(byte addr[8]);


void readTempIntoArray(void)
{
     Log.info("Do Temp");

    // read the 3 sensors and store in array.
    ReadTemperatures(indexArrayTemperatures);

   

    indexArrayTemperatures++;
    if (indexArrayTemperatures >= TemperaturesArraySize)
      indexArrayTemperatures = 0;

}


/*******************************************************************************************/
/*******************************************************************************************/
/*******************************************************************************************/
// TEMPERATURE FUNCTIONS
// Read sensors and store into temperatures array
void ReadTemperatures(int index)
{
  //Temperatures
  float tempWater = 0.0;
  float tempAir = 0.1;
  float tempBox = 0.2;
 
  // Start read of temp data
  ds.reset();        // first clear the 1-wire bus
  ds.write(0xCC, 0); // Send a all ROMS message
  ds.write(0x44, 0); // Start conversion

  // Read temps
  lastTryRead = TRYREADTEMPWATER;
  tempWater = readTemp(waterThermometer);

  lastTryRead = TRYREADTEMPAIR;
  tempAir = readTemp(airThermometer);
  lastTryRead = TRYREADTEMPBOX;
  tempBox = readTemp(boxThermometer);
  lastTryRead = TRYREADALLWORK;

  // Start filling in array. Stored in unsigned value so shift and offset temps.
  arrayTemperatures[index][colTimeStamp] = Time.now();
  arrayTemperatures[index][colTempWater] = static_cast<unsigned int>((tempWater + 50.0) * 100.0);
  arrayTemperatures[index][colTempAir] = static_cast<unsigned int>((tempAir + 50.0) * 100.0);
  arrayTemperatures[index][colTempBox] = static_cast<unsigned int>((tempBox + 50.0) * 100.0);
}

// Read the temperatures array and send back a formated string for off boarding.
String TemperaturesJson(unsigned int lastSend)
{
  String data = "{\"site\":\"PYC\",\"temp\":[";

  for (size_t i = 0; i < arraySize(arrayTemperatures); ++i)
  {
    if (arrayTemperatures[i][colTimeStamp] > lastSend)
    {
      data.concat(String::format("{\"Time\":%lu,\"Air\":%3.1f, \"Box\":%3.1f, \"Water\":%3.1f},",
                                 arrayTemperatures[i][colTimeStamp],
                                 static_cast<float>(arrayTemperatures[i][colTempAir]) / 100.0 - 50.0, // Recover from unsigned value.
                                 static_cast<float>(arrayTemperatures[i][colTempBox]) / 100.0 - 50.0,
                                 static_cast<float>(arrayTemperatures[i][colTempWater]) / 100.0 - 50.0));
    }
    // if (!(data.length() < 558)) // max is 622, 61 characters in a data line
    //   break;

    Log.info("Temp string lenght: %d", data.length());
    if (!(data.length() < 558))
    {
      // max is 622, 68 characters in a data line
      Log.info("Temp SKIPPING");
      break;
    }
  }

  data.remove(data.length() - 1); // Trim off extra comma
  data.concat("]}");
  return data;
}
// Read a dallas temp sensor
float readTemp(byte addr[8])
{
  // Read the temperature from one of the dallas sensors.
  // Input : Sensor serial number on one bus
  // Output; temp in degress celsius.
  // This assumes the temp sensors have already been sent the conversion command atleast 750 mSec ago.
  // Based on info from
  //https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf
  //https://github.com/tomdeboer/SparkCoreDallasTemperature

  // Given an address of a sensor, return the temp

  byte i;
  byte data[12];

  float celsius;

  ds.reset();
  ds.select(addr);
  ds.write(0xBE, 0); // Read Scratchpad

  // transfer data
  for (i = 0; i < 9; i++)
  { // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  int16_t raw = (data[1] << 8) | data[0];
  celsius = (float)raw * 0.0625;

  return celsius;
}
