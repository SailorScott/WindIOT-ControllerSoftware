/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/ScottsProjects/WindIOT-ControllerSoftware/src/WindIOTCellular.ino"
/*
 * Project WindIoTCellular
 * Description:
 * Author: Scott Nichols
 * Date:
 */

//  This file contains the code for state control, battery management and offboarding of data.

#include "Particle.h"

// Contains sensor reading code, storage and formating for off boarding.
#include "WindIOT_Temps.h"
#include "WindIOT_Wind.h"
#include "WindIOT_Pressure_Wave.h"
void setup();
void loop();
void fnOffBoardData();
void fnTempMeasurement();
void fnWindMeasurement();
void fnWavePressureMeasurement();
void fnWaveDelayStart();
void fnWaveStartMeasurements();
void fnTurnOffComms();
void fnCheckBattery();
void PublishData();
String readBatterySystemStats();
void watchdogHandler();
#line 16 "c:/ScottsProjects/WindIOT-ControllerSoftware/src/WindIOTCellular.ino"
SerialLogHandler logHandler;

// --------------------------------------------------------------------------------------------
// program control and announcement
int led1 = D7; // Onboard Blue LED

// --------------------------------------------------------------------------------------------
// Software Timers
Timer timerWindMeasurement(10 * 1000, fnWindMeasurement);      // every 10 seconds get wind speed and direction
Timer timerTempMeasurement(20 * 60 * 1000, fnTempMeasurement); // 20*60*1000 // every 20 minutes make temperature measurments

Timer timerWaveDelayStart(8 * 60 * 1000, fnWaveDelayStart, true);         // Start after 8 minutes
Timer timerWaveStartMeasurements(10 * 60 * 1000, fnWaveStartMeasurements); // every 10 minutes start taking samples.
Timer timerWavePressureMeasurement(200, fnWavePressureMeasurement);       // pressure sample when enabled

Timer timerOffboardData(20 * 60 * 1000, fnOffBoardData);          // every 20 minutes turn on comms and publish data
Timer timerOneTimeTurnOffComms(150 * 1000, fnTurnOffComms, true); // After first 2.5 minutes after start, turn off comms to save battery. should have time update
Timer timerCheckBattery(20 * 60 * 1000, fnCheckBattery);          // 20*60*1000 // every 20 minutes read SOC

bool doWindMeasurement = false;
bool doTempMeasurement = false;
bool doWavePressureMeasurement = false;
bool doneReadingWave = false;
bool doWaveAnalysisAndStore = false;
bool doTurnOnRadio = false;
bool doOffBoardData = false;

// --------------------------------------------------------------------------------------------
// Watchdog timer - to catch system resets. Report back with battery data.
ApplicationWatchdog *wd;
void watchdogHandler(void);
retained int countCompassFail = 0;
retained int countAngleFail = 0;
retained int countTempWaterFail = 0;
retained int countTempAirFail = 0;
retained int countTempBoxFail = 0;
retained int countPressureFail = 0;

#define EEPROMRESETCOUNT 0 // Memory address for tracking resets. Also copied into wind and temps as needed.
#define TRYREADCOMPASS 1
#define TRYREADANGLE 2
#define TRYREADTEMPWATER 3
#define TRYREADTEMPAIR 4
#define TRYREADTEMPBOX 5
#define TRYREADPRESSURE 6
#define TRYREADALLWORK 10

retained int lastTryRead = TRYREADALLWORK; // Varialble to keep track of what sensor was read last.

// --------------------------------------------------------------------------------------------
// Battery mangement
String readBatterySystemStats(void);
FuelGauge fuel;
double batterySOC = 0; // Variable to keep track of LiPo state-of-charge (SOC)

bool Send20Min = true; // Normal mode is 20 min send, when low battery send hourly.
// --------------------------------------------------------------------------------------------
// Publishing data
bool flagCanTurnOffComms = false;
SYSTEM_MODE(SEMI_AUTOMATIC); // need to control connection state.
SYSTEM_THREAD(ENABLED);
// --------------------------------------------------------------------------------------------
// Setup threading
// https://community.particle.io/t/electron-not-recognizing-single-threaded-block/21750
// https://github.com/rickkas7/particle-threads
void startupFunction();
void StartOffboardComms(void *param);

// The mutex is initialized in startupFunction()
STARTUP(startupFunction());

Thread threadStartComms;
system_tick_t lastThreadTime = 0;

os_mutex_t mutex;

// Note: threadFunction will be called before setup(), so you can't initialize the mutex there!
// STARTUP() is a good place to do it
void startupFunction()
{

  Log.info("In startup Function");
  // Create the mutex
  os_mutex_create(&mutex);

  // Initially lock it, so when the thread tries to lock it, it will block.
  // It's unlocked when time to turn on comms()
  os_mutex_lock(mutex);

  // Start watchdog. Reset the system after 60 seconds if
  // the application is unresponsive.
  wd = new ApplicationWatchdog(60000, watchdogHandler, 1536);
}
// --------------------------------------------------------------------------------------------
// setup() runs once, when the device is first turned on.
void setup()
{
  System.enableFeature(FEATURE_RETAINED_MEMORY);

  Particle.connect(); // Connect and get the time

  // Put initialization like pinMode and begin functions here.
  pinMode(led1, OUTPUT);

  // Set time zone to Eastern USA daylight saving time
  Time.zone(-4);

  // one wire for compass and angle
  Wire.begin();

  // hall effect switch ones per rotation of cups
  attachInterrupt(A0, countOneSpin, RISING);

  // Setup the BMP390 pressure setting config
  setupPressureSensor();

  // current time for start of counting.
  TimeAtWindStartCounting = millis();

  timerTempMeasurement.start();
  timerWindMeasurement.start();
  timerWaveDelayStart.start();
  timerOneTimeTurnOffComms.start();
  delay(1000); // let the measurements finish.
  timerOffboardData.start();
  threadStartComms = Thread("treadStartComms", StartOffboardComms);

  batterySOC = fuel.getSoC(); // when starting get the state of charge.

  // initialize watchdog resets counters
  uint16_t resets;
  EEPROM.get(EEPROMRESETCOUNT, resets);
  if (resets == 0xFFFF)
  {
    // EEPROM was empty -> initialize value
    resets = 0;
  }

  EEPROM.put(EEPROMRESETCOUNT, resets);
}

// loop() runs over and over again, as quickly as it can execute.
void loop()
{

  if (doTempMeasurement && ((countTempAirFail + countTempWaterFail + countTempBoxFail) < 4))
  {
    doTempMeasurement = false;
    readTempIntoArray();

    Log.info(readBatterySystemStats());
  }

  if (doWindMeasurement)
  {
    doWindMeasurement = false;
    readWindIntoArray(((countAngleFail + countCompassFail) < 4)); // pass in if we failed.
  }

  if (doWavePressureMeasurement)
  {
    doWavePressureMeasurement = false;
    if (countPressureFail < 4) // if less then 4 failures make a measurement.
      doneReadingWave = WriteSinglePressureIntoArray();

      if (doneReadingWave) {
        Log.info("Done Reading Wave!!!!");
        timerWavePressureMeasurement.stop();
        doWindMeasurement = false;
        // flag to do analysis.
        doWaveAnalysisAndStore = true;
      }
  }

  if (doWaveAnalysisAndStore)
  {
      doWaveAnalysisAndStore = false;
      waveAnalysisAndStore(1); // loop through and find average, peaks
  }

  if (doTurnOnRadio)
  {
    doTurnOnRadio = false;
    os_mutex_unlock(mutex); // allow the thread to run that turns on radio.
  }

  if (doOffBoardData) // Radio on and connected to Particle
  {
    doOffBoardData = false;

    PublishData();

    // Close connection and turn off comms
    if (flagCanTurnOffComms)
    {
      Log.info("Disconect and turn off cell");
      Particle.disconnect();
      Cellular.off();
    }
  }

  if ((batterySOC < 50.0) && Send20Min) // got enough juice?
  {
    Send20Min = false;
    timerOffboardData.changePeriod(60 * 60 * 1000); // Make 1 hour. Next transmit could be longer then hour.
  }

  if ((batterySOC > 55.0) && !Send20Min) // got enough juice?
  {
    Send20Min = true;
    timerOffboardData.changePeriod(20 * 60 * 1000); // Make 1 hour. Next transmit could be longer then hour.
  }

  if (batterySOC < 25) // Battery almost dead!! Go to sleep while battery charges.
  {
    // Sleep untill ~noon the next day. Hopefully have enough juice to resume
    int hours2Sleep = 1;
    int hourOfDay = Time.hour();

    if (hourOfDay < 12)
    {
      hours2Sleep = 12 - hourOfDay;
    }
    else
    {
      hours2Sleep = 12 + (24 - hourOfDay);
    }

    System.sleep(SLEEP_MODE_DEEP, 3600 * hours2Sleep);
  }

} // end LOOP

void StartOffboardComms(void *param)
{
  while (true)
  {
    // Block until unlocked by the timer to send data off board.
    os_mutex_lock(mutex);

    int countTries = 0;
    Log.info("Do turn on radio");
    // turn on
    if (!Cellular.ready())
    {
      Cellular.on();
      Cellular.connect();
      delay(5000);
      do
      {
        delay(5000);
        countTries++;
        Log.info("Cell on? %d, connecting: %d", countTries, Cellular.connecting());
      } while (!Cellular.ready() && (countTries < 30));

      Log.info("past Cell on..");
    }

    if (Cellular.ready() && !Particle.connected())
    {
      Log.info("Connecting particle");

      Particle.connect();
      countTries = 0;
      do
      {
        delay(5000);
        countTries++;
        Log.info("Connect Particle waits %d, connected: %d", countTries, Particle.connected());
      } while (!Particle.connected() && countTries < 20);
    }

    // Reset counters an flags
    if (!Particle.connected() && countTries == 20)
    { // did not successfully connect. Close down cell.
      Log.info("Did not connect, shut down cell");
      doOffBoardData = false;
      Particle.disconnect();
      Cellular.off();
    }
    else
    {
      doOffBoardData = true; // if on then send data.
    }
    Log.info("End thread function, connected?  %d", doOffBoardData);
  }
  // You must not return from the thread function
}

/******************************************************************************************/
// Timer calls, simple turn on flags
void fnOffBoardData() // Turn on comms and publish data. Called by timer timerOffboardData
{
  doTurnOnRadio = true;
}
void fnTempMeasurement()
{
  doTempMeasurement = true;
}
void fnWindMeasurement()
{
  doWindMeasurement = true;
}
// ***** Pressures and wave timers **********
// after a delay, we start measureing wavesets very 20 minutes. A wave set is sampled every 0.2 seconds.
void fnWavePressureMeasurement() // a single measurement every 0.2 seconds
{
  doWavePressureMeasurement = true;
}

void fnWaveDelayStart() // one time
{
  timerWaveStartMeasurements.start();
}
void fnWaveStartMeasurements() // every 20 min start a series of measurements.
{
  timerWavePressureMeasurement.start(); 
}
// ***************
void fnTurnOffComms()
{
  flagCanTurnOffComms = true;
  if (!doOffBoardData) // if not sending any data, then close.
  {
    Particle.disconnect();
    Cellular.off();
  }
}
void fnCheckBattery()
{
  batterySOC = fuel.getSoC();
}
/*******************************************************************************************/
/*******************************************************************************************/
/******************************************************************************************/
// Format the data strings and send off board.
void PublishData()
{
  static unsigned int lastSuccessfulPublish = 10; // tracks when last publish occurs. Used to send only newer data.
  unsigned int attempt2Publish;
  bool successfulPublish = false;

  if (Particle.connected())
  {
    Log.info("In Publish Data, connected");

    String temps = TemperaturesJson(lastSuccessfulPublish);

    String windData = WindDataJson(lastSuccessfulPublish);

    String pressData = PressureWaveJson(lastSuccessfulPublish);

    String batteryStats = readBatterySystemStats();

    attempt2Publish = Time.now();

    Particle.publish("BatteryStats", batteryStats, PRIVATE);

    Particle.publish("WindMPH", windData, PRIVATE); // assuming if one works, both work. Plus wind more important

    Particle.publish("PressWave", pressData, PRIVATE); 

    successfulPublish = Particle.publish("TempC", temps, PRIVATE);

    delay(5000); // let the data get transfered.

    if (successfulPublish)
    {
      lastSuccessfulPublish = attempt2Publish;
    }
  }
}

String readBatterySystemStats()
{ // Build up string with battery stats

  batterySOC = fuel.getSoC();

  uint16_t resets;
  EEPROM.get(EEPROMRESETCOUNT, resets);

  // fuel.getAlert() returns a 0 or 1 (0=alert not triggered)
 String data = String::format("{\"site\":\"PYC\",\"Time\":%lu,\"SOC\":%3.1f,\"ResetsCntr\":%u,\"AngleFail\":%u,\"CompassFail\":%u,\"TempAirFail\":%u,\"TempWaterFail\":%u, \"TempBoxFail\":%u, \"PressureFail\":%u}",
                               Time.now(), batterySOC, resets, countAngleFail, countCompassFail, countTempAirFail, countTempWaterFail, countTempBoxFail, countPressureFail);

  return data;
}

void watchdogHandler()
{
  // Do as little as possible in this function, preferably just
  // calling System.reset().
  // Do not attempt to Particle.publish(), use Cellular.command()
  // or similar functions. You can save data to a retained variable
  // here safetly so you know the watchdog triggered when you
  // restart.
  // In 2.0.0 and later, RESET_NO_WAIT prevents notifying the cloud of a pending reset

  uint16_t resets;
  EEPROM.get(EEPROMRESETCOUNT, resets);
  resets = resets + 1;
  EEPROM.put(EEPROMRESETCOUNT, resets);

  // Increase counts of fails tracking.
  switch (lastTryRead)
  {
  case TRYREADANGLE:
    countAngleFail = countAngleFail + 1;
    break;

  case TRYREADCOMPASS:
    countCompassFail = countCompassFail + 1;
    break;

  case TRYREADTEMPAIR:
    countTempAirFail = countTempAirFail + 1;
    break;

  case TRYREADTEMPWATER:
    countTempWaterFail = countTempWaterFail + 1;
    break;

  case TRYREADTEMPBOX:
    countTempBoxFail = countTempBoxFail + 1;
    break;

  case TRYREADPRESSURE:
    countPressureFail = countPressureFail + 1;
    break;
  }

  Log.info("Watchdog Triggered!");
  System.reset(RESET_NO_WAIT);
}
