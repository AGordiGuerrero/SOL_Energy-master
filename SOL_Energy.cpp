
/*
SOL_Energy.cpp - Library for Energyer variable monitoring
Created by A.Gordillo-Guerrero, Nov, 2022.
Released into the public domain.

Include:

- Temperature and Humidity control using one DHT sensor
- Lecture of four analog channels using Adafruit_ADS1015 and I2C protocol
- Lecture of battery voltaje using one ADC channel.
- Reading of a digital or capacitive button using one input pin.


*/


//#include "Arduino.h"
#include "SOL_Energy.h"
//#include "Wire.h" // for IMU 6050 control
//#include "I2Cdev.h"
//#include "Adafruit_ADS1X15.h"
//#include "DHT.h"

// Uncomment for debug via serial port
#define SerialDEBUG

/// IMPORTANT HARDWARE DEFINITIONS
#if defined(ESP8266)
#pragma message "Compiling for ESP8266..."

#elif defined(ESP32)
#pragma message "Compiling for ESP32..."

#elif defined(AVR)
#pragma message "Compiling for AVR structure..."
#else

#error "This ain't a ESP8266 or ESP32, dumbo!"
#endif

#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

//Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */


//DHT temperature and humidity sensor
//DHT dht(_DHTPIN, DHTTYPE);


//////////////////////////////////
// Constructor //////////////////
// Includes definition of dht and OneWire objects, separated by commas
energyPlant::energyPlant(int DHTPin, int buttonPin, int relayPin, int oneWirePin):
  dht(DHTPin, DHTTYPE),oneWire(oneWirePin),sensors(&oneWire){

  // Pin mode definition
  // Relay board outputs
  pinMode(relayPin , OUTPUT);

  //DHT *dht = NULL;
  //dht = new DHT(DHTPin, DHTTYPE);

  // private variable asignation
  _DHTPin=DHTPin;
  _buttonPin=buttonPin;
  _relayPin=relayPin;
  _oneWirePin=oneWirePin;

}



///////////////////////////////////
// Initializator //////////////////
//   To assure the correct startup of modules
///////////////////////////////////

void energyPlant::init()
{

  // Initializing serial communication
  Serial.begin(9600); //Start Serial monitor in 9600

  // Turn OFF all relays at start
  turnRelayOFF();


  // Checking for strange values of sensors at startup
  Serial.println("Checking sensor values...");

  Serial.println("Checking temperature and humidity from DHT...");

  // DHT sensor initiallization
  dht.begin();
  delay(1000);
  // H_DHT and T_DHT check...
  readDHT();
  checkH(); // setting H_DHT flags
  checkT(); // setting typedef int MyCustomType; flags
  if(flagT){ Serial.println("      T_DHT is in measurement range."); }
      else {Serial.println("     Take care: temperature is not in measurement range." ); }
  if(flagH){ Serial.println("      H_DHT is in measurement range."); }
      else {Serial.println("     Take care: humidity is not in measurement range." ); }

  // OneWire sensor initiallization
  Serial.println("Checking temperature from One Wire Dallas sensors");
  sensors.begin();
  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

   // report parasite power requirements
   Serial.print("Parasite power is: ");
   if (sensors.isParasitePowerMode()) Serial.println("ON");
   else Serial.println("OFF");

   if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");

   //Serial.print("Device 0 Address: ");
   //printAddress(insideThermometer);
   //Serial.println();

   // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
   sensors.setResolution(insideThermometer, 9);

   Serial.print("Device 0 Resolution: ");
   Serial.print(sensors.getResolution(insideThermometer), DEC);
   Serial.println();



/*
  // Initializing ADS1115
  /if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115.");
    delay(3000);
  }
*/
/*
// Battery check...
  Serial.println("Checking battery...");
  delay(1000);
  readVbat_I2C();
  checkVbat(); // setting Battery flags
  if(flagVbat){ Serial.println("      Battery is in measurement range."); }
      else {Serial.println("     Take care: Battery is not in measurement range." ); }

*/

/*
// Weight sensors
Serial.println("Measuring heights...");
delay(1000);
readWeight_Nsamples_discard(60);
Serial.println("Checking weight...");
checkWeights();
if((weight<WeightLowerThresh)||(weight>WeightUpperThresh)){ Serial.println("Take care. Strange weight."); ; Serial.print("weight= "); Serial.print(weight);}
else {Serial.println("Weight ok!" ); }
*/

Serial.println();

}



void energyPlant::getStatus()
{

	Serial.println();
	Serial.println("  Composter's physical variable:");
  Serial.print("    Temperature (ºC): ");   Serial.print(T_DHT);
  if(flagT){ Serial.println("      T_DHT is in measurement range."); }
      else {Serial.println("     Take care: temperature is not in measurement range." ); }
  Serial.print("   Humidity (%): ");   Serial.print(H_DHT);
  if(flagH){ Serial.println("      H_DHT is in measurement range."); }
      else {Serial.println("     Take care: humidity is not in measurement range." ); }
/*
	Serial.println("    Weight (g): ");   Serial.print(weight);
  if(flagWeight){ Serial.println("      Weigth is in measurement range."); }
      else {Serial.println("     Take care: weight is not in measurement range." ); }

  Serial.print("    Battery Voltage (V): "); Serial.println(Vbat);
  if(flagVbat){ Serial.println("    Voltage battery is in action range."); }
      else {Serial.println("     Take care: Voltage battery is not in action range." ); }

*/

	Serial.println();

}


void energyPlant::readandcheckAll()
{
    readDHT();
    checkH();
    checkT();
  //  readVbat_I2C();
  //  readWeight_I2C_onesample();
  //  checkVbat();
  //  checkWeight();
//    readHeights_Nsamples_discard(Nsamples);
//    checkHeights();

}



void energyPlant::readDHT()
{

  H_DHT = dht.readHumidity();
  // Read temperature as Celsius (the default)
  T_DHT = dht.readTemperature();

  if (isnan(H_DHT) || isnan(T_DHT)) {
     Serial.println(F("Failed to read from DHT sensor!"));
     return;
   }

#ifdef SerialDEBUG
  Serial.print("Temperature (Celsius): ");
  Serial.println(T_DHT);
  Serial.print("Humidity (%): ");
  Serial.println(H_DHT);
#endif

}

void energyPlant::readOneWire_onesensor()
{

  // method 2 - faster
  T_OneWire1 = sensors.getTempC(insideThermometer);
  if(T_OneWire1 == DEVICE_DISCONNECTED_C)
  {
    Serial.println("Error: Could not read temperature data from OneWire sensors");
    return;
  }


#ifdef SerialDEBUG
  Serial.print("Temperature One Wire 1 (Celsius): ");
  Serial.println(T_OneWire1);
#endif

}





// Read analog value from battery voltage divider using ADS1115 ADC converter ( port Ain3)
void energyPlant::readVbat_I2C()
{

int16_t adc3;
  // Reading via ADS1115
  //Vbat = ads.readADC_SingleEnded(3);

    // cuadratic fitting
    // f(x)=5.58E-06*x*x - 0.00664*x +6.685
    // f(x)=4.02E-06*x*x - 0.00277*x +4.522
    //    Vbat = round(4.02E-06*Vbat*Vbat - 0.00277*Vbat +4.522);
adc3 = ads.readADC_SingleEnded(3);

Vbat = ads.computeVolts(adc3);

//Vbat = round(Vbat/10); // subtract 1.3V more

        //    Vbat = round(4.02E-06*Vbat*Vbat - 0.00277*Vbat +4.522);
//            Vbat = round(4.02E-06*Vbat*Vbat - 0.00277*Vbat + 3.222); // subtract 1.3V more

//#ifdef SerialDEBUG
  Serial.print("Battery voltaje (V): ");
  Serial.println(Vbat);
//#endif

}


// Reading via ADS1115
void energyPlant::readWeight_I2C_onesample()
{
  weight = ads.readADC_SingleEnded(0);
    // linear fitting using "MedidasPresion_20210314.ods"
    // f(x)= 0.06159*x -23.4666
  // PComp = round(0.06159*PComp -23.4666);

#ifdef SerialDEBUG
   	Serial.print("Weight one sample: ");     	Serial.println(weight);
#endif

}


// Reading via ADS1115
void energyPlant::readWeight_I2C_Nsamples(int Nsamples)
{

	int8_t   i;
  float averageW=0; //sample average
  int tmp; //temporal

  // Reading compressor pressure
  //read until Nsamples are obtained, or readcount_max trial measures have been performed
  for (i=0; i < Nsamples; i++){
    		tmp = ads.readADC_SingleEnded(0);

      // linear fitting using "MedidasPresion_20210314.ods"
    		// f(x)= 0.06159*x -23.4666
        //averageP += round(0.06159*tmp -23.4666);
        averageW += round(tmp); // direct voltage measurement
	}
  weight = round(averageW/Nsamples);

#ifdef SerialDEBUG
	    Serial.print("Average Weight: "); 	Serial.println(weight);
#endif

}



/// IMPORTANT HARDWARE DEFINITIONS
#if defined(ESP32)
#pragma message "Compiling for ESP32..."

void energyPlant::readButton()
{
  // Reading variables from th capacitive button
  int touchValue = touchRead(_buttonPin);

  Serial.print("TouchValue: ");
  Serial.println(touchValue);

  // check if the touchValue is below the threshold
  if(touchValue < CapThresh){

    if (flagDebounce)
      analogbuttonpressed();

    flagDebounce=1; // flag 1 when the capacitive touch button is bellow threshold

  }
  else{flagDebounce=0;} // to avoid undessired events

}

#else // if not for ESP32 it is a simple button
void energyPlant::readButton()
{
  // Reading variables from th capacitive button
  int buttonValue = digitalRead(_buttonPin);

  Serial.print("ButtonhValue: ");
  Serial.println(buttonValue);

  // check if the button Value is LOW two consecutive times
  if(buttonValue == LOW ){

    if (flagDebounce)
      analogbuttonpressed();

    flagDebounce=1; // flag 1 when the capacitive touch button is bellow threshold

  }
  else{flagDebounce=0;} // to avoid undessired events

}

#endif


void energyPlant::analogbuttonpressed() {

  Serial.println("Analog button pressed...");
  delay(1000); // for debug

/*
  uint16_t reading = analogRead(_switchPin);
  // print out the values you read:
  //sprintf(buff, "%d  \n", reading);
  //Serial.println(buff);

  // make correspondence between read value and switch position
  // it is composed of a simple voltage divider
  if(reading<800){
    Serial.println("Changing to Mode AUTO");
    mode=0;
    letsControlPressure=0; // we do not control height
    letsControlAngle=1; // we do control angle
    NTrialsFront=0; // Put to zero the number of control trials
    NTrialsRear=0; // Put to zero the number of control trials
    flagDeflated=0; // to start deflating
//    texMode0.setText("AUTOLEVEL MODE");
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Auto Mode ON");
#endif
// in auto mode we choose the parameters of the Parking mode
    desHFront=desHParkFront;
    desHRear=desHParkRear;
    desPFront=desPParkFront;
    desPRear=desPParkRear;
  }
  else if(reading<1800){
    Serial.println("Changing to Mode PARK");
    mode=1;
    letsControlPressure=1; // we have to control height
    letsControlAngle=0; // we do not control angle
    flagCorrectRear=0;
    flagCorrectFront=0;
    NTrialsFront=0; // Put to zero the number of control trials
    NTrialsRear=0; // Put to zero the number of control trials
//    texMode0.setText("PARK MODE");
    desHFront=desHParkFront;
    desHRear=desHParkRear;
    desPFront=desPParkFront;
    desPRear=desPParkRear;
  }
  else if(reading<3000){
      Serial.println("Changing to Mode ROAD");
    mode=2;
    letsControlPressure=1; // we have to control height
    letsControlAngle=0; // we do not control angle
    flagCorrectRear=0;
    flagCorrectFront=0;
    NTrialsFront=0; // Put to zero the number of control trials
    NTrialsRear=0; // Put to zero the number of control trials
   //   texMode0.setText("ROAD MODE");
    desHFront=desHRoadFront;
    desHRear=desHRoadRear;
    desPFront=desPRoadFront;
    desPRear=desPRoadRear;

  }
  else {
    Serial.println("Changing to Mode TRAIL");
    mode=3;
    letsControlPressure=1; // we have to control height
    letsControlAngle=0; // we do not control angle
    flagCorrectRear=0;
    flagCorrectFront=0;
    NTrialsFront=0; // Put to zero the number of control trials
    NTrialsRear=0; // Put to zero the number of control trials
  //    texMode0.setText("TRAIL MODE");
    desHFront=desHTrailFront;
    desHRear=desHTrailRear;
    desPFront=desPTrailFront;
    desPRear=desPTrailRear;
  }

*/

}



void energyPlant::checkVbat()
{

  if( (Vbat<VbatLowerThresh) || (Vbat>VbatUpperThresh) ){ flagVbat=0; }
  else { flagVbat=1; }

}

void energyPlant::checkH()
{

  if( (H_DHT<H_LowerThresh) || (H_DHT>H_UpperThresh) || isnan(H_DHT) ){ flagH=0; }
  else { flagH=1; }

}
void energyPlant::checkT()
{

  if( (T_DHT<T_LowerThresh) || (T_DHT>T_UpperThresh) || isnan(T_DHT) ){ flagT=0; }
  else { flagT=1; }

}


void energyPlant::checkWeight()
{

  if( (weight<WeightLowerThresh) || (weight>WeightUpperThresh) || isnan(weight) )
  { flagWeight=0; }
  else { flagWeight=1; }

}



void energyPlant::turnRelayOFF()
{

   // Turn OFF all relays
  digitalWrite(_relayPin, LOW);

}
