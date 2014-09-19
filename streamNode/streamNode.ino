#include <Wire.h> //I2C needed for sensors
#include "MPL3115A2.h" //Pressure sensor
#include "HTU21D.h" //Humidity sensor
#include <SoftwareSerial.h> //Needed for GPS
#include <TinyGPS++.h> //GPS parsing
#include <SPI.h>  //For CC3000
#include <SFE_CC3000.h>  //Also for CC3000
#include <SFE_CC3000_Client.h>  //As is this
#include <Progmem.h>  //Expaned memory
#include <OneWire.h>  //For DS18S20 thermometer

static const int RXPin = 5, TXPin = 4; //GPS is attached to pin 4(TX from GPS) and pin 5(RX into GPS)
SoftwareSerial ss(RXPin, TXPin);  //For GPS input
SoftwareSerial pHSerial(17, 16);  //For pH sensor output
TinyGPSPlus gps;  //GPS instance
MPL3115A2 myPressure; //Create an instance of the pressure sensor
HTU21D myHumidity; //Create an instance of the humidity sensor
// DS18S20 Temperature chip i/o
OneWire ds(18);  // on pin 18

//Hardware pin definitions, all of which are fixed to the weather shield
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 3;  //Windspeed interrupt
const byte RAIN = 2;  //Rain gauge interrupt
const byte STAT1 = 7;  //Status 1
const byte STAT2 = 8;  //Status 2
const byte GPS_PWRCTL = 6; //Pulling this pin low puts GPS to sleep but maintains RTC and RAM

// analog I/O pins
const byte REFERENCE_3V3 = A3;  //Analog reference voltage (3.3 VDC from board)
const byte LIGHT = A1;  //Light sensor input
const byte BATT = A2;  //Battery level input
const byte WDIR = A0;  //Wind direction potentiometer input
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

long lastWindCheck = 0;  //How long has it been since last we checked the wind?
volatile long lastWindIRQ = 0;  //Wind interrupt request counter
volatile byte windClicks = 0;  //Actual wind instance counter

//We need to keep track of the following variables:
//Wind speed/dir each update (no storage)
//Wind gust/dir over the day (no storage)
//Wind speed/dir, avg over 2 minutes (store 1 per second)
//Wind gust/dir over last 10 minutes (store 1 per minute)
//Rain over the past hour (store 1 per minute)
//Total rain over date (store one per day)

byte windspdavg[120]; //120 bytes to keep track of 2 minute average
int winddiravg[120]; //120 ints to keep track of 2 minute average
int windgust_10m[10]; //10 ints to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile int rainHour[60]; //60 inting numbers to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
int windspeedmph = 0; // [mph instantaneous wind speed]
int windgustmph = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
int windspdmph_avg2m = 0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
int windgustmph_10m = 0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
int humidity = 0; // [%]
int tempf = 0; // [temperature F]
int rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
volatile int dailyrainin = 0; // [rain inches so far today in local time]
//int baromin = 30.03;// [barom in] - It's hard to calculate baromin locally, do this in the agent
int pressure = 0;
//int dewptf; // [dewpoint F] - It's hard to calculate dewpoint locally, do this in the agent

int batt_lvl = 11.8; //[analog value from 0 to 1023]
int light_lvl = 455; //[analog value from 0 to 1023]

//Variables used for GPS
//int flat, flon; // 39.015024 -102.283608686
//unsigned long age;
//int year;
//byte month, day, hour, minute, second, hundredths;

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

    if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rainHour[minutes] += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }
}

#define FLOWSENSORPIN 2
// count how many pulses!
volatile uint16_t pulses = 0;
// track the state of the pulse pin
volatile uint8_t lastflowpinstate;
// you can try to keep time of how long it is between pulses
volatile uint32_t lastflowratetimer = 0;
// and use that to calculate a flow rate
volatile int flowrate;
// Interrupt is called once a millisecond, looks for any pulses from the sensor!
SIGNAL(TIMER0_COMPA_vect) {
  uint8_t x = digitalRead(FLOWSENSORPIN);
  
  if (x == lastflowpinstate) {
    lastflowratetimer++;
    return; // nothing changed!
  }
  
  if (x == HIGH) {
    //low to high transition!
    pulses++;
  }
  lastflowpinstate = x;
  flowrate = 1000.0;
  flowrate /= lastflowratetimer;  // in hertz
  lastflowratetimer = 0;
}
char ph_data[20];
////////////////////////////////////
// CC3000 Shield Pins & Variables //
////////////////////////////////////
// Don't change these unless you're using a breakout board.
#define CC3000_INT      21   // Needs to be an interrupt pin (D2/D3)
#define CC3000_EN       20   // Can be any digital pin
#define CC3000_CS       19  // Preferred is pin 10 on Uno
#define IP_ADDR_LEN     4   // Length of IP address in bytes

////////////////////
// WiFi Constants //
////////////////////
char ap_ssid[] = "RedRover";                // SSID of network
char ap_password[] = "";        // Password of network
unsigned int ap_security = 'WLAN_SEC_UNSEC'; // Security of network
// ap_security can be any of: WLAN_SEC_UNSEC, WLAN_SEC_WEP, 
//  WLAN_SEC_WPA, or WLAN_SEC_WPA2
unsigned int timeout = 3000;             // Milliseconds
char server[] = "data.sparkfun.com";      // Remote host site: 54.86.132.254=data.sparkfun.com

// Initialize the CC3000 objects (shield and client):
SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);
SFE_CC3000_Client client = SFE_CC3000_Client(wifi);

/////////////////
// Phant Stuff //
/////////////////
const String publicKey = "MGGvD5py95U3KE3q8Qlq";
const String privateKey = "nzzox2krM2hVldVqDXZq";
const byte NUM_FIELDS = 13;
const String fieldNames[NUM_FIELDS] = {"P","RH","Tair","Tstream","conductivity","depth1","depth2","flow","light","pH","rain","winddir","windmag"};
String fieldData[NUM_FIELDS];

void setup()
{
  Serial.begin(115200);
  ss.begin(9600); //Begin listening to GPS over software serial at 9600. This should be the default baud of the module.
  pHSerial.begin(38400);
  pHSerial.print("e\r");
  delay(50);
  pHSerial.print("e\r");
  delay(50);
  pinMode(STAT1, OUTPUT); //Status LED Blue
  pinMode(STAT2, OUTPUT); //Status LED Green
  
  pinMode(GPS_PWRCTL, OUTPUT);
  digitalWrite(GPS_PWRCTL, HIGH); //Pulling this pin low puts GPS to sleep but maintains RTC and RAM
  
  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor
  
  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

  //Configure the pressure sensor
  myPressure.begin(); // Get sensor online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

  //Configure the humidity sensor
  myHumidity.begin();

  seconds = 0;
  lastSecond = millis();

  // attach external interrupt pins to IRQ functions
  attachInterrupt(0, rainIRQ, FALLING);
  attachInterrupt(1, wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();

  Serial.println("Weather Shield online!");
  // Set Up WiFi:
  setupWiFi();
  pinMode(FLOWSENSORPIN, INPUT);
  digitalWrite(FLOWSENSORPIN, HIGH);
  lastflowpinstate = digitalRead(FLOWSENSORPIN);
  useInterrupt(true);
  Serial.println(F("=========== Ready to Stream ==========="));
}

void loop()
{
   byte j;
  byte present = 0;
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      Serial.print("No more addresses.\n");
      ds.reset_search();
      return;
  }

  Serial.print("R=");
  for( j = 0; j < 8; j++) {
    Serial.print(addr[j], HEX);
    Serial.print(" ");
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.print("CRC is not valid!\n");
      return;
  }

  if ( addr[0] == 0x10) {
      Serial.print("Device is a DS18S20 family device.\n");
  }
  else if ( addr[0] == 0x28) {
      Serial.print("Device is a DS18B20 family device.\n");
  }
  else {
      Serial.print("Device family is not recognized: 0x");
      Serial.println(addr[0],HEX);
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
  for ( j= 0; j < 9; j++) {           // we need 9 bytes
    data[j] = ds.read();
  }  
  if(millis() - lastSecond >= 1000)
  {
    digitalWrite(STAT1, HIGH); //Blink stat LED
    
    lastSecond += 1000;

    //Take a speed and direction reading every second for 2 minute average
    if(++seconds_2m > 119) seconds_2m = 0;

    //Calc the wind speed and direction every second for 120 second to get 2 minute average
    int currentSpeed = get_wind_speed();
    //int currentSpeed = random(5); //For testing
    int currentDirection = get_wind_direction();
    windspdavg[seconds_2m] = (int)currentSpeed;
    winddiravg[seconds_2m] = currentDirection;
    //if(seconds_2m % 10 == 0) displayArrays(); //For testing

    //Check to see if this is a gust for the minute
    if(currentSpeed > windgust_10m[minutes_10m])
    {
      windgust_10m[minutes_10m] = currentSpeed;
      windgustdirection_10m[minutes_10m] = currentDirection;
    }

    //Check to see if this is a gust for the day
    if(currentSpeed > windgustmph)
    {
      windgustmph = currentSpeed;
      windgustdir = currentDirection;
    }
    
    if(++seconds > 59)
    {
      seconds = 0;

      if(++minutes > 59) minutes = 0;
      if(++minutes_10m > 9) minutes_10m = 0;

      rainHour[minutes] = 0; //Zero out this minute's rainfall amount
      windgust_10m[minutes_10m] = 0; //Zero out this minute's gust
    }

    //Report all readings every second
    printWeather();

    digitalWrite(STAT1, LOW); //Turn off stat LED
  }
  int liters = pulses;
  liters /= 7.5;
  liters /= 60.0;
  smartdelay(800); //Wait 1 second, and gather GPS data
  setupWiFi();
  calcWeather();
  fieldData[0]=String(pressure);  // P
  fieldData[1]=String(humidity);  // RH
  fieldData[2]=String(tempf);    // Tair
  fieldData[3]="Tstream";        // Tstream
  digitalWrite(52, HIGH);
  fieldData[4]=String(analogRead(A4));  // Conductivity
  digitalWrite(52, LOW);
  digitalWrite(50, HIGH);
  fieldData[5]=String(analogRead(A5));  // Depth1
  digitalWrite(50, LOW);
  digitalWrite(48, HIGH);
  fieldData[6]=String(analogRead(A6));  // Depth2
  digitalWrite(48, LOW);
  fieldData[7]=String(liters);    // Flow
  fieldData[8]=String(light_lvl);  // Light
  pHSerial.print("R\r");
  fieldData[9]=String(pHSerial.readBytesUntil(13,ph_data,20));  // pH
  fieldData[10]=String(rainin);    // Rain
  fieldData[11]=String(analogRead(A8));  //WindDir
  fieldData[12]=String(analogRead(A9));  //WindMag
  // Post data:
  Serial.println("Posting!");
  postData(); // the postData() function does all the work, 
  for(int i=0; i<NUM_FIELDS; i++){
    Serial.print(fieldData[i]);
    if(i!=12){
      Serial.print(",");
    }
  }
  Serial.println();
  delay(50000);
}

void postData()
{
  
  // Make a TCP connection to remote host
  if ( !client.connect(server, 80) )
  {
    // Error: 4 - Could not make a TCP connection
    Serial.println(F("Error: 4"));
  }
  
  // Post the data! Request should look a little something like:
  // GET /input/publicKey?private_key=privateKey&light=1024&switch=0&time=5201 HTTP/1.1\n
  // Host: data.sparkfun.com\n
  // Connection: close\n
  // \n
  client.print("GET /input/");
  client.print(publicKey);
  client.print("?private_key=");
  client.print(privateKey);
  for (int i=0; i<NUM_FIELDS; i++)
  {
    client.print("&");
    client.print(fieldNames[i]);
    client.print("=");
    client.print(fieldData[i]);
  }
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(server);
  client.println("Connection: close");
  client.println();
  
  while (client.connected())
  {
    if ( client.available() )
    {
      char c = client.read();
      Serial.print(c);
    }      
  }
  Serial.println();
}

void setupWiFi()
{
  ConnectionInfo connection_info;
  int i;
  
  // Initialize CC3000 (configure SPI communications)
  if ( wifi.init() )
  {
    Serial.println(F("CC3000 Ready!"));
  }
  else
  {
    // Error: 0 - Something went wrong during CC3000 init!
    Serial.println(F("Error: 0"));
  }
  
  // Connect using DHCP
  Serial.print(F("Connecting to: "));
  Serial.println(ap_ssid);
  if(!wifi.connect(ap_ssid, ap_security, ap_password, timeout))
  {
    // Error: 1 - Could not connect to AP
    Serial.println(F("Error: 1"));
  }
  
  // Gather connection details and print IP address
  if ( !wifi.getConnectionInfo(connection_info) ) 
  {
    // Error: 2 - Could not obtain connection details
    Serial.println(F("Error: 2"));
  }
  else
  {
    Serial.print(F("My IP: "));
    for (i = 0; i < IP_ADDR_LEN; i++)
    {
      Serial.print(connection_info.ip_address[i]);
      if ( i < IP_ADDR_LEN - 1 )
      {
        Serial.print(".");
      }
    }
    Serial.println();
  }
}
//While we delay for a given amount of time, gather GPS data
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}


//Calculates each of the variables that wunderground is expecting
void calcWeather()
{
  //Calc winddir
  winddir = get_wind_direction();

  //Calc windspeed
  windspeedmph = get_wind_speed();

  //Calc windgustmph
  //Calc windgustdir
  //Report the largest windgust today
  windgustmph = 0;
  windgustdir = 0;

  //Calc windspdmph_avg2m
  int temp = 0;
  for(int i = 0 ; i < 120 ; i++)
    temp += windspdavg[i];
  temp /= 120.0;
  windspdmph_avg2m = temp;

  //Calc winddir_avg2m
  temp = 0; //Can't use winddir_avg2m because it's an int
  for(int i = 0 ; i < 120 ; i++)
    temp += winddiravg[i];
  temp /= 120;
  winddir_avg2m = temp;

  //Calc windgustmph_10m
  //Calc windgustdir_10m
  //Find the largest windgust in the last 10 minutes
  windgustmph_10m = 0;
  windgustdir_10m = 0;
  //Step through the 10 minutes  
  for(int i = 0; i < 10 ; i++)
  {
    if(windgust_10m[i] > windgustmph_10m)
    {
      windgustmph_10m = windgust_10m[i];
      windgustdir_10m = windgustdirection_10m[i];
    }
  }

  //Calc humidity
  humidity = myHumidity.readHumidity();
  //int temp_h = myHumidity.readTemperature();
  //Serial.print(" TempH:");
  //Serial.print(temp_h, 2);

  //Calc tempf from pressure sensor
  tempf = myPressure.readTempF();
  //Serial.print(" TempP:");
  //Serial.print(tempf, 2);

  //Total rainfall for the day is calculated within the interrupt
  //Calculate amount of rainfall for the last 60 minutes
  rainin = 0;  
  for(int i = 0 ; i < 60 ; i++)
    rainin += rainHour[i];

  //Calc pressure
  pressure = myPressure.readPressure();

  //Calc dewptf

  //Calc light level
  light_lvl = get_light_level();

  //Calc battery level
  batt_lvl = get_battery_level();
  
}

//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
int get_light_level()
{
  int operatingVoltage = analogRead(REFERENCE_3V3);

  int lightSensor = analogRead(LIGHT);
  
  operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V
  
  lightSensor = operatingVoltage * lightSensor;
  
  return(lightSensor);
}

//Returns the voltage of the raw pin based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
//Battery level is connected to the RAW pin on Arduino and is fed through two 5% resistors:
//3.9K on the high side (R1), and 1K on the low side (R2)
int get_battery_level()
{
  int operatingVoltage = analogRead(REFERENCE_3V3);

  int rawVoltage = analogRead(BATT);
  
  operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V
  
  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin
  
  rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage
  
  return(rawVoltage);
}

//Returns the instataneous wind speed
int get_wind_speed()
{
  int deltaTime = millis() - lastWindCheck; //750ms

  deltaTime /= 1000.0; //Covert to seconds

  int windSpeed = (int)windClicks / deltaTime; //3 / 0.750s = 4

  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();

  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

  /* Serial.println();
   Serial.print("Windspeed:");
   Serial.println(windSpeed);*/

  return(windSpeed);
}

//Read the wind direction sensor, return heading in degrees
int get_wind_direction() 
{
  unsigned int adc;

  adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
}


//Prints the various variables directly to the port
//I don't like the way this function is written but Arduino doesn't support ints under sprintf
void printWeather()
{
  calcWeather(); //Go calc all the various sensors

  Serial.println();
  Serial.print("$,winddir=");
  Serial.print(winddir);
  Serial.print(",windspeedmph=");
  Serial.print(windspeedmph, 1);
  /*Serial.print(",windgustmph=");
  Serial.print(windgustmph, 1);
  Serial.print(",windgustdir=");
  Serial.print(windgustdir);
  Serial.print(",windspdmph_avg2m=");
  Serial.print(windspdmph_avg2m, 1);
  Serial.print(",winddir_avg2m=");
  Serial.print(winddir_avg2m);
  Serial.print(",windgustmph_10m=");
  Serial.print(windgustmph_10m, 1);
  Serial.print(",windgustdir_10m=");
  Serial.print(windgustdir_10m);*/
  Serial.print(",humidity=");
  Serial.print(humidity, 1);
  Serial.print(",tempf=");
  Serial.print(tempf, 1);
  Serial.print(",rainin=");
  Serial.print(rainin, 2);
  Serial.print(",dailyrainin=");
  Serial.print(dailyrainin, 2);
  Serial.print(",pressure=");
  Serial.print(pressure, 2);
  Serial.print(",batt_lvl=");
  Serial.print(batt_lvl, 2);
  Serial.print(",light_lvl=");
  Serial.print(light_lvl, 2);

  Serial.print(",lat=");
  Serial.print(gps.location.lat(), 6);
  Serial.print(",lat=");
  Serial.print(gps.location.lng(), 6);
  Serial.print(",altitude=");
  Serial.print(gps.altitude.meters());
  Serial.print(",sats=");
  Serial.print(gps.satellites.value());

  char sz[32];
  Serial.print(",date=");
  sprintf(sz, "%02d/%02d/%02d", gps.date.month(), gps.date.day(), gps.date.year());
  Serial.print(sz);

  Serial.print(",time=");
  sprintf(sz, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  Serial.print(sz);

  Serial.print(",");
  Serial.println("#");

}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
  }
}
