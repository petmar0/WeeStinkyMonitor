#include <SoftwareSerial.h> //Needed for GPS
#include <SPI.h>  //For CC3000
#include <SFE_CC3000.h>  //Also for CC3000
#include <SFE_CC3000_Client.h>  //As is this
#include <Progmem.h>  //Expaned memory
#include <OneWire.h>  //For DS18S20 thermometer

// DS18S20 Temperature chip i/o
OneWire ds(18);  // on pin 18

SoftwareSerial pHSerial(17, 16);

#define FLOWSENSORPIN 20
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
/*
Hardware Connections:
 
 Uno Pin    CC3000 Board    Function
 
 +5V        VCC or +5V      5V
 GND        GND             GND
 2          INT             Interrupt
 7          EN              WiFi Enable
 10         CS              SPI Chip Select
 51         MOSI            SPI MOSI
 50         MISO            SPI MISO
 52         SCK             SPI Clock
*/
////////////////////////////////////
// Don't change these unless you're using a breakout board.
#define CC3000_INT      2   // Needs to be an interrupt pin (D2/D3)
#define CC3000_EN       7   // Can be any digital pin
#define CC3000_CS       10  // Preferred is pin 10 on Uno
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
const byte NUM_FIELDS = 6;
const String fieldNames[NUM_FIELDS] = {"Tstream","conductivity","depth","ph","q","turbidity"};    //Tstream conductivity depth ph q turbidity
String fieldData[NUM_FIELDS];

void setup()
{
  Serial.begin(115200);
  pHSerial.begin(38400);
  pHSerial.print("e\r");
  delay(50);
  pHSerial.print("e\r");
  delay(50);
  // Set Up WiFi:
  setupWiFi();
  pinMode(FLOWSENSORPIN, INPUT);
  digitalWrite(FLOWSENSORPIN, HIGH);
  lastflowpinstate = digitalRead(FLOWSENSORPIN);
  useInterrupt(true);
  digitalWrite(22, HIGH);
  digitalWrite(24, HIGH);
  digitalWrite(26, HIGH);
  digitalWrite(28, HIGH);
  digitalWrite(30, HIGH);
  Serial.println(F("=========== Ready to Stream ==========="));
}

void loop()
{
  byte j;
  byte present = 0;
  byte data[12];
  byte addr[8];
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
  pHSerial.print("R\r");
  int pH=pHSerial.readBytesUntil(13,ph_data,20);
  int liters = pulses;
  liters /= 7.5;
  liters /= 60.0;
  setupWiFi();
  //Tstream conductivity depth ph q turbidity
  fieldData[0]=String(int(data));        // Tstream
  fieldData[1]=String(analogRead(A0));  // Conductivity
  fieldData[2]=String(analogRead(A1));  // Depth
  fieldData[3]=String(pH);  // pH
  fieldData[4]=String(analogRead(A2));  // Turbidity
  fieldData[5]=String(liters);    // Flow
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
