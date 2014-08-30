/*****************************************************************
Phant_CC3000.ino
Post data to SparkFun's data stream server system (phant) using
an Arduino and the CC3000 Shield.
Jim Lindblom @ SparkFun Electronics
Original Creation Date: July 3, 2014

This sketch uses an Arduino Uno to POST sensor readings to 
SparkFun's data logging streams (http://data.sparkfun.com). A post
will be initiated whenever pin 3 is connected to ground.

Before uploading this sketch, there are a number of global vars
that need adjusting:
1. WiFi Stuff: Fill in your SSID, WiFi Passkey, and encryption
   setting using three variables available.
2. Phant Stuff: Fill in your data stream's public, private, and 
data keys before uploading!

This sketch requires that you install this library:
* SFE_CC3000: https://github.com/sparkfun/SFE_CC3000_Library

Hardware Hookup:
  * These components are connected to the Arduino's I/O pins:
    * D3 - Active-low momentary button (pulled high internally)
    * A0 - Photoresistor (which is combined with a 10k resistor
           to form a voltage divider output to the Arduino).
    * D5 - SPST switch to select either 5V or 0V to this pin.
  * A CC3000 Shield sitting comfortable on top of your Arduino.
 
Development environment specifics:
    IDE: Arduino 1.0.5
    Hardware Platform: RedBoard & CC3000 Shield (v10)

This code is beerware; if you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, please 
buy us a round!

Much of this code is largely based on Shawn Hymel's WebClient
example in the SFE_CC3000 library.

Distributed as-is; no warranty is given.
*****************************************************************/
// SPI and the pair of SFE_CC3000 include statements are required
// for using the CC300 shield as a client device.
#include <SPI.h>
#include <SFE_CC3000.h>
#include <SFE_CC3000_Client.h>
// Progmem allows us to store big strings in flash using F().
// We'll sacrifice some flash for extra DRAM space.
#include <Progmem.h>

////////////////////////////////////
// CC3000 Shield Pins & Variables //
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
unsigned int ap_security = WLAN_SEC_UNSEC; // Security of network
// ap_security can be any of: WLAN_SEC_UNSEC, WLAN_SEC_WEP, 
//  WLAN_SEC_WPA, or WLAN_SEC_WPA2
unsigned int timeout = 30000;             // Milliseconds
char server[] = "data.sparkfun.com";      // Remote host site

// Initialize the CC3000 objects (shield and client):
SFE_CC3000 wifi = SFE_CC3000(CC3000_INT, CC3000_EN, CC3000_CS);
SFE_CC3000_Client client = SFE_CC3000_Client(wifi);

/////////////////
// Phant Stuff //
/////////////////
const String publicKey = "MGGvD5py95U3KE3q8Qlq";
const String privateKey = "nzzox2krM2hVldVqDXZq";
const byte NUM_FIELDS = 14;
const String fieldNames[NUM_FIELDS] = {"P","RH","Tair","Tstream","conductivity","depth1","depth2","flow","light","pH","rain","t","winddir","windmag"};
String fieldData[NUM_FIELDS];

//////////////////////
// Input Pins, Misc //
//////////////////////
const int triggerPin = 3;
const int lightPin = A0;
const int switchPin = 5;
String name = "Anonymouse";
boolean newName = true;

void setup()
{
  Serial.begin(115200);
  
  // Setup Input Pins:
  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(lightPin, INPUT_PULLUP);
  
  // Set Up WiFi:
  setupWiFi();
  
  Serial.println(F("=========== Ready to Stream ==========="));
  Serial.println(F("Press the button (D3) to send an update"));
  Serial.println(F("Type your name (no spaces!), followed by '!' to update name"));
}

void loop()
{
  // If the trigger pin (3) goes low, send the data.
  if (!digitalRead(triggerPin))
  {
    // Gather data:
    fieldData[0] = String(analogRead(lightPin));
    fieldData[1] = String(digitalRead(switchPin));
    fieldData[2] = name;
    
    // Post data:
    Serial.println("Posting!");
    postData(); // the postData() function does all the work, 
                // check it out below.
    delay(1000);
  }
  
  // Check for a new name input:
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == '!')
    {
      newName = true;
      Serial.print("Your name is ");
      Serial.println(name);
    }
    else if (newName)
    {
      newName = false;
      name = "";
      name += c;
    }
    else
    {
      name += c;
    }
  }
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
