/**
 * This sketch will use a BMP280 sensor to show temperature, pressure, and estimated altitude.
 * The BMP280 uses the I2C bus to communicate with the microcontroller.
 * The ESP-12E SCL pin is D1 (GPIO5), and SDA is D2 (GPIO4).
 */
#include <Adafruit_BMP280.h>		// The Adafruit library for BMP280 sensor.
#include <Adafruit_Sensor.h>		// The Adafruit sensor library.
#include <ESP8266WiFi.h>			// This header is added to the IDE libraries after the ESP8266 is added in board manager.
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <Wire.h>						// This is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include "networkVariables.h"		// I use this file to hide my network information from random people browsing my GitHub repo.
#include <ThingSpeak.h>

#define BMP280_I2C_ADDRESS 0x76	// Confirmed working I2C address as of 2021-08-21, for the GY-BM model https://smile.amazon.com/gp/product/B07S98QBTQ/.

/**
 * Declare network variables.
 * Adjust the commented-out variables to match your network and broker settings.
 * The commented-out variables are stored in "networkVariables.h", which I do not upload to GitHub.
 */
//const char* wifiSsid = "yourSSID";
//const char* wifiPassword = "yourPassword";
//const char* mqttBroker = "yourBrokerAddress";
//const int mqttPort = 1883;
const char* mqttTopic = "ajhWeather";
char clientAddress[16];
char macAddress[18];
const float seaLevelPressure = 1009.8;		// Adjust this to the sea level pressure (in hectopascals) for your local weather conditions.
// Provo Airport: https://forecast.weather.gov/data/obhistory/KPVU.html
const int wifiLED = 2;		// This LED is on the ESP8266 module itself (next to the antenna).
const int mqttLED = 16;		// This LED is on the NodeMCU board, near the microUSB port.
// ThingSpeak variables
unsigned long myChannelNumber = 1;
//const char* myWriteAPIKey = "yourWriteKey";

// Create class objects.
Adafruit_BMP280 bmp280;
WiFiClient espClient;
PubSubClient mqttClient( espClient );


/**
 * mqttConnect() will attempt to (re)connect the MQTT client.
 */
void mqttConnect()
{
	// Loop until MQTT has connected.
	while( !mqttClient.connected() )
	{
		digitalWrite( mqttLED, 1 );						// Turn the MQTT LED off.
		Serial.print( "Attempting MQTT connection..." );
		if( mqttClient.connect( "ESP8266 Client" ) ) // Attempt to mqttConnect using the designated clientID.
		{
			Serial.println( "connected!" );
			digitalWrite( mqttLED, 0 );					// Turn the MQTT LED on.
		}
		else
		{
			Serial.print( " failed, return code: " );
			Serial.print( mqttClient.state() );
			Serial.println( " try again in 2 seconds" );
			// Wait 2 seconds before retrying.
			delay( 2000 );
		}
	}
	Serial.println( "MQTT is connected!\n" );
} // End of mqttConnect() function.


/**
 * The setup() function runs once when the device is booted, and then loop() takes over.
 */
void setup()
{
	// Start the Serial communication to send messages to the computer.
	Serial.begin( 115200 );
	delay( 10 );
	Serial.println( '\n' );
	pinMode( wifiLED, OUTPUT );	// Initialize digital pin WiFi LED as an output.
	pinMode( mqttLED, OUTPUT );	// Initialize digital pin MQTT LED as an output.


	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort );

	// Announce WiFi parameters.
	String logString = "WiFi connecting to SSID: ";
	logString += wifiSsid;
	Serial.println( logString );

	// Connect to the WiFi network.
	Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : "Failed!" );
	WiFi.begin( wifiSsid, wifiPassword );
	ThingSpeak.begin( espClient );  // Initialize ThingSpeak

	int i = 0;
	/*
     WiFi.status() return values:
     0 : WL_IDLE_STATUS when WiFi is in process of changing between statuses
     1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
     3 : WL_CONNECTED after successful connection is established
     4 : WL_CONNECT_FAILED if wifiPassword is incorrect
     6 : WL_DISCONNECTED if module is not configured in station mode
  */
	// Loop until WiFi has connected.
	while( WiFi.status() != WL_CONNECTED )
	{
		digitalWrite( wifiLED, 1 );	// Turn the WiFi LED off.
		delay( 1000 );
		Serial.println( "Waiting for a connection..." );
		Serial.print( "WiFi status: " );
		Serial.println( WiFi.status() );
		Serial.print( ++i );
		Serial.println( " seconds" );
	}

	// Print that WiFi has connected.
	Serial.println( '\n' );
	Serial.println( "WiFi connection established!" );
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );
	Serial.print( "IP address: " );
	snprintf( clientAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
	Serial.println( clientAddress );
	digitalWrite( wifiLED, 0 );	// Turn the WiFi LED on.

	Serial.println( "Attempting to connect to the BMP280." );
	if( !bmp280.begin( BMP280_I2C_ADDRESS ) )
	{
		Serial.println( "Could not find a valid BMP280 sensor, Check wiring!" );
		while( 1 )
			;
	}
	Serial.println( "Connected to the BMP280!\n" );
} // End of setup() function.


/**
 * The loop() function begins after setup(), and repeats as long as the unit is powered.
 */
void loop()
{
	// These next 3 lines act as a "heartbeat", to give local users an indication that the system is working.
	digitalWrite( wifiLED, 1 );	// Turn the WiFi LED off to alert the user that a reading is about to take place.
	delay( 1000 );						// Wait for one second.
	digitalWrite( wifiLED, 0 );	// Turn the WiFi LED on.

	Serial.println();
	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
	{
		// Reconnect to the MQTT broker.
		mqttConnect();
	}
	mqttClient.loop();

	// Get temperature, pressure and altitude from the Adafruit BMP280 library.
	// Temperature is always a floating point in Centigrade units. Pressure is a 32 bit integer in Pascal units.
	float temperature = bmp280.readTemperature();	 				// Get temperature.
	float pressure = bmp280.readPressure();			 				// Get pressure.
	float altitude_ = bmp280.readAltitude( seaLevelPressure );	// Get altitude based on the sea level pressure for your location.

	// Prepare a String to hold the JSON.
	char mqttString[256];
	// Write the readings to the String in JSON format.
	snprintf( mqttString, 256, "{\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"tempC\": %.1f,\n\t\"presP\": %.1f,\n\t\"altM\": %.1f\n}", macAddress, clientAddress, temperature, pressure, altitude_ );
	// Publish the JSON to the MQTT broker.
	mqttClient.publish( mqttTopic, mqttString );
	// Print the JSON to the Serial port.
	Serial.println( mqttString );

	// Set the ThingSpeak fields.
	ThingSpeak.setField(1, temperature);
	ThingSpeak.setField(2, pressure);
	ThingSpeak.setField(3, altitude_);
	int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
	if(x == 200)
	{
   	Serial.println("Thingspeak update successful.");
   }
   else
	{
   	Serial.println("Problem updating channel. HTTP error code " + String(x));
   }

	Serial.println( "Pausing for 60 seconds..." );
	delay( 60000 );	// Wait for 60 seconds.
} // End of loop() function.
