/**
 * This sketch will use a BMP280 sensor to show temperature, pressure, and estimated altitude.
 * The BMP280 uses the I2C bus to communicate with the microcontroller.
 * The ESP8266/ESP-12E SCL pin is D1 (GPIO5), and SDA is D2 (GPIO4).
 * @copyright   Copyright © 2022 Adam Howell
 * @licence     The MIT License (MIT)
 */
#include "ESP8266WiFi.h"						// This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#include <Wire.h>						// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <Adafruit_BMP280.h>		// The Adafruit library for BMP280 sensor.
#include <Adafruit_Sensor.h>		// The Adafruit sensor library.
#include "privateInfo.h"			// I use this file to hide my network information from random people browsing my GitHub repo.
#include <ThingSpeak.h>

#define BMP280_I2C_ADDRESS 0x76	// Confirmed working I2C address as of 2021-08-21, for the GY-BM model https://smile.amazon.com/gp/product/B07S98QBTQ/.

/**
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 * If you do not want to use that file, you can set them here instead.
 */
//const char* wifiSsid = "your WiFi SSID";
//const char* wifiPassword = "your WiFi password";
//const char* mqttBroker = "your broker address";
//const int mqttPort = 1883;
const char* mqttTopic = "espWeather";
const char* sketchName = "ESP8266BMP280ThingSpeak";
const char* notes = "Lolin ESP8266 with BMP280";
const int wifiLED = 2;											// This LED for the Lolin devkit is on the ESP8266 module itself (next to the antenna).
const char* espControlTopic = "espControl";				// This is a topic we subscribe to, to get updates.  Updates may change publishDelay, seaLevelPressure, or request an immediate poll of the sensors.

char ipAddress[16];
char macAddress[18];
unsigned int loopCount = 0;									// This is a counter for how many loops have happened since power-on (or overflow).
unsigned long publishDelay = 60000;							// This is the loop delay in miliseconds.
unsigned long lastPublish = 0;
float seaLevelPressure = 1014.5;								// Adjust this to the sea level pressure (in hectopascals) for your local weather conditions.
// Provo Airport: https://forecast.weather.gov/data/obhistory/KPVU.html
// ThingSpeak variables
unsigned long myChannelNumber = 1;
//const char* ThingSpeakWriteKey = "yourWriteKey";

// Create class objects.
Adafruit_BMP280 bmp280;
WiFiClient espClient;
PubSubClient mqttClient( espClient );


void onReceiveCallback( char* topic, byte* payload, unsigned int length )
{
	Serial.print( "Message arrived [" );
	Serial.print( topic );
	Serial.print( "] " );
	for( int i = 0; i < length; i++ )
	{
		char receivedChar = ( char )payload[i];
		Serial.print( receivedChar );
		// Note that some boards (like this one) consider 'HIGH' to be off.
		if( receivedChar == '0' )
			digitalWrite( LED_BUILTIN, HIGH );		// Turn the LED off.
		if( receivedChar == '1' )
			digitalWrite( LED_BUILTIN, LOW );		// Turn the LED on.
	}
	Serial.println();
}


void wifiConnect( int maxAttempts )
{
	// Announce WiFi parameters.
	String logString = "WiFi connecting to SSID: ";
	logString += wifiSsid;
	Serial.println( logString );

	// Connect to the WiFi network.
	Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : " - Failed!" );
	WiFi.begin( wifiSsid, wifiPassword );

	int i = 1;
	/*
     WiFi.status() return values:
     0 : WL_IDLE_STATUS when WiFi is in process of changing between statuses
     1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
     3 : WL_CONNECTED after successful connection is established
     4 : WL_CONNECT_FAILED if wifiPassword is incorrect
     6 : WL_DISCONNECTED if module is not configured in station mode
  */
	// Loop until WiFi has connected.
	while( WiFi.status() != WL_CONNECTED && i < maxAttempts )
	{
		delay( 1000 );
		Serial.println( "Waiting for a connection..." );
		Serial.print( "WiFi status: " );
		Serial.println( WiFi.status() );
		Serial.print( i++ );
		Serial.println( " seconds" );
	}

	// Print that WiFi has connected.
	Serial.println( '\n' );
	Serial.println( "WiFi connection established!" );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );
	Serial.print( "IP address: " );
	snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
	Serial.println( ipAddress );

	digitalWrite( wifiLED, 0 );	// Turn the WiFi LED on.
} // End of wifiConnect() function.


void mqttConnect( int maxAttempts )
{
	int i = 0;
	// Loop until MQTT has connected.
	while( !mqttClient.connected() && i < maxAttempts )
	{
		Serial.print( "Attempting MQTT connection..." );
		// Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
		if( mqttClient.connect( macAddress ) )
		{
			Serial.println( "connected!" );
			mqttClient.subscribe( espControlTopic );		// Subscribe to the designated MQTT topic.
		}
		else
		{
			Serial.print( " failed, return code: " );
			Serial.print( mqttClient.state() );
			Serial.println( " try again in 2 seconds" );
			// Wait 5 seconds before retrying.
			delay( 5000 );
		}
		i++;
	}
} // End of mqttConnect() function.


void setup()
{
	// Start the Serial communication to send messages to the computer.
	Serial.begin( 115200 );
	while ( !Serial )
		delay( 100 );
	Serial.println( '\n' );
	Serial.print( sketchName );
	Serial.println( " is beginning its setup()." );
	Serial.println( __FILE__ );
	pinMode( wifiLED, OUTPUT );	// Initialize digital pin WiFi LED as an output.

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort );
	mqttClient.setCallback( onReceiveCallback );				 // Assign the onReceiveCallback() function to handle MQTT callbacks.

	Serial.println( "Attempting to connect to the BMP280..." );
	if( !bmp280.begin( BMP280_I2C_ADDRESS ) )
	{
		Serial.println( "Could not find a valid BMP280 sensor, Check wiring!" );
		while( 1 )
			;
	}
	Serial.println( "Connected to the BMP280!\n" );

	// Set the MAC address variable to its value.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );

	wifiConnect( 20 );
	ThingSpeak.begin( espClient );  // Initialize ThingSpeak
	// This ensures we take a reading in the first loop().
	lastPublish = 0;
} // End of setup() function.


void loop()
{
	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
	{
		// Reconnect to the MQTT broker.
		mqttConnect( 10 );
	}
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	// ToDo: Move all this into a function, and call it from setup() and from loop().
	unsigned long time = millis();
	// When time is less than publishDelay, subtracting publishDelay from time causes an overlow which results in a very large number.
	if( ( time > publishDelay ) && ( time - publishDelay ) > lastPublish )
	{
		loopCount++;
		// These next 3 lines act as a "heartbeat", to give local users a visual indication that the system is working.
		digitalWrite( wifiLED, 1 );	// Turn the WiFi LED off to alert the user that a reading is about to take place.
		delay( 1000 );						// Wait for one second.
		digitalWrite( wifiLED, 0 );	// Turn the WiFi LED on.

		Serial.println( sketchName );
		Serial.print( "Connected to broker at \"" );
		Serial.print( mqttBroker );
		Serial.print( ":" );
		Serial.print( mqttPort );
		Serial.println( "\"" );
		Serial.print( "Listening for control messages on topic \"" );
		Serial.print( espControlTopic );
		Serial.println( "\"." );

		// Get temperature, pressure and altitude from the Adafruit BMP280 library.
		// Temperature is always a floating point in Centigrade units. Pressure is a 32 bit integer in Pascal units.
		float temperature = bmp280.readTemperature();	 				// Get temperature.
		float pressure = bmp280.readPressure();			 				// Get pressure.
		float altitude_ = bmp280.readAltitude( seaLevelPressure );	// Get altitude based on the sea level pressure for your location. Note that "altitude" is a keyword, hence the underscore.

		// Prepare a String to hold the JSON.
		char mqttString[256];
		// Write the readings to the String in JSON format.
		snprintf( mqttString, 256, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"tempC\": %.2f,\n\t\"pressure\": %.1f,\n\t\"altitude\": %.1f,\n\t\"uptime\": %d,\n\t\"notes\": \"%s\"\n}", sketchName, macAddress, ipAddress, temperature, pressure, altitude_, loopCount, notes );
		// Publish the JSON to the MQTT broker.
		mqttClient.publish( mqttTopic, mqttString );
		// Print the JSON to the Serial port.
		Serial.println( mqttString );

		// Set the ThingSpeak fields.
		ThingSpeak.setField( 1, temperature );
		ThingSpeak.setField( 2, pressure );
		ThingSpeak.setField( 3, altitude_ );
		int x = ThingSpeak.writeFields( myChannelNumber, ThingSpeakWriteKey );
		if( x == 200 )
		{
	   	Serial.println( "Thingspeak update successful." );
	   }
	   else
		{
	   	Serial.println( "Problem updating channel. HTTP error code " + String( x ) );
	   }
	   lastPublish = millis();
	  	Serial.print( "Next publish in " );
		Serial.print( publishDelay / 1000 );
		Serial.println( " seconds.\n" );
	}
} // End of loop() function.
