/**
 * This sketch will use a BMP280 sensor to show temperature, pressure, and estimated altitude.
 * The BMP280 uses the I2C bus to communicate with the microcontroller.
 * The ESP-12E SCL pin is D1 (GPIO5), and SDA is D2 (GPIO4).
 */
#include <Adafruit_BMP280.h> // Include Adafruit library for BMP280 sensor
#include <Adafruit_Sensor.h> // Include Adafruit sensor library
#include <ESP8266WiFi.h>	  // Network Client for the WiFi chipset.
#include <PubSubClient.h>	  // PubSub is the MQTT API.
#include <Wire.h>				  // Include Wire library, required for I2C devices

#define BMP280_I2C_ADDRESS 0x76

// Define constants.
const char* wifiSsid = "Red";
const char* wifiPassword = "8012254722";
char clientAddress[16];
char macAddress[18];
const char* mqttBroker = "192.168.55.200";
const char* mqttTopic = "ajhWeather";
const int mqttPort = 2112;


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
		Serial.print( "Attempting MQTT connection..." );
		if( mqttClient.connect( "ESP8266 Client" ) ) // Attempt to mqttConnect using the designated clientID.
		{
			Serial.println( "connected" );
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
	pinMode( 2, OUTPUT );	// Initialize digital pin LED_BUILTIN as an output.
	digitalWrite( 2, LOW ); // Turn the LED on.

	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort ); // Set the MQTT client parameters.

	// Connect to the WiFi network.
	Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : "Failed!" );
	WiFi.begin( wifiSsid, wifiPassword );
	Serial.print( "WiFi connecting to " );
	Serial.println( wifiSsid );

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
	Serial.println();
	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
	{
		// Reconnect to the MQTT broker.
		mqttConnect();
	}
	mqttClient.loop();

	// Get temperature, pressure and altitude from the Adafruit BMP280 library.
	float temperature = bmp280.readTemperature();	 // Get temperature.
	float pressure = bmp280.readPressure();			 // Get pressure.
	float altitude_ = bmp280.readAltitude( 1016.8 ); // Get altitude (this should be adjusted to your local forecast).

	// Format the readings into JSON.
	char mqttString[256];
	// Publish the readings to the MQTT broker in JSON format.
	snprintf( mqttString, 256, "{\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"temp\": %.1f,\n\t\"pres\": %.1f,\n\t\"alt\": %.1f\n}", macAddress, clientAddress, temperature, pressure, altitude_ );
	// Publish the JSON to the MQTT broker.
	mqttClient.publish( mqttTopic, mqttString );
	// Print the JSON to the Serial port.
	Serial.println( mqttString );

	delay( 60000 ); // Wait for 60 seconds.
} // End of loop() function.
