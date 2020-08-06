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
const char* mqttBroker = "192.168.55.200";
const char* mqttTopic = "ajhWeather";
const int mqttPort = 2112;


// Create class objects.
Adafruit_BMP280 bmp280;
WiFiClient espClient;
PubSubClient mqttClient( espClient );


// Connect to the MQTT broker.
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


void setup()
{
	Serial.begin( 115200 );
	delay( 10 );
	Serial.println( '\n' );

	mqttClient.setServer( mqttBroker, mqttPort ); // Set the MQTT client parameters.

	WiFi.begin( wifiSsid, wifiPassword ); // Connect to the WiFi network.
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
	Serial.print( "MAC address: " );
	Serial.println( WiFi.macAddress() );
	Serial.print( "IP address: " );
	Serial.println( WiFi.localIP() );

	// Connect to the MQTT broker.
	// mqttConnect();
	if( !bmp280.begin( BMP280_I2C_ADDRESS ) )
	{
		Serial.println( "Could not find a valid BMP280 sensor, Check wiring!" );
		while( 1 )
			;
	}

	// Store client IP address into clientAddress.
	sprintf( clientAddress, "IP: %d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );

	Serial.println( "Attempting to connect to the BMP280." );
	if( !bmp280.begin( BMP280_I2C_ADDRESS ) )
	{
		Serial.println( "Could not find a valid BMP280 sensor, Check wiring!" );
		while( 1 )
			;
	}
	Serial.println( "Connected to the BMP280!\n" );
} // End of setup() function.


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

	// Publish the client IP address to the MQTT broker.
	mqttClient.publish( mqttTopic, clientAddress );

	// Get temperature, pressure and altitude from the Adafruit BMP280 library.
	float temperature = bmp280.readTemperature();	 // Get temperature.
	float pressure = bmp280.readPressure();			 // Get pressure.
	float altitude_ = bmp280.readAltitude( 1016.8 ); // Get altitude (this should be adjusted to your local forecast).

	char mqttString[256];
	snprintf( mqttString, 256, "{\"ip\":\"%s\",\"temp\":%.1f,\"pres\":%.1f,\"alt\":%.1f}", clientAddress, temperature, pressure, altitude_ );
	mqttClient.publish( mqttTopic, mqttString );

	// Shamelessly stolen from: https://stackoverflow.com/a/62803431/2803488
	// char displayTemperature[64] = "Temperature: ";
	// char displayPressure[64] = "Pressure: ";
	// char displayAltitude[64] = "Altitude: ";

	// snprintf( strchr( displayTemperature, '\0' ), sizeof( displayTemperature ), "%.1f °C", temperature );
	// snprintf( strchr( displayPressure, '\0' ), sizeof( displayPressure ), "%.1f hPa", pressure );
	// snprintf( strchr( displayAltitude, '\0' ), sizeof( displayAltitude ), "%.1f m", altitude_ );

	// Serial.println( displayTemperature );
	// mqttClient.publish( mqttTopic, displayTemperature );
	// Serial.println( displayPressure );
	// mqttClient.publish( mqttTopic, displayPressure );
	// Serial.println( displayAltitude );
	// mqttClient.publish( mqttTopic, displayAltitude );

	delay( 60000 ); // Wait for 60 seconds.
} // End of loop() function.
