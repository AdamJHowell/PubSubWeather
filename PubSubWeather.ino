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


// Define constants.
const char* wifiSsid = "Red";
const char* wifiPassword = "8012254722";
char clientAddress[16];
const char* mqttBroker = "192.168.55.200";
const char* mqttTopic = "ajhWeather";
const int mqttPort = 2112;
const int ESP8266_LED = 16;
const int ESP12_LED = 2;
const int BMP280_I2C_ADDRESS = 0x76;
const int LED_ON = 0; // The ESP8266 LED logic is backwards.
const int LED_OFF = 1;


// Create class objects.
Adafruit_BMP280 bmp280;
WiFiClient espClient;
PubSubClient mqttClient( espClient );


// Connect to WiFi (if needed) and MQTT.
String networkConnect()
{
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
	Serial.print( "IP address:\t" );
	Serial.println( WiFi.localIP() );

	// Loop until MQTT has connected.
	while( !mqttClient.connected() )
	{
		Serial.print( "Attempting MQTT connection..." );
		if( mqttClient.connect( "ESP8266 Client" ) ) // Attempt to networkConnect using the designated clientID.
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
	Serial.println( "WiFi and MQTT are connected!\n" );
	return WiFi.localIP().toString();
} // End of networkConnect() function.


void setup()
{
	Serial.begin( 115200 );
	delay( 10 );
	Serial.println( '\n' );

	mqttClient.setServer( mqttBroker, mqttPort ); // Set the MQTT client parameters.

	pinMode( ESP12_LED, OUTPUT );	  // Initialize ESP12_LED as an output.
	pinMode( ESP8266_LED, OUTPUT ); // Initialize ESP8266_LED as an output.

	WiFi.begin( wifiSsid, wifiPassword ); // Connect to the WiFi network.
	networkConnect();

	// Store client IP address into clientAddress.
	sprintf( clientAddress, "IP:%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
	// Publish the client IP address to the MQTT broker.
	mqttClient.publish( mqttTopic, clientAddress );

	Serial.println( "Attempting to networkConnect to the BMP280." );
	if( !bmp280.begin( BMP280_I2C_ADDRESS ) )
	{
		Serial.println( "Could not find a valid BMP280 sensor, Check wiring!" );
		while( 1 )
			;
	}
} // End of setup() function.


void loop()
{
	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
	{
		// Reconnect to WiFi and/or MQTT.
		networkConnect();
	}
	mqttClient.loop();

	// Get temperature, pressure and altitude from library.
	float temperature = bmp280.readTemperature();	// Get temperature.
	float pressure = bmp280.readPressure();			// Get pressure.
	float altitude = bmp280.readAltitude( 1016.8 ); // Get altitude (this should be adjusted to your local forecast).

	char* outLine;
	snprintf( outLine, 32, "Temperature: %s °C ( %s °F )\n", temperature, ( ( temperature * 9 / 5 ) + 32 ) );
	mqttClient.publish( mqttTopic, outLine );
	Serial.print( outLine );

	snprintf( outLine, 32, "Pressure: %s hPa ( %s inHg )\n", pressure, ( pressure * 0.000295 ) );
	mqttClient.publish( mqttTopic, outLine );
	Serial.print( outLine );

	snprintf( outLine, 32, "Altitude: %s m ( %s feet )\n\n", altitude, ( altitude * 3.281 ) );
	mqttClient.publish( mqttTopic, outLine );
	Serial.print( outLine );

	digitalWrite( ESP12_LED, LED_ON );	// Turn the LED on.
	delay( 5000 );								// Wait for two seconds.
	digitalWrite( ESP12_LED, LED_OFF ); // Turn the LED off.
	delay( 5000 );								// Wait for two seconds.
} // End of loop() function.
