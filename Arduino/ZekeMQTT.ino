/***************************************************
  Adafruit MQTT Library Ethernet Example

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Alec Moore
  Derived from the code written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>

/*************************************************************/ 
/************************* Command *****************************/ 
/*************************************************************/ 

/************************* Relay Setup *****************************/ 
#define commandPinRelay1	5
#define commandPinRelay2	6
#define commandPinRelay3	7
#define commandPinRelay4	8


/*************************************************************/ 
/************************* State *****************************/ 
/*************************************************************/ 

/************************* LDR Setup *****************************/ 
#define statePinLDR			A0
/************************* DHT22 Temperature sensor Setup *****************************/ 
#include <DHT.h>
DHT dht;
/************************* PIR Setup *****************************/ 
#define statePinPIR			3


/*****************************************************************************/ 
/************************* Ethernet Client Setup *****************************/
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

//Uncomment the following, and set to a valid ip if you don't have dhcp available.
//IPAddress iotIP (192, 168, 0, 42);
//Uncomment the following, and set to your preference if you don't have automatic dns.
//IPAddress dnsIP (8, 8, 8, 8);
//If you uncommented either of the above lines, make sure to change "Ethernet.begin(mac)" to "Ethernet.begin(mac, iotIP)" or "Ethernet.begin(mac, iotIP, dnsIP)"


/*****************************************************************************/ 
/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "tierney.homeip.net"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "Arduino_Theatre"
#define AIO_KEY         ""
#define AIO_GUID		"ZekesGUID"


/************ Global State (you don't need to change this!) ******************/

//Set up the ethernet client
EthernetClient client;

// Store the MQTT server, client ID, username, and password in flash memory.
// This is required for using the Adafruit MQTT library.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
// Set a unique MQTT client ID using the AIO key + the date and time the sketch
// was compiled (so this should be unique across multiple devices for a user,
// alternatively you can manually set this to a GUID or other random value).
const char MQTT_CLIENTID[] PROGMEM  = __TIME__ AIO_USERNAME;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }


/****************************** Feeds ***************************************/

// Setup a feeds for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char HEARTBEAT_FEED[] PROGMEM = "/rcu/" AIO_GUID "/heartbeat/state";
Adafruit_MQTT_Publish heartbeat = Adafruit_MQTT_Publish(&mqtt, HEARTBEAT_FEED);

const char TEMPERATURE_FEED[] PROGMEM = "/rcu/" AIO_GUID "/temperature/state";
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);

const char HUMIDITY_FEED[] PROGMEM = "/rcu/" AIO_GUID "/humidity/state";
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);

const char PIR_FEED[] PROGMEM = "/rcu/" AIO_GUID "/pir/state";
Adafruit_MQTT_Publish pir = Adafruit_MQTT_Publish(&mqtt, PIR_FEED);

const char LDR_FEED[] PROGMEM = "/rcu/" AIO_GUID "/ldr/state";
Adafruit_MQTT_Publish ldr = Adafruit_MQTT_Publish(&mqtt, LDR_FEED);

const char RELAY1_FEED_STATE[] PROGMEM = "/rcu/" AIO_GUID "/relay1/state";
Adafruit_MQTT_Publish relay1State = Adafruit_MQTT_Publish(&mqtt, RELAY1_FEED_STATE);

const char RELAY2_FEED_STATE[] PROGMEM = "/rcu/" AIO_GUID "/relay2/state";
Adafruit_MQTT_Publish relay2State = Adafruit_MQTT_Publish(&mqtt, RELAY2_FEED_STATE);

const char RELAY3_FEED_STATE[] PROGMEM = "/rcu/" AIO_GUID "/relay3/state";
Adafruit_MQTT_Publish relay3State = Adafruit_MQTT_Publish(&mqtt, RELAY3_FEED_STATE);

const char RELAY4_FEED_STATE[] PROGMEM = "/rcu/" AIO_GUID "/relay4/state";
Adafruit_MQTT_Publish relay4State = Adafruit_MQTT_Publish(&mqtt, RELAY4_FEED_STATE);

/*******************************************/ 
// Setup a feeds for subscribing to commands.
/*******************************************/ 
const char RELAY1_FEED[] PROGMEM = "/rcu/" AIO_GUID "/relay1/com";
Adafruit_MQTT_Subscribe relay1 = Adafruit_MQTT_Subscribe(&mqtt, RELAY1_FEED);
const char RELAY2_FEED[] PROGMEM = "/rcu/" AIO_GUID "/relay2/com";
Adafruit_MQTT_Subscribe relay2 = Adafruit_MQTT_Subscribe(&mqtt, RELAY2_FEED);
const char RELAY3_FEED[] PROGMEM = "/rcu/" AIO_GUID "/relay3/com";
Adafruit_MQTT_Subscribe relay3 = Adafruit_MQTT_Subscribe(&mqtt, RELAY3_FEED);
const char RELAY4_FEED[] PROGMEM = "/rcu/" AIO_GUID "/relay4/com";
Adafruit_MQTT_Subscribe relay4 = Adafruit_MQTT_Subscribe(&mqtt, RELAY4_FEED);
// Request publish all settings feedback.
const char PUBLISH_ALL_FEED[] PROGMEM = "/rcu/" AIO_GUID "/publishAll/com";
Adafruit_MQTT_Subscribe publishAll = Adafruit_MQTT_Subscribe(&mqtt, PUBLISH_ALL_FEED);

/*************************** Sketch Code ************************************/

// Only read temp or humidity every tempHumidityDelay milliseconds
// to prevent blocking loop by reading device every time it loops.
unsigned long humidityTimer = millis();
unsigned long tempTimer = millis();
unsigned long tempHumidityDelay = 15000;

void setup() {
	// Setup serial connection
	Serial.begin(115200);
	Serial.println(F("Zekes MEGA Room Control Unit"));

	// Initialise the Client
	Serial.print(F("\nInitialising the Client..."));
	Ethernet.begin(mac);
	delay(1000); //give the ethernet a second to initialize
	mqtt.subscribe(&relay1);
	mqtt.subscribe(&relay2);
	mqtt.subscribe(&relay3);
	mqtt.subscribe(&relay4);
	mqtt.subscribe(&publishAll);
	// Initialise sensor pins
	dht.setup(2, dht.DHT22);
	pinMode(statePinLDR, INPUT);
	pinMode(statePinPIR, INPUT);
	// Initialise relay output pins
	pinMode(commandPinRelay1, OUTPUT);
	pinMode(commandPinRelay2, OUTPUT);
	pinMode(commandPinRelay3, OUTPUT);
	pinMode(commandPinRelay4, OUTPUT);
	// Turn relays off by default
	digitalWrite(commandPinRelay1, HIGH);
	digitalWrite(commandPinRelay2, HIGH);
	digitalWrite(commandPinRelay3, HIGH);
	digitalWrite(commandPinRelay4, HIGH);
}

// Declare heartbeat counter.
uint32_t x=0;

// Set up variables for relay states. This is used by PublishAll routine when publishing relay states.
// Default to 0 as all relays are initialised to off in Setup routine.
short relay1LastState = 0;
short relay2LastState = 0;
short relay3LastState = 0;
short relay4LastState = 0;

String myRelayMessageDefault = "Not Set";
String myRelayMsg1 = "0";
String myRelayMsg2 = "0";
String myRelayMsg3 = "0";
String myRelayMsg4 = "0";
String myPublishAllMsg = "0";

void loop() {
	// Ensure the connection to the MQTT server is alive (this will make the first
	// connection and automatically reconnect when disconnected).  See the MQTT_connect
	// function definition further below.
	MQTT_connect();

	// this is our 'wait for incoming subscription packets' busy subloop
	Adafruit_MQTT_Subscribe *subscription;
	while ((subscription = mqtt.readSubscription(1000))) 
	{
		if (subscription == &relay1) 
		{
			Serial.print(F("******** Relay 1 received: "));
			uint32_t myRelayRead = (uint32_t)relay1.lastread;
			myRelayMsg1 = (char *)myRelayRead;
			//String myRelayMsg1 = (char *)relay1.lastread;
			Serial.println(myRelayMsg1);
			if (myRelayMsg1 == "0")
			{
				digitalWrite(commandPinRelay1, HIGH);
				relay1LastState = 0;
			}
			else
			{
				digitalWrite(commandPinRelay1, LOW);
				relay1LastState = 1;
			}
		}
		if (subscription == &relay2) 
		{
			Serial.print(F("******** Relay 2 received: "));
			myRelayMsg2 = (char *)relay2.lastread;
			Serial.println(myRelayMsg2);
			if (myRelayMsg2 == "0")
			{
				digitalWrite(commandPinRelay2, HIGH);
				relay2LastState = 0;
			}
			else
			{
				digitalWrite(commandPinRelay2, LOW);
				relay2LastState = 1;
			}
		}
		if (subscription == &relay3) 
		{
			Serial.print(F("******** Relay 3 received: "));
			myRelayMsg3 = (char *)relay3.lastread;
			Serial.println(myRelayMsg3);
			if (myRelayMsg3 == "0")
			{
				digitalWrite(commandPinRelay3, HIGH);
				relay3LastState = 0;
			}
			else
			{
				digitalWrite(commandPinRelay3, LOW);
				relay3LastState = 1;
			}
		}
		if (subscription == &relay4) 
		{
			Serial.print(F("******** Relay 4 received: "));
			myRelayMsg4 = (char *)relay4.lastread;
			Serial.println(myRelayMsg4);
			if (myRelayMsg4 == "0")
			{
				digitalWrite(commandPinRelay4, HIGH);
				relay4LastState = 0;
			}
			else
			{
				digitalWrite(commandPinRelay4, LOW);
				relay4LastState = 1;
			}
		}
		if (subscription == &publishAll) 
		{
			Serial.print(F("******** Publish all request received: "));
			myPublishAllMsg = (char *)publishAll.lastread;
			Serial.println(myPublishAllMsg);
			PublishAllSensors();
		}
	}



	// Now we can publish stuff!

	// Send heartbeat reading.
	Serial.println();
	Serial.println();
	Serial.print(F("\nSending heartbeat "));
	Serial.print(x);
	Serial.print("...");
	if (! heartbeat.publish(x++))
	{
		Serial.println(F("Failed"));
	} 
	else
	{
		Serial.println(F("OK!"));
	}
	// Send temperature reading.
	static float myTemp = 0;
	static float myTempLast = 0;
	// Only read every tempHumidityDelay milliseconds
	if (tempTimer < millis())
	{
		myTemp = dht.getTemperature();
		tempTimer = millis() + tempHumidityDelay;
	}
	if (myTemp != myTempLast)
	{
		Serial.print(F("\nSending	temperature "));
		Serial.print(myTemp);
		Serial.print("...");
		if (! temperature.publish(myTemp)) 
		{
			Serial.println(F("Failed"));
		} 
		else 
		{
			Serial.println(F("OK!"));
		}
		myTempLast = myTemp;
	}
	// Send humidity reading.
	static float myHumidity = 0;
	static float myHumidityLast = 0;
	if (humidityTimer < millis())
	{
		myHumidity = dht.getHumidity();
		humidityTimer = millis() + tempHumidityDelay;
	}
	if (myHumidityLast != myHumidity)
	{
		Serial.print(F("\nSending	humidity "));
		Serial.print(myHumidity); // randomVal1);
		Serial.print("...");
		if (! humidity.publish(myHumidity)) 
		{
			Serial.println(F("Failed"));
		} 
		else 
		{
			Serial.println(F("OK!"));
		}
		myHumidityLast = myHumidity;
	}

	// Send PIR reading.
	uint32_t myPIR = digitalRead(statePinPIR);
	static uint32_t myPIRLast = 0;
	if (myPIRLast != myPIR)
	{
		Serial.print(F("\nSending PIR "));
		Serial.print(myPIR);
		Serial.print("...");
		if (! pir.publish(myPIR)) 
		{
			Serial.println(F("Failed"));
		} 
		else 
		{
			Serial.println(F("OK!"));
		}
		myPIRLast = myPIR;
	}

	// Send Light Dependant Resistor reading.
	uint32_t myLDR = analogRead(statePinLDR);
	static uint32_t myLDRLast = 0;
	if (myLDRLast != myLDR)
	{
		Serial.print(F("\nSending LDR "));
		Serial.print(myLDR);
		Serial.print("...");
		if (! ldr.publish(myLDR)) 
		{
			Serial.println(F("Failed"));
		} 
		else 
		{
			Serial.println(F("OK!"));
		}
		myLDRLast = myLDR;
	}
	
	// Publish relay 1 state if it has been changed to give feedback that relay has changed
	if (myRelayMsg1 != myRelayMessageDefault)
	{
		Serial.print(F("\nSending relay1 state "));
		Serial.print(myRelayMsg1);
		Serial.print("...");
		if (myRelayMsg1 == "0")
		{
			if (! relay1State.publish((uint32_t)(0)))
			{
				Serial.println(F("Failed"));
			} 
			else
			{
				Serial.println(F("OK!"));
				myRelayMsg1 = myRelayMessageDefault;
			}
		}
		else
		{
			if (! relay1State.publish((uint32_t)1))
			{
				Serial.println(F("Failed"));
			} 
			else
			{
				Serial.println(F("OK!"));
				myRelayMsg1 = myRelayMessageDefault;
			}
		}
	}

	// Publish relay 2 state if it has been changed to give feedback that relay has changed
	if (myRelayMsg2 != myRelayMessageDefault)
	{
		Serial.print(F("\nSending relay2 state "));
		Serial.print(myRelayMsg2);
		Serial.print("...");
		if (myRelayMsg2 == "0")
		{
			if (! relay2State.publish((uint32_t)(0)))
			{
				Serial.println(F("Failed"));
			} 
			else
			{
				Serial.println(F("OK!"));
				myRelayMsg2 = myRelayMessageDefault;
			}
		}
		else
		{
			if (! relay2State.publish((uint32_t)1))
			{
				Serial.println(F("Failed"));
			} 
			else
			{
				Serial.println(F("OK!"));
				myRelayMsg2 = myRelayMessageDefault;
			}
		}
	}

	// Publish relay 3 state if it has been changed to give feedback that relay has changed
	if (myRelayMsg3 != myRelayMessageDefault)
	{
		Serial.print(F("\nSending relay3 state "));
		Serial.print(myRelayMsg3);
		Serial.print("...");
		if (myRelayMsg3 == "0")
		{
			if (! relay3State.publish((uint32_t)(0)))
			{
				Serial.println(F("Failed"));
			} 
			else
			{
				Serial.println(F("OK!"));
				myRelayMsg3 = myRelayMessageDefault;
			}
		}
		else
		{
			if (! relay3State.publish((uint32_t)1))
			{
				Serial.println(F("Failed"));
			} 
			else
			{
				Serial.println(F("OK!"));
				myRelayMsg3 = myRelayMessageDefault;
			}
		}
	}


	// Publish relay 4 state if it has been changed to give feedback that relay has changed
	if (myRelayMsg4 != myRelayMessageDefault)
	{
		Serial.print(F("\nSending relay4 state "));
		Serial.print(myRelayMsg4);
		Serial.print("...");
		if (myRelayMsg4 == "0")
		{
			if (! relay4State.publish((uint32_t)(0)))
			{
				Serial.println(F("Failed"));
			} 
			else
			{
				Serial.println(F("OK!"));
				myRelayMsg4 = myRelayMessageDefault;
			}
		}
		else
		{
			if (! relay4State.publish((uint32_t)1))
			{
				Serial.println(F("Failed"));
			} 
			else
			{
				Serial.println(F("OK!"));
				myRelayMsg4 = myRelayMessageDefault;
			}
		}
	}

	// ping the server to keep the mqtt connection alive
	if(! mqtt.ping()) {
	mqtt.disconnect();
	}
}

void PublishAllSensors(void)
{
	// Send temperature reading.
	float myTemp = 0;
	delay(dht.getMinimumSamplingPeriod());
	myTemp = dht.getTemperature();
	Serial.print(F("\nSending	temperature "));
	Serial.print(myTemp);
	Serial.print("...");
	if (! temperature.publish(myTemp)) 
	{
		Serial.println(F("Failed"));
	} 
	else 
	{
		Serial.println(F("OK!"));
	}
	// Send humidity reading.
	delay(dht.getMinimumSamplingPeriod());
	float myHumidity = dht.getHumidity();
	Serial.print(F("\nSending	humidity "));
	Serial.print(myHumidity); // randomVal1);
	Serial.print("...");
	if (! humidity.publish(myHumidity)) 
	{
		Serial.println(F("Failed"));
	} 
	else 
	{
		Serial.println(F("OK!"));
	}

	// Send PIR reading.
	uint32_t myPIR = digitalRead(statePinPIR);
	Serial.print(F("\nSending PIR "));
	Serial.print(myPIR);
	Serial.print("...");
	if (! pir.publish(myPIR)) 
	{
		Serial.println(F("Failed"));
	} 
	else 
	{
		Serial.println(F("OK!"));
	}

	// Send Light Dependant Resistor reading.
	uint32_t myLDR = analogRead(statePinLDR);
	Serial.print(F("\nSending LDR "));
	Serial.print(myLDR);
	Serial.print("...");
	if (! ldr.publish(myLDR)) 
	{
		Serial.println(F("Failed"));
	} 
	else 
	{
		Serial.println(F("OK!"));
	}
	
	// Publish relay 1's last state.
	Serial.print(F("\nSending relay1 state "));
	Serial.print(relay1LastState);
	Serial.print("...");
	if (relay1LastState == 0)
	{
		if (! relay1State.publish((uint32_t)(0)))
		{
			Serial.println(F("Failed"));
			// If it fails to publish then force resend/s through main loop when this routine returns.
			myRelayMsg1 = "0";
		} 
		else
		{
			Serial.println(F("OK!"));
		}
	}
	else
	{
		if (! relay1State.publish((uint32_t)1))
		{
			Serial.println(F("Failed"));
			// If it fails to publish then force resend/s through main loop when this routine returns.
			myRelayMsg1 = "1";
		} 
		else
		{
			Serial.println(F("OK!"));
		}
	}

	// Publish relay 2's last state.
	Serial.print(F("\nSending relay2 state "));
	Serial.print(relay2LastState);
	Serial.print("...");
	if (relay2LastState == 0)
	{
		if (! relay2State.publish((uint32_t)(0)))
		{
			Serial.println(F("Failed"));
			// If it fails to publish then force resend/s through main loop when this routine returns.
			myRelayMsg2 = "0";
		} 
		else
		{
			Serial.println(F("OK!"));
		}
	}
	else
	{
		if (! relay2State.publish((uint32_t)1))
		{
			Serial.println(F("Failed"));
			// If it fails to publish then force resend/s through main loop when this routine returns.
			myRelayMsg2 = "1";
		} 
		else
		{
			Serial.println(F("OK!"));
		}
	}

	// Publish relay 3's last state.
	Serial.print(F("\nSending relay3 state "));
	Serial.print(relay3LastState);
	Serial.print("...");
	if (relay3LastState == 0)
	{
		if (! relay3State.publish((uint32_t)(0)))
		{
			Serial.println(F("Failed"));
			// If it fails to publish then force resend/s through main loop when this routine returns.
			myRelayMsg3 = "0";
		} 
		else
		{
			Serial.println(F("OK!"));
		}
	}
	else
	{
		if (! relay3State.publish((uint32_t)1))
		{
			Serial.println(F("Failed"));
			// If it fails to publish then force resend/s through main loop when this routine returns.
			myRelayMsg3 = "1";
		} 
		else
		{
			Serial.println(F("OK!"));
		}
	}

	// Publish relay 4's last state.
	Serial.print(F("\nSending relay4 state "));
	Serial.print(relay4LastState);
	Serial.print("...");
	if (relay4LastState == 0)
	{
		if (! relay4State.publish((uint32_t)(0)))
		{
			Serial.println(F("Failed"));
			// If it fails to publish then force resend/s through main loop when this routine returns.
			myRelayMsg4 = "0";
		} 
		else
		{
			Serial.println(F("OK!"));
		}
	}
	else
	{
		if (! relay4State.publish((uint32_t)1))
		{
			Serial.println(F("Failed"));
			// If it fails to publish then force resend/s through main loop when this routine returns.
			myRelayMsg4 = "1";
		} 
		else
		{
			Serial.println(F("OK!"));
		}
	}
}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
	return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
	   Serial.println(mqtt.connectErrorString(ret));
	   Serial.println("Retrying MQTT connection in 5 seconds...");
	   mqtt.disconnect();
	   delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}
