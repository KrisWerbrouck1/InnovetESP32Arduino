# ESP32

## Algemeen

De ESP32 is net als de ESP8266 een microcontroller van ESpressif. Enkele
kenmerken van de ESP32 zijn:

-   Dual core 32-bit microprocessor
-   Draadloze Wifi communicatie 802.11 b/g/n
-   Bluetooth communicatie
-   12-bit SAR ADC (Successive approximation analog to digital convertor)
-   8 bit DAC (digital tot analog convertor)
-   SPI-bus (Serial Peripheral Interface)
-   I²C-bus (Inter-IC-bus) Spreek uit als I kwadraat C bus
-   PWM-modules (Pulse-width modulation)

Er worden door verschillende fabrikanten ontwikkelbord met de ESP32
verkocht. Enkele veel voorkomende borden zijn:

Wemos D32-Pro V2 <https://www.wemos.cc/en/latest/d32/d32_pro.html>

Leverancier:
<https://opencircuit.nl/Product/Wemos-D32-Pro-V2.0.0-ESP32-CH340C-16MB-Flash>

![D32Pro](./assets/D32Pro.png)

FireBeatle ESP32 van dfrobot
<https://wiki.dfrobot.com/FireBeetle_ESP32_IOT_Microcontroller(V3.0)__Supports_Wi-Fi_&_Bluetooth__SKU__DFR0478>

Leverancier:
<https://opencircuit.nl/Product/FireBeetle-ESP32-IoT-Microcontroller>

![Fire32](./assets/Fire32.png)

De ESP32 kan zowel geprogrammeerd worden in C met arduino IDE als met micropython. In deze cursus opteren we voor arduino IDE.

## Arduino IDE

De ESP32 is niet standaard aanwezig in arduino IDE. Toevoegen kan via:

<https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/>

Te kiezen bord bij Wemos D32-Pro V2:

![D32ProArduino](./assets/D32ProArduino.png)

Te kiezen bord bij FireBeatle ESP32 van dfrobot:

![Fire32Arduino](./assets/Fire32Arduino.png)

## Knipperled

Voorbeeldprogramma met delay's:

```cpp
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

```

Voorbeeldprogramma zonder delay's:

```cpp
unsigned long previousMillis = 0;       //laatste maal toestandsverandering
unsigned long currentMillis = 0;       //tijd sinds opstart arduino

boolean ledState = LOW;                // ledstatus

const long interval = 1000;           // interval in milliseconden
const int ledPin =  LED_BUILTIN;      // pin nummer van de ingebouwde led

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  currentMillis = millis();
  if ((currentMillis - previousMillis) >= interval) //wanneer verschil tussen laatste toestandsverandering en opstarttijd groter of gelijk is aan interval
  { ledState=!ledState;
    previousMillis = currentMillis;    //update previousMillis
    digitalWrite(LED_BUILTIN,ledState);
    Serial.println(ledState);
  }

}
```

Opdrachten:

-   Zoek in de documentatie de pin van de ingebouwde led op het bord op. Laat de ingebouwde led knipperen.
-   Plaats een externe led en laat deze knipperen.
-   Laat 2 leds afwisselend knipperen.

## Weergave seriële monitor

Voorbeeldprogramma

```cpp
void setup() {
Serial.begin(115200);
}

void loop() {
delay(1000);
Serial.println("hello from ESP32");
}
```

Opdracht:
-   Test bovenstaande programma.

## Digitale ingang

Sluit volgende schakeling aan op een zelfgekozen ingang

![Schakeling met drukknop](./assets/drukknop.png)

Voorbeeldcode

```cpp
#define drukknopPin 4

boolean drukknopStatus;

void setup() {
 pinMode(drukknopPin, INPUT); // pin 12 als ingang plaatsen
 Serial.begin(115200);
}

void loop() {
  drukknopStatus = digitalRead(drukknopPin); //binnenlezen toestand ingang
  Serial.println(drukknopStatus);
}
```

Opdrachten:
-   Test de voorbeeldcode met de drukknop uit.
-   Laat een led oplichten wanneer de drukknop ingedrukt is.
-   Sluit 2 drukknoppen aan. De ene drukknop is de start drukknop de andere de stopdrukknop. Laat een led oplichten wanneer de startdrukknop ingedrukt is.

## Analoge invoer

De ESP32 beschikt over een 12 bit ADC.

Voorbeeldprogramma

```cpp
int sensorPin = A6;    // select the input pin 
int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(115200);
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  Serial.print("Sensorwaarde: ");
  Serial.println (sensorValue);
  delay(100);
}
```

## DHT11 temperatuur- en luchtvochtigheidssensor

De data wordt digitaal verzonden van de DHT11 naar de arduino.

De aansluitpinnen:

![DHT11](./assets/DHT11.png)
of
![DHT11](./assets/DHT11V2.png)

Voorbeeldprogramma:

```cpp
#include "DHT.h"
//here we use 14 of ESP32 to read data
#define DHTPIN 14
//our sensor is DHT11 type
#define DHTTYPE DHT11
//create an instance of DHT sensor
DHT dht(DHTPIN, DHTTYPE);
void setup()
{ Serial.begin(115200);
  Serial.println("DHT11 sensor!");
  //call begin to start sensor
  dht.begin();
}

void loop() {
  //use the functions which are supplied by library.
  float hum = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float temp = dht.readTemperature();

  // print the result to Terminal
  Serial.print("Humidity: ");
  Serial.print(hum);
  Serial.print(" \t");
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" *C ");
  //we delay a little bit for next read
  delay(2000);
}
```

## MAC-adres

Voorbeeldcode om het mac-adres weer te geven

```cpp
#include <WiFi.h>

void setup(){
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
}

void loop(){
}
```

## Overzicht beschikbare wifi netwerken en de signaalsterkte

```cpp
#include "WiFi.h"

void setup() {
  Serial.begin(115200);
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("Setup done");
}

void loop() {
  Serial.println("scan start");
  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0) {
    Serial.println("no networks found");
  } 
 else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; i++) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.println(")");
      delay(10);
    }
  }

  // Wait a bit before scanning again
  delay(5000);
}
```


## Verbinding met WPA2 personal en weergave IP-adres

De ESP wordt in station mode verbonden met een WPA2 personal netwerk.
Het IP-adres wordt weergegeven.

Meer info over station mode:
<https://nodemcu.readthedocs.io/en/release/modules/wifi/>


Voorbeeld:
```cpp
#include <WiFi.h>

#define WIFI_SSID "SSID"  //SSID WPA2 personal wifi netwerk
#define WIFI_PSK "PASWOORD" // paswoord WPA2 personal wifi netwerk

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("WIFI:\tsetup");
  WiFi.mode(WIFI_STA); //Station mode
  Serial.print("\tconnecting ");
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.print("\n\tGot wireless IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // put your main code here, to run repeatedly:

}

```


## HTTP en HTML
### HTTP

HTTP staat voor Hyper Text Transfer Protocol. HTTP is ontworpen voor communicatie tussen client en server. Met een browser zoals chome of firefox (de client) is het mogelijk webpagina’s op te vragen op een ESP32. (server)

![HTTP request](./assets/HTTP1.png)

### HTTP request

Als je in de adresbalk van je webbrowser een bepaalde website opvraagt, dan wordt er van de client naar de server een http request gestuurd. Er bestaan verschillende request methodes zoals GET, POST,PUT,... In de cursus wordt enkel GET en POST gebruikt.

[Voorbeeld GET met php bestand](http://www.sensor-cube.be/opleidingiot/formget.php)

Via het php bestand formget.php kan je informatie doorgeven via GET. Test dit uit. 

Wanneer je het woord test meegeeft krijg je volgende URL:

[http://www.sensor-cube.be/opleidingiot/formget.php?naam=test](http://www.sensor-cube.be/opleidingiot/formget.php?naam=test)

Via functietoets F12 op het toetsenbord kan meer info weergegeven worden in de browser.

![Weergave GET in browser](./assets/GET1.png)

[Voorbeeld POST met php bestand](http://www.sensor-cube.be/opleidingiot/formpost.php)

Via het php bestand formpost.php wordt informatie verstuurd. Test dit uit.

![Weergave POST in browser](./assets/POST1.png)

###	Http-response

Een typische http-response van de server naar de client ziet er als volgt uit:

```HTML
HTTP/1.0 200 OK
Content-Type:text/plain
Alles goed?
```

Of
```HTML
HTTP/1.0 200 OK
Content-Type:text/html
<html>
<h1>Hello from ESP8266 </h1>
</html>
```
Een http-response die de server naar de client stuurt, geeft informatie mee die de client kan gebruiken om de content correct weer te geven.
De status lijn “HTTP/1.0 200 OK” of “HTTP/1.1 200 OK” moet altijd minimaal in de http response staan.
* HTTP/1.0 geeft de gebruikte versie van http aan. Enkel versie 1.0, 1.1 en 2.0 worden nog ondersteund.
* 200 is een code die de status van de opgevraagde data weergeeft.

Een overzicht van veel voorkomende statuscodes:
* 200 OK - succesvol opgevraagde content
* 301 Moved Permanently – de content moet via de nieuwe URL gehaald worden
* 400 Bad Request - de request bevat fouten
* 404 Not Found - de opgevraagde content bestaat niet
* 500 Internal Server Error - server foutje


#	Microcontroller als simpele webserver

Bron: <https://randomnerdtutorials.com/esp32-web-server-arduino-ide/>

In onderstaande voorbeeld wordt de microcontroller in het netwerk geplaatst. De microcontroller doet dienst als server om een webpagina weer te geven. Vul zeker het juist SSID en paswoord van het netwerk in.

```cpp 
/*********
  Complete project details at http://randomnerdtutorials.com
*********/

// Load Wi-Fi library
#include <WiFi.h>

// Replace with your network credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_SSID_PASSWORD";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(115200);
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();


            // Display the HTML web page
            client.println("<!DOCTYPE html>");
            // Web Page Heading
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
            client.println("<title>ESP32 Webpagina</title>");
            client.println("</head>");
            // Web Page body
            client.println("<body><h1>ESP32 Web Server</h1>");
            client.println("<p>Een tekst</p>");
            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          }
          else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
```

Test het programma uit.

### Onderzoeksvragen

Wat is het nut van volgende stuk code:

```cpp 
while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
```

Wat is het nut van volgende regel code:
```cpp 
Serial.println("WiFi connected.");
Serial.println("IP address: ");
Serial.println(WiFi.localIP());
```	

Waarvoor staat 80 in volgende regel code:
```cpp 
WiFiServer server(80);
```	

Wat wordt bewaard in de string header met volgende code:
```cpp 
if (client.available()) {                   // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
```	

Wat wordt bepaald met volgende teken:
```cpp 
\n

```

in volgende code
```cpp 
if (c == '\n') { 
```

Wat is het nut van volgende regels code:
```cpp 
client.println("HTTP/1.1 200 OK");
client.println("Content-type:text/html");
client.println("Connection: close");
client.println();
```

Wat is het nut van volgende regels code:
```cpp
client.println("<!DOCTYPE html>");
// Web Page Heading
client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
client.println("<title>ESP32 Webpagina</title>");
 client.println("</head>");
// Web Page body
client.println("<body><h1>ESP32 Web Server</h1>");
client.println("<p>Een tekst</p>");
client.println("</body></html>");
```

### Opdracht

Geef de analoge ingangsspanning weer op een webpagina. Maak gebruik van een potentiometer.

### Aansturen actuatoren

Bron: <https://randomnerdtutorials.com/esp32-web-server-arduino-ide/>

In volgende voorbeeld wordt via de parameters:
* /on de ingebouwde led ingeschakeld.
* /off de ingebouwde led uitgeschakeld.

Voorbeeldcode:

```cpp
// Load Wi-Fi library
#include <WiFi.h>

// Replace with your network credentials
const char* ssid = "IOT";
const char* password = "IOT-Wifi";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variable to store the current output state
String outputLedState = "off";


// Assign output variable to GPIO pins
const int outputLed = LED_BUILTIN;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(115200);
  // Initialize the output variable as output
  pinMode(outputLed, OUTPUT);

  // Set output to LOW
  digitalWrite(outputLed, LOW);
 
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /on") >= 0) {
              Serial.println("Builtin led on");
              outputLedState = "on";
              digitalWrite(outputLed, HIGH);
            } else if (header.indexOf("GET /off") >= 0) {
              Serial.println("Builtin led off");
              outputLedState = "off";
              digitalWrite(outputLed, LOW);
            } 
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for build in led  
            client.println("<p>Builtin led - State " + outputLedState + "</p>");
            // If the builtin led is off, it displays the ON button       
            if (outputLedState=="off") {
              client.println("<p><a href=\"/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
```

### Opdracht

Stuur 2 leds aan via een webbrowser.




Bron:

<https://nl.wikipedia.org/wiki/ESP32>


[![Netlify Status](https://api.netlify.com/api/v1/badges/f0f5c858-598f-479c-b0da-169debeca8c6/deploy-status)](https://app.netlify.com/sites/innovetesp32arduino/deploys)