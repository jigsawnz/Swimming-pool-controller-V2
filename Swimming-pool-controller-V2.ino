/*  The MIT License (MIT)
 *
 *  Copyright (c) [2015] [George Timmermans]
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

//********************************************************************

#include "SPI.h"
#include "ILI9341_t3.h"
#include "Adafruit_Sensor.h"
#include "DHT.h"
#include "DHT_U.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "EEPROM.h"

//********************************************************************

/* Pinout teensy 3.0
 *                             -----
 *                      GND *  |___|  * Vin (3.7 to 5.5 volts)
 * TX                     0 *         * AGND
 * RX                     1 *         * 3.3V (100 mA max)
 * Relay1                 2 *         * 23-A9
 * Relay2                 3 *         * 22-A8
 * Relay3                 4 *         * 21-A7
 * Relay4                 5 * _______ * 20-A6
 * speaker                6 *|       |* 19-A5
 *                        7 *|       |* 18-A4
 * ONE_WIRE_BUS           8 *|       |* 17-A3
 * TFT_DC                 9 *|_______|* 16-A2
 * TFT_CS                10 *         * 15-A1
 * TFT_MOSI              11 *   ###   * 14-A0        DHTPIN
 * TFT_MISO              12 * * * * * * 13(led)      TFT_SCLK   
 */
 
//********************************************************************
                                            
// TFT display
const uint8_t TFT_DC   =    9;
const uint8_t TFT_CS   =   10;
const uint8_t TFT_MOSI =   11;
const uint8_t TFT_MISO =   12;
const uint8_t TFT_SCLK =   13;
const uint8_t TFT_RST  =  255;  // 255 = unused, connect to 3.3V
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

// Landscape mode
const uint16_t WIDTH  = tft.height();
const uint16_t HEIGTH = tft.width();

// Dome air temperature and humidity sensor
const uint8_t DHTPIN  = 14;
const uint8_t DHTTYPE = DHT22;
DHT           dht(DHTPIN, DHTTYPE, 30);

// Waterproof DS18B20 temperature sensors
const uint8_t ONE_WIRE_BUS          = 8;
const uint8_t TEMPERATURE_PRECISION = 10; //9 = 0.5C, 10 = 0.25C, 11 = 0.1125C, 12 = 0.0625C 
OneWire       oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
const int8_t  DEVICE_DISCONNECTED = -127; //temperature reading when device is disconnected.

DeviceAddress swimmingPool = {0x28, 0xAC, 0x7A, 0x56, 0x06, 0x00, 0x00, 0xEF};
DeviceAddress solarPanel   = {0x28, 0x85, 0x62, 0x5C, 0x06, 0x00, 0x00, 0x96};
DeviceAddress Thermometer2 = {0x28, 0x57, 0x2C, 0x5C, 0x06, 0x00, 0x00, 0x77};
DeviceAddress Thermometer3 = {0x28, 0xAF, 0xC1, 0x5C, 0x06, 0x00, 0x00, 0x92};

// Relays
const uint8_t RELAY_VALVE_PIN = 2; 
const uint8_t RELAY_PUMP_PIN  = 3; 
const uint8_t RELAY3_PIN      = 4;
const uint8_t RELAY4_PIN      = 5;

// Alarm speaker
const uint8_t SPEAKER_PIN     = 6;
bool speakerState = false;

// Arrays for recording sensor data.
const uint16_t MAX = 288;  // data every 5 min, total of 24 hours
double domeTempArray     [ MAX ];
double swimmingPoolArray [ MAX ];
double solarPanelArray   [ MAX ];
//double domeTempArray2  [ MAX ];
//double domeTempArray3  [ MAX ];
double humArray          [ MAX ];

// Intervals on which to perform certain actions.
const uint32_t REFRESH_GUI_INTERVAL   = 5000;
const uint32_t STORE_DATA_INTERVAL    = 5000; // 300000 (save every 5 min, 5 x 60 X 1000 = 300000 milliseconds
const uint32_t SENSOR_DATA_INTERVAL   = 2500; // set to slowest sensor (dht21 , 2.5seconds refresh.
const uint32_t UPDATE_RELAY_INTERVAL  = 2500; // 2.5seconds refresh
const uint32_t PUMP_OVERRIDE_INTERVAL = 7200000; //(time out after 120 minutes, 120 x 60 X 1000 = 7200000 milliseconds
const uint32_t SPEAKER_INTERVAL       = 1000;

// Store sensor readings and easy referencing.
double domeTemp, poolTemp, solarTemp, thermo2, thermo3, humidity;
enum sensors {DOMETEMP, POOLTEMP, SOLARTEMP, THERMO2, THERMO3, HUMIDITY, TEMPCONTROL};
double tempDifference;

// Store incoming byte from serial and bluetooth communications.
char inByte = 'e';

// Store system status in EEPROM
uint8_t address = 0;
byte value;

//********************************************************************
// System flags
bool systemON      = false;
bool valveOPEN     = false;
bool pumpON        = false; 
bool pumpOVERRIDE  = false;
bool sensorFAILURE = false;
//********************************************************************
// Temperature setpoints for relay controls 
const double VALVE_OPEN    =  5.0;
const double VALVE_CLOSED  =  2.0;
const double MAX_POOL_TEMP = 35.0;
//********************************************************************
unsigned long storeDataTimer      = millis();
unsigned long refreshGUITimer     = millis();
unsigned long refreshSensorsTimer = millis();
unsigned long updateRelayTimer    = millis();
unsigned long pumpOverrideTimer   = 0;
unsigned long speakerTimer        = millis();
//********************************************************************

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600); // Bleutooth.
  
  sensors.begin();
  sensors.setResolution(swimmingPool, TEMPERATURE_PRECISION);
  sensors.setResolution(solarPanel,   TEMPERATURE_PRECISION);
  sensors.setResolution(Thermometer2, TEMPERATURE_PRECISION);
  sensors.setResolution(Thermometer3, TEMPERATURE_PRECISION);
  
  pinMode(RELAY_VALVE_PIN, OUTPUT); 
  pinMode(RELAY_PUMP_PIN,  OUTPUT); 
  pinMode(RELAY3_PIN,      OUTPUT); 
  pinMode(RELAY4_PIN,      OUTPUT);
  pinMode(SPEAKER_PIN,     OUTPUT);
  digitalWrite(RELAY_VALVE_PIN, HIGH);
  digitalWrite(RELAY_PUMP_PIN,  HIGH);
  digitalWrite(RELAY3_PIN,      HIGH);
  digitalWrite(RELAY4_PIN,      HIGH);
  digitalWrite(SPEAKER_PIN,      LOW);
  
  tft.begin();
  tft.setRotation(1);  // Landscape mode
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
  tft.println(F("Solar Pool Controller"));
  tft.println(F("By George Timmermans"));

  Serial.println(F("Solar Pool Controller"));
  Serial.println(F("By George Timmermans"));
  Serial.println();
  delay(1000);
  
  printMenu();
  
  delay(2500);

  systemON = EEPROM.read(address); 
}

//********************************************************************

void loop(void) {
  // handel overflow of the millis() function
  if (storeDataTimer > millis())  
    storeDataTimer = millis();
  if (refreshGUITimer > millis())  
    refreshGUITimer = millis();  
  if (refreshSensorsTimer > millis())  
    refreshSensorsTimer = millis();  
  if (pumpOVERRIDE && pumpOverrideTimer > millis())
    pumpOverrideTimer = millis(); 
  if (sensorFAILURE && speakerTimer > millis())
    speakerTimer = millis(); 
  
  if (Serial.available() > 0) // Read incoming serial data (USB)
  {
    inByte = Serial.read();
    Serial.print(F("USB received: "));
    Serial.println(char(inByte));
    Serial.println();
    Serial1.print(F("USB received:"));
    Serial1.println(char(inByte));
    Serial1.println();
    refreshGUITimer = REFRESH_GUI_INTERVAL + 1;
  }
  if (Serial1.available() > 0) // Read incoming serial data (Bluetooth)
  {
    inByte = Serial1.read();
    Serial.print(F("UART received: "));
    Serial.println(char(inByte));
    Serial.println();
    Serial1.print(F("UART received:"));
    Serial1.println(char(inByte));
    Serial1.println();
    refreshGUITimer = REFRESH_GUI_INTERVAL + 1;
  }
  
  switch (inByte) { 
    case 'a':  // Show air temperature graph on TFT display  
      if ( millis() - refreshGUITimer >= REFRESH_GUI_INTERVAL ) 
      {
        drawSensorDataGraph(DOMETEMP); 
        refreshGUITimer = millis();
      }  
      break;
    case 'b':  // Show pool temperature graph on TFT display  
      if ( millis() - refreshGUITimer >= REFRESH_GUI_INTERVAL ) 
      {
        drawSensorDataGraph(POOLTEMP); 
        refreshGUITimer = millis();
      }
      break;
    case 'c':  // Show solar panel temperature graph on TFT display
      if ( millis() - refreshGUITimer >= REFRESH_GUI_INTERVAL ) 
      {
        drawSensorDataGraph(SOLARTEMP); 
        refreshGUITimer = millis();
      }
      break;
    case 'd':  // Show air humiodity graph on TFT display
      if ( millis() - refreshGUITimer >= REFRESH_GUI_INTERVAL ) 
      {
        drawSensorDataGraph(HUMIDITY);
        refreshGUITimer = millis();
      }
      break;
      case 'e':  // Show temperature difference between pool temperature and solarpanel graph on TFT display
      if ( millis() - refreshGUITimer >= REFRESH_GUI_INTERVAL ) 
      {
        drawSensorDataGraph(TEMPCONTROL);
        refreshGUITimer = millis();
      }
      break;
    case 'p':  //  Override swimming pool pump for two hours.
      if (systemON) 
      {
        pumpOVERRIDE = !pumpOVERRIDE;
        if (pumpOVERRIDE)
        {
          Serial.println(F("Pump override is ON\n"));
          Serial1.println(F("Pump override is ON\n"));
          pumpOverrideTimer = millis();
        }
        else
          Serial.println(F("Pump override is OFF\n"));
          Serial1.println(F("Pump override is OFF\n"));
      }
      else
      {
        Serial.println(F("Turn system ON before overriding pump!\n"));
        Serial1.println(F("Turn system ON before overriding pump!\n"));
      }
      inByte = 'x';
      break;
    case 'x':  // Display system status and latest sensor readings
      if ( millis() - refreshGUITimer >= REFRESH_GUI_INTERVAL ) 
      {
      printSystemStatus(); 
      refreshGUITimer = millis();
      }
      break;
    case 'w':
      // easy way of preventing the refreshing the visual data.
      break;
    case '1':  // Turn system ON
      systemON = true;
      EEPROM.write(address, systemON);
      Serial.println(F("System is ON"));
      Serial.println();
      inByte = 'x';
      break;
    case '0':  // Turn system OFF
      systemON = false;
      EEPROM.write(address, systemON);
      digitalWrite(RELAY_VALVE_PIN, HIGH);
      digitalWrite(RELAY_PUMP_PIN,  HIGH); 
      pumpON = false;
      valveOPEN = false;
      pumpOVERRIDE = false;
      Serial.println(F("System is OFF"));
      Serial.println();
      inByte = 'x';
      break;
    case '?':  // Print menu options
      printMenu();
      inByte = 'w';
      break;  
    default:
      clearTFTScreen();
      Serial.print(inByte);
      Serial1.print(inByte);
      tft.setTextColor(ILI9341_RED);
      tft.print(inByte);
      tft.setTextColor(ILI9341_YELLOW);
      sendToAll(F(" :is not a valid option!\n"));
      sendToAll(F("press '?' for Menu\n\n"));
      inByte = 'w';  
  }  

  // Store sensor data for graphs
  if (millis() - storeDataTimer >= STORE_DATA_INTERVAL)  
  { 
    storeSensorData();
    storeDataTimer = millis();
  }

  // Update sensor data with new readings
  if (millis() - refreshSensorsTimer >= SENSOR_DATA_INTERVAL) 
  { 
    readSensors();
    refreshSensorsTimer = millis();
  }

  // Turn pump OFF after two hours when in override mode.
  if (pumpOVERRIDE) 
  {
    if (millis() - pumpOverrideTimer >= PUMP_OVERRIDE_INTERVAL)
    {
      pumpOVERRIDE = false;
      Serial.println(F("Pump override: timed out\n"));
      Serial1.println(F("Pump override: timed out\n"));
    }
  }

  // Making beeping sound if a critical sensor isn't working.
  if(sensorFAILURE)
  {
    if(millis() - speakerTimer >= SPEAKER_INTERVAL)
    {
      digitalWrite(SPEAKER_PIN, speakerState );
      speakerState = !speakerState;
      speakerTimer = millis();
    }
  }

  // Turn relays ON or OFF
  if (millis() - updateRelayTimer >= UPDATE_RELAY_INTERVAL) 
  { 
    controlRelay();
    updateRelayTimer = millis();
  }
}

//********************************************************************

// Open or close valve depending on the temperature difference between swimmingpool and solarpanel
void controlRelay()  {
  if (systemON)
  {
    if (poolTemp != DEVICE_DISCONNECTED && solarTemp != DEVICE_DISCONNECTED)
    { 
      if (pumpOVERRIDE)
      {
        if (tempDifference >= VALVE_OPEN)
        {
          if (pumpON)
          {
            if (valveOPEN)
            {
               // Do nothing
            }
            else // valveOPEN == false
            {
              valveOPEN = true;
              digitalWrite(RELAY_VALVE_PIN, LOW);
            }
          }
          else // pumpON == false
          {
            if (valveOPEN)
            {
              pumpON = true;
              digitalWrite(RELAY_PUMP_PIN, LOW);
            }
            else // valveOPEN == false
            {
              pumpON = true;
              digitalWrite(RELAY_PUMP_PIN, LOW);
              valveOPEN = true;
              digitalWrite(RELAY_VALVE_PIN, LOW);
            }
          }
        }
        else // tempDifference >= valveOPEN == false
        {
          if (tempDifference <= VALVE_CLOSED)
          {
            if (pumpON)
            {
              if (valveOPEN)
              {
                valveOPEN = false;
                digitalWrite(RELAY_VALVE_PIN, HIGH);         
              }
              else // valveOPEN == false
              {
                // Do nothing
              }
            }
            else // pumpON = false
            {
              if (valveOPEN)
              {
                pumpON = true;
                digitalWrite(RELAY_PUMP_PIN, LOW);
                valveOPEN = false;
                digitalWrite(RELAY_VALVE_PIN, HIGH);
              }
              else // valveOPEN == false
              {
                pumpON = true;
                digitalWrite(RELAY_PUMP_PIN, LOW);
              }
            }
          }
          else // tempDifference <= VALVE_CLOSED == false
          {
            if (pumpON)
            {
              // Do nothing         
            }
            else // pumpON = false
            {
              pumpON = true;
              digitalWrite(RELAY_PUMP_PIN, LOW);
            }
          }
        }
      }
      else // pumpOVERRIDE = false
      {
        if (tempDifference >= VALVE_OPEN)
        {
          if (pumpON)
          {
            if (valveOPEN)
            {
              // Do nothing
            }
            else // valveOPEN == false
            {
              valveOPEN = true;
              digitalWrite(RELAY_VALVE_PIN, LOW);
            }
          }
          else // pumpON == false
          {
            if (valveOPEN)
            {
              pumpON = true;
              digitalWrite(RELAY_PUMP_PIN, LOW);
            }
            else // valveOPEN == false
            {
              pumpON = true;
              digitalWrite(RELAY_PUMP_PIN, LOW);
              valveOPEN = true;
              digitalWrite(RELAY_VALVE_PIN, LOW); 
            }
          }
        }
        else // tempDifference >= VALVE_OPEN == false
        {
          if (tempDifference <= VALVE_CLOSED)
          {
            if (pumpON)
            {
              if (valveOPEN)
              {
                pumpON = false;
                digitalWrite(RELAY_PUMP_PIN, HIGH);
                valveOPEN = false;
                digitalWrite(RELAY_VALVE_PIN, HIGH); 
              }
              else // valveOPEN == false
              {
                pumpON = false;
                digitalWrite(RELAY_PUMP_PIN, HIGH);
              }
            }
            else // pumpON == false
            {
              if (valveOPEN)
              {
                valveOPEN = false;
                digitalWrite(RELAY_VALVE_PIN, HIGH);
              }
               else // valveOPEN == false
              {
                // Do nothing
              }
            }
          }
          else // tempDifference <= VALVE_CLOSED == false
          {
            // Do nothing
          }
        }
      }
    } 
    else // Temp sensore failure
    {
      if (poolTemp == DEVICE_DISCONNECTED)
      {
      Serial.println("Pool temp sensor OFFLINE!"); 
      Serial1.println("Pool temp sensor OFFLINE!"); 
      }
      if (solarTemp == DEVICE_DISCONNECTED)
      {
      Serial.println("Solar temp sensor OFFLINE!"); 
      Serial1.println("Solar temp sensor OFFLINE!"); 
      }
    }
  }
}

//********************************************************************

// Update sensor data with new readings
void readSensors()
{
  double h = dht.readHumidity();
  double t = dht.readTemperature();
  
  if isnan(h)
    h = DEVICE_DISCONNECTED;
  if isnan(t)
    t = DEVICE_DISCONNECTED;

  domeTemp = t;
  humidity = h; 
    
  sensors.requestTemperatures();
  
  poolTemp  = sensors.getTempC(swimmingPool);
  solarTemp = sensors.getTempC(solarPanel);
  thermo2   = sensors.getTempC(Thermometer2);
  thermo3   = sensors.getTempC(Thermometer3);
  tempDifference = solarTemp - poolTemp;

  // In case of sensor failure turn on audio alarm.
  if ((poolTemp == DEVICE_DISCONNECTED) || (solarTemp == DEVICE_DISCONNECTED)) // Add more sensors if deemed critical to systems operations.
    sensorFAILURE = true;
  else
  {
    sensorFAILURE = false;
    digitalWrite(SPEAKER_PIN, LOW);
  }
}

//********************************************************************

// Save sensor data for plotting graphs
void storeSensorData()
{
  static uint16_t i = 0;

  if ( i < MAX )
  {
    domeTempArray[ i ] = domeTemp;  
    swimmingPoolArray[ i ] = poolTemp;  
    solarPanelArray[ i ] = solarTemp;  
    //domeTempArray2[ i ] = domeTemp2;    
    //domeTempArray3[ i ] = domeTemp3;  
    humArray[ i ] = humidity;
    i++;
  } 
  else  // shift all data left and add latest reading to the end of the array
  {
    for ( uint16_t j = 0; j < MAX - 1; j++ )
    {
      domeTempArray[ j ] = domeTempArray[ j + 1 ];
      domeTempArray[ MAX - 1 ] = domeTemp;

      swimmingPoolArray[ j ] = swimmingPoolArray[ j + 1 ];
      swimmingPoolArray[ MAX - 1 ] = poolTemp;

      solarPanelArray[ j ] = solarPanelArray[ j + 1 ];
      solarPanelArray[ MAX - 1 ] = solarTemp;

      /*
      domeTempArray2[ j ] = domeTempArray2[ j + 1 ];
      domeTempArray2[ MAX - 1 ] = domeTemp2;
      
      domeTempArray3[ j ] = domeTempArray3[ j + 1 ];
      tempArray3[ MAX - 1 ] = temp3;
      */
      
      humArray[ j ] = humArray[ j + 1 ];
      humArray[ MAX - 1 ] = humidity; 
    }
  }
}

//********************************************************************

//Display latest sensor reading near the top of the graph when plotting the graph.
void viewSensorData(uint8_t sensor)  
{
  double data = -127;
  String sensorString = "";
  String unit = "";
  
  switch (sensor) {   
    case DOMETEMP:
      data = domeTemp;
      sensorString = F("Dome Temp:  ");
      unit = F("C");
      break;
    case POOLTEMP: 
      data = poolTemp; 
      sensorString = F("Pool Temp:  ");  
      unit = F("C");
      break;
    case SOLARTEMP:    
      data = solarTemp;
      sensorString = F("Solar Temp: ");
      unit = F("C");
      break;
    case THERMO2:    
      data = thermo2;
      sensorString = F("Thermo2:   ");
      unit = F("C");
      break;
    case THERMO3:    
      data = thermo2;
      sensorString = F("Thermo3:   ");
      unit = F("C");
      break;
    case HUMIDITY:    
      data = humidity;
      sensorString = F("Humidity:  ");
      unit = F("%");
      break;  
    case TEMPCONTROL:    
      data = tempDifference;
      sensorString = F("Temp Diff: ");
      unit = F("C");
      break;  
  }

  tft.fillRect(WIDTH / 2, 0, WIDTH / 2, 16, ILI9341_BLACK);
  tft.setCursor(WIDTH * 0.3, 0);
  if (data > DEVICE_DISCONNECTED)
  {
    tft.print(sensorString);
    if ((data > -100.0 && data <= -10.0) || (data >= 100))
      tft.print(F(" "));
    else if ((data > -10.0 && data <= -0.1) || (data >= 10))
      tft.print(F("  "));
    else if ((data >= 0.0 && data < 10))
      tft.print(F("   "));
    tft.print(data, 1);
    tft.println(unit);
  } 
  else 
  {
    tft.print(sensorString);
    tft.setTextColor(ILI9341_RED);
    tft.print(F(" ERROR!"));
    tft.setTextColor(ILI9341_YELLOW);
  }
}

//********************************************************************

// plot the graph from the stored data points
void drawSensorDataGraph(uint8_t sensor) 
{
  drawGraph();
  viewSensorData(sensor);

  uint16_t x0,y0,x1,y1,color;
  
  switch (sensor) 
  {
    case DOMETEMP:    
      graphScale(2, -1);
      for (uint16_t i = 0; i < MAX - 1; i++)
      {
        //tft.drawPixel(WIDTH - 300 + i, HEIGTH - 20 - (int)(domeTempArray[ i ]     * 5), ILI9341_ORANGE);
        x0 = WIDTH - 300 + i;
        if ((int)(domeTempArray[ i ]) == DEVICE_DISCONNECTED)
        {
          y0 = HEIGTH - 20 - 0;
          color = ILI9341_RED;      
        }
        else 
        {
          y0 = HEIGTH - 20 - (int)(domeTempArray[ i ] * 5);
          color = ILI9341_ORANGE;
        }
          
        x1 = WIDTH - 300 + i + 1;
        if ((int)(domeTempArray[ i + 1 ]) == DEVICE_DISCONNECTED)
        {
          y1 = HEIGTH - 20 - 0;
          color = ILI9341_RED;
        }
        else
        {
          y1 = HEIGTH - 20 - (int)(domeTempArray[ i + 1 ] * 5);
          color = ILI9341_ORANGE;
        }
        tft.drawLine( x0, y0, x1, y1, color);
      }
      break;
    case POOLTEMP: 
      graphScale(2, -1);
      for (uint16_t i = 0; i < MAX - 1; i++)
      {
        //tft.drawPixel(WIDTH - 300 + i, HEIGTH - 20 - (int)(swimmingPoolArray[ i ] * 5), ILI9341_ORANGE);
        x0 = WIDTH - 300 + i;
        if ((int)(swimmingPoolArray[ i ]) == DEVICE_DISCONNECTED)
        {
          y0 = HEIGTH - 20 - 0;
          color = ILI9341_RED;
        }
        else
        {
          y0 = HEIGTH - 20 - (int)(swimmingPoolArray[ i ] * 5);
          color = ILI9341_ORANGE;
        }
        
        x1 = WIDTH - 300 + i + 1;
        if ((int)(swimmingPoolArray[ i + 1 ]) == DEVICE_DISCONNECTED)
        {
          y1 = HEIGTH - 20 - 0;
          color = ILI9341_RED;
        }
        else
        {
          y1 = HEIGTH - 20 - (int)(swimmingPoolArray[ i + 1 ] * 5);
          color = ILI9341_ORANGE;
        }
        tft.drawLine( x0, y0, x1, y1, color);
      }
      break;
    case SOLARTEMP:    
      graphScale(1, SOLARTEMP);
      for (uint16_t i = 0; i < MAX - 1; i++)
      {
        //tft.drawPixel(WIDTH - 300 + i, HEIGTH - 20 - (int)(solarPanelArray[ i ]   * 2), ILI9341_ORANGE);
        x0 = WIDTH - 300 + i;
        if ((int)(solarPanelArray[ i ]) == DEVICE_DISCONNECTED)
        {
          y0 = HEIGTH - 20 - 0;
          color = ILI9341_RED;
        }
        else
        {
          y0 = HEIGTH - 20 - (int)(solarPanelArray[ i ] * 2);
          color = ILI9341_ORANGE;
        }
        
        x1 = WIDTH - 300 + i + 1;
        if ((int)(solarPanelArray[ i + 1]) == DEVICE_DISCONNECTED)
        {
          y1 = HEIGTH - 20 - 0;
          color = ILI9341_RED;
        }
        else
        {
          y1 = HEIGTH - 20 - (int)(solarPanelArray[ i + 1 ] * 2);
          color = ILI9341_ORANGE;
        }
        tft.drawLine( x0, y0, x1, y1, color);
      }
      break;
    case THERMO2:   
      //graphScale(2, -1);
      //for (uint16_t i = 0; i < MAX; i++)
      //  tft.drawPixel(WIDTH - 300 + i, HEIGTH - 20 - (int)(tempArray2[ i ] * 5), ILI9341_ORANGE);
      break;
    case THERMO3:    
      //graphScale(2, -1);
      //for (uint16_t i = 0; i < MAX; i++)
      //  tft.drawPixel(WIDTH - 300 + i, HEIGTH - 20 - (int)(tempArray3[ i ] * 5), ILI9341_ORANGE);
      break;
    case HUMIDITY:
      graphScale(1, HUMIDITY);
      for (uint16_t i = 0; i < MAX - 1; i++)
      {
        //tft.drawPixel(WIDTH - 300 + i, HEIGTH - 20 - (int)(humArray[ i ] * 2), ILI9341_CYAN);
        x0 = WIDTH - 300 + i;
        if ((int)(humArray[ i ]) == DEVICE_DISCONNECTED)
        {
          y0 = HEIGTH - 20 - 0;
          color = ILI9341_RED;
        }
        else
        {
          y0 = HEIGTH - 20 - (int)(humArray[ i ] * 2);
          color = ILI9341_CYAN;
        }
        
        x1 = WIDTH - 300 + i + 1;
        if ((int)(humArray[ i + 1]) == DEVICE_DISCONNECTED)
        {
          y1 = HEIGTH - 20 - 0;
          color = ILI9341_RED;
        }
        else
        {
          y1 = HEIGTH - 20 - (int)(humArray[ i + 1 ] * 2);
          color = ILI9341_CYAN;
        }
        tft.drawLine( x0, y0, x1, y1, color);
      }
      break;
    case TEMPCONTROL:    
      graphScale(1, TEMPCONTROL);
      for (uint16_t i = 0; i < MAX - 1; i++)
      {
        if ((int)(swimmingPoolArray[ i ] * 2) != (int)(solarPanelArray[ i ] * 2))
        {
          //tft.drawPixel(WIDTH - 300 + i, HEIGTH - 20 - (int)(swimmingPoolArray[ i ] * 2), ILI9341_CYAN);
          x0 = WIDTH - 300 + i;
          if ((int)(swimmingPoolArray[ i ]) == DEVICE_DISCONNECTED)
          {
            y0 = HEIGTH - 20 - 0;
            color = ILI9341_RED;
          }
          else
          {
            y0 = HEIGTH - 20 - (int)(swimmingPoolArray[ i ] * 2);
            color = ILI9341_CYAN;
          }
        
          x1 = WIDTH - 300 + i + 1;
          if ((int)(swimmingPoolArray[ i + 1 ]) == DEVICE_DISCONNECTED)
          {
            y1 = HEIGTH - 20 - 0;
            color = ILI9341_RED;
          }
          else
          {
            y1 = HEIGTH - 20 - (int)(swimmingPoolArray[ i + 1 ] * 2);
            color = ILI9341_CYAN;
          }
          tft.drawLine( x0, y0, x1, y1, color);
          
          //tft.drawPixel(WIDTH - 300 + i, HEIGTH - 20 - (int)(solarPanelArray[ i ]   * 2), ILI9341_ORANGE);
          x0 = WIDTH - 300 + i;
          if ((int)(solarPanelArray[ i ]) == DEVICE_DISCONNECTED)
          {
            y0 = HEIGTH - 20 - 0;
            color = ILI9341_RED;
          }
          else
          {
            y0 = HEIGTH - 20 - (int)(solarPanelArray[ i ] * 2);
            color = ILI9341_ORANGE;
          }
        
          x1 = WIDTH - 300 + i + 1;
          if ((int)(solarPanelArray[ i + 1]) == DEVICE_DISCONNECTED)
          {
            y1 = HEIGTH - 20 - 0;
            color = ILI9341_RED;
          }
          else
          {
            y1 = HEIGTH - 20 - (int)(solarPanelArray[ i + 1 ] * 2);
            color = ILI9341_ORANGE;
          }
          tft.drawLine( x0, y0, x1, y1, color);
        } 
        else 
        { //Both values are the same, resulting in trying to paint the same pixel
          //tft.drawPixel(WIDTH - 300 + i, HEIGTH - 20 - (int)(solarPanelArray[ i ]   * 2), ILI9341_GREEN); 
          x0 = WIDTH - 300 + i;
          if ((int)(solarPanelArray[ i ]) == DEVICE_DISCONNECTED)
          {
            y0 = HEIGTH - 20 - 0;
            color = ILI9341_RED;
          }
          else
          {
            y0 = HEIGTH - 20 - (int)(solarPanelArray[ i ] * 2);
            color = ILI9341_GREEN;
          }
          
          x1 = WIDTH - 300 + i + 1;
          if ((int)(solarPanelArray[ i + 1 ]) == DEVICE_DISCONNECTED)
          {
            y1 = HEIGTH - 20 - 0;
            color = ILI9341_RED;
          }
          else
          {
            y1 = HEIGTH - 20 - (int)(solarPanelArray[ i + 1 ] * 2);
            color = ILI9341_GREEN;
          }
          tft.drawLine( x0, y0, x1, y1, color);
        }
      }
      break;  
  } 
}

//********************************************************************

// Draw the empty graph template 
void drawGraph()  // Draw the axis and dots.
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.write(24); 
  
  tft.setCursor(WIDTH - 20, HEIGTH - 16);
  tft.print(F("h")); 
  tft.setCursor(WIDTH - 10, HEIGTH - 16);
  tft.write(26); 
  
  tft.setTextSize(1);
  tft.setCursor(12,  HEIGTH - 16);
  tft.print(F("0"));
  tft.setCursor(89,  HEIGTH - 16);
  tft.print(F("6")); 
  tft.setCursor(158, HEIGTH - 16);
  tft.print(F("12")); 
  tft.setCursor(230, HEIGTH - 16);
  tft.print(F("16")); 
  tft.setTextSize(2);
  
  for (uint16_t i = 20; i <= 308; i = i + 12)  
    for (uint16_t j = 20; j < HEIGTH; j = j + 25)
      tft.drawPixel(i, j, ILI9341_WHITE); 
  
  tft.drawFastVLine(19,  20, 201, ILI9341_WHITE);
  tft.drawFastHLine(19, 221, 289, ILI9341_WHITE);
}

//********************************************************************

//currently two different scales for Y axis, 100 and 40. And print data unit.
void graphScale( uint8_t scale, int8_t unit) 
{
  switch (scale){
    case 1: // data values between 0 and 100
      if (unit == HUMIDITY)
      {
        tft.setCursor(0, 16);
        tft.print(F("H")); 
      }
      if (unit == TEMPCONTROL || unit == SOLARTEMP)
      {
        tft.setCursor(0, 16);
        tft.print(F("T"));   
      }
      tft.setTextSize(1);
      tft.setCursor(5, 170 - 4);
      tft.print(F("25")); 
      tft.setCursor(5, 120 - 4);
      tft.print(F("50")); 
      tft.setCursor(5,  70 - 4);
      tft.print(F("75")); 
      tft.setTextSize(2);
      break;
    case 2: // data values between 0 and 40
      tft.setCursor(0, 16);
      tft.print(F("T")); 
  
      tft.setTextSize(1);
      tft.setCursor(5, 170 - 4);
      tft.print(F("10")); 
      tft.setCursor(5, 120 - 4);
      tft.print(F("20")); 
      tft.setCursor(5,  70 - 4);
      tft.print(F("30")); 
      tft.setTextSize(2);
      break;
  }
}

//********************************************************************

// display system status and latest sensor data
void printSystemStatus()
{
  clearTFTScreen();
  sendToAll(F("System is:        "));
  if (systemON)
  {
    tft.setTextColor(ILI9341_GREEN);  
    sendToAll(F("ENABLED\n\n"));
  } else {
    tft.setTextColor(ILI9341_RED);
    sendToAll(F("DISABLED\n"));
  }
  tft.setTextColor(ILI9341_YELLOW);
 
  sendToAll(F("Dome Air Temp:    "));
  sendToAll(domeTemp);
  sendToAll(F("C\n"));
  sendToAll(F("Pool Temp:        "));
  sendToAll(poolTemp);
  sendToAll(F("C\n"));
  sendToAll(F("Solar Panel Temp: "));
  sendToAll(solarTemp);
  sendToAll(F("C\n"));
  sendToAll(F("Temp Sensor3:     "));
  sendToAll(thermo2);
  sendToAll(F("C\n"));
  sendToAll(F("Temp Sensor4:     "));
  sendToAll(thermo3);
  sendToAll(F("C\n"));
  sendToAll(F("Dome Humidity:    "));
  sendToAll(humidity);
  sendToAll(F("%\n"));
  sendToAll(F("Temp difference:  "));
  sendToAll(tempDifference);
  sendToAll(F("C\n"));
  sendToAll(F("Solar valve:      "));
  if (valveOPEN)
  {
    tft.setTextColor(ILI9341_GREEN);  
    sendToAll(F("OPEN\n"));
  } else {
    tft.setTextColor(ILI9341_RED);
    sendToAll(F("CLOSED\n"));
  }
  tft.setTextColor(ILI9341_YELLOW);
  sendToAll(F("Filter pump:      "));
  if (pumpON)
  {
    tft.setTextColor(ILI9341_GREEN);  
    sendToAll(F("ON\n"));
  } else {
    tft.setTextColor(ILI9341_RED);
    sendToAll(F("OFF\n"));
  }
  tft.setTextColor(ILI9341_YELLOW);
  sendToAll(F("Pump override:    "));
  if (pumpOVERRIDE)
  {
    tft.setTextColor(ILI9341_GREEN);  
    sendToAll(F("ON\n"));
    tft.setTextColor(ILI9341_YELLOW); 
    sendToAll(F("Pump time left:   "));
    sendToAll((PUMP_OVERRIDE_INTERVAL - (millis() - pumpOverrideTimer)) / 60000.0);
    sendToAll(F("Min\n\n"));
    
  } else {
    tft.setTextColor(ILI9341_RED);
    sendToAll(F("OFF\n\n"));
  }
  tft.setTextColor(ILI9341_YELLOW);
}

//********************************************************************

void printMenu()
{
  clearTFTScreen();
  
  sendToAll(F("MENU OPTIONS:\n"));  
  sendToAll(F("a - Dome air temp\n"));
  sendToAll(F("b - Swimming pool temp\n"));
  sendToAll(F("c - Solar panel temp\n"));
  sendToAll(F("d - Dome humidity\n"));
  sendToAll(F("e - Temp difference\n"));
  sendToAll(F("x - system status\n"));
  sendToAll(F("1 - Turn system ON\n"));
  sendToAll(F("0 - Turn system OFF\n"));
  sendToAll(F("? - print menu\n\n")); 
}

//********************************************************************

void sendToAll(String sendDataString)
{
  Serial.print(sendDataString);
  Serial1.print(sendDataString);
  tft.print(sendDataString);
}

//********************************************************************

void sendToAll(double sendDataDouble)
{
  if ((sendDataDouble > -100.0 && sendDataDouble <= -10.0) || (sendDataDouble >= 100))
  {
    Serial.print(F(" "));
    Serial1.print(F(" "));
    tft.print(F(" "));
  }
  else if ((sendDataDouble > -10.0 && sendDataDouble <= -0.1) || (sendDataDouble >= 10))
  {
    Serial.print(F("  "));
    Serial1.print(F("  "));
    tft.print(F("  "));
  }
  else if ((sendDataDouble >= 0.0 && sendDataDouble < 10))
  {
    Serial.print(F("   "));
    Serial1.print(F("   "));
    tft.print(F("   "));
  }
  
  Serial.print(sendDataDouble);
  Serial1.print(sendDataDouble);
  if (sendDataDouble != DEVICE_DISCONNECTED)
    tft.setTextColor(ILI9341_GREEN); 
  else
    tft.setTextColor(ILI9341_RED);
  tft.print(sendDataDouble,1);
  tft.setTextColor(ILI9341_YELLOW);
}

//********************************************************************

void clearTFTScreen()
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
}

//********************************************************************

