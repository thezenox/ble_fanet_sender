#include <Arduino.h>

#define FANET_MAN 0xBD // manufacturer
#define FANET_ID 0x0001 // without manufacturer
char * pilot_name = "YourName";

#include <Wire.h>
#include <bluefruit.h>
#include "TinyGPSPlus.h"
#include <FanetLora.h>
#include <Adafruit_DotStar.h>

#define PIN_LORA_MISO MISO  // P0.15
#define PIN_LORA_MOSI MOSI  // P0.20
#define PIN_LORA_SCK SCK    // P0.13

#define PIN_LORA_CS A5      // P0.03
#define PIN_LORA_RESET A4   // P0.02
#define PIN_LORA_DIO0 2     // P1.02


FanetLora fanet;
long frequency = FREQUENCY868;
uint8_t radioChip = RADIO_SX1276;
uint8_t rfMode = 0x03;
MacAddr macaddr;

TinyGPSPlus tgps;
BLEUart bleuart; // Peripheral uart service

// Ultrabip BLE service & characteristics for UART NMEA data
BLEClientService        flight_data_service(0xFFE0);
BLEClientCharacteristic flight_data_characteristic(0xFFE1);

#define PIN_DOTSTAR_CLK 6  // P1.09
#define PIN_DOTSTAR_DATA 8 // P0.08
#define LED_COUNT 1
Adafruit_DotStar strip(LED_COUNT, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BRG);

bool connected = false; // if a smartphone is connected for data forwarding
float climb = 0; // filtered climb
uint32_t last_ble_package = 0; // millis() of last package

/*------------------------------------------------------------------*/
/* Peripheral
 *------------------------------------------------------------------*/
void prph_connect_callback(uint16_t conn_handle){
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));

  Serial.print("[Prph] Connected to ");
  Serial.println(peer_name);
  connected = true;
}

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason){


  Serial.print("Prph Disconnected, reason = 0x"); Serial.println(reason, HEX);
  connected = false;
}

// Forward data to peer connected
void forward_data(uint8_t *data, int len){
  if ( connected || bleuart.notifyEnabled() ){
    bleuart.write(data, len);
  }else{
    // Serial.println("[Cent] Peripheral role not connected");
  }  
}

#define BUFFERLEN 450
void parse_data(uint8_t *data, int len){
  static uint8_t d[BUFFERLEN];
  static int d_pos = 0;

  last_ble_package = millis()-2; // -2ms to avoid strange things that happen without it...

if(d_pos+len > BUFFERLEN){
  d_pos =0;
  Serial.println("overflow");
  return;
}
// parse GPS NMEA string
  for(int i=0; i< len; i++){
    tgps.encode(data[i]);
  }
// pare varimeter NMEA string
  const char* LXWP0 = "$LXWP0";
  memcpy(&d[d_pos], data, len);
  d_pos += len;

  if (d[d_pos-1] == '\n'){ // if line is complete
    // just get vario
    int comma = 0;
    char buff [10] = "";
    int buffpos =0;
    float vario = -100;
    if(memcmp(d, LXWP0,6 )==0){
      for( int i=0; i< d_pos; i++){
        if(d[i] == ','){
          comma ++;
          if(comma == 5) {
            climb = atof(buff);
          }
          buffpos = 0;
        } else {
          buff[buffpos]=d[i];
          buffpos++;
        }
      }
      if(vario !=-100){ // if found add to lowpass filter
        climb += vario*0.1;
        climb = climb/1.1;
      }
    }
    // set led green til we get valid gps data
    if (!tgps.location.isValid()){
      strip.setPixelColor(0,50,0,0);
      strip.show();
      Serial.write(d,d_pos);
    }
    //
    // Forward data from our peripheral to Mobile
    forward_data(d, d_pos);
    d_pos = 0;
  }
}

void prph_bleuart_rx_callback(uint16_t conn_handle){
  (void) conn_handle;
  // Forward data from Mobile to our peripheral
  char str[20+1] = { 0 };
  bleuart.read(str, 20);

  Serial.print("[Prph] RX: ");
  Serial.println(str);  

// uart tx is desabled
  //if ( clientUart.discovered() )
  //{
  //  clientUart.print(str);
  //}else
  //{
  //  bleuart.println("[Prph] Central role not connected");
  //}
}

void startAdv(void){
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 32);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(0);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void scan_callback(ble_gap_evt_adv_report_t* report){
  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  Serial.print("\n");

  // adress filtering
  if(report->peer_addr.addr[5] == 0x57){ // <-- change for your device
    Serial.print("Connecting\n");
    Bluefruit.Central.connect(report);
  }
  Bluefruit.Scanner.resume();
}

void connect_callback(uint16_t conn_handle){
  Serial.println("Connected");
  //startAdv(); // start advertising BLE uart sevice if connected to variometer --> uncomment for "relay" mode
  Serial.print("Discovering flight data Service ... ");
  if ( !flight_data_service.discover(conn_handle) ){
    Serial.println("Found NONE");
    Bluefruit.disconnect(conn_handle); // disconnect since we couldn't find flight data service
    return;
  }

  Serial.print("Discovering flightdata characteristic ... ");
  if ( !flight_data_characteristic.discover() ){
    Serial.println("not found !!!");  
    Serial.println("flightdata characteristic is mandatory but not found");
    Bluefruit.disconnect(conn_handle);
    return;
  }

  if ( flight_data_characteristic.enableNotify() ){
    Serial.println("Ready to receive flightdata NMEA values");
  }else{
    Serial.println("Couldn't enable notify for flightdata. Increase DEBUG LEVEL for troubleshooting");
  }
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason){
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}


void flightdata_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len){
  parse_data(data, len);
}

// ################# FANET functions ####################

void send_name(){
  fanet.setRFMode(rfMode);
  fmac.setRegion(tgps.location.lat(),tgps.location.lng());
  fanet.sendName(pilot_name);
}

void send_tracking(){

    rfMode = 0x0B; // Enable FANET & FLARM
    FanetLora::trackingData data;
    data.timestamp = 0;
    data.type =  0x11; //tracking-type (11... online tracking 7X .... ground tracking)
    data.lat = tgps.location.lat(); //latitude
    data.lon = tgps.location.lng(); //longitude
    data.altitude = tgps.altitude.meters(); //altitude [m]
    data.aircraftType = FanetLora::aircraft_t::paraglider; //
    data.speed = tgps.speed.kmph(); //km/h
    data.climb = climb; //m/s
    data.heading = tgps.course.deg(); //deg
    data.OnlineTracking = true;

    fanet.setRFMode(rfMode); // wake from deepsleep
    fmac.setRegion(tgps.location.lat(),tgps.location.lng()); // set frequency
    if(data.speed < 6 && abs(data.climb) <1){
      fanet.sendGroundTracking(&data);
    } else {
      fanet.writeMsgType1(&data);
    }
}



void setup(){
  Serial.begin(115200);
 // while ( !Serial ) delay(10);   // for nrf52840 with native usb

  strip.begin();
  strip.setBrightness(80);
  strip.show(); // turn led off

  Serial.println("Bluefruit52 FANET BLE Sender");
  Serial.println("--------------------------------------\n");

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(1, 1);
  Bluefruit.setName("Breezedude BLE FANET");

  Bluefruit.autoConnLed(false); // disable BLE LED to save power
  Bluefruit.setConnLedInterval(250); // Increase Blink rate to different from PrPh advertising mode

    // Configure and Start BLE Uart Service
  bleuart.begin();
  bleuart.bufferTXD(true);
  bleuart.setRxCallback(prph_bleuart_rx_callback);

  // Initialize HRM client
  flight_data_service.begin();

  // set up callback for receiving measurement
  flight_data_characteristic.setNotifyCallback(flightdata_notify_callback);
  flight_data_characteristic.begin();

  // Callbacks for Peripheral (=Smartphone)
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);
  Bluefruit.Periph.setConnSlaveLatency(1);
  Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);

  // Callbacks for Central (= Variometer)
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);
  //Bluefruit.Central.setConnIntervalMS(10,1000);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Filter only accept HRM service
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  //Bluefruit.Scanner.filterUuid(flight_data_service.uuid); // not working, maybe service is not advertised
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds

  fanet.autobroadcast = false;
  fanet.setRFMode(rfMode);
  macaddr = MacAddr(FANET_MAN, FANET_ID);
  fanet.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS,PIN_LORA_RESET, PIN_LORA_DIO0,-1,frequency,14,radioChip, macaddr);

// Set LED red = not connected
  strip.setPixelColor(0,0,50,0);
  strip.show();
}

void loop(){
  static uint32_t last_tracking_message = 0;
  static uint32_t last_name_message =0;
  static bool sending = false;
  fanet.run();

  if(millis()- last_tracking_message > 5*1000){
    if (tgps.location.isValid() && tgps.hdop.value()<2.1 && tgps.location.age() < 2000){
    last_tracking_message = millis();
    Serial.printf("Millis: %lu Climb: %0.2f GPS: %3.6f, %3.6f Speed: %0.2f Course: %0.2f Alt: %0.2f HDOP: %0.2f\n", millis(), climb, tgps.location.lat(), tgps.location.lng(), tgps.speed.kmph(), tgps.course.deg(), tgps.altitude.meters(), tgps.hdop.value());
    send_tracking();
    sending = true;
    strip.setPixelColor(0,0,0,255);
    strip.show();
    }
  }

  if(!sending && millis()- last_name_message > 4*60*1000){
    if (tgps.location.isValid() && tgps.location.age() < 2000){
      
    last_name_message = millis();
    send_name();
    Serial.printf("%lu Sending Name\n", millis());
    sending = true;
    }
  }
  // wait for fanet package to complete
  if(sending && fanet.checkSendComplete()){
    sending = false;
    Serial.printf("%lu Send complete\n", millis());
    fanet.end(); // set RFM95 to sleep
    strip.setPixelColor(0,0,0,0);
    strip.show();
  }
  
  // power off if variometer is disconnected for > 1 min (maybe someone forgot to turn off this device)
  if(millis()- last_ble_package > 60000){
    Serial.println(millis()- last_ble_package);
    Serial.printf("%lu Power off\n", millis());
    Serial.flush();
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    sd_power_system_off(); // this function puts the whole nRF52 to deep sleep (no Bluetooth).  If no sense pins are setup (or other hardware interrupts), the nrf52 will not wake up.
  }
}


/*
  // $LXWP0,logger_stored, airspeed, airaltitude,v1[0],v1[1],v1[2],v1[3],v1[4],v1[5], hdg, windspeed*CS<CR><LF>
  //
  // 0 loger_stored : [Y|N] (not used in LX1600)
  // 1 IAS [km/h] ----> Condor uses TAS!
  // 2 baroaltitude [m]
  // 3-8 vario values [m/s] (last 6 measurements in last second)
  // 9 heading of plane (not used in LX1600)
  // 10 windcourse [deg] (not used in LX1600)
  // 11 windspeed [km/h] (not used in LX1600)
  //
  // e.g.:
  // $LXWP0,Y,222.3,1665.5,1.71,,,,,,239,174,10.1
  */