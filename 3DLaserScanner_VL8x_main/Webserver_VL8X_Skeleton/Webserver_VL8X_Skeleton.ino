#include <WiFi.h>
#include <WebServer.h>

/*______________Setup Server on ESP_______________________________________________________ */

//_______________Define-Constants_______________________________________
//********************************************************************+
const char* ssid = "LaserScanner";    //Server name/SSID
const char* password = "3desp32";     //Server password
IPAddress local_ip(192,168,1,1);      //Sever default ip and stuff
IPAddress gateway(192,168,1,1); 
IPAddress subnet(255,255,255,0); 
WebServer server(80);                 //default HTTP-Port

//variables that will change:
//
// e.g.: 
//


//Adafruit_VL6180X vl = Adafruit_VL6180X();  startet Lib-methoden und funktionen


//_________________________________SETUP_________________________________
//***********************************************************************
void setup() {
  Serial.begin(115200);
  //***********Setup Sensor etc..

  //*******************************

  //***setup Soft Access Point für Webserver-Host auf ESP -> into: softAP_init()
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
  //***************************

  //**********Server_handles aufrufen-> into: ON_2_handles()
  server.on("/", handle_OnConnect);     //request nach /root abfangen; führt zu Start der StartUp/Kalibrierung im Code
  //server.on("/start", handle_startScan); ->zum start scan
  server.on("/download", handle_downloadData);  //-> measurements herunterladen
  //server.on("/led2on", handle_led1on);
  //server.on("/led2off", handle_led1off);
  server.onNotFound(handle_NotFound); //ungültige URL liefert E404
  //****************************

  server.begin();
  Serial.println("HTTP server started");
}


//_________________________________LOOP_________________________________
//**********************************************************************
void loop() {
  server.handleClient();              //http-Anfragen ausführen mit entsprechenden handles

  //HIER: Scan code + entsprechende Ansteuerung der Motoren, sowie daten speichern usw...
}


//______________________Funktionen-Deklaration_________________________________
//******************************************************************************
void handle_OnConnect() {
  //HIER: Start conditions
  Serial.println("Start WebServer zur Kommunikation mit 3D-Laserscanner");
  //SCAN_status from button click Pin???
  SCAN_status = 1;
  //SCAN_prog from number of stepps
  server.send(200, "text/html", SendHTML(SCAN_status)); // SendHTML um Seite für entsprechende Zustände zu generieren
  //Hier Ausgabe an Bildschirm mit Startup und Initialisierung..
}

/*void handle_scanStatus() {
 var SCAN_status init & nutzung usw.....
}*/

void handle_downloadData() {
  //var SCAN_data init & nutzung usw... mit abfrage Messdaten und Ausgabe
  SCAN_data = 1;
  server.send(200, "text/html", SendHTML(SCAN_Data)); --> Neue SendHTML_download nötig 
} 

/*void handle_scanFinished() {
  .....
} */

/*void handle_scanProg() {
  var SCAN_prog init & nutzung usw....
} */

//void handle_.....
