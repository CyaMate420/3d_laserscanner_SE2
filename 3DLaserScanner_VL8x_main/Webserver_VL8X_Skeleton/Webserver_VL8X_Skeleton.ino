
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
int SCAN_status = 0;
int SCAN_data = 0;

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
  SCAN_data = 1; //zum Test -> entspricht Scan ist fertig
  server.send(200, "text/html", SendHTML_Download(SCAN_data)); //--> Neue SendHTML_download nötig 
} 

/*void handle_scanFinished() {
  .....
} */

/*void handle_scanProg() {
  var SCAN_prog init & nutzung usw....
} */

//void handle_.....

/*void handle_led1on() {
  LED1status = HIGH;
  Serial.println("GPIO4 Status: ON");
  server.send(200, "text/html", SendHTML(...));  //200=OK
}*/

/*void handle_led1off() {
  LED1status = LOW;
  Serial.println("GPIO4 Status: OFF");
  server.send(200, "text/html", SendHTML(....)); 
}*/

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String SendHTML_Download (uint8_t scanData) {
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";   //für webbrowser
  ptr +="<title>Download Measurement Data</title>\n";  //title page w/ <title>-tag

  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n"; //Desgin & Layout
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h2 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";  //Layout

  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n"; //Button design allgemein
  ptr +=".button-on {background-color: #3498db;}\n"; //button aussehen
  ptr +=".button-on:active {background-color: #2980b9;}\n"; //button wenn geklickt

  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n"; //Layout
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="</body>\n";

  ptr +="<h1>3D-LaserScanner by mst2.se2 students</h1>\n";  //Webpage headings..
   ptr +="<h2>Hier Matlab-Code ausgabe:</h2>\n"; //AUSGABE MATLAB-CODE

  if(scanData)
  {ptr +="<p>Generated</p><a class=\"button button-on\" href=\"/download\">New Scan?</a>\n";} //"download" mit scan_start oder progress Seite ersetzen 
  else
  {ptr +="<p>Generate MatLab-Code from completed Measurement: ON</p><a class=\"button button-on\" href=\"/start\">ON</a>\n";}

  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}


String SendHTML(uint8_t scanStat){ //generiert Seite für entsprechenden handle-> für verschiedene Zwecke neuen SEndHTML, download data and progress displaying
  String ptr = "<!DOCTYPE html> <html>\n";  //indicates that  HTML-Code will be sent in the follwing  
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";   //für webbrowser
  ptr +="<title>Scanner measurements</title>\n";  //title page w/ <title>-tag

  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n"; //Desgin Einstellungen Webpage
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h2 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";  //Layout
  
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n"; //Button design allgemein
  ptr +=".button-on {background-color: #3498db;}\n"; //button aussehen
  ptr +=".button-on:active {background-color: #2980b9;}\n"; //button wenn geklickt

  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n"; //Layout

  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="</body>\n";

  ptr +="<h1>3D-LaserScanner by mst2.se2 students</h1>\n";  //Webpage headings..
   ptr +="<h2>Web Application Using Access Point(AP) Mode From ESP32 WROOM</h2>\n";
    ptr +="<h3>Current Scan Status:</h3>\n";
  
  if(scanStat) //Erst so anzeigen wenn Scan ab geschlossen ist -> momentan nur Test
  {ptr +="<p>Generate MatLab-Code from completed Measurement: Ready</p><a class=\"button button-on\" href=\"/download\">Ready</a>\n";} //diesen teil erst nachdem SCAN_prog fertig ist ->ANPASSEN
  else
  {ptr +="<p>LED1 Status: OFF</p><a class=\"button button-off\" href=\"/led1on\">ON</a>\n";} //->ANPASSEN

  //if(scanProg)
  //{//ptr +="<p>LED2 Status: ON</p><a class=\"button button-off\" href=\"/led2off\">OFF</a>\n";} -> ANPASSEN
  //else
  //{//ptr +="<p>LED2 Status: OFF</p><a class=\"button button-on\" href=\"/led2on\">ON</a>\n";} -> ANPASSEN

  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}
