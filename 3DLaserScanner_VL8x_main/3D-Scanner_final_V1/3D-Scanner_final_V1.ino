//formatierte 3D-Scanner .ino

/***************************************************************************************************************/
/*                                                INCLUDES                                                     */
/***************************************************************************************************************/

#include <Adafruit_VL6180X.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <WiFi.h>
#include <WebServer.h>
//#include <AccelStepper.h>               // no need for it due to just toggle GPIO pins ? 



/***************************************************************************************************************/
/*                                                DEFINES                                                      */
/***************************************************************************************************************/
/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv GPIO-PINS vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

//Button
#define BUTTON  5         // Button pin

//Stepper motor 
#define STEP_PIN_1 32      // Motor1 step pin  (disc)
#define DIR_PIN_1  33      // Motor1 direction pin (disc)
#define STEP_PIN_2 26      // Motor2 step pin  (threaded rod)
#define DIR_PIN_2  27      // Motor2 direction pin (threaded rod)

//LCD 
#define RS  19            // LCD RS pin 
#define EN  23            // LCD EN pin
#define D4  18            // LCD D4 pin
#define D5  17            // LCD D5 pin
#define D6  16            // LCD D6 pin
#define D7  15            // LCD D7 pin

//WebServer defines
const char* ssid = "LaserScanner";    //Server name/SSID
const char* password = "3desp32";     //Server password
IPAddress local_ip(192,168,1,1);      //Sever default ip and stuff
IPAddress gateway(192,168,1,1); 
IPAddress subnet(255,255,255,0); 
WebServer server(80);                 //default HTTP-Port

/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

//LCD defines
#define LCD_COLUMNS   16          // amount of columns LCD-screen
#define LCD_ROWS      2           // amount of rows LCD-screen

//motor defines 
#define CLOCKWISE           1
#define COUNTERCLOCKWISE    0
#define UP                  1           // ausprobieren 
#define DOWN                0           // ausprobieren 

// customazable defines
#define HEIGHT_STEP_SIZE            4           // 2mm height difference between two measured levels
#define MAX_HEIGHT                  40          // 50mm difference between plate and top measure level
#define MEASUREMENTS_PER_POINT      10          // amount of measurement repititions per point

// fixed defines
#define ANGLE_STEP_SIZE_MUL10       18          // 1,8° per step (*10 -> due to improved performance with operating with integer)
#define THREADED_ROD_PITCH          8           // in mm/360° (i.e. 8mm/360°)
#define REFERENCE_DISTANCE_MUL10    750         // the distance between sensor and center of the rotating plate
#define MAX_DISTANCE                1500        // the maximum distance between object and sensor -> above is error 
#define MEASUREMENTS_PER_ROTATION   (3600/ANGLE_STEP_SIZE_MUL10)         
#define MEASURED_LEVELS             (MAX_HEIGHT/HEIGHT_STEP_SIZE)
#define AMOUNT_MEASURED_POINTS      (MEASURED_LEVELS*MEASUREMENTS_PER_ROTATION) 

//error-code defines
#define NO_ERROR                0
#define NO_SENSOR_FOUND         99
#define ARRAY_OVERFLOW_ERROR    98



/***************************************************************************************************************/
/*                                                STRUCTS                                                      */
/***************************************************************************************************************/

typedef struct scan_handle{                                     
  uint16_t alpha_mul_10[MEASUREMENTS_PER_ROTATION*MEASURED_LEVELS];          // alpha*10  -> reduce information loss due to integer calculation 
  uint16_t radius_mul_10[MEASUREMENTS_PER_ROTATION*MEASURED_LEVELS];         // radius*10 -> reduce information loss due to integer calculation 
  uint8_t height[MEASUREMENTS_PER_ROTATION*MEASURED_LEVELS];                 // height in mm 
  uint8_t error;                                                             // saves Error Code when an error occurs. (NO_ERROR by default).
  uint    index;                                                             // Information about the current processed array data
}scan_handle;



typedef struct motor_handle{
  uint8_t step_pin;
  uint8_t dir_pin;
}motor_handle;



/***************************************************************************************************************/
/*                                                PROTOTYPES                                                   */
/***************************************************************************************************************/

void scan_prepare(scan_handle* xy);
void scan_wait_for_start(scan_handle* xy);
void scan_run(scan_handle* xy);
void scan_finish(scan_handle* xy);
void scan_error(scan_handle* xy);
uint8_t button_state(void);
uint8_t server_button_state(void);
void measure_one_rotation(scan_handle* xy, uint8_t current_height);
void step_motor(motor_handle* motor, uint8_t direction, uint8_t amount_of_steps);
void display_scan_progress(uint8_t current_value, uint8_t max_value);
void check_scan_error(scan_handle* xy);
void transmit_matlab_code(scan_handle* xy);
void reset_scan_handle(scan_handle* xy);


/***************************************************************************************************************/
/*                                                VARIABLES                                                    */
/***************************************************************************************************************/

scan_handle current_scan = {{0}, {0}, {0}, NO_ERROR, 0};   // maybe init in loop ? -> faster access ?  
motor_handle disc_motor = {STEP_PIN_1, DIR_PIN_1};
motor_handle threaded_rod_motor = {STEP_PIN_2, DIR_PIN_2};
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);                // not sure if in setup or here ? -> test it 
Adafruit_VL6180X vl = Adafruit_VL6180X();

typedef enum scanState_from_server {ROOT_PREPARING, READY, SCAN, DOWNLOAD} scan_state;    //different phases of scanning routine
scan_state serverState = (scan_state) -1;  //type-cast muss                               //Global var for matching current scannin state from server
                                    // '-> mit unzulässigem state initialisiert zum Starten des Scan-Vorgangs durch Verbindung zum Webserver

/***************************************************************************************************************/
/*                                                SETUP                                                        */
/***************************************************************************************************************/

void setup() {                // evtl in Funktionen auslagern ? serial_init(), lcd_init() .....

  //serial
  Wire.begin();
  Serial.begin(115200);

  while (!Serial) {           // wait for serial port to open on native usb devices
    delay(1);
  }

  //LCD
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  lcd.clear();
  lcd.print("Initialisieren...");
  delay(2000);

  //Stepper motors
  pinMode(STEP_PIN_1, OUTPUT);  
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);

  //Button
  pinMode(BUTTON, INPUT);

   //**********Setup Soft Access Point für Webserver-Host auf ESP -> into: softAP_init()
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  //**********Server_handles aufrufen-> into: ON_2_handles()?
  server.on("/", handle_OnConnect);           //Landing-Page und INfos über Preparing zustand
  server.on("/ready", handle_Ready4Scan);     //after finishing Scanner preparing-> start scan here with serverButton (Start Scan) click oder hardwareButton
  server.on("/running", handle_RunningScan);   //showing Scan progress and generating button to copy MatLab-Code from next page
  server.on("/download", handle_DownloadData);  //displaying MatLab Code with measurements to "download" (copy-paste) - also: restart whole procedure
  server.onNotFound(handle_NotFound);           //ungültige URL liefert E404
  server.begin();
  Serial.println("HTTP server started");


  //VL6180x
  if (! vl.begin()) {
    current_scan.error = NO_SENSOR_FOUND;               // error code for not founding sensor
  }

  lcd.clear();
  lcd.print("Please Log into Webserver..");
  delay(2000);

}



/***************************************************************************************************************/
/*                                                LOOP                                                         */
/***************************************************************************************************************/

void loop() {
  /*  while(1){
    uint tmp_data = 0;
    for(uint16_t measurement = 1; measurement <= 200; measurement++){             // calculate average out of xxx repititions
      tmp_data += vl.readRange();                                
      //xy->error = vl.readRangeStatus();
    }    
    tmp_data = tmp_data/200;
    Serial.println(tmp_data);
    }*/


  server.handleClient();                          //start specific http-requests --> startet bei /root 

  switch (serverState) {
    case ROOT_PREPARING: //--> hier nach Button click auf /(root)
      check_scan_error(&current_scan);
      scan_prepare(&current_scan);               // LCD: "Initialisieren: Bitte noch kein Gegenstand auf Platte stellen !"    && setzt serverState nach Abschluss auf READY
      serverState = READY; //nah Abschluss der Calibration -> Zustand weiterschalten 
    break;

    case READY: 
      handle_Ready4Scan();
      scan_wait_for_start(&current_scan);        // LCD: "Taste zum starten Drücken" && loop for button_state     && server_ready4Scan -> per taster oder ServerButton zu:
    break;

    case SCAN:
      scan_run(&current_scan);                   // LCD: "Scanning... (PROGRESS BAR ?)" && MOTOR1/2 Drehen + VL6180x messen und in data speichern      && server_runningScan -> sobald abgeschlossen per ServerButton zu:
      serverState = DOWNLOAD;
    break;

    case DOWNLOAD:
      scan_finish(&current_scan);                // LCD: "Scan beendet. Übertrage Daten..."  -> scan_transmit ?      && server_download ->per button zu on connect zum neustarten
    break;

    default:  //Webserver nicht aufgerufen oder noch kein Start der Calibration
      lcd.clear();
      lcd.print("Waiting for Server input..");
      Serial.println("default CASE");
      delay(2500);
    break;
  }     
}



/***************************************************************************************************************/
/*                                                FUNCTIONS                                                    */
/***************************************************************************************************************/
/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv loop-state functions vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

void scan_prepare(scan_handle* xy){           // drive motor in correct position and calibrate reference distance init variables/data 
  check_scan_error(xy);
  //PLACEHOLDER
  lcd.clear();
  lcd.print("ph: scan_prepare");
  delay(2500);
  //PLACEHOLDER

  // calibration (raw)
  while(vl.readRange() >= 25){
    step_motor(&threaded_rod_motor, DOWN, 100);
  }
  delay(1000);
  uint16_t ref_distance = vl.readRange();
  Serial.println(ref_distance);

  // calibration (fine)
  while(vl.readRange() <= (ref_distance+5)){
    step_motor(&threaded_rod_motor, UP, 1);
    delay(100);
  }
  ref_distance = vl.readRange();
  Serial.println(ref_distance);
}



void scan_wait_for_start(scan_handle* xy){
  check_scan_error(xy);

  lcd.clear();
  lcd.print("Zum Starten Taste");
  lcd.setCursor(0,1);
  lcd.print("druecken.");

  //while( (!button_state()) || (!server_button_state()) ){ };      // wait until Button is pressed or Server sends Input (scanState == SCAN)
  while( button_state() == 0 || serverState != SCAN){}; 

   server.handleClient(); //Abfragen des http-request (sobald button auf /ready gedrückt zu handle_runningScan -> stateScan = SCAN)

    lcd.clear();
    lcd.print("Scan starten");
    delay(500);
    for(uint8_t i=0; i<2; i++){
    lcd.print(".");
    delay(500);
  }
}



void scan_run(scan_handle* xy){
  for(int current_height = 0; current_height < MAX_HEIGHT; current_height += HEIGHT_STEP_SIZE)
  {           
    check_scan_error(xy);
    display_scan_progress(current_height, MAX_HEIGHT);              // maybe smarter to input index and sizeof array as parameter -> better percentage resolution
    measure_one_rotation(xy, current_height);
    delay(10);
    static uint16_t steps = (MEASUREMENTS_PER_ROTATION*HEIGHT_STEP_SIZE)/THREADED_ROD_PITCH;
    step_motor(&threaded_rod_motor, UP, steps);                     // depends on threaded rod pitch (Gewindesteigung) i.e. Tr8*8(P2) -> 8mm/360° => 50 steps for 2mm in height
    delay(10);
  }                                                                 // or 8mm/200steps  => 2mm/50steps
}



void scan_finish(scan_handle* xy){                // reset xy->error, data, index and transmit data
  check_scan_error(xy);
  transmit_matlab_code(xy);
  reset_scan_handle(xy);                          // data->{0}, index=0, error=0 ... 
}



void scan_error(scan_handle* xy){                 // print error code on LCD an reset error = restart scan
  lcd.clear();
  lcd.print("Ups, ein Fehler");
  lcd.setCursor(0,1);
  lcd.print("ist augetreten!");
  delay(2500);
  lcd.clear();
  lcd.print("Fehlercode:");
  lcd.setCursor(0,1);
  lcd.print(xy->error);
  delay(2500);
  xy->error = 0;
}


/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv Webserver functions vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

void handle_OnConnect() {
  Serial.println("Start WebServer zur Kommunikation mit 3D-Laserscanner"); 
  server.send(200, "text/html", SendHTML(serverState)); // SendHTML um Seite für entsprechende Zustände zu generieren
  //Hier Ausgabe an Bildschirm mit Startup und Initialisierung und Button für calibration start
}

void handle_Ready4Scan() { // /ready
  Serial.println("Ready4scan");
  if (serverState == -1 || serverState == 3) {
    serverState = ROOT_PREPARING;           //starten des calibration zustands
  server.send(200, "text/html", SendHTML(serverState));
  } else {
  Serial.println("READY ZUSTAND");
  server.send(200, "text/html", SendHTML(serverState));
  }
}

void handle_RunningScan() { // /running
  Serial.println("runningScan");
  if (serverState == 1) {
    serverState = SCAN;
    server.send(200, "text/html", SendHTML(serverState));
  } else {
    Serial.println("refresh oder zu download");
    server.send(200, "text/html", SendHTML(serverState));
  }
}

void handle_DownloadData() { // /download
  Serial.println("download data");
  server.send(200, "text/html", SendHTML_Download(serverState, &current_scan)); //übertragen der Daten auf Website 
} 

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String SendHTML(scan_state state){  //generiert Seite für entsprechenden handle-> if-Abfragen für verschiedene Zustände
  
  String ptr = "<!DOCTYPE html> <html>\n";  //indicates that  HTML-Code will be sent in the follwing  
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";   //für webbrowser
  ptr +="<title>3D-LaserScanner by mst2.se2</title>\n";  //title page w/ <title>-tag

  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n"; //Desgin Einstellungen Webpage
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h2 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";  //Layout
  
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n"; //Button design allgemein
  ptr +=".button-on {background-color: #3498db;}\n"; //button aussehen
  ptr +=".button-on:active {background-color: #2980b9;}\n"; //button wenn geklickt

  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n"; //Layout

  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="</body>\n";

  ptr +="<h1>3D-LaserScanner routine</h1>\n";  //Webpage headings..
   
  
  if(serverState == -1) {   // using /root
  ptr +="<h2>Web Application Using Access Point(AP) Mode From ESP32 WROOM connected to 3D-LaserScanner</h2>\n";
  ptr +="<h3>Scanner needs to calibrate and prepare scanning instruments..</h3>\n";

   ptr +="<p>Start Instrument Boot Up</p><a class=\"button button-on\" href=\"/ready\">START</a>\n"; 
  } 

  if (serverState == 0) {   // using /Ready4Scan
    ptr +="<h2>Please wait before installing object!</h2>\n";
    ptr +="<h3>Calibration is running..</h3>\n";

   ptr +="<p>LOADING..</p><a class=\"button button-off\" href=\"/ready\">Refresh</a>\n";
  }
  else if (serverState == 1) {    // /using /Ready4Scan
    ptr +="<h2>Calibration finished!</h2>\n";
    ptr +="<h3>Place object and start scan on Button below or on Scanner itself.</h3>\n";

    ptr +="<p>After placing object:Y</p><a class=\"button button-on\" href=\"/running\">Start Scan</a>\n"; 
  } 
  else if (serverState == 2) { // using /RunningScan
    ptr +="<h2>Scan started!</h2>\n";
    ptr +="<h3>Progress is shown on LCD-Display..</h3>\n";

    ptr +="<p>Generate scanned 3d_modell:</p><a class=\"button button-off\" href=\"/running\">WAIT for FINISH</a>\n"; 
  } 
  else {    // using /RunningScan
    ptr +="<h3>Scan finished..</h3>\n";

    ptr +="<p>Generate scanned 3d_modell:</p><a class=\"button button-off\" href=\"/download\">DOWNLOAD</a>\n";  //zu /DownloadData
  }

  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

String SendHTML_Download (scan_state state, scan_handle* xy) {
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

  ptr +="<h2>MatLab-Code generiert</h2>\n";                   //AUSGABE MATLAB-CODE
  ptr +="<p>Folgenden Code kopieren und in Matlab einfuegen:  </p>\n";

  //Ausgabe MatLab-Code
  uint16_t amount_array_elements = (sizeof(xy->alpha_mul_10))/(sizeof(xy->alpha_mul_10[0]));      // data of alpha, radius and height has to be equal ! 

  ptr +="<p>alpha=[";
  for(int i=0; i<amount_array_elements; i++){
    if(i != (amount_array_elements-1)){
      ptr += xy->alpha_mul_10[i];
      ptr +=",";
    }
    else{
      ptr += xy->alpha_mul_10[i];
      ptr +="]' * (pi/1800); </p>\n";             // divided by 10 due to information loss with integer processing
    }
  }

  ptr +="<p>radius=[";
  for(int i=0; i<amount_array_elements; i++){
    if(i != (amount_array_elements-1)){
      ptr += xy->radius_mul_10[i];
      ptr +=",";
    }
    else{
      ptr += xy->radius_mul_10[i];
      ptr +="]' * (1/10);</p>\n";              // divided by 10 due to information loss with integer processing
    }
  }
  
  ptr +="<p>height=[";
  for(int i=0; i<amount_array_elements; i++){
    if(i != (amount_array_elements-1)){
      ptr += xy->height[i];
      ptr +=",";
    }
    else{
      ptr += xy->height[i];
      ptr +="]' * (1/10);</p>\n";             // divided by 10 due to information loss with integer processing
    }
  }

  ptr +="<p>[x y z] = pol2cart(alpha,radius,height);</p>\n";
  ptr +="<p>plot3(x,y,z);</p>\n";
  

  if(serverState == 3) //Option für neuen Scan 
  {ptr +="<p>Generated</p><a class=\"button button-on\" href=\"/ready\">Start New Scan?</a>\n";}   //bei neuem scan-> neue calibration 

  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}


/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv private functions vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

uint8_t button_state(void){                   // returns information if button is pressed
  return digitalRead(BUTTON);
}


uint8_t server_button_state(void){            // asks server if button is klicked. 
  //PLACEHOLDER
  return 0;                                   // returns button state: 0->no button klicked | 1->button klicked
}



void measure_one_rotation(scan_handle* xy, uint8_t current_height){                                 // measures one rotation and puts measured data*10 (avoids information loose cause of integer calculation) in xy->alpha xy->radius & xy->height
  for(uint16_t current_alpha = 0; current_alpha <3600; current_alpha += ANGLE_STEP_SIZE_MUL10)      // repeat untill 360 degree reached
  {
    uint16_t tmp_data = 0;
    for(uint16_t measurement = 1; measurement <= MEASUREMENTS_PER_POINT; measurement++){             // calculate average out of xxx repititions
      tmp_data += vl.readRange();                                
      //xy->error = vl.readRangeStatus();
    }       

    uint index = xy->index;
    xy->alpha_mul_10[index] = current_alpha;
    xy->radius_mul_10[index] = (REFERENCE_DISTANCE_MUL10 - ( (tmp_data*10)/MEASUREMENTS_PER_POINT) );           // "*10" due to information lost processing with integers ************************************************IMPORTANT: CHECK IF DISTANCE IS TO HIGH !!! IF YES -> WRITE "0"
    xy->height[index] = current_height;
    xy->index++;
    if(index > ((sizeof(xy->radius_mul_10))/sizeof(xy->radius_mul_10[0]))){    // check if size of array is big enough for data
      xy->error = ARRAY_OVERFLOW_ERROR;
    }
    
    step_motor(&disc_motor, CLOCKWISE, 1);
    
  }
}


void step_motor(motor_handle* motor, uint8_t direction, uint8_t amount_of_steps){
  digitalWrite(motor->dir_pin, direction);
  for(int i=0; i<amount_of_steps; i++){
    digitalWrite(motor->step_pin, HIGH);
    delay(10);                                   // test different delays !
    digitalWrite(motor->step_pin, LOW);
    delay(10);                                   // test different delays !
  }
}



void display_scan_progress(uint8_t current_value, uint8_t max_value){   // shows a progressbar in the bottom row and displays the percentage done 
  if(!current_value){
    lcd.clear();
    lcd.print("Scanfortschritt:");
  }

  uint8_t progress_percent = (current_value*100)/(max_value);           // calculates current progress in percentage
  uint8_t progress_scaled  = ((progress_percent*LCD_COLUMNS)/100);      // progress scaled to LCD screen

  lcd.setCursor(progress_scaled,1); 
  uint8_t st1 = ((LCD_COLUMNS-1)/2);
  uint8_t st2 = ((LCD_COLUMNS-1)/2)+2;
  if((progress_scaled<st1) ||(progress_scaled>st2)){      //progressbar, but in the middle a gap for displaying the percentage
    lcd.print(".");
  }
  
  lcd.setCursor(((LCD_COLUMNS-1)/2),1);                       // show percentage in the middle of the row
  lcd.print(progress_percent);
  lcd.print("%");
}



void check_scan_error(scan_handle* xy){                // checks if error != 0, if yes displays error code ! 
  if(xy->error){
    scan_error(xy);
  }
}



void transmit_matlab_code(scan_handle* xy){
  lcd.clear();
  lcd.print("Daten uebertragen.");
  lcd.setCursor(0,1);
  lcd.print("Bitte warten.");

  uint16_t amount_array_elements = (sizeof(xy->alpha_mul_10))/(sizeof(xy->alpha_mul_10[0]));      // data of alpha, radius and height has to be equal ! 
  Serial.println(" ");
  Serial.println("Folgenden Code kopieren und in Matlab einfuegen:");
  Serial.println(" ");

  Serial.print("alpha=[");
  for(int i=0; i<amount_array_elements; i++){
    if(i != (amount_array_elements-1)){
      Serial.print(xy->alpha_mul_10[i]);
      Serial.print(",");
    }
    else{
      Serial.print(xy->alpha_mul_10[i]);
      Serial.print("]' * (pi/1800);");             // divided by 10 due to information loss with integer processing
    }
  }
  Serial.println("");
  
  Serial.print("radius=[");
  for(int i=0; i<amount_array_elements; i++){
    if(i != (amount_array_elements-1)){
      Serial.print(xy->radius_mul_10[i]);
      Serial.print(",");
    }
    else{
      Serial.print(xy->radius_mul_10[i]);
      Serial.print("]' * (1/10);");             // divided by 10 due to information loss with integer processing
    }
  }
  Serial.println("");

  Serial.print("height=[");
  for(int i=0; i<amount_array_elements; i++){
    if(i != (amount_array_elements-1)){
      Serial.print(xy->height[i]);
      Serial.print(",");
    }
    else{
      Serial.print(xy->height[i]);
      Serial.print("]' * (1/10);");             // divided by 10 due to information loss with integer processing
    }
  }
  Serial.println("");

  Serial.println("[x y z] = pol2cart(alpha,radius,height);");
  Serial.println("plot3(x,y,z);");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
}



void reset_scan_handle(scan_handle* xy){
  uint16_t amount_array_elements = (sizeof(xy->alpha_mul_10))/(sizeof(xy->alpha_mul_10[0]));

  for(int i=0; i<amount_array_elements; i++){
    xy->alpha_mul_10[i] = 0;
    xy->radius_mul_10[i] = 0;
    xy->height[i] = 0;
  }
  xy->error = 0;
  xy->index = 0;
}
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/