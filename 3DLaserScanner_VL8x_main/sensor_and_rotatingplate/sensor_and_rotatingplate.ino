//Board: NodeMCU-32S

#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>


#define POLLING 50
#define BUTTON  5
#define STEP_PIN 32
#define DIR_PIN  33

#define STEPS_PER_REV 200      // the number of steps in one revolution of your motor (28BYJ-48)
#define MEASUREMENTS_PER_REV    200
#define D_REF   80              //Abstand Sensor Drehscheibe Mittelpunkt
#define STEPPER_SPEED 200         // delay between high/low of step signal

/* Put your SSID & Password */
const char* ssid = "3D-Scanner";  // Network name
const char* password = "123456789";  //Network password
uint8_t is_scanning = 0;


/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

// declare an object of WebServer library
WebServer server(80);

//LiquidCrystal lcd(12, 11, 5, 4, 3, 2); bei arduino
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);
Adafruit_VL6180X vl = Adafruit_VL6180X();



void setup() {
  //-------------------------------------- serial init ---------------------------------------------
  Wire.begin();
  Serial.begin(115200);

  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }
  
  //-------------------------------------- web server init ----------------------------------------
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
  server.on("/", handle_OnConnect);
  //server.on("/StartScan", handle_Scanning);
  server.onNotFound(handle_NotFound);
  server.begin();
  Serial.println("HTTP server started");

  //-------------------------------------- Stepper Motor init --------------------------------------

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  //-------------------------------------- Button initialisieren ----------------------------------
  pinMode(BUTTON, INPUT);

  //-------------------------------------- lcd init -----------------------------------------------
  lcd.begin(16, 2);   //anzahl spalten, anzahl zeilen
  lcd.clear();
  lcd.print("Initialising...");
  delay(2000);

  //-------------------------------------- VL6180x init ------------------------------------------
  if (! vl.begin()) {
    Serial.println("Sensor not found!");
    lcd.clear();
    lcd.print("Sensor not found!");
    while (1);
  }
  Serial.println("Sensor found!");
  lcd.clear();
  lcd.print("Sensor found!");
  delay(2000);
  Serial.println("Press button to start");
  lcd.clear();
  lcd.print("Press button to ");
  lcd.setCursor(0,1);
  lcd.print("start scan.");
  lcd.setCursor(0,0);
  delay(1000);
}


//--------------------------------------------------------- loop ---------------------------------------------------------------------------
void loop() {

  static uint8_t state = 0;
  static int16_t polar_coordinates[MEASUREMENTS_PER_REV][2] = {0};    //10 zeilen * 2 spalten matrix
  server.handleClient();

  if(digitalRead(BUTTON))
  {
    if(state == 0){
    lcd.clear();
    lcd.print("Button pressed...");
    Serial.println("Button pressed...");
    delay(1000);
    lcd.clear();
    lcd.print("Start scanning...");
    Serial.println("Start scanning...");
    delay(1000);
    state = 1;
    is_scanning = 1;
    server.handleClient();
    }

    for(int i=0; i<MEASUREMENTS_PER_REV; i++) 
    {
      if(i!=0){
      //stepper.step(STEPS_PER_REV/MEASUREMENTS_PER_REV);
        digitalWrite(STEP_PIN, HIGH);
        delay(STEPPER_SPEED);
        digitalWrite(STEP_PIN, LOW);
        delay(STEPPER_SPEED);
      }
      uint16_t alpha = i*360/MEASUREMENTS_PER_REV;                  // da 1,8° -> float !

      uint16_t distance = get_average_distance();

      int16_t radius = D_REF - (int)distance;
      polar_coordinates[i][0] = alpha;
      polar_coordinates[i][1] = radius;
      
      lcd.clear();
      //lcd.print("Alpha: "); lcd.print(alpha); lcd.print("deg (false)");
      lcd.print("Messung Nr.: "); lcd.print(i+1);
      lcd.setCursor(0,1);
      lcd.print("Distance: "); lcd.print(distance); lcd.print("mm");  
      server.handleClient();
    }

    lcd.clear();
    lcd.print("Scan finished !");
    Serial.println("Scan finished !");
    is_scanning = 2;
    server.handleClient();
    delay(3000);
    Serial.println("Calculating MATLAB-Code...");
    lcd.clear();
    lcd.print("Calculating ");
    lcd.setCursor(0,1);
    lcd.print("MATLAB-Code");
    for(int i=0; i<3; i++){
    delay(1000);
    lcd.print(".");
    }

    matlab_code(&polar_coordinates[0][0]);
  }
  else
  {
    if(state){
    lcd.clear();
    lcd.print("Press Button to ");
    lcd.setCursor(0,1);
    lcd.print("start");
    lcd.setCursor(0,0);
    Serial.println("Press button to start");
    delay(1000);
    state = 0;
    }
  }
}








//--------------------------------------------------------------------------------------------------
//private functions
// Webserver functions
void handle_OnConnect() {
  server.send(200, "text/html", SendHTML("Platzhalter: hier steht später Matlab-Code\n", is_scanning)); 
}

String SendHTML(String Matlab_code, uint8_t is_scanning){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>3D Scanner Sensortechnik</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>ESP32 Web Server</h1>\n";
  ptr +="\n";
  if(is_scanning == 0){
    ptr += "<p>start scan by pushing button</p>\n";
  }
  else if(is_scanning == 1){
    ptr += "<p>scanning</p>\n";
  }
  else if(is_scanning == 2){
    ptr += "<p>scan finished</p>\n";
    ptr += "<p>folgenden Code kopieren und in Matlab einfügen:</p>\n";
    ptr += Matlab_code;
  }
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}


// Matlab string erzeugen
void matlab_code(int16_t *polar_coordinates)
{
  Serial.println("Folgenden Code kopieren und in Matlab einfuegen:");
  Serial.print("alpha=[");
  for(int i=0; i<MEASUREMENTS_PER_REV; i++)
  {
      Serial.print(polar_coordinates[i*2]);
      if(i<(MEASUREMENTS_PER_REV-1)){
        Serial.print(",");
      }
      else{
        Serial.print("]' * (pi/180); ");
      }
  }
  Serial.println("");

  Serial.print("radius=[");
  for(int i=0; i<MEASUREMENTS_PER_REV; i++){
      Serial.print(polar_coordinates[(i*2)+1]);
      if(i<(MEASUREMENTS_PER_REV-1)){
        Serial.print(",");
      }
      else{
        Serial.print("]'; ");
      }   
  }
  Serial.println("");

  Serial.println("polar(alpha, radius)");
  Serial.println("");
  Serial.println("");
  Serial.println("");
}


// VL6180x functions
uint16_t get_average_distance(void)
{
  uint16_t range = 0;
  for(int n=1; n<=POLLING; n++)
  {
    range = range+vl.readRange();
  }
  range=range/POLLING;

  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    //lcd.clear();
    //lcd.print(range); lcd.print("mm");
  }

  // Some error occurred, print it out!
  
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    //Serial.println("System error");
    lcd.clear();
    lcd.print("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    //Serial.println("ECE failure");
    lcd.clear();
    lcd.print("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    //Serial.println("No convergence");
    lcd.clear();
    lcd.print("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    //Serial.println("Ignoring range");
    lcd.clear();
    lcd.print("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    //Serial.println("Signal/Noise error");
    lcd.clear();
    lcd.print("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    //Serial.println("Raw reading underflow");
    lcd.clear();
    lcd.print("Raw underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    //Serial.println("Raw reading overflow");
    lcd.clear();
    lcd.print("Raw overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    //Serial.println("Range reading underflow");
    lcd.clear();
    lcd.print("Range underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    //Serial.println("Range reading overflow");
    lcd.clear();
    lcd.print("Range overflow");
  }
  
  return range;
}


