//formatierte 3D-Scanner .ino
// 07/03/2022 19:22

/***************************************************************************************************************/
/*                                                INCLUDES                                                     */
/***************************************************************************************************************/

#include <Adafruit_VL6180X.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <WiFi.h>
#include <WebServer.h>


/***************************************************************************************************************/
/*                                                DEFINES                                                      */
/***************************************************************************************************************/
/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv GPIO-PINS vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

//Button
#define BUTTON 5  // Button pin
#define POTI   34

//Stepper motor
#define STEP_PIN_1 32  // Motor1 step pin  (disc)
#define DIR_PIN_1 33   // Motor1 direction pin (disc)
#define STEP_PIN_2 26  // Motor2 step pin  (threaded rod)
#define DIR_PIN_2 27   // Motor2 direction pin (threaded rod)

//LCD
#define RS 19  // LCD RS pin
#define EN 23  // LCD EN pin
#define D4 18  // LCD D4 pin
#define D5 17  // LCD D5 pin
#define D6 16  // LCD D6 pin
#define D7 15  // LCD D7 pin


/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

//LCD defines
#define LCD_COLUMNS 16  // amount of columns LCD-screen
#define LCD_ROWS 2      // amount of rows LCD-screen

//motor defines
#define CLOCKWISE 1
#define COUNTERCLOCKWISE 0
#define UP 1
#define DOWN 0

// customazable defines
#define HEIGHT_STEP_SIZE        2         // 2mm height difference between two measured levels
#define MAX_HEIGHT              120       // 50mm difference between plate and top measure level
#define MEASUREMENTS_PER_POINT  1         // amount of measurement repititions per point

// fixed defines
#define ANGLE_STEP_SIZE_MUL10     18      // 1,8° per step (*10 -> due to improved performance with operating with integer)
#define THREADED_ROD_PITCH        8          // in mm/360° (i.e. 8mm/360°)
#define REFERENCE_DISTANCE_MUL10  750  // the distance between sensor and center of the rotating plate
#define MAX_DISTANCE              1500             // the maximum distance between object and sensor -> above is error
#define MEASUREMENTS_PER_ROTATION (3600 / ANGLE_STEP_SIZE_MUL10)
#define MEASURED_LEVELS           (MAX_HEIGHT / HEIGHT_STEP_SIZE)
#define AMOUNT_MEASURED_POINTS    (MEASURED_LEVELS * MEASUREMENTS_PER_ROTATION)
#define DISTANCE_SENSOR_PLATE     25       // used for calibration
#define DISC_MOTOR_SPEED          10       // delay between the HIGH and LOW signal for stepper motor (10 = fast, 100 = slow)
#define THREADED_ROD_MOTOR_SPEED  10       //
#define OFFSET_LOWEST_LEVEL       100      // offset between lowest measured level and disk (avoids inacuraccy due to disk)



/***************************************************************************************************************/
/*                                                STRUCTS                                                      */
/***************************************************************************************************************/

typedef enum scan_state { INITIALISING,
                          CALIBRATING,
                          WAITING,
                          RUNNING,
                          FINISHED } scan_state;

typedef enum scan_error_type { NONE = 0,
                               NO_SENSOR_FOUND = 100,
                               ARRAY_OVERFLOW_ERROR } scan_error_type;

typedef enum button_state { NOT_PRESSED,
                            PRESSED } button_state;


typedef struct scan_handle {
  uint16_t alpha_mul_10[AMOUNT_MEASURED_POINTS];   // alpha*10  -> reduce information loss due to integer calculation
  uint16_t radius_mul_10[AMOUNT_MEASURED_POINTS];  // radius*10 -> reduce information loss due to integer calculation
  uint16_t height[AMOUNT_MEASURED_POINTS];         // height in mm
  scan_error_type error;                           // saves Error Code when an error occurs. (NO_ERROR by default).
  uint index;                                      // Information about the current processed array data
  scan_state state;
  uint16_t progress;
  button_state server_button_state;
} scan_handle;



typedef struct motor_handle {
  uint8_t step_pin;
  uint8_t dir_pin;
  uint8_t speed;
} motor_handle;



/***************************************************************************************************************/
/*                                                PROTOTYPES                                                   */
/***************************************************************************************************************/

void scan_calibrate(scan_handle* xy);
void scan_wait_for_start(scan_handle* xy);
void scan_run(scan_handle* xy);
void scan_finish(scan_handle* xy);
void scan_error(scan_handle* xy);
button_state get_button_state(void);
void measure_one_rotation(scan_handle* xy, uint16_t current_height);
void step_motor(motor_handle* motor, uint8_t direction, uint16_t amount_of_steps);
uint16_t display_scan_progress(uint16_t current_value, uint16_t max_value);
void check_scan_error(scan_handle* xy);
void transmit_matlab_code(scan_handle* xy);
void reset_scan_handle(scan_handle* xy);

String SendHTML_Download(scan_handle* xy);

/***************************************************************************************************************/
/*                                                VARIABLES                                                    */
/***************************************************************************************************************/

//Handles
scan_handle current_scan = { { 0 }, { 0 }, { 0 }, NONE, 0, INITIALISING, 0, NOT_PRESSED };
motor_handle disc_motor = { STEP_PIN_1, DIR_PIN_1, DISC_MOTOR_SPEED };
motor_handle threaded_rod_motor = { STEP_PIN_2, DIR_PIN_2, THREADED_ROD_MOTOR_SPEED };

//IDK
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
Adafruit_VL6180X vl = Adafruit_VL6180X();

//WebServer
const char* ssid = "3D-Scanner";    // Server name/SSID
const char* password = "12345678";  // Server password
button_state server_button_state = NOT_PRESSED;
IPAddress local_ip(192, 168, 1, 1);  // Sever default ip and stuff
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);  // default HTTP-Port

int analog = MAX_HEIGHT;

/***************************************************************************************************************/
/*                                                SETUP                                                        */
/***************************************************************************************************************/

void setup() {
  //serial
  Wire.begin();
  Serial.begin(115200);

  while (!Serial) {  // wait for serial port to open on native usb devices
    delay(1);
  }

  //Webserver
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  //server.setContentLength(150000);  
  delay(100);

  server.on("/", server_initialising);            // Landing-Page und INfos über Preparing zustand
  server.on("/calibration", server_calibrating);  // after finishing Scanner preparing-> start scan here with serverButton (Start Scan) click oder hardwareButton
  server.on("/waiting", server_waiting);          // showing Scan progress and generating button to copy MatLab-Code from next page
  server.on("/running", server_running);          // displaying MatLab Code with measurements to "download" (copy-paste) - also: restart whole procedure
  server.on("/finished", server_finished);
  server.onNotFound(server_NotFound);  // ungültige URL liefert E404
  server.begin();
  delay(100);
  server.handleClient();
  delay(100);

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

  //VL6180x
  if (!vl.begin()) {
    current_scan.error = NO_SENSOR_FOUND;  // error code for not founding sensor
  }
}



/***************************************************************************************************************/
/*                                                LOOP                                                         */
/***************************************************************************************************************/

void loop() {
  scan_calibrate(&current_scan);       // LCD: "Initialisieren: Bitte noch kein Gegenstand auf Platte stellen !"    && setzt serverState nach Abschluss auf READY
  scan_wait_for_start(&current_scan);  // LCD: "Taste zum starten Drücken" && loop for button_state     && server_ready4Scan -> per taster oder ServerButton zu:
  scan_run(&current_scan);             // LCD: "Scanning... (PROGRESS BAR ?)" && MOTOR1/2 Drehen + VL6180x messen und in data speichern      && server_runningScan -> sobald abgeschlossen per ServerButton zu:
  scan_finish(&current_scan);          // LCD: "Scan beendet. Übertrage Daten..."  -> scan_transmit ?      && server_download ->per button zu on connect zum neustarten
}



/***************************************************************************************************************/
/*                                                FUNCTIONS                                                    */
/***************************************************************************************************************/
/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv loop-state functions vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

void scan_calibrate(scan_handle* xy) {  // drive motor in correct position and calibrate reference distance init variables/data
  xy->state = CALIBRATING;
  server.handleClient();

  lcd.clear();
  lcd.print("Kalibrieren...");
  delay(2500);

  // calibration (raw)
  int tmp = 0;
  while ((tmp = vl.readRange()) >= (DISTANCE_SENSOR_PLATE)) {
    step_motor(&threaded_rod_motor, DOWN, 50);
    Serial.print("Aktuell gemessener Wert in calibration raw: ");
    Serial.println(tmp);
    server.handleClient();
  }
  delay(1000);
  uint16_t ref_distance = vl.readRange();
  Serial.println("ref_distance:");
  Serial.println(ref_distance);

  // calibration (fine)
  while ((tmp = vl.readRange()) <= (ref_distance + 10)) {  // "+10" to balance out variance of sensor(1mm)
    step_motor(&threaded_rod_motor, UP, 1);
    Serial.print("Aktuell gemessener Wert in calibration fine: ");
    Serial.println(tmp);
    server.handleClient();
    delay(100);
  }

  step_motor(&threaded_rod_motor, UP, OFFSET_LOWEST_LEVEL);  // Offset ->  to be sure, that not the edge of the plate is detected !
}


void scan_wait_for_start(scan_handle* xy) {
  xy->state = WAITING;
  server.handleClient();

  lcd.clear();
  lcd.print("Objekthoehe:");
  lcd.setCursor(0, 1);

  button_state tmp = NOT_PRESSED;
  while ((tmp == NOT_PRESSED)) {
    server.handleClient();
    tmp = server_button_state;

    
    analog = analogRead(POTI);
    analog = map(analog,0,4096,0,MAX_HEIGHT);

    lcd.setCursor(0, 1);  
    lcd.print(analog);
    lcd.print("mm       ");
    Serial.println(analog);
  };
  server_button_state = NOT_PRESSED;

  lcd.clear();
  lcd.print("Scan starten");
  delay(500);
  for (uint8_t i = 0; i < 2; i++) {
    lcd.print(".");
    delay(500);
  }
}



void scan_run(scan_handle* xy) {
  xy->state = RUNNING;
  server.handleClient();
  display_scan_progress(0, analog);

  for (uint16_t current_height = 0; current_height < analog; current_height += HEIGHT_STEP_SIZE) {
    static const uint16_t steps = (MEASUREMENTS_PER_ROTATION * HEIGHT_STEP_SIZE) / (THREADED_ROD_PITCH);
    xy->progress = display_scan_progress(current_height, analog);
    server.handleClient();

    if (current_height != 0) {
      step_motor(&threaded_rod_motor, UP, steps);  // depends on threaded rod pitch (Gewindesteigung) i.e. Tr8*8(P2) -> 8mm/360° => 50 steps for 2mm in height
      delay(10);
    }

    measure_one_rotation(xy, current_height);
    delay(10);
  }  // or 8mm/200steps  => 2mm/50steps

  xy->progress = display_scan_progress(analog, analog);
}



void scan_finish(scan_handle* xy) {  // reset xy->error, data, index and transmit data
  xy->state = FINISHED;
  server.handleClient();

  transmit_matlab_code(xy);

  button_state tmp = NOT_PRESSED;
  while ((tmp == NOT_PRESSED)) {
    server.handleClient();
    tmp = server_button_state;
  };
  server_button_state = NOT_PRESSED;

  reset_scan_handle(xy);
}



void scan_error(scan_handle* xy) {  // print error code on LCD an reset error = restart scan
  lcd.clear();
  lcd.print("Ups, ein Fehler");
  lcd.setCursor(0, 1);
  lcd.print("ist augetreten!");
  delay(2500);
  lcd.clear();
  lcd.print("Fehlercode:");
  lcd.setCursor(0, 1);
  lcd.print(xy->error);
  delay(2500);
  xy->error = NONE;
}



/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv Webserver functions vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

void server_initialising(void) {
  server.send(200, "text/html", SendHTML(&current_scan));
}



void server_calibrating(void) {
  server_button_state = PRESSED;
  current_scan.state = CALIBRATING;
  server.send(200, "text/html", SendHTML(&current_scan));
}



void server_waiting(void) {
  server.send(200, "text/html", SendHTML(&current_scan));
}



void server_running(void) {
  server_button_state = PRESSED;
  current_scan.state = RUNNING;
  server.send(200, "text/html", SendHTML(&current_scan));
}



void server_finished(void) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Length", (String)200000);
  server.send(200, "text/html", "");
  String tmp = SendHTML(&current_scan);

  //int size = sizeof(tmp);
  //Serial.print("size :");
  //Serial.print(size);
  //server.sendContent_P( SendHTML(&current_scan));
  server.sendContent(tmp);
}


void server_NotFound(void) {
  server.send(404, "text/plain", "Not found");
}



String SendHTML(scan_handle* xy) {

  String ptr = "<!DOCTYPE html> <html>\n";                                                                        //indicates that  HTML-Code will be sent in the follwing
  ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";  //für webbrowser
  ptr += "<title>3D-LaserScanner by mst2.se2</title>\n";                                                          //title page w/ <title>-tag

  ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";                                                       //Desgin Einstellungen Webpage
  ptr += "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h2 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";  //Layout

  ptr += ".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";  //Button design allgemein
  ptr += ".button-on {background-color: #3498db;}\n";                                                                                                                                                                      //button aussehen
  ptr += ".button-on:active {background-color: #2980b9;}\n";                                                                                                                                                               //button wenn geklickt

  ptr += "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";  //Layout

  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "</body>\n";

  ptr += "<h1>3D-Scanner</h1>\n";  //Webpage headings..


  if (xy->state == INITIALISING) {
    ptr += "<h2>Please wait</h2>\n";
    ptr += "<h3>Initialising...</h3>\n";
    ptr += "<a class=\"button button-on\" href=\"/\">Refresh</a>\n";

  }

  else if (xy->state == CALIBRATING) {
    ptr += "<h2>Please wait</h2>\n";
    ptr += "<h3>Calibrating...</h3>\n";
    ptr += "<a class=\"button button-on\" href=\"/\">Refresh</a>\n";

  }

  else if (xy->state == WAITING) {
    ptr += "<h2>Calibration successfull!</h2>\n";
    ptr += "<h3>Start scan by pushing button</h3>\n";

    ptr += "<a class=\"button button-on\" href=\"/running\">Start Scan</a>\n";
    
  }

  else if (xy->state == RUNNING) {
    ptr += "<h2>Scan running!</h2>\n";
    ptr += "<h3>Current Progress: ";
    ptr += xy->progress;
    ptr += "%</h3>\n";
    ptr += "<a class=\"button button-on\" href=\"/\">Refresh</a>\n";

  }

  else if (xy->state == FINISHED) {  // using /RunningScan
    String tmp = SendHTML_Download(xy);
    ptr += tmp;
    ptr += "<a class=\"button button-on\" href=\"/calibration\">Reset Scan</a>\n";
    xy->progress = 0;
  }

  ptr += "</body>\n";
  ptr += "</html>\n";
  return ptr;
}



String SendHTML_Download(scan_handle* xy) {
  String rtn = "<h2>Scan finished</h2>\n";  //AUSGABE MATLAB-CODE
  rtn += "<p>Folgenden Code kopieren und in Matlab einfuegen:  </p>\n";

  //Ausgabe MatLab-Code
  //uint amount_array_elements = (sizeof(xy->alpha_mul_10))/(sizeof(xy->alpha_mul_10[0]));      // data of alpha, radius and height has to be equal !
  uint amount_array_elements = (xy->index);
  rtn += "<p>load census; steps=200;</p>\n";

  rtn += "<p>alpha=[";
  for (uint i = 0; i < amount_array_elements; i++) {
    String tmp = String(xy->alpha_mul_10[i]);

    if (i != (amount_array_elements - 1)) {
      rtn += tmp;
      rtn += ",";
    } else {
      rtn += tmp;
      rtn += "]' * (pi/1800); </p>\n";  // divided by 10 due to information loss with integer processing
    }
  }

  rtn += "<p>radius=[";
  for (uint i = 0; i < amount_array_elements; i++) {
    String tmp = String(xy->radius_mul_10[i]);
    
    if (i != (amount_array_elements - 1)) {
      rtn += tmp;
      rtn += ",";
    } else {
      rtn += tmp;
      rtn += "]' * (1/10);</p>\n";  // divided by 10 due to information loss with integer processing
    }
  }

  rtn += "<p>height=[";
  for (int i = 0; i < amount_array_elements; i++) {
    String tmp = String(xy->height[i]);
    if (i != (amount_array_elements - 1)) {
      rtn += tmp;
      rtn += ",";
    } else {
      rtn += tmp;
      rtn += "]';</p>\n";  // divided by 10 due to information loss with integer processing
    }
  }

  rtn += "<p>H=length(height)/steps;</p>\n";
  rtn += "<p>alpha2=zeros(H,steps);</p>\n";
  rtn += "<p>radius2=zeros(H,steps);</p>\n";
  rtn += "<p>height2=zeros(H,steps);</p>\n";
  rtn += "<p>g=zeros(H,steps);</p>\n";
  rtn += "<p>alpha3=zeros(H,steps+1);</p>\n";
  rtn += "<p>radius3=zeros(H,steps+1);</p>\n";
  rtn += "<p>height3=zeros(H,steps+1);</p>\n";

  rtn += "p=0.95;</p>\n";
  rtn += "<p>xxi=(0:2*pi/steps:(2*pi-(2*pi/steps)));</p>\n";
  rtn += "<p>for i=1:H</p>\n";
  rtn += "<p>for j=1:steps</p>\n";
  rtn += "<p>alpha2(i,j)=alpha(j+(i-1)*steps,1);</p>\n";
  rtn += "<p>radius2(i,j)=radius(j+(i-1)*steps,1);</p>\n";
  rtn += "<p>height2(i,j)=height(j+(i-1)*steps,1);</p>\n";
  rtn += "<p>end</p>\n";
  rtn += "<p>end</p>\n";
  rtn += "<p>for i=1:1:H</p>\n";
  rtn += "<p>g(i,:)=csaps(alpha2(i,:), radius2(i,:), p, xxi);</p>\n";
  rtn += "<p>end</p>\n";


  rtn += "<p>alpha3(:,steps+1)=alpha2(:,1);</p>\n";
  rtn += "<p>radius3(:,steps+1)=g(:,1);</p>\n";
  rtn += "<p>height3(:,steps+1)=height2(:,1);</p>\n";

  rtn += "<p>for i=1:H</p>\n";
  rtn += "<p>for j=1:steps</p>\n";
  rtn += "<p>alpha3(i,j)=alpha2(i,j);</p>\n";
  rtn += "<p>radius3(i,j)=g(i,j);</p>\n";
  rtn += "<p>height3(i,j)=height2(i,j);</p>\n";
  rtn += "<p>end</p>\n";
  rtn += "<p>end</p>\n";

  rtn += "<p>[x,y,z]=pol2cart(alpha3,radius3,height3);</p>\n";

  rtn += "<p>surf (x,y,z)</p>\n";

  return rtn;
}



/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv private functions vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

button_state get_button_state(void) {  // returns information if button is pressed
  button_state tmp = NOT_PRESSED;
  if (digitalRead(BUTTON)) {
    tmp = PRESSED;
  };
  return tmp;
}



void measure_one_rotation(scan_handle* xy, uint16_t current_height) {  // measures one rotation and puts measured data*10 (avoids information loss due of integer calculation) in xy->alpha xy->radius & xy->heigh

  for (uint16_t current_alpha = 0; current_alpha < 3600; current_alpha += (ANGLE_STEP_SIZE_MUL10))  // repeat untill 360 degree reached
  {
    uint16_t tmp_data = 0;
    for (uint16_t measurement = 1; measurement <= MEASUREMENTS_PER_POINT; measurement++) {  // calculate average out of xxx repititions
      tmp_data += vl.readRange();
      Serial.println(tmp_data);
    }

    uint index = xy->index;
    xy->alpha_mul_10[index] = current_alpha;
    xy->radius_mul_10[index] = ((REFERENCE_DISTANCE_MUL10) - ((tmp_data * 10) / (MEASUREMENTS_PER_POINT)));  // "*10" due to information lost processing with integers
    xy->height[index] = current_height;
    xy->index++;
    if (index > ((sizeof(xy->radius_mul_10)) / sizeof(xy->radius_mul_10[0]))) {  // check if size of array is big enough for data
      xy->error = ARRAY_OVERFLOW_ERROR;
    }

    step_motor(&disc_motor, CLOCKWISE, 1);
    server.handleClient();
    delay(100);
  }
}



void step_motor(motor_handle* motor, uint8_t direction, uint16_t amount_of_steps) {  // steps motor clock-, counterclockwise
  digitalWrite(motor->dir_pin, direction);
  for (uint16_t i = 0; i < amount_of_steps; i++) {
    digitalWrite(motor->step_pin, HIGH);
    delay(motor->speed);
    digitalWrite(motor->step_pin, LOW);
    delay(motor->speed);
  }
}



uint16_t display_scan_progress(uint16_t current_value, uint16_t max_value) {  // shows a progressbar in the bottom row and displays the percentage done
  lcd.clear();
  lcd.print("Scanfortschritt:");

  uint16_t progress_percent = (current_value * 100) / (max_value);      // calculates current progress in percentage
  uint16_t progress_scaled = ((progress_percent * LCD_COLUMNS) / 100);  // progress scaled to LCD screen

  for (uint16_t i = 0; i <= progress_scaled; i++) {
    lcd.setCursor(((LCD_COLUMNS / 2) - 1), 1);  // show percentage in the middle of the row
    lcd.print(progress_percent);
    lcd.print("%");
    if ((i < ((LCD_COLUMNS / 2) - 1)) || (i > ((LCD_COLUMNS / 2) + 1))) {
      lcd.setCursor(i, 1);
      lcd.print(".");
    }
  }
  return progress_percent;
}



void check_scan_error(scan_handle* xy) {  // checks if error != 0, if yes displays error code !
  if (xy->error) {
    scan_error(xy);
  }
}



void transmit_matlab_code(scan_handle* xy) {
  lcd.clear();
  lcd.print("Daten uebertragen.");
  lcd.setCursor(0, 1);
  lcd.print("Bitte warten.");

  //uint amount_array_elements = (sizeof(xy->alpha_mul_10)) / (sizeof(xy->alpha_mul_10[0]));  // data of alpha, radius and height has to be equal !
  uint amount_array_elements = xy->index;
  Serial.println(" ");
  Serial.println("Folgenden Code kopieren und in Matlab einfuegen:");
  Serial.println(" ");

  Serial.print("alpha=[");
  for (uint i = 0; i < amount_array_elements; i++) {
    if (i != (amount_array_elements - 1)) {
      Serial.print(xy->alpha_mul_10[i]);
      Serial.print(",");
    } else {
      Serial.print(xy->alpha_mul_10[i]);
      Serial.print("]' * (pi/1800);");  // divided by 10 due to information loss with integer processing
    }
  }
  Serial.println("");

  Serial.print("radius=[");
  for (uint i = 0; i < amount_array_elements; i++) {
    if (i != (amount_array_elements - 1)) {
      Serial.print(xy->radius_mul_10[i]);
      Serial.print(",");
    } else {
      Serial.print(xy->radius_mul_10[i]);
      Serial.print("]' * (1/10);");  // divided by 10 due to information loss with integer processing
    }
  }
  Serial.println("");

  Serial.print("height=[");
  for (uint i = 0; i < amount_array_elements; i++) {
    if (i != (amount_array_elements - 1)) {
      Serial.print(xy->height[i]);
      Serial.print(",");
    } else {
      Serial.print(xy->height[i]);
      Serial.print("]';");  // divided by 10 due to information loss with integer processing
    }
  }
  Serial.println("");

  Serial.println("[x y z] = pol2cart(alpha,radius,height);");
  Serial.println("plot3(x,y,z);");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
}



void reset_scan_handle(scan_handle* xy) {
  uint amount_array_elements = (sizeof(xy->alpha_mul_10)) / (sizeof(xy->alpha_mul_10[0]));

  for (uint i = 0; i < amount_array_elements; i++) {
    xy->alpha_mul_10[i] = 0;
    xy->radius_mul_10[i] = 0;
    xy->height[i] = 0;
  }
  xy->error = NONE;
  xy->index = 0;
  xy->state = CALIBRATING;
  xy->progress = 0;
  xy->server_button_state = NOT_PRESSED;
}
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/