//formatierte 3D-Scanner .ino

/***************************************************************************************************************/
/*                                                INCLUDES                                                     */
/***************************************************************************************************************/

#include <Adafruit_VL6180X.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
//#include <WiFi.h>
//#include <WebServer.h>
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

/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

//LCD defines
#define LCD_COLUMNS   16          // amount of columns LCD-screen
#define LCD_ROWS      2           // amount of rows LCD-screen

//motor defines 
#define CLOCKWISE           1
#define COUNTERCLOCKWISE    0
#define UP                  1           // ausprobieren 
#define DOWN                0           // ausprobieren 

//scanner defines
#define MEASUREMENTS_PER_ROTATION   200         // abhängig von minimalem Winkel pro Step von Motor bzw maximalen steps pro rotation von motor (äquivalent)
// !-> MEASUREMENTS_PER_ROTATION = 3600/ANGLE_STEP_SIZE_MUL10...... i.e. 200 = 3600/18
#define MEASURED_LEVELS             25          // abhängig von Gewindestangenlänge und Höhendifferenz zwei verschiedener "Ebenen"-Scans
// !-> MEASURED_LEVELS = MAX_HEIGHT/HEIGHT_STEP_SIZE...... i.e. 25 = 50/2
#define ANGLE_STEP_SIZE_MUL10       18          // 1,8° per step (*10 -> due to improved performance with operating with integer)
#define HEIGHT_STEP_SIZE            2           // 2mm height difference between two measured levels
#define MAX_HEIGHT                  50          // 50mm difference between plate and top measure level
#define REFERENCE_DISTANCE_MUL10    750          // the distance between sensor and center of the rotating plate
#define MAX_DISTANCE                150         // the maximum distance between object and sensor -> above is error 
#define MEASUREMENTS_PER_POINT      50           // amount of measurement repititions per point   

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

  //Web-server
  //html "server_on_connect" hier einfügen


  //VL6180x
  if (! vl.begin()) {
    current_scan.error = NO_SENSOR_FOUND;               // error code for not founding sensor
  }
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

  check_scan_error(&current_scan);
  scan_prepare(&current_scan);                            // LCD: "Initialisieren: Bitte noch kein Gegenstand auf Platte stellen !"                             && server_on connect, bzw server_scan_finished
  scan_wait_for_start(&current_scan);                     // LCD: "Taste zum starten Drücken" && loop for button_state                                          && server_wait_for_input
  scan_run(&current_scan);                                // LCD: "Scanning... (PROGRESS BAR ?)" && MOTOR1/2 Drehen + VL6180x messen und in data speichern      && server_scanning
  scan_finish(&current_scan);                             // LCD: "Scan beendet. Übertrage Daten..."  -> scan_transmit ?                                        && server_scan_finished
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
}



void scan_wait_for_start(scan_handle* xy){
  check_scan_error(xy);

  lcd.clear();
  lcd.print("Zum Starten Taste");
  lcd.setCursor(0,1);
  lcd.print("druecken.");

  //while( (!button_state()) || (!server_button_state()) ){ };      // wait until Button is pressed or Server sends Input
  while( button_state() == 0){};

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
    step_motor(&threaded_rod_motor, UP, 50);                        // depends on threaded rod pitch (Gewindesteigung) i.e. Tr8*8(P2) -> 8mm/360° => 50 steps for 2mm in height
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