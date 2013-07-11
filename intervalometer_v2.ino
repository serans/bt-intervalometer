/**
 * INTERVALOMETER
 */

#include <TimerOne.h> 
#include <Servo.h>

#define BUFFER_SIZE 80
#define EOL '\n'
#define EOT 4

char line_buffer[BUFFER_SIZE];

#define ID_PIC_PERIOD     0  // period between shots (in seconds)
#define ID_TOTAL_PIC_N    1  // total number of pictures to be taken
#define ID_USE_FOCUS      2  // @TODO wether or not to use the focus
#define ID_SHUTTER_MILLIS 3  // number of millis to keep the shutter open
#define ID_USE_ROTATION   4  // wether or not to use camera rotation via servos
#define ID_ROTATION_START 5  // rotation initial position
#define ID_ROTATION_INC   6  // rotation increment between shots
#define ID_ROTATION_INC_P 7  // number of pics to be taken at each rotation angle
#define ID_STATUS         8  // status of the intervalometer (see STATUS_XXXX constants)
#define ID_NUM_PICS_TAKEN 9  // number of pictures already taken
#define ID_ROTATION_CURRENT 10 // current rotation of the servo

#define STATUS_READY      0  // initial state
#define STATUS_SHOOTING   1  // when shutter is on
#define STATUS_WAITING    2  // waiting for the next shot

#define NUM_STATE_VALUES  11

#define PIN_SHUTTER       13
#define PIN_FOCUS         12
#define PIN_SERVO         11

// GLOBAL VARS
char* state_k[] = {
  "pic_period",
  "total_pic_n",
  "use_focus",
  "shutter_millis",
  "use_rotation",
  "rotation_start",
  "rotation_inc",
  "rotation_inc_p",
  "status",
  "num_pics_taken",
  "rotation_current",
};

unsigned short state_v[NUM_STATE_VALUES];

char* status_name[] = {
  "status_ready",
  "status_shooting",
  "status_waiting",
};

byte buffer_pos;
char buffer[BUFFER_SIZE];
unsigned long shutter_timestamp;
unsigned long init_timestamp;
boolean shutter_interrupt = false;

Servo rotation_servo;
byte rotation_servo_period;

//FUNCTIONS
void assignVars(char *line, char *keys[], unsigned short *values, byte N);
void dumpState();
void shoot();
void applyConf();
void afterPictureIsTaken();

void setup () {

  //DEFAULTS
  state_v[ID_PIC_PERIOD] = 1;
  state_v[ID_TOTAL_PIC_N] = 1;
  state_v[ID_SHUTTER_MILLIS] = 100;
  state_v[ID_STATUS] = STATUS_READY;
  
  state_v[ID_USE_ROTATION] = 0;
  state_v[ID_ROTATION_START] = 0;
  state_v[ID_ROTATION_INC] = 1;
  
  //INITIALIZATIONS
  Timer1.initialize();
  pinMode(PIN_SHUTTER, OUTPUT);
  pinMode(PIN_FOCUS, OUTPUT);  
  rotation_servo.attach(PIN_SERVO);
  Serial.begin(115200);

  Serial.println("D:START");
}

//LOOP
void loop () {
  
  readSerialInput();
  
  takePictures();
  afterPictureIsTaken();
  
}

void afterPictureIsTaken() {
  if(shutter_interrupt) {  
    shutter_interrupt=false;
    dumpState();
    
    if(state_v[ID_USE_ROTATION] == 1)
      if(state_v[ID_ROTATION_INC_P] <= rotation_servo_period) {
        rotation_servo_period = 0;
        state_v[ID_ROTATION_CURRENT]+=state_v[ID_ROTATION_INC];
        rotation_servo.write(state_v[ID_ROTATION_CURRENT]);
      }
  }
}

void takePictures() {
  if(state_v[ID_STATUS] == STATUS_WAITING){ 
    if( state_v[ID_NUM_PICS_TAKEN] >= state_v[ID_TOTAL_PIC_N] ) {
      state_v[ID_STATUS] = STATUS_READY;
    } else if(millis()-shutter_timestamp >= long(state_v[ID_PIC_PERIOD])*1000) {
      shoot();
    }
  }
}

void readSerialInput() {
  while(Serial.available()) {
    char c = Serial.read();
    
    if( c == EOL || c == EOT ) {
      line_buffer[buffer_pos+1] = EOL;
      interpretateLine(line_buffer);
      clearBuffer(&line_buffer[0], BUFFER_SIZE);     
      buffer_pos = 0;
    } else if( buffer_pos < BUFFER_SIZE-2 ) {
      line_buffer[buffer_pos] = c;
      buffer_pos++;
    }
    
  }
}

void interpretateLine(char *line) {
  
  if(strcmp(line,"STATUS") == 0) {
    Serial.println("OK");
    dumpState();
  } else if(strcmp(line,"RESUME") == 0) {
    Serial.println("OK");
    state_v[ID_STATUS] = STATUS_WAITING;
  } else if(strcmp(line,"STOP") == 0) {
    Serial.println("OK");
    state_v[ID_STATUS] = STATUS_READY;
  } else if(strcmp(line,"START") == 0) {
    Serial.println("OK");
    applyConf();
  }else{
    assignVars(line_buffer, state_k, state_v, NUM_STATE_VALUES);
  }

  Serial.write(EOT);
}

/*
 * Interpretates a line, assigning a value to the corresponding keys
 */
void assignVars(char *line, char *keys[], unsigned short *values, byte N) {
  //Serial.println(line);
  int i;
  char *val=line;
  boolean assigned = false;
  
  while(*val != '\0') {
    if( *val == ':' ) {
      *val = '\0';
      val++;
      break;
    }
    val++;
  }

  for(i=0; i<N; i++) {
    if(strcmp(keys[i],line) == 0) {
        values[i] = atoi(val);
        assigned = true;
    }
  }
  
  if(assigned) Serial.println("OK");
  else Serial.println("PARSING ERROR");
}

void applyConf() {
  state_v[ID_NUM_PICS_TAKEN] = 0;
  init_timestamp = millis();
  shoot();
  
  if(state_v[ID_USE_ROTATION] == 1) {
    state_v[ID_ROTATION_CURRENT] = state_v[ID_ROTATION_START];
    rotation_servo.write(state_v[ID_ROTATION_CURRENT]);
    rotation_servo_period = 0;
  }
}

void dumpState() {
  int i;

  for (i=0; i<NUM_STATE_VALUES; i++) {
    Serial.print(state_k[i]);
    Serial.print(":");
    if(i == ID_STATUS) Serial.println(status_name[state_v[i]]);
    else Serial.println(state_v[i]);
  }

  Serial.print("elapsed_time:");
  if(state_v[ID_STATUS] == STATUS_WAITING) Serial.println(millis()-init_timestamp);
  else Serial.println(0);

  Serial.println();
}

/*
 * Takes a picture
 */
void shoot() {

  state_v[ID_STATUS] = STATUS_SHOOTING;
  unsigned long T;
  T = long(state_v[ID_SHUTTER_MILLIS]) * 1000;

  shutter_timestamp = millis();
 
  digitalWrite( PIN_SHUTTER, 1);

  Timer1.initialize(T);
  Timer1.attachInterrupt( closeShutter );

  rotation_servo_period++;
}

void closeShutter() {
  digitalWrite( PIN_SHUTTER, 0);
  shutter_interrupt = true;
  Timer1.stop();
  state_v[ID_NUM_PICS_TAKEN]++;
  state_v[ID_STATUS] = STATUS_WAITING;
}


void clearBuffer(char *buffer, short N) {
  int i;
  for(i=0;i<N;i++) buffer[i]='\0';
}

