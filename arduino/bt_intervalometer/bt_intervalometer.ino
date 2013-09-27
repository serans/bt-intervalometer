/**
 * INTERVALOMETER
 */
 
#include <SoftwareSerial.h> 
#include <GroveSBT.h>
#include <TimerOne.h> 

#ifdef USE_SERVO
  #include <Servo.h>
#endif

#define BUFFER_SIZE 80
#define EOL '\n'
#define EOT 4

//CONFIGURATION CONST
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

//PINS CONFIGURATION
#define PIN_FOCUS         8
#define PIN_SERVO         9
#define PIN_SHUTTER       13

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
boolean shutter_closed_flag = false;

#ifdef USE_SERVO
  Servo rotation_servo;
#endif

byte rotation_servo_period;

//FUNCTIONS
void assignVars(char *keys[], unsigned short *values, byte N);
void onBtReady();
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
  pinMode(PIN_SHUTTER, OUTPUT);
  pinMode(PIN_FOCUS, OUTPUT);  
  
  Timer1.attachInterrupt(NULL);

  #ifdef USE_SERVO
    rotation_servo.attach(PIN_SERVO);
  #endif

  groveSBT_init();
  groveSBT_onReady = onBtReady;
  groveSBT_onNewLine = interpretateLine;
  
  Serial.begin(9600);
  Serial.println("BEGINING");
}

void loop () {

  groveSBT_loop();      

  takePictures();
  
  if(shutter_closed_flag) {  
    shutter_closed_flag=false;
    state_v[ID_NUM_PICS_TAKEN]++;
    afterPictureIsTaken();
  }
}

/**
 */
void onBtReady() { 
  groveSBT_inq(); 
}

/**
 * After a picture is taken:
 *  - Check if more pictures are to be taken
 *  - Rotates the servo if necessary
 */
void afterPictureIsTaken() {

   if( state_v[ID_NUM_PICS_TAKEN] < state_v[ID_TOTAL_PIC_N] ) {
       if(state_v[ID_USE_ROTATION] == 1) {
         if(state_v[ID_ROTATION_INC_P] <= rotation_servo_period) {
           rotation_servo_period = 0;
           state_v[ID_ROTATION_CURRENT]+=state_v[ID_ROTATION_INC];
           #ifdef USE_SERVO
             rotation_servo.write(state_v[ID_ROTATION_CURRENT]);
           #endif
         }
       }
       state_v[ID_STATUS] = STATUS_WAITING;
   } else {
       state_v[ID_STATUS] = STATUS_READY;
   }

   dumpState();
}

/*
 * When the timing is right, opens the shutter and sets an interruption 
 * to close the shutter.
 */
void takePictures() {
  if(state_v[ID_STATUS] == STATUS_WAITING){ 
    if(millis()-shutter_timestamp >= long(state_v[ID_PIC_PERIOD])*1000) {
      shoot();
    }
  }
}

void shoot() {
  state_v[ID_STATUS] = STATUS_SHOOTING;
  unsigned long T;
  T = long(state_v[ID_SHUTTER_MILLIS]) * 1000;

  shutter_timestamp = millis();
 
  digitalWrite( PIN_SHUTTER, 1);

  Timer1.detachInterrupt();
  Timer1.initialize(T);
  Timer1.attachInterrupt( closeShutter );

  rotation_servo_period++;
}

/*
 * Called by Timer1 to close the shutter
 */
void closeShutter() {
  Timer1.stop();
  digitalWrite( PIN_SHUTTER, 0);
  shutter_closed_flag = true;
}

void interpretateLine() {

  if(groveSBT_buffer_equals_string("STATUS\n")) {
    groveSBT_write("OK\n");
    groveSBT_init();
    dumpState();
  } else if(groveSBT_buffer_equals_string("RESUME\n")) {
    groveSBT_write("OK\n");
    groveSBT_init();
    state_v[ID_STATUS] = STATUS_WAITING;
  } else if(groveSBT_buffer_equals_string("STOP\n")) {
    groveSBT_write("OK\n");
    groveSBT_init();
    state_v[ID_STATUS] = STATUS_READY;
  } else if(groveSBT_buffer_equals_string("START\n")) {
    init_timestamp = millis();
    groveSBT_write("OK\n");
    groveSBT_init();
    applyConf();
    shoot();
  }else{
    assignVars(state_k, state_v, NUM_STATE_VALUES);
    groveSBT_init();
  }

}

/*
 * reads a line in the format "key:value" and sets *values accordingly
 */
void assignVars(char *keys[], unsigned short *values, byte N) {
  int i,j;
  char key[10];
  char val[4];
  char c;
  
  boolean key_found = false;
  boolean value_found = false;

  for(i=0; groveSBT_available(); i++) {
    key[i] = groveSBT_read();
    if(key[i] == ':') {
      key[i] = '\0';
      key_found = true;
      break;
    }
  }
  
  if(key_found) {
    for(i=0; groveSBT_available(); i++) {
      val[i] = groveSBT_read();
      if(val[i] == '\n') {
        val[i] == '\0';
        value_found = true;
        break;
      }
    }
  }
  
  if(value_found) {
    for(i=0; i<N; i++) {
      if(strcmp(keys[i],key) == 0) {
          values[i] = atoi(val);
          groveSBT_write("OK\n");
          return;
      }
    }
  }

}

void applyConf() {
  state_v[ID_NUM_PICS_TAKEN] = 0;
 
  if(state_v[ID_USE_ROTATION] == 1) {
    state_v[ID_ROTATION_CURRENT] = state_v[ID_ROTATION_START];
    
    #ifdef USE_SERVO
      rotation_servo.write(state_v[ID_ROTATION_CURRENT]);
    #endif
    
    rotation_servo_period = 0;
  }
}

void dumpState() {
  int i;

  for (i=0; i<NUM_STATE_VALUES; i++) {
    groveSBT_write(state_k[i]);
    groveSBT_write(":");
    
    if(i == ID_STATUS) 
      groveSBT_write(status_name[state_v[i]]);
    else 
      groveSBT_write(state_v[i]);
    
    groveSBT_write("\n");
  }

  groveSBT_write("elapsed_time:");
  if(state_v[ID_STATUS] == STATUS_WAITING) groveSBT_write(millis()-init_timestamp);
  else groveSBT_write(0);

  groveSBT_write("\n\n");
}
