/*

  Bloodbowl Scoreboard

 */

#include <Wire.h>
#include <avr/pgmspace.h>

#define START_OF_DATA 0x10       //data markers
#define END_OF_DATA 0x20         //data markers

// Colorduino ONE I2C 5
#define DEST_I2C_ADDR1 0x05       //set destination I2C address (match firmware in Colorduino)

// Colorduino TWO I2C 6
#define DEST_I2C_ADDR2 0x06       //set destination I2C address (match firmware in Colorduino)

#define SCREENSIZEX 8            //num of LEDs accross
#define SCREENSIZEY 8            //num of LEDs down

// --------------------------------------------------------------------------------
/* Initialise */
// --------------------------------------------------------------------------------
// delay time between faces
unsigned long delaytime=200;

int playerBtnPins[] = {7,8};   // Buttons for adding score for P1 & P2
int counters[] = {0, 0};       /* Counter for 8x8 Matrix #2  */
int buttonState = 0;           // current state of the button
int lastButtonState[] = {0,0};
boolean startCounter = false;   /* Controlled by a start button */
int startBtnPin = 9;

/* Hall Sensor */
int hallSensorPins[] =     {2, 3};         // PIN2 = 1sT Player, PIN3 = 2nd Player
int hallStates[] =         {0, 0};
boolean lastHallStates[] = {false,false};
int hallTimeouts[] =       {0, 0};

int ledPin =  13;

/* RTC */
#include "RTClib.h" // Rtc Lib
RTC_DS1307 RTC;

/* 8 Digit 7 Segment Display - Variables */
#define MAXCOUNTER 180
int counter = MAXCOUNTER;         /* Counter for 8 Digit 7 Segment Counter */
int RTCPrevSeconds = 0;        /* Store the RTC seconds value */
#define MAX7219DIN 4
#define MAX7219CS 5
#define MAX7219CLK 6

/* Rounds */
int roundBtnPin = 10;
int roundBtnState = 0;
int lastRoundBtnState = 0;
int roundsCounter[] = {0,0};

// --------------------------------------------------------------------------------

byte display_byte[3][64];        //display array - 64 bytes x 3 colours

const unsigned char font8_8[92][8] PROGMEM = {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp
    { 0x00, 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x00 },   // !
    { 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00 },   // "
    { 0x00, 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14, 0x00 },   // #
    { 0x00, 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12, 0x00 },   // $
    { 0x00, 0x00, 0x62, 0x64, 0x08, 0x13, 0x23, 0x00 },   // %
    { 0x00, 0x00, 0x36, 0x49, 0x55, 0x22, 0x50, 0x00 },   // &
    { 0x00, 0x00, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00 },   // '
    { 0x00, 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00, 0x00 },   // (
    { 0x00, 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00, 0x00 },   // )
    { 0x00, 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14, 0x00 },   // *
    { 0x00, 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00 },   // +
    { 0x00, 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00, 0x00 },   // ,
    { 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00 },   // -
    { 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00 },   // .
    { 0x00, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00 },   // /
    { 0x00, 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00 },   // 0
    { 0x00, 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00 },   // 1
    { 0x00, 0x00, 0x42, 0x61, 0x51, 0x49, 0x46, 0x00 },   // 2
    { 0x00, 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31, 0x00 },   // 3
    { 0x00, 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00 },   // 4
    { 0x00, 0x00, 0x27, 0x45, 0x45, 0x45, 0x39, 0x00 },   // 5
    { 0x00, 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00 },   // 6
    { 0x00, 0x00, 0x01, 0x71, 0x09, 0x05, 0x03, 0x00 },   // 7
    { 0x00, 0x00, 0x36, 0x49, 0x49, 0x49, 0x36, 0x00 },   // 8
    { 0x00, 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00 },   // 9
    { 0x00, 0x00, 0x00, 0x36, 0x36, 0x00, 0x00, 0x00 },   // :
    { 0x00, 0x00, 0x00, 0x56, 0x36, 0x00, 0x00, 0x00 },   // ;
    { 0x00, 0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00 },   // <
    { 0x00, 0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00 },   // =
    { 0x00, 0x00, 0x00, 0x41, 0x22, 0x14, 0x08, 0x00 },   // >
    { 0x00, 0x00, 0x02, 0x01, 0x51, 0x09, 0x06, 0x00 },   // ?
    { 0x00, 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E, 0x00 },   // @
    { 0x00, 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C, 0x00 },   // A
    { 0x00, 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00 },   // B
    { 0x00, 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00 },   // C
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00 },   // D
    { 0x00, 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00 },   // E
    { 0x00, 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01, 0x00 },   // F
    { 0x00, 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A, 0x00 },   // G
    { 0x00, 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00 },   // H
    { 0x00, 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00 },   // I
    { 0x00, 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00 },   // J
    { 0x00, 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00 },   // K
    { 0x00, 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00 },   // L
    { 0x00, 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00 },   // M
    { 0x00, 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00 },   // N
    { 0x00, 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00 },   // O
    { 0x00, 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00 },   // P
    { 0x00, 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00 },   // Q
    { 0x00, 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00 },   // R
    { 0x00, 0x00, 0x46, 0x49, 0x49, 0x49, 0x31, 0x00 },   // S
    { 0x00, 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00 },   // T
    { 0x00, 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00 },   // U
    { 0x00, 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00 },   // V
    { 0x00, 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00 },   // W
    { 0x00, 0x00, 0x63, 0x14, 0x08, 0x14, 0x63, 0x00 },   // X
    { 0x00, 0x00, 0x07, 0x08, 0x70, 0x08, 0x07, 0x00 },   // Y
    { 0x00, 0x00, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00 },   // Z
    { 0x00, 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00, 0x00 },   // [
    { 0x00, 0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55, 0x00 },   // 55
    { 0x00, 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00, 0x00 },   // ]
    { 0x00, 0x00, 0x04, 0x02, 0x01, 0x02, 0x04, 0x00 },   // ^
    { 0x00, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00 },   // _
    { 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x00, 0x00 },   // '
    { 0x00, 0x00, 0x20, 0x54, 0x54, 0x54, 0x78, 0x00 },   // a
    { 0x00, 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00 },   // b
    { 0x00, 0x00, 0x38, 0x44, 0x44, 0x44, 0x20, 0x00 },   // c
    { 0x00, 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00 },   // d
    { 0x00, 0x00, 0x38, 0x54, 0x54, 0x54, 0x18, 0x00 },   // e
    { 0x00, 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02, 0x00 },   // f
    { 0x00, 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C, 0x00 },   // g
    { 0x00, 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00 },   // h
    { 0x00, 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00 },   // i
    { 0x00, 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00, 0x00 },   // j
    { 0x00, 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00 },   // k
    { 0x00, 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00 },   // l
    { 0x00, 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00 },   // m
    { 0x00, 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00 },   // n
    { 0x00, 0x00, 0x38, 0x44, 0x44, 0x44, 0x38, 0x00 },   // o
    { 0x00, 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18, 0x00 },   // p
    { 0x00, 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC, 0x00 },   // q
    { 0x00, 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x00 },   // r
    { 0x00, 0x00, 0x48, 0x54, 0x54, 0x54, 0x20, 0x00 },   // s
    { 0x00, 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20, 0x00 },   // t
    { 0x00, 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00 },   // u
    { 0x00, 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00 },   // v
    { 0x00, 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00 },   // w
    { 0x00, 0x00, 0x44, 0x28, 0x10, 0x28, 0x44, 0x00 },   // x
    { 0x00, 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C, 0x00 },   // y
    { 0x00, 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00 },   // z
    { 0x00, 0x00, 0x00, 0x06, 0x09, 0x09, 0x06, 0x00 }    // horiz lines
};

//setup for plasma
typedef struct {
  unsigned char r;
  unsigned char g;
  unsigned char b;
} ColorRGB;


//a color with 3 components: h, s and v
typedef struct {
  unsigned char h;
  unsigned char s;
  unsigned char v;
} ColorHSV;

// --------------------------------------------------------------------------------

void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
  pinMode(playerBtnPins[0], INPUT); /* Player 1 */
  pinMode(playerBtnPins[1], INPUT); /* Player 2 */

  pinMode(ledPin, OUTPUT);
  pinMode(hallSensorPins[0], INPUT); /* Player 1 magnet */
  pinMode(hallSensorPins[1], INPUT); /* Player 2 magnet */

  pinMode(startBtnPin, INPUT);  /* Start counter button */
  pinMode(roundBtnPin, INPUT);  /* Next Round button */

  Serial.begin(9600); // Starting Serial Terminal

  RTC.begin();                  /* Start the clock */

  /* 8 Digit 7 Segment Display - Initialise */
  RTCPrevSeconds=currentSeconds();
  MAX7219init();
  MAX7219brightness(15);
}

void loop(){
  wrapCounter();                /* Wrap counter back to 0 after getting to 9 */
  buttonEvent();                /* Add one if button pushed */
  buttonStart();                /* Control start of counter */
  buttonRound();                /* Next round button */
  hallEvent();                  /* Add one if magnet is sensed by hall sensor */
  displayCounters();             /* Display 0-9 on the 8x8 screen */

  //blankRounds();

  calcCounter();                /* Calc and display when necessary */
}

/* When the reset button is hit we'll store the time from the RTC */

// --------------------------------------------------------------------------------
// Button code
// --------------------------------------------------------------------------------

/* Manual override switch to add one to the counter */
void buttonEvent(){
  for(int i=0;i<2;i++){
    // read the pushbutton input pin:
    buttonState = digitalRead(playerBtnPins[i]);

    // compare the buttonState to its previous state
    if (buttonState != lastButtonState[i]) {

      // if the state has changed, increment the counter
      if (buttonState == HIGH) {

        // if the current state is HIGH then the button went from off to on:
        counters[i]++;
      }else{
        delay(50);
      }
    }

    // save the current state as the last state, for next time through the loop
    lastButtonState[i] = buttonState;
  }
}

// --------------------------------------------------------------------------------
// Hall code
// --------------------------------------------------------------------------------
/*
   Someone puts a magnet on the hall sensor, increase counter by 1
   When hall sensor is triggered wait 1 minute before it allows increasing by one.
*/
void hallEvent(){

  for(int i=0;i<2;i++){

    Serial.print("hallEvent");
    Serial.println(i);

    /* Hall Sensor */
    hallStates[i] = digitalRead(hallSensorPins[i]);

    /* hallTimeouts[i] (20 = 1sec) */
    if (hallTimeouts[i] > 0){
      hallTimeouts[i]--;
      digitalWrite(ledPin, HIGH); /* Turn on light */

    }else{
      digitalWrite(ledPin, LOW); /* Turn off light */

      if (hallStates[i] == LOW && lastHallStates[i] == HIGH) {

        /* Piece is placed on sensor */
        counters[i]++;
        hallTimeouts[i] = 200;
        delay(50);
      }
    }

    lastHallStates[i] = hallStates[i];

  }

}


// --------------------------------------------------------------------------------
// Rounds code
// --------------------------------------------------------------------------------
void blankRounds(){
  for(byte i=4;i<8;i++){
    MAX7219senddata(i+1,15);
  }
}

void nextRound(){

  /* Check if Player 1 round equals Player 2 round */
  if (roundsCounter[0] == roundsCounter[1]){

    /* If it does, then player 1 adds a round */
    if (roundsCounter[0] < 16){
      roundsCounter[0]++;                  /* Increase round */
    }else{
      roundsCounter[0] = 0;
    }

  }else{

    /* Else player 2 adds a round */
    if (roundsCounter[1] < 16){
      roundsCounter[1]++;                  /* Increase round */
    }else{
      roundsCounter[1] = 0;
    }
  }

}

void displayRound(){

  /* Player1 */
  int roundCounterPos8=roundsCounter[0] / 10;
  int roundCounterPos7=roundsCounter[0] % 10;
  roundCounterPos7 = roundCounterPos7 + 240; /* Add dot */
  MAX7219senddata(8,roundCounterPos8); /* Display */
  MAX7219senddata(7,roundCounterPos7); /* Display */

  /* Player 2 */
  int roundCounterPos6=roundsCounter[1] / 10;
  int roundCounterPos5=roundsCounter[1] % 10;
  MAX7219senddata(6,roundCounterPos6); /* Display */
  MAX7219senddata(5,roundCounterPos5); /* Display */
}

void gameOver(){
  /* Display a loop flashing it off and on for 5 seconds */
  for(byte i=0;i<10;i++){
    displayRound();
    delay(500);
    blankRounds();
    delay(500);
  }
  roundsCounter[0]=0;
  roundsCounter[1]=0;
  displayRound();
}

void buttonRound(){
  roundBtnState = digitalRead(roundBtnPin);

  if (roundBtnState != lastRoundBtnState){
    if (roundBtnState == HIGH){
      resetCounter();
      displayCountdown();
      startCounter = false;
      nextRound();

      if (roundsCounter[0] == 16 || roundsCounter[1] == 16){
        gameOver();
      }else{
        displayRound();
      }
    }

    lastRoundBtnState = roundBtnState;
  }
}


// --------------------------------------------------------------------------------
// Counterdown code
// --------------------------------------------------------------------------------
void resetCounter(){
  counter=MAXCOUNTER;
}

int currentSeconds(){
  DateTime now = RTC.now();
  return now.second();
}

void adjustCounter(){
  if (counter <= 0){
    //resetCounter();             /* wrap around to 3:00 */
  } else {
    counter--;
    displayCountdown();
  }
}

void buttonStart(){
  if (digitalRead(startBtnPin) == HIGH){
    startCounter = true;
  }
}

void calcCounter(){
  if (startCounter == true){
    if (RTCPrevSeconds != currentSeconds()){
      adjustCounter();
      RTCPrevSeconds=currentSeconds();
    }
  }else{
    resetCounter();
    displayCountdown();
  }
}

void blankCountdown(){
  for(byte i=0;i<4;i++){
    MAX7219senddata(i+1,15);
  }
}

int counterMinute(){
  int minute = counter / 60;    /* Calculate minute */
  minute += 240;                /* Add dot */
  return minute;
}

int counterSecondPart2(){
  int secs = counter % 60;
  return secs / 10;
}

int counterSecondPart1(){
  int secs = counter % 60;
  return secs % 10;
}

void displayCountdown(){
  blankCountdown();

  Serial.println(counter);

  /* Display the countdown */
  MAX7219senddata(3,counterMinute());
  MAX7219senddata(2,counterSecondPart2());
  MAX7219senddata(1,counterSecondPart1());
}

// --------------------------------------------------------------------------------
// 8 Digit 7 Segment Display - Methods
// --------------------------------------------------------------------------------


void MAX7219brightness(byte b){  //0-15 is range high nybble is ignored
  MAX7219senddata(10,b);        //intensity
}

void MAX7219init(){
  pinMode(MAX7219DIN,OUTPUT);
  pinMode(MAX7219CS,OUTPUT);
  pinMode(MAX7219CLK,OUTPUT);
  digitalWrite(MAX7219CS,HIGH);   //CS off
  digitalWrite(MAX7219CLK,LOW);   //CLK low
  MAX7219senddata(15,0);        //test mode off
  MAX7219senddata(12,1);        //display on
  MAX7219senddata(9,255);       //decode all digits
  MAX7219senddata(11,7);        //scan all
  for(int i=1;i<9;i++){
    MAX7219senddata(i,0);       //blank all
  }
}

void MAX7219senddata(byte reg, byte data){
  digitalWrite(MAX7219CS,LOW);   //CS on
  for(int i=128;i>0;i=i>>1){
    if(i&reg){
      digitalWrite(MAX7219DIN,HIGH);
    }else{
      digitalWrite(MAX7219DIN,LOW);
    }
    digitalWrite(MAX7219CLK,HIGH);
    digitalWrite(MAX7219CLK,LOW);   //CLK toggle
  }
  for(int i=128;i>0;i=i>>1){
    if(i&data){
      digitalWrite(MAX7219DIN,HIGH);
    }else{
      digitalWrite(MAX7219DIN,LOW);
    }
    digitalWrite(MAX7219CLK,HIGH);
    digitalWrite(MAX7219CLK,LOW);   //CLK toggle
  }
  digitalWrite(MAX7219CS,HIGH);   //CS off
}

// --------------------------------------------------------------------------------
// Display general functions
// --------------------------------------------------------------------------------

void wrapCounter(){
  if (counters[0] > 9) {
    counters[0] = 0;
  }
  if (counters[1] > 9) {
    counters[1] = 0;
  }
}
void displayCounters(){
  char counter1Char = char(counters[0] + 48); // Because char is converting ASCII counters 48=zero
  displayText(counter1Char,255,0,0,0,1);

  char counter2Char = char(counters[1] + 48); // Because char is converting ASCII counters 48=zero
  displayText(counter2Char,255,0,0,0,2);
}

// --------------------------------------------------------------------------------
// Display code for colorduino
// --------------------------------------------------------------------------------

//update display buffer using x,y,r,g,b format
void display(byte x, byte y, byte r, byte g, byte b) {
  byte p = (y*8)+x;   //convert from x,y to pixel number in array
  display_byte[0][p] = r;
  display_byte[1][p] = g;
  display_byte[2][p] = b;
}

//send display buffer to display
void update_display(byte addr) {
  BlinkM_sendBuffer(addr, 0, display_byte[0]);
  BlinkM_sendBuffer(addr, 1, display_byte[1]);
  BlinkM_sendBuffer(addr, 2, display_byte[2]);
}

//send data via I2C to a client
//takes approx 192microseconds from leonardo --> colourduino for this transaction
static void BlinkM_sendBuffer(byte addr, byte col, byte* disp_data) {
  for (int i = 0; i < 4; i++) {
    Wire.beginTransmission(addr);
    Wire.write(START_OF_DATA);
    Wire.write(col);
    Wire.write(i);
    for (int j = 0; j < 16; j++) {
      Wire.write(disp_data[i*16+j]);
    }
    Wire.write(END_OF_DATA);
    Wire.endTransmission();
  }
}

void displayText(char chr, unsigned char R, unsigned char G, unsigned char B, char bias, int screen) {
  unsigned char x,y;
  unsigned char i,j,temp;
  unsigned char Char;
  unsigned char chrtemp[24] = {0};
  float value;
  ColorRGB colorRGB;
  if ((bias > 8) || (bias < -8)) {
    return;
  }
  Char = chr - 32;
  j = 8 - bias;
  for(i = 8;i> 0;i--)
  {
    chrtemp[j] = pgm_read_byte(&(font8_8[Char][i]));
    j++;
  }
  for(i = 0; i < 8; i++)
  {
    temp = chrtemp[i+8];
    for(j = 0; j < 8; j++)
    {
      if(temp & 0x80)
      {
        colorRGB.r = R;
        colorRGB.g = G;
        colorRGB.b = B;
        display(i, j, colorRGB.r, colorRGB.g, colorRGB.b);
      }
      else
      {
        colorRGB.r = 0;
        colorRGB.g = 0;
        colorRGB.b = 0;
        display(i, j, colorRGB.r, colorRGB.g, colorRGB.b);
      }
      temp = temp << 1;
    }
  }

  if (screen == 1){
    update_display(DEST_I2C_ADDR1);
  } else {
    update_display(DEST_I2C_ADDR2);
  }

}
