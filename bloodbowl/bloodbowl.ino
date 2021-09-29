/*

  Bloodbowl Scoreboard

 */

#include <Wire.h>
#include <avr/pgmspace.h>

#define START_OF_DATA 0x10       //data markers
#define END_OF_DATA 0x20         //data markers
#define DEST_I2C_ADDR 0x05       //set destination I2C address (must match firmware in Colorduino module)

#define SCREENSIZEX 8            //num of LEDs accross
#define SCREENSIZEY 8            //num of LEDs down

// --------------------------------------------------------------------------------
/* Initialise */
// --------------------------------------------------------------------------------
// delay time between faces
unsigned long delaytime=200;

const int buzzer = 9; //buzzer to arduino pin 9
int inPin = 2;   // choose the input pin (for a pushbutton)
int counter = 0;
int buttonState = 0;         // current state of the button
int lastButtonState = 0;

/* Hall Sensor */
int hallSensorPin = 4;
int ledPin =  13;
int hallState = 0;
boolean lastHallState = false;

int hallTimeout = 0;

/* Ultrasonic Setup */
const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6; // Echo Pin of Ultrasonic Sensor
int sonicReads[10];
int sonicReadsPos = 0;
int sonicAvg = 0;

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
typedef struct
{
  unsigned char r;
  unsigned char g;
  unsigned char b;
} ColorRGB;


//a color with 3 components: h, s and v
typedef struct
{
  unsigned char h;
  unsigned char s;
  unsigned char v;
} ColorHSV;

// --------------------------------------------------------------------------------

void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
  pinMode(inPin, INPUT);         /* declare pushbutton */

  pinMode(ledPin, OUTPUT);
  pinMode(hallSensorPin, INPUT);

  Serial.begin(9600); // Starting Serial Terminal
}

void loop(){
  /* wrapCounter();                /\* Wrap counter back to 0 after getting to 9 *\/ */
  /* displayCounter();             /\* Display 0-9 on the 8x8 screen *\/ */
  /* buttonEvent();                /\* Add one if button pushed *\/ */
  /* hallEvent();                  /\* Add one if magnet is sensed by hall sensor *\/ */
  ultrasonicEvent();            /* Ultrasonic sensor */
}

void ultrasonicEvent(){
   long duration, inches, cm;
   pinMode(pingPin, OUTPUT);

   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH);
   inches = microsecondsToInches(duration);
   cm = microsecondsToCentimeters(duration);
   Serial.print(inches);
   Serial.print("in, ");
   Serial.print(cm);
   Serial.print("cm");
   Serial.println();
   int avg=sonicCalcAvg(cm);
   displayUltrasonic(avg);
   delay(100);
}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

int sonicCalcAvg(int cm){
  if (sonicReadsPos > 9){
    int sonicTotal=0;
    for (int i=0; i < 9; i++){
      sonicTotal = sonicTotal + sonicReads[i];
    }
    sonicAvg = sonicTotal / 9;
    sonicReadsPos = 0;

  }else{
    sonicReads[sonicReadsPos] = cm;
    sonicReadsPos++;
  }
  return sonicAvg;
}

void displayUltrasonic(int avg){
  int counter = avg - 3;
  char counterChar = char(counter + 48); // Because char is converting ASCII counters 48=zero
  displayText(counterChar,255,0,0,0);
}

void wrapCounter(){
  if (counter > 9) {
    counter = 0;
  }
}

void displayCounter(){
  char counterChar = char(counter + 48); // Because char is converting ASCII counters 48=zero
  displayText(counterChar,255,0,0,0);
}

/* Manual override switch to add one to the counter */
void buttonEvent(){
  // read the pushbutton input pin:
  buttonState = digitalRead(inPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {

    // if the state has changed, increment the counter
    if (buttonState == HIGH) {

      // if the current state is HIGH then the button went from off to on:
      counter++;

    } else {
      delay(50);
    }
  }

  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
}

/*
   Someone puts a magnet on the hall sensor, increase counter by 1
   When hall sensor is triggered wait 1 minute before it allows increasing by one.
*/
void hallEvent(){
  /* Hall Sensor */
  hallState = digitalRead(hallSensorPin);

  /* hallTimeout (20 = 1sec) */
  if (hallTimeout > 0){
    hallTimeout--;
    digitalWrite(ledPin, HIGH); /* Turn on light */

  }else{
    digitalWrite(ledPin, LOW); /* Turn off light */

    if (hallState == LOW && lastHallState == HIGH) {

      /* Piece is placed on sensor */
      counter++;
      hallTimeout = 200;
      delay(50);
    }
  }

  lastHallState = hallState;
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

void displayText(char chr, unsigned char R, unsigned char G, unsigned char B, char bias) {
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
  update_display(DEST_I2C_ADDR);
}
