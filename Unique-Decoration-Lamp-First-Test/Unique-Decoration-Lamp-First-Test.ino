#include <SPI.h>

//*************************************************ARDUINO PIN USAGE***********************************************//

#define latch_pin 2
#define blank_pin 4
#define data_pin 11
#define clock_pin 13


//*************************************************COLORWHEEL VARIABLES***********************************************//

#define myPI      3.14159265358979323846
#define myDPI     1.2732395
#define myDPI2    0.40528473

#define COLOUR_WHEEL_LENGTH 256

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
uint8_t colourB[COLOUR_WHEEL_LENGTH];
int16_t ColPos = 0;
uint16_t colourPos;
uint8_t R, G, B;

//*****************************************************BAM VARIABLES*******************************************************//


byte anode[8];//byte to write to the anode shift register, 8 of them, shifting the ON level in each byte in the array

byte red[4][8];
byte blue[4][8];
byte green[4][8];


#define     BAM_RESOLUTION 4
const byte  Size_X = 8;
const byte  Size_Y = 8;

int level=0;
int anodelevel=0;
int BAM_Bit, BAM_Counter=0;

//******************************************************SET UP***********************************************************//

void setup(){

SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV2);

noInterrupts();

TCCR1A = B00000000;
TCCR1B = B00001011;
TIMSK1 = B00000010;
OCR1A=30;

anode[0]=B00000001;
anode[1]=B00000010;
anode[2]=B00000100;
anode[3]=B00001000;
anode[4]=B00010000;
anode[5]=B00100000;
anode[6]=B01000000;
anode[7]=B10000000;

pinMode (latch_pin, OUTPUT); 

pinMode(data_pin, OUTPUT);
pinMode(clock_pin, OUTPUT);

SPI.begin();
interrupts();
clearfast();
fill_colour_wheel();
}

//******************************************************MAIN PROGRAM**********************************************************//

void loop()
{
clearfast();
  
for (int y=0; y<8; y++)
  {
    for (int x=0; x<8; x++)
    {
      LED(y, x, 15, 0, 0);
      delay(200);
    }
  }
delay(2000); 
clearfast();

for (int y=0; y<8; y++)
  {
    for (int x=0; x<8; x++)
    {
      LED(y, x, 0, 15, 0);
      delay(200);
    }
  }
delay(2000); 
clearfast();
  
for (int y=0; y<8; y++)
  {
    for (int x=0; x<8; x++)
    {
      LED(y, x, 0, 0, 15);
      delay(200);
    }
  }
delay(2000); 
clearfast();

colorMorphTable(50);
clearfast();

get_colour(colourPos+rand()%64, &R, &G, &B);
fillTable(R,G,B);
delay(5000);
clearfast();
increment_colour_pos(5);

fillTable_colorwheel();
}

//******************************************************SET LED WITH COORDINATES & COLOR*********************************************************//

void LED(int Y, int X, int R, int G, int B)
{
  Y = constrain(Y, 0, Size_Y - 1);
  X = constrain(X, 0, Size_X - 1); 
  R = constrain(R, 0, (1 << BAM_RESOLUTION) - 1);
  G = constrain(G, 0, (1 << BAM_RESOLUTION) - 1);
  B = constrain(B, 0, (1 << BAM_RESOLUTION) - 1);
  
  //int WhichByte = Y;
  
  //int WhichBit = X ;

    for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
    {
      //*** RED ***
      bitWrite(red[BAM][Y], X, bitRead(R, BAM));
      //*** GREEN ***
      bitWrite(green[BAM][Y], X, bitRead(G, BAM));
      //*** BLUE ***
      bitWrite(blue[BAM][Y], X, bitRead(B, BAM));
    }
}
ISR(TIMER1_COMPA_vect)
{
  PORTD |= 1<<blank_pin;
   
  if(BAM_Counter==8)
  BAM_Bit++;
  else
  if(BAM_Counter==24)
  BAM_Bit++;
  else
  if(BAM_Counter==56)
  BAM_Bit++;  
  BAM_Counter++;
  
  switch (BAM_Bit)
  {
  case 0:
      //Red
        SPI.transfer(red[0][level]);
              
      //Green
        SPI.transfer(green[0][level]);

      //Blue
        SPI.transfer(blue[0][level]);

      break;
    case 1:       
      //Red
        SPI.transfer(red[1][level]);
      
      //Green
        SPI.transfer(green[1][level]);

      //Blue
        SPI.transfer(blue[1][level]);

      break;
    case 2:      
      //Red
        SPI.transfer(red[2][level]);
        
       //Green
        SPI.transfer(green[2][level]);

      //Blue
        SPI.transfer(blue[2][level]);

      break;
    case 3:
      //Red
        SPI.transfer(red[3][level]);
        
      //Green
        SPI.transfer(green[3][level]);

      //Blue
        SPI.transfer(blue[3][level]);


  if(BAM_Counter==120)
    {
  BAM_Counter=0;
  BAM_Bit=0;
    }
  break;
  }
    SPI.transfer(anode[anodelevel]);//finally, send out the anode level byte
    PORTD |= 1<<latch_pin;
    PORTD &= ~(1<<latch_pin);
    delayMicroseconds(3); 
    PORTD &= ~(1<<blank_pin);//Blank pin LOW to turn on the LEDs with the new data
    delayMicroseconds(3);
    
    anodelevel++;
    level++;
    if(anodelevel==8)
    anodelevel=0;
    if(level==8)
    level=0;
    pinMode(blank_pin, OUTPUT);
} 

//******************************************************DELAY & CLEAR LED*********************************************************//

void delay_ms(uint16_t x)
{
  uint8_t y, z;
  for ( ; x > 0 ; x--){
    for ( y = 0 ; y < 90 ; y++){
      for ( z = 0 ; z < 6 ; z++){
        asm volatile ("nop");
      }
    }
  }
}


void clearfast ()
{
for (unsigned char j=0; j<8; j++)
        {
        red[0][j]   = 0;
        red[1][j]   = 0;
        red[2][j]   = 0;
        red[3][j]   = 0;
        green[0][j] = 0;
        green[1][j] = 0;
        green[2][j] = 0;
        green[3][j] = 0;
        blue[0][j]  = 0;
        blue[1][j]  = 0;
        blue[2][j]  = 0;
        blue[3][j]  = 0;
        }
}

//******************************************************FILL COLOR & MORPH COLOR*********************************************************//
void fillTable(byte R, byte G, byte B)
{  // This subroutine fills the cube with a colour
    for (byte x=0; x<8; x++)
    {  // scan thru every column
      for (byte y=0; y<8; y++)
      {  // scan thru every panel
        LED(y, x, R, G, B);
      }
    }
}

void colorMorph(int time) {
  int red, green, blue;
  int keepColorTime = time * 150;
  
  delay(keepColorTime);
  // RED + GREEN
  for(int green = 0; green <= 15; green++) {
    fillTable(15, green, 0);
    delay(time);
  }
  delay(keepColorTime);
  // GREEN - RED
  for(int red = 15; red >= 0; red --) {
    fillTable(red, 15, 0);
    delay(time);
  }
  delay(keepColorTime);
  // GREEN + BLUE
  for(int blue = 0; blue <= 15; blue++) {
    fillTable(0, 15, blue);
    delay(time);
  }
  delay(keepColorTime);
  // BLUE - GREEN
  for(int green = 15; green >= 0; green --) {
    fillTable(0, green, 15);
    delay(time);
  }
  delay(keepColorTime);
  // BLUE + RED
  for(int red = 0; red <= 15; red++) {
    fillTable(red, 0,15);
    delay(time);
  }
  delay(keepColorTime);
  // RED - BLUE + GREEN
  green = 0;
  for(int blue = 15; blue >= 0; blue --) {
    fillTable(15, green, blue);
    delay(time);
    green++;
  }
  delay(keepColorTime);
  // GREEN - RED + BLUE
  blue = 0;
  for(int red = 15; red >= 0; red --) {
    fillTable(red, 15, blue);
    delay(time);
    blue++;
  }
  delay(keepColorTime);
  // GREEN + RED + BLUE
  for(int red = 0; red <= 15; red++) {
    fillTable(red, 15, 15);
    delay(time);
  }
  delay(keepColorTime);
  // RED - GREEN - BLUE
  blue = 15;
  for(int green = 15; green >= 0; green --) {
    fillTable(15, green, blue);
    delay(time);
    blue--;
  }
}

void colorMorphTable(int time) 
{
  for(int red = 0; red <= 15; red++) {
    fillTable(red, 0,0);
    delay(time);
  }
  colorMorph(time);
  for(int red = 15; red >= 0; red--) {
    fillTable(red, 0,0);
    delay(time);
  }
  delay(500);
}

void fillTable_colorwheel(){  // This subroutine fills the cube with a colour
  uint8_t R, G, B;
  for (byte inter=0; inter<30; inter++)
  {
    for (byte x=0; x<8; x++)
    {
      for (byte y=0; y<8; y++)
      {
        get_colour(colourPos + 10*x + 10*y, &R, &G, &B);
        LED(y, x, R, G, B);      
      }
      increment_colour_pos(5);
      delay(1);
    }
  
    for (byte y=0; y<8; y++)
    {
      for (byte x=0; x<8; x++)
      {  
        get_colour(colourPos + 10*x + 10*y, &R, &G, &B);
        LED(y, x, R, G, B);      
      }
      increment_colour_pos(5);
      delay(1);
    }
  delay(500);
}
}

//**************************************************COLORWHEEL FUNCTION*************************************************************************//

void fill_colour_wheel(void) 
{
  float red, green, blue;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;
    blue = (I == 2 ? 1 : 0)*s + (I == 0 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;
    colourB[phase] = blue;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (p >= COLOUR_WHEEL_LENGTH)
    p -= COLOUR_WHEEL_LENGTH;

  *R = colourR[p];
  *G = colourG[p];
  *B = colourB[p];
}

void get_next_colour(uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (++ColPos >= COLOUR_WHEEL_LENGTH)
    ColPos -= COLOUR_WHEEL_LENGTH;

  *R = colourR[ColPos];
  *G = colourG[ColPos];
  *B = colourB[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= COLOUR_WHEEL_LENGTH)
  {
    colourPos -= COLOUR_WHEEL_LENGTH;
  }
}


//**********************************************************MATH FUNCTION*****************************************************************//

//FAST SINE APPROX
float mySin(float x){
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI){
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI)){
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//FAST COSINE APPROX
float myCos(float x){
  return mySin(x+myPI/2);
}

float myTan(float x){
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in){
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1){
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++){
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++){
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//MAP NUMBERS TO NEW RANGE
float myMap(float in, float inMin, float inMax, float outMin, float outMax){
  float out;
  out = (in-inMin)/(inMax-inMin)*(outMax-outMin) + outMin;
  return out;
}

//ROUND A NUMBER
int16_t myRound(float in){
  int8_t s = in/myAbs(in);
  return (int16_t)(s*(myAbs(in) + 0.5));
}

//ABSOLUTE VALUE
float myAbs(float in){
  return (in)>0?(in):-(in);
} 

//***************************************************************************************************************************//
