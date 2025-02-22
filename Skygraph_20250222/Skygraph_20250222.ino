#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdio.h>
// OLED FeatherWing buttons map to different pins depending on board.
// The I2C (Wire) bus may also be different.
 // 32u4, M0, M4, nrf52840 and 328p
  #define WIRE Wire
  #define VX 128
  #define VY 32
Adafruit_SSD1306 display = Adafruit_SSD1306(VX, VY, &WIRE);

//https://zenn.dev/takeboww/articles/0bc38c2be05027

// Conncet the port of the stepper motor driver
int outPorts[] = {4,5,6,7};
byte exphase[4] = {3,6,12,9};
int spD, spD1, spD2, spD5, spD10;
int speed = 416 ; // 3.3ms 
// #define BaseClk 33.3
unsigned long previousMillis = 0;        // 最後に更新した時刻
long Mstep = 0;  // steping motor position 
long Tshot = 0;  // total shot 
bool  Mdirection = true;

// Connect the relay port 
#define RELAY 8
// BUTTON 
#define BUTTON_A  2
// Coonect the PWN LED port
#define PWM_LED 3
// Connect the Variable registor (Potentio meter)
#define VR1  A0


// interval Timer setting 
#define TmBackLash  3 // backrusu
#define TmOpen  33 // open time 
#define TmClose  7  // close time 
#define LpNum  3 // multi seen times
#define TmReverse  1

long  timer,timer0 ;
int   count;
int   led_sts;
bool  Shutter_sts;
int   loopNum;
int   processEnd;
int   processNum;
bool   proc0[100] ; // 1:west 0:east 
int    proc1[100] ; // time(sec)
bool   proc2[100] ; // shutter on


void setup() {
  int i, j;

  spD1 = 17;
  spD2 = 16;
  spD10 = 1;
  spD5 = spD1 / 5;
 // set process  連続何枚とって　バックするか決める
  j = 0;
  proc0[j] = true;
  proc1[j] = TmBackLash;
  proc2[j] = false;
  j++;

  for ( i = 0 ; i< LpNum; i++) {
    proc0[j] = true;
    proc1[j] = TmOpen;
    proc2[j] = true;
    j++;
    proc0[j] = true;
    proc1[j] = TmClose;
    proc2[j] = false;
    j++;
  }
  proc0[j] = false;
  proc1[j] = TmReverse;
  proc2[j] = false;
  j++;
  processEnd = j;
  

 // set pins to output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i < 4; i++) {
    pinMode(outPorts[i], OUTPUT);
  }
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  pinMode(PWM_LED , OUTPUT);
  analogWrite(PWM_LED, 8);
  

  Serial.begin(9600);

 // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  // text display tests
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  delay(3000);

  timer0 = millis();
  Mstep = 0;
  loopNum = 0;

  drawStatus(0,1,0,0);
  delay(1000);
}


void loop(){

  char s[80];
  long timer1;
  int  ii, tm, j;

  for ( ii = 0; ii < processEnd ; ii++ ) {
    drawStatus(Tshot, loopNum , ii ,proc1[ii]-tm );
    display.display();
   
    timer1 = timerSecond ();
    Mdirection =  proc0[ii]  ;
    if ( Shutter_sts == 0 && proc2[ii] == true )
      Tshot++;
    Shutter_sts = proc2[ii]; 
    ShutterOn( Shutter_sts);
    for ( tm = 0, j = 0  ; tm < proc1[ii]; j++ ){
      tm = timerSecond() - timer1;
      if ( Mdirection) {
        moveWest();
        delay(spD1 );
        moveWest();
        delay(spD1 );
        moveWest();
        delay(spD2 );
      } 
      else {
        moveEast();
        delay(spD5);
      } 
    }
    sprintf( s,"%3d %2d : %2d %2d %2d %7ld", loopNum, ii, proc0[ii],proc1[ii], proc2[ii], Mstep);
    Serial.println( s );

  }
  loopNum++;
}

long timerSecond( void){    // second from starting 
  return( (millis() - timer0 )/1000 );
}

bool ShutterOn( bool i) {
  digitalWrite(RELAY, i );
}

 
void moveWest(void) {
  byte r;
 
  r = exphase[0];
  outOneStep( r);

  exphase[0] = exphase [1];
  exphase[1] = exphase [2];
  exphase[2] = exphase [3];
  exphase[3] = r;

  Mstep++;
}

void moveEast(void){
  byte r;
  r = exphase[0];
  outOneStep(r);

  exphase[0] = exphase [3];
  exphase[3] = exphase [2];
  exphase[2] = exphase [1];
  exphase[1] = r;

  Mstep--;
}

void outOneStep(byte p) {
  // Define a variable, use four low bit to indicate the state of port
  digitalWrite(outPorts[0], p & 1 );
  digitalWrite(outPorts[1], p & 2 );
  digitalWrite(outPorts[2], p & 4 );
  digitalWrite(outPorts[3], p & 8 );
}


/******
*******/

void drawStatus ( int Tnum, int Lp, int j ,int t) {
  char buf[22];
  long s;
  int Lnum0, Lnum1, adcValue,btnValue;


  s = timerSecond();
  sprintf(buf,"%6d",s);
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.print  ("SkyGrp Cntl ");
  display.println(buf);
  
 
  sprintf(buf,"%4d",Tnum);
  display.print("Ttl");
  display.print(buf);
  sprintf( buf,"%2d",Lp);
  display.print("(");
  display.print(buf);
  display.print(")");
  
  Lnum0 = j/2 ;
  sprintf(buf,"%2d",Lnum0 );
  display.print(buf);
  display.print("/");
  Lnum1 = processEnd/2;
  sprintf(buf,"%2d",Lnum1);
  display.print(buf);
  display.print("-");
  
  sprintf(buf,"%3d",t);
  display.println(buf);

  sprintf(buf,"%6d",Mstep);
  display.print("Step");
  display.print(buf);
  if ( Mdirection ) 
    display.print(" WEST");
  else
    display.print(" EAST");
  
  if (Shutter_sts)
    display.println(" OPN");
  else
    display.println(" CLS");
  
  
  btnValue = digitalRead(BUTTON_A)?1:0;
  adcValue = analogRead(VR1);
  sprintf( buf,"%1d", btnValue);
  display.print("Btn");
  display.print(buf);
  sprintf(buf,"%2x", adcValue );
  display.print(" Anlg");
  display.print(buf);
  
  display.display(); // actually display all of the above
  
}

