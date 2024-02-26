#include <Adafruit_DotStar.h>

/* Hello future me, or other people interested in Adafruit DotStar's led library.
      Setup 
 To setup an Arduino (uno) to correctly use LED strips, know this.
 - Every strip has 2 Digital Input (DI) pins. set them accordingly.
 - Include the DotStar Library.
 - #define NUMPIXELS, DATAPIN, and CLOCKPIN. the "PIN" variables correspond to the DI pins you set.
 - then, depending on if you want to use a full strip or a small number of pixels:
 
 IF using 20 Pixels or Higher:
 Use a power supply. Preferrably a battery into a bus with multiple outputs.
 Plug in the thicker wires of the strip into the - and + 5V inputs. USE 5 VOLTS. IF HIGHER OR LOWER THINGS MAY GO KAPOOT.
 be sure to plug in the smaller 5V and GND wire into the ardino.  

 IF not:
 plug in the 5V wire into the 5V input, and GND to any GND input. easy as pie.

        Pixel Commands
 strip.show(); - the most important of the other commands. shows any changes made once and once only.
 strip.setPixelColor(pixel, r,g,b); "pixel" is the specific pixel you want to change the color for. use ints if you want to spice them up!
 strip.clear(); self explanatory. clears the strip changes.
 strip.begin(); Initalizes the strip to be ready for use.
 


*/
#define NUMPIXELS 67// Number of LEDs in strip
#define DATAPIN    9
#define CLOCKPIN   8
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR); /* Sometimes your led strip doesnt have the right colors. switch the RGB letters around and itll work. */

int led = -NUMPIXELS / 2;
int maxled = NUMPIXELS-1;
int led2 = 0;
int note;
int notecheck = 5;
int teamcheck = 6;
int pin2 = 2;
int pin3 = 3; 
int pin4 = 4;
int team;
void setup() {
  // put your setup code here, to run once:
strip.begin();
strip.clear();
strip.show();
pinMode(notecheck, INPUT);
pinMode(teamcheck, INPUT);
Serial.begin(9600);
}
void patternChase(int r, int g, int b) {
       note = digitalRead(notecheck);
       team = digitalRead(teamcheck);       
       strip.setPixelColor(led, r, g, b);
       led++;
       strip.setPixelColor(led2, r * 3, g * 3, b * 3);
       led2++;
       led2++;
       strip.show();
       if (led >= maxled) { 
           led2 = 0;
           led = -NUMPIXELS / 2;
       } 
}

void loop() {
  // put your main code here, to run repeatedly:
 
note = digitalRead(notecheck);
team = digitalRead(teamcheck);
Serial.println(pin2);

if (digitalRead(pin2) == 0 && digitalRead(pin3) == 0 && digitalRead(pin4) == 0){
strip.fill(strip.Color(50,0,0),0,maxled); //red no game piece
}
else if (digitalRead(pin2) == 0 && digitalRead(pin3) == 0 && digitalRead(pin4) == 1){
strip.fill(strip.Color(50,25,0),0,maxled); //yellow searching
}
else if (digitalRead(pin2) == 0 && digitalRead(pin3) == 1 && digitalRead(pin4) == 0){
strip.fill(strip.Color(0,25,0),0,maxled); //green have game piece
}
else if (digitalRead(pin2) == 1 && digitalRead(pin3) == 0 && digitalRead(pin4) == 0){
strip.fill(strip.Color(0,0,25),0,maxled); //blue ready to shoot
}
else if (digitalRead(pin2) == 1 && digitalRead(pin3) == 1 && digitalRead(pin4) == 0){
strip.fill(strip.Color(50,0,20),0,maxled); //purple
}
else if (digitalRead(pin2) == 0 && digitalRead(pin3) == 1 && digitalRead(pin4) == 1){
strip.fill(strip.Color(50,8,0),0,maxled); //orange 
}

strip.show();

/*  if (note == HIGH && note == HIGH) {
      patternChase(5,5,0); 
     }
   else if (team == HIGH && note == LOW) {
      patternChase(5,0,0);
     } 
   else if (note == LOW && team == LOW) {
   patternChase(0,0,5);
   }
   */
}
