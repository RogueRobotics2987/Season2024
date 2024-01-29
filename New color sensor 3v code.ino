#include <Wire.h>
#include <Adafruit_NeoPixel.h>
 
//These values are the readings of the whitepoint of the led. Take the back of vinyl sticker and put it againts face of sensor
#define LED_RED 7540.0f
#define LED_GREEN 14470.0f
#define LED_BLUE 7270.0f
 
//Calculate the balancing factors
#define BAL_RED (LED_GREEN/LED_RED)
#define BAL_GREEN (LED_GREEN/LED_GREEN) 
#define BAL_BLUE (LED_GREEN/LED_BLUE)
 
#define I2C_ADDR 0x52
#define PIXELNUM 8
 
Adafruit_NeoPixel pixels(PIXELNUM, 13, NEO_GRB + NEO_KHZ800);
 
uint8_t readBuff[9];
uint16_t ir=0;
uint16_t red=0;
uint16_t green=0;
uint16_t blue=0;
 
void setup() {
  Wire.begin();
  Serial.begin(115200);
  pixels.begin();
  i2cWrite(0x00,0b0110);  //enable light sensor and activate rgb mode
  i2cWrite(0x04,0b01000000); //set to 16 bit resolution for 25ms response time and set measurement rate to 25ms
}
 
void loop() {
  i2cRead(0x0A,readBuff,12);
 
  ir=(readBuff[1]<<8)|readBuff[0];
  green=(readBuff[4]<<8)|readBuff[3];
  blue=(readBuff[7]<<8)|readBuff[6];
  red=(readBuff[10]<<8)|readBuff[9];
 
  red*=BAL_RED;
  green*=BAL_GREEN;
  blue*=BAL_BLUE;
 
if (red < 10000 && red > 900 && green < 3500 && green > 400 && blue < 2800 && blue > 100){
  Serial.println("ORANGE");
}


  Serial.print(ir);
  Serial.print(" ");
  Serial.print(red);
  Serial.print(" ");
  Serial.print(green);
  Serial.print(" ");
  Serial.print(blue);
  Serial.println(" ");
 
  //Normalize the readings to brightest channel then apply log scale to better discern the colors.
  float maxV=max(blue,max(red,green));
  red=255*pow(red/maxV,5);
  green=255*pow(green/maxV,5);
  blue=255*pow(blue/maxV,5);
 
  if(blue>254){ //max blue?
    red=0;
    green=0;
    blue=255;
  }
  else if(green>254){ //max green?
    red=0;
    green=255;
    blue=0;
  }
  else if(red>254){   //Max red occurs when it's red or yellow
    if(green<25){     //Check green channel, if it's below threadhold then it's not yellow
      red=255;
      green=0;
      blue=0;
    }
    else{             //red is max and green is significant portion so we can assume it's yellow (though white will also trigger this since we don't check blue channel)
      red=255;
      green=150;
      blue=0;
    }
  }
 
  //Reduce value for neopixels so it's not blinding
  red=red>>2;
  green=green>>2;
  blue=blue>>2;
 
  
 
  for(uint8_t i=0;i<PIXELNUM;i++){
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
  }
  pixels.show();
  
  delay(250);
}
 
 
void i2cWrite(uint8_t reg, uint8_t val){
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}
void i2cRead(uint8_t reg,uint8_t *val,uint16_t len){
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDR, len);
    for(uint8_t i=0;i<len;i++){
      val[i]=Wire.read();
    }




}
