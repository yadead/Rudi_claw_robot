/*
  Purpose: Basic example of IIC RGB Color Sensor TCS34725
  Notes: 
    1. All Libraries required or you can install "Adafruit TCS34725" from library manager
    2. See wiring schematic, dedicated SCL/SDA shoudl be preffered
  Author: Ben Jones 25/4/24
  Contact: benjmain.jones21@det.nsw.edu.au
*/

#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"


int servoPins[5] = {5, 6, 9, 10, 11};


byte gammatable[256];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);



class ColorSensor {
  private:
    int redThreshold;
    int greenThreshold;
    int blueThreshold;
    
  public:
    ColorSensor(int redThresh, int greenThresh, int blueThresh) {
      redThreshold = redThresh;
      greenThreshold = greenThresh;
      blueThreshold = blueThresh;
    }

    void start() {
      if (tcs.begin()) {

      } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1);
      }
      
      for (int i = 0; i < 256; i++) {
        float x = i / 255.0;
        x = pow(x, 2.5) * 255;
        gammatable[i] = 255 - x;
      }
    }

    void readColorValues(int &red, int &green, int &blue) {
      uint16_t rawR, rawG, rawB, clear;
      tcs.getRawData(&rawR, &rawG, &rawB, &clear);

      red = gammatable[rawR > 255 ? 255 : rawR];
      green = gammatable[rawG > 255 ? 255 : rawG];
      blue = gammatable[rawB > 255 ? 255 : rawB];
    }

    bool redyes(int red) {
      return red > redThreshold;
    }

    bool greenyes(int green) {
      return green > greenThreshold;
    }

    bool blueyes(int blue) {
      return blue > blueThreshold;
    }
};



class RobotArm {
  private:
    Servo servos[5];
    int positions[5][5];
    
  public:
    RobotArm(int servoPins[5], int positionsArray[5][5]) {
      for (int i = 0; i < 5; i++) {
        servos[i].attach(servoPins[i]);
      }
      setPositions(positionsArray);
    }
    
    void setPositions(int positionsArray[5][5]) {
      for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
          positions[i][j] = positionsArray[i][j];
        }
      }
    }
    
    void moveToPosition(int posIndex) {
      for (int i = 0; i < 5; i++) {
        servos[i].write(positions[posIndex][i]);
        delay(1000);
      }
    }

    void moveToColorPosition(int colorPosIndex) {
      moveToPosition(1);
      delay(1000);
      moveToPosition(colorPosIndex);
      delay(1000);
      moveToPosition(0);
    }
};




int positions[5][5] = {
  {90, 90, 90, 90, 40}, // defaultPos
  {90, 55, 140, 145, 82}, // pickupPos
  {180, 55, 140, 145, 40}, // redPos
  {0, 55, 140, 145, 40}, // greenPos
  {90, 125, 40, 45, 40}  // bluePos
};


RobotArm robotArm(servoPins, positions);
ColorSensor colorSensor(160, 100, 100);


void setup() {
  Serial.begin(9600);
  robotArm.moveToPosition(0);
  colorSensor.start();
}

void loop() {
  int red, green, blue;
  colorSensor.readColorValues(red, green, blue);

  if (colorSensor.redyes(red)) {
    robotArm.moveToColorPosition(2);
  } else if (colorSensor.greenyes(green)) {
    robotArm.moveToColorPosition(3);
  } else if (colorSensor.blueyes(blue)) {
    robotArm.moveToColorPosition(4);
  }

  delay(100);
}










//  if (int(red) >= 160) {
//    Serial.println(int("red"));
//    servo1.write(defaultpos.servo1);
//    servo2.write(defaultpos.servo2);
//    servo3.write(defaultpos.servo3);
//    servo4.write(defaultpos.servo4);
//    servo5.write(defaultpos.servo5);
//  }
//  else if (int(green) >= 160) {
//    Serial.println(int("green"));
//  }
//  else if (int(blue) >= 160) {
//    Serial.println(int("blue"));
//  }
//  else {
//    Serial.println("nothingdetected");
//  }
//  class robotclaw{
//      public:
//        int servo1;
//        int servo2;
//        int servo3;
//        int servo4;
//        int servo5;
//  };
//  int main() {
//      robotclaw defaultpos;
//      robotclaw clawpos;
//      robotclaw redpos;
//      robotclaw greenpos;
//      robotclaw bluepos;
//
//
//    //robotclaw defaultpos;
//    defaultpos.servo1 = "x";
//    defaultpos.servo2 = "x";
//    defaultpos.servo3 = "x";
//    defaultpos.servo4 = "x";
//    defaultpos.servo5 = "x";
//  
//    //robotclaw clawpos;
//    clawpos.servo1 = "x";
//    clawpos.servo2 = "x";
//    clawpos.servo3 = "x";
//    clawpos.servo4 = "x";
//    clawpos.servo5 = "x";
//
//    //robotclaw redpos;
//    redpos.servo1 = "x";
//    redpos.servo2 = "x";
//    redpos.servo3 = "x";
//    redpos.servo4 = "x";
//    redpos.servo5 = "x";
//
//    //robotclaw greenpos;
//    greenpos.servo1 = "x";
//    greenpos.servo2 = "x";
//    greenpos.servo3 = "x";
//    greenpos.servo4 = "x";
//    greenpos.servo5 = "x";
//
//    //robotclaw bluepos;
//    bluepos.servo1 = "x";
//    bluepos.servo2 = "x";
//    bluepos.servo3 = "x";
//    bluepos.servo4 = "x";
//    bluepos.servo5 = "x";
//
//
//
//
//
//  }
//
//}
//if Red > 50
//  start redcube
//if green > 50
//  start greencube
//if blue > 50
//  start bluecube
//
//  redcube:
//  pickuppos, wait 1, redpos wait 1, defaultpos
//
//  greencube:
//  pickuppos, wait 1, Greenpos, wait 1, defaultpos
//
//  bluecube:
//  pickuppos, wait 1, Bluepos, wait 1, defaultpos
//
//
//  //position angles
//
//  Defaultpos
//  base = 
//  claw = 
//  servo1 =
//  servo2 =
//  servo3 =
//
//  Clawpos
//  base = 
//  wait 0.5 claw = 
//  servo1 =
//  servo2 =
//  servo3 =
//
//  Redpos
//  base = 
//  wait 0.5 claw = 
//  servo1 =
//  servo2 =
//  servo3 =
//
//  greenpos
//  base = 
//  wait 0.5 claw = 
//  servo1 =
//  servo2 =
//  servo3 =
//
//  bluepos
//  base = 
//  wait 0.5 claw = 
//  servo1 =
//  servo2 =
//  servo3 =