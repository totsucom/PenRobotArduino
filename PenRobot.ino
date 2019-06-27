/*
Trinket-M0用ロボット制御プログラム

https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
ボードマネージャで Adafruit SAMD Boards by Adafruit をインストールしてください。

電源電圧は4-6Vとなっていますが、IOピンの電圧は3.3Vなので注意。
*/
#define DEBUG
#define ARRAY_LEN(ARR) (sizeof(ARR) / sizeof((ARR)[0]))

#include "MyServo.h"
#include "PointXY.h"

MyServo *leftServo;

float angles[] = {0.0F, 10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F, 80.0F, 90.0F, 100.0F, 110.0F, 120.0F, 130.0F, 140.0F, 150.0F, 160.0F, 170.0F, 180.0F};
int   pulses[] = { 607,   713,   818,   920,  1022,  1120,  1207,  1302,  1388,  1478,   1563,   1650,   1740,   1830,   1923,   2017,   2120,   2222,   2325};


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  leftServo = new MyServo(
    4,            //GPIO
    angles,
    pulses,
    ARRAY_LEN(angles), //data count of array
    0.6F,         //servo speed = degree per milliseconds
    90.0F,        //default angle
    0.0F, 180.0F, //min, max angle
    500, 2400     //min, max pulse
  );   

  PointXY a(100,100);
  PointXY b(101,101);
  a = b;
  PointXY c = a + b;
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    int pulse = atoi(s.c_str());
    Serial.print("Pulse(");
    Serial.print(pulse);
    Serial.print(") => ");
    Serial.println(leftServo->calcAngle(pulse));
  }
}
