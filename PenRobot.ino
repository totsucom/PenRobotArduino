/*
Trinket-M0用ロボット制御プログラム

https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
ボードマネージャで Adafruit SAMD Boards by Adafruit をインストールしてください。

電源電圧は4-6Vとなっていますが、IOピンの電圧は3.3Vなので注意。
*/

//#define DEBUG  //デバッグモード
#define ARRAY_LEN(ARR) (sizeof(ARR) / sizeof((ARR)[0]))  //配列の大きさを返すマクロ

#include "MyServo.h"
#include "PointXY.h"
#include "Motion.h"
#include "CommandBuffer.h"

//↓↓Trinket-M0にあるDotStarLED(RGBカラーLED)を操作する。ステータス表示に使う
#include "Adafruit_DotStar.h"
#include <SPI.h>
#define DATAPIN   7
#define CLOCKPIN  8
#define COLOR_IDLING  0x000000
#define COLOR_ORIGIN  0x004000
#define COLOR_MOVING  0x000040
Adafruit_DotStar strip = Adafruit_DotStar(1, DATAPIN, CLOCKPIN, DOTSTAR_BGR);
//↑↑ここまで

#define RED_LED 13 //Trinket-M0にある赤色LED。エラー表示に使う

float current_x, current_y;     //現在のペンの位置
bool current_position = false;  //current_x,yが有効か？
bool pen_status = false;        //ペンの状態 trueで下がっている

typedef enum {
    IDLING,
    ORIGIN,
    MOVING
} STATUS;
STATUS status = IDLING;

//↓↓以下サーボの定義
#define SERVO_SPEED 0.6F     //サーボ動作速度 度/ミリ秒　動作が確実に完了する時間を計算するため、内部でさらに この値*20% + 20ms の時間的余裕をみている
#define SERVO_MIN_PULSE 500  //サーボの有効パルス幅(最小)
#define SERVO_MAX_PULSE 2400 //サーボの有効パルス幅(最大)

#define LEFT_SERVO 0 //GPIO
float leftAngles[] = {4.0F, 10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F, 80.0F, 90.0F, 100.0F, 110.0F, 120.0F, 130.0F, 140.0F, 150.0F, 160.0F, 170.0F, 180.0F, 184.0F};
int   leftPulses[] = { 550,   620,   730,   830,   930,  1030,  1135,  1225,  1310,  1405,   1485,   1575,   1655,   1745,   1840,   1930,   2025,   2130,   2240,   2270};
float leftDefaultAngle = 184.0F;

#define RIGHT_SERVO 4 //GPIO
float rightAngles[] = {4.0F, 10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F, 80.0F, 90.0F, 100.0F, 110.0F, 120.0F, 130.0F, 140.0F, 150.0F, 160.0F, 170.0F, 180.0F, 184.0F};
int   rightPulses[] = { 570,   650,   760,   860,   960,  1050,  1140,  1230,  1315,  1405,   1500,   1580,   1670,   1760,   1845,   1940,   2030,   2130,   2245,   2290};
float rightDefaultAngle = 4.0F;

#define UPDOWN_SERVO 3 //GPIO
//ペンの高さを調節するパラメータ。これはデフォルト値だが、コマンドから調整できる
float updownOffAngleAtNear = 90.0F;     //Near:y=35mm
float updownOffAngleAtFar = 90.0F;      //Far:y=110mm
float updownWriteAngleAtNear = 88.0F;
float updownWriteAngleAtFar = 88.0F;

//左(腕)サーボの初期設定
MyServo leftServo = MyServo(
    LEFT_SERVO,   //GPIO
    leftAngles,   //補正データ(角度)
    leftPulses,   //補正データ(パルス)
    ARRAY_LEN(leftAngles),    //補正データの数
    SERVO_SPEED,              //サーボ速度
    leftDefaultAngle,         //原点角度
    leftAngles[0], leftAngles[ARRAY_LEN(leftAngles) - 1], //有効な角度の範囲
    SERVO_MIN_PULSE, SERVO_MAX_PULSE     //有効なパルス幅
  );   

//右(腕)サーボの初期設定
MyServo rightServo = MyServo(
    RIGHT_SERVO,   //GPIO
    rightAngles,   //補正データ(角度)
    rightPulses,   //補正データ(パルス)
    ARRAY_LEN(rightAngles),   //補正データの数
    SERVO_SPEED,              //サーボ速度
    rightDefaultAngle,        //原点角度
    rightAngles[0], rightAngles[ARRAY_LEN(rightAngles) - 1], //有効な角度の範囲
    SERVO_MIN_PULSE, SERVO_MAX_PULSE     //有効なパルス幅
  );   

//上下サーボの初期設定
MyServo updownServo = MyServo(
    UPDOWN_SERVO,   //GPIO
    leftAngles,   //補正データ(角度) ※あまり精度はいらないので左腕サーボの補正値を流用
    leftPulses,   //補正データ(パルス)
    ARRAY_LEN(leftAngles),    //補正データの数
    SERVO_SPEED,              //サーボ速度
    updownOffAngleAtNear,     //原点角度
    leftAngles[0], leftAngles[ARRAY_LEN(leftAngles) - 1], //有効な角度の範囲
    SERVO_MIN_PULSE, SERVO_MAX_PULSE     //有効なパルス幅
  );   

//エラーLEDを一定時間表示(しないと見えない)する
bool current_error = false; //現在のエラー状態
bool display_error = false; //表示中のエラー状態
long off_delay;             //この時刻(millis)まで最低限LEDをON

//情報表示管理
bool show_information = false;//表示したい
long show_delay;            //show_delayまで何もなければ表示

long aligin_delay;          //IDLE一定時間後に原点復帰


/**
 * 赤色LEDでエラーを表示する
 */
void setError(bool flag)
{
  if (flag) {
    digitalWrite(RED_LED, HIGH);
    current_error = true;
    display_error = true;
    off_delay = millis() + 200; //最低200ms点灯
  } else {
    current_error = false;
    if (display_error && millis() >= off_delay) {
      display_error = false;
      digitalWrite(RED_LED, LOW);
    }
  }
}

/**
 * 赤色LEDの状態を更新する(ループで呼ぶ)
 */
void updateError()
{
  if (!current_error && display_error && millis() >= off_delay) {
    display_error = false;
    digitalWrite(RED_LED, LOW);
  }
}

//ロボットの寸法
#define UPPER_ARM_LENGTH 40.0F
#define LOWER_ARM_LENGTH 60.0F
#define ARM_TO_PEN 14.0F
#define SERVO_POS_X 11.0F  //サーボ軸のX座標。右腕サーボは負の値になる
#define SERVO_POS_Y 0.0F   //サーボ軸のY座標

/**
 * サーボ角度から現在位置を計算
 * 成功時はtrueを返し、current_x,current_yに位置を設定
 */
bool calcCurrentPosition() {

  if (current_position) {
    //すでに計算済み
    return true;
  }

  //左肘の座標を計算
  PointXY L1 = PointXY(0, UPPER_ARM_LENGTH);
  L1.rotate(leftServo.getCurrentAngle(), false);
  L1.add(SERVO_POS_X, SERVO_POS_Y);

  //右肘の座標を計算
  PointXY R1 = PointXY(0, -UPPER_ARM_LENGTH);
  R1.rotate(rightServo.getCurrentAngle(), false);
  R1.add(-SERVO_POS_X, SERVO_POS_Y);

  //左手と右手の交点を計算 候補は2点 cp1 cp2
  PointXY cp1, cp2;
  if (!PointXY::circleCrossPoints(L1.x, L1.y, LOWER_ARM_LENGTH, R1.x, R1.y, LOWER_ARM_LENGTH, &cp1, &cp2)) {
    //あり得ない
    return false;
  }

  //肘の曲がりが弱いほうが正解
  //float ang1 = PointXY::angle(L1.x - SERVO_POS_X, L1.y - SERVO_POS_Y, cp1.x - L1.x, cp1.y - L1.y);
  //float ang2 = PointXY::angle(L1.x - SERVO_POS_X, L1.y - SERVO_POS_Y, cp2.x - L1.x, cp2.y - L1.y);

  //交点座標のy値の大きなほうが正解
  PointXY w;
  if (cp1.y > cp2.y) { //(ang1 > ang2) {
    //cp1が正解

    //左肘→手首ベクトル
    w.set(cp1.x - L1.x, cp1.y - L1.y);

  } else {
    //cp2が正解

    //左肘→手首ベクトル
    w.set(cp2.x - L1.x, cp2.y - L1.y);
  }

  //ペン先座標を計算
  w.mul((ARM_TO_PEN + LOWER_ARM_LENGTH) / LOWER_ARM_LENGTH);
  w.add(L1.x, L1.y);

  current_x = w.x;
  current_y = w.y;
  current_position = true;
  return true;
}

/*
 * 指定座標における、サーボ角度を計算
 */
bool calcAngle(float x, float y, float *leftAngle, float *rightAngle) {

  //左腕の肘にあたる座標を計算(候補は２点 cp1 cp2)
  PointXY cp1, cp2;
  if (!PointXY::circleCrossPoints(SERVO_POS_X, SERVO_POS_Y, UPPER_ARM_LENGTH, x, y, ARM_TO_PEN + LOWER_ARM_LENGTH, &cp1, &cp2)) {
    //作業範囲を超えた場合
#ifdef DEBUG
    Serial.println("calcAngle error1");
#endif
    return false;
  }

  //cp1の角度はどうか？（角度をマイナスしているのはPC上の回転方向(clockwise)とサーボの回転方向(unticlock)が逆なため）
  //左サーボ軸座標(SERVO_POS_X, SERVO_POS_Y)
  float ang = -PointXY::signedAngle(0.0F, 1.0F, cp1.x - SERVO_POS_X, cp1.y - SERVO_POS_Y);

  PointXY v;
  if (ang >= leftAngles[0] && ang <= leftAngles[ARRAY_LEN(leftAngles) - 1]) {
    //cp1が正解

    //左手手首の座標を計算（左手でペンを持ち、右手は左手手首を持っている）
    v.set(x - cp1.x, y - cp1.y);
    v.mul(LOWER_ARM_LENGTH / (ARM_TO_PEN + LOWER_ARM_LENGTH));
    v.add(cp1.x, cp1.y);

  } else {

    //cp2の角度ではどうか？
    ang = -PointXY::signedAngle(0.0F, 1.0F, cp2.x - SERVO_POS_X, cp2.y - SERVO_POS_Y);

    if (ang >= leftAngles[0] && ang <= leftAngles[ARRAY_LEN(leftAngles) - 1]) {
      //cp2が正解

      //左手手首の座標を計算（左手でペンを持ち、右手は左手手首を持っている）
      v.set(x - cp2.x, y - cp2.y);
      v.mul(LOWER_ARM_LENGTH / (ARM_TO_PEN + LOWER_ARM_LENGTH));
      v.add(cp2.x, cp2.y);

    } else {

      //このパターンはあるのか?
#ifdef DEBUG
      Serial.println("calcAngle error2");
#endif
      return false;
    }
  }
  *leftAngle = ang;


  //右腕の肘にあたる座標を計算(候補は２点 cp1 cp2)
  if (!PointXY::circleCrossPoints(-SERVO_POS_X, SERVO_POS_Y, UPPER_ARM_LENGTH, v.x, v.y, LOWER_ARM_LENGTH, &cp1, &cp2)) {
    //作業範囲を超えた場合
#ifdef DEBUG
    Serial.println("calcAngle error3");
#endif
    return false;
  }

  //cp1の角度はどうか？（角度をマイナスしているのはPC上の回転方向(clockwise)とサーボの回転方向(unticlock)が逆なため）
  //右サーボ軸座標(-SERVO_POS_X, SERVO_POS_Y)
  ang = -PointXY::signedAngle(0.0F, -1.0F, cp1.x + SERVO_POS_X, cp1.y - SERVO_POS_Y);

  if (ang < rightAngles[0] || ang > rightAngles[ARRAY_LEN(rightAngles) - 1]) {
    //cp1は不正解

    //cp2ではどうか?
    ang = -PointXY::signedAngle(0.0F, -1.0F, cp2.x + SERVO_POS_X, cp2.y - SERVO_POS_Y);

    if (ang < rightAngles[0] || ang > rightAngles[ARRAY_LEN(rightAngles) - 1]) {
      //cp2も不正解(あり得る?)
#ifdef DEBUG
      Serial.println("calcAngle error4");
      Serial.println(cp1.toString());
      Serial.println(cp2.toString());
#endif
      return false;
    }
  }
  *rightAngle = ang;
  return true;
}

/**
 * y座標から上下サーボの角度を計算する
 */
float calcUpdownAngle(float y)
{
  if (pen_status) {
    return (y - 35.0F) / (110.0F - 35.0F) * (updownWriteAngleAtFar - updownWriteAngleAtNear) + updownWriteAngleAtNear;
  } else {
    return (y - 35.0F) / (110.0F - 35.0F) * (updownOffAngleAtFar - updownOffAngleAtNear) + updownOffAngleAtNear;
  }
}

Move move;
Turn turn;

/* XYサーボを指定座標に動かす指示 */
void MoveTo(float x, float y)
{
#ifdef DEBUG
  Serial.print("MoveTo(");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.println(")");
#endif
  
  //新しい座標をセット
  move.set(current_x, current_y, x, y);

  //リセット
  turn.stop();

  //残りの処理はDoIt()に任せる
}

/* 左右のサーボを指定角度に動かす指示 */
void TurnTo(float left_ang, float right_ang)
{
  //使わない
  move.stop();

  //新しい角度をセット
  turn.set(leftServo.getCurrentAngle(), left_ang, rightServo.getCurrentAngle(), right_ang);

  //残りの処理はDoIt()に任せる
}

/**
 * XYサーボを指定座標に動かす。ループで呼ぶ
 * 関数がfalseを返すと、次の座標指示ができる
 */
bool DoIt()
{
  static bool working =false;
  //Serial.println("DoIt()");
  
  //サーボの動作が完了するまで待つ
  if (leftServo.isRunning() || rightServo.isRunning() || updownServo.isRunning()) {
    return true;
  }

  //移動完了
  if (!move.isRunning() && !turn.isRunning()) {

    if (working) {

#ifdef DEBUG
      Serial.println("no more position, position calculated.");
#endif
  
      //原点でなかったらアイドリング表示にする  BUSY<=>IDLEが連続してしまうため、メインループに移行
      /*if (status != ORIGIN) {
        setStatus(IDLING);
        show_information = true;
        show_delay = millis() + 500UL;
      }*/
  
      //座標を再計算
      calcCurrentPosition();

      working = false;
    }
    
    return false;
  }
  working = true;

  //次の角度を取得
  float aL, aR;
  if (turn.calc(&aL, &aR)) {

#ifdef DEBUG
    Serial.println("got next angle");
#endif

    //サーボを動かす
    leftServo.setAngle(aL);
    rightServo.setAngle(aR);
    current_position = false;

    //上下サーボも追随して動かす
    if (calcCurrentPosition()) {
      updownServo.setAngle(calcUpdownAngle(current_y));
      setError(false);
    } else {
      setError(true);
    }

    if (aL == leftDefaultAngle && aR == rightDefaultAngle) {
      //原点座標になった
      setStatus(ORIGIN);
      showStatus();
    } else {
      //動作中
      setStatus(MOVING);
    }
    return true;
  }

  //次の座標を取得
  float nx, ny;
  if (!move.calc(&nx, &ny)) {
    //座標が無い（多分ここまで来ない）
    return false;
  }

#ifdef DEBUG
  Serial.print("got next position (");
  Serial.print(nx);
  Serial.print(",");
  Serial.print(ny);
  Serial.println(")");
#endif

  //サーボ角度を取得
  float angL, angR;
  if (!calcAngle(nx, ny, &angL, &angR)) {

    //計算できない場合はエラー中止
    move.stop();
    turn.stop();
    setError(true);
    return false;
  } else {
    setError(false);
  }

#ifdef DEBUG
  Serial.print("got next angle (");
  Serial.print(angL);
  Serial.print(",");
  Serial.print(angR);
  Serial.println(")");
#endif

  //新しい角度をセット
  turn.set(leftServo.getCurrentAngle(), angL, rightServo.getCurrentAngle(), angR);

/*
  //最初の角度を取得
  if (turn.calc(&aL, &aR)) {

    //サーボを動かす
    leftServo.setAngle(aL);
    rightServo.setAngle(aR);
    current_position = false;

    //上下サーボも動かす
    updownServo.setAngle(calcUpdownAngle(ny));
  }
*/
  return true;
}

/**
 * ステータスを設定しDotStarLEDの変更する
 */
void setStatus(STATUS new_status) {
  if (status != new_status) {
    status = new_status;
    if (status == ORIGIN) {
      strip.setPixelColor(0, COLOR_ORIGIN);
      Serial.println("{\"status\":\"ORIGN\"}");
    } else if (status == MOVING) {
      strip.setPixelColor(0, COLOR_MOVING);
      Serial.println("{\"status\":\"BUSY\"}");
    } else {
      strip.setPixelColor(0, COLOR_IDLING);
      Serial.println("{\"status\":\"IDLE\"}");
    }
    strip.show();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  //エラーLED
  pinMode(RED_LED, OUTPUT);
  setError(false);

  //ステータスLED
  setStatus(ORIGIN);

  //初期位置を計算
  calcCurrentPosition();
}

//受信コマンド
enum {
 cmd_move_x = 128,  // <f>　　　ペンをX方向に相対移動する 単位[mm]  例 x-1.5
 cmd_move_y,        // <f>　　　ペンをY方向に相対移動する 単位[mm]
 cmd_angle_l,       // <f>　　　左サーボを相対回転する　　単位[度]
 cmd_angle_r,       // <f>　　　右サーボを相対回転する　　単位[度]
 cmd_angle_u,       // <f>　　　上下サーボを相対回転する　単位[度]
 cmd_move_to_near,  //  　　　　ペンの位置を近距離側へ移動(snコマンド用)
 cmd_set_near_angle,// <f>　　 上下サーボの近距離側のペンの書く高さを絶対角度で設定する　単位[度]
 cmd_move_to_far,   // 　　　　 ペンの位置を遠距離側へ移動(sfコマンド用)
 cmd_set_far_angle, // <f>　　 上下サーボの遠距離側のペンの書く高さを絶対角度で設定する　単位[度]
 cmd_pen_up,        // 　　　　 ペンを上げる
 cmd_pen_down,      // 　　　　 ペンを下げる（書くポジション）
 cmd_move_to,       // <f><f>  ペンを上げた状態で絶対座標に移動する　　　　単位[mm]　例 m-20,50.5
 cmd_line_to,       // <f><f>  ペンを下げた状態で絶対座標に移動する（書く）
 cmd_align,         // 　　　　　ペンを上げて原点位置に戻る
 cmd_info,          // 　　　　　角度や座標の情報を表示
 cmd_last
};

//受信コマンド別必要なバイト数
//dataSize[command - 128]
byte dataSize[] = {
 5,  // <f>　　　ペンをX方向に相対移動する 単位[mm]  例 x-1.5
 5,  // <f>　　　ペンをY方向に相対移動する 単位[mm]
 5,  // <f>　　　左サーボを相対回転する　　単位[度]
 5,  // <f>　　　右サーボを相対回転する　　単位[度]
 5,  // <f>　　　上下サーボを相対回転する　単位[度]
 1,  // 　　　　 ペンの位置を近距離側へ移動(snコマンド用)
 5,  // <f>　　 上下サーボの近距離側のペンの書く高さを絶対角度で設定する　単位[度]
 1,  // 　　　　 ペンの位置を遠距離側へ移動(sfコマンド用)
 5,  // <f>　　 上下サーボの遠距離側のペンの書く高さを絶対角度で設定する　単位[度]
 1,  // 　　　　 ペンを上げる
 1,  // 　　　　 ペンを下げる（書くポジション）
 9,  // <f><f>  ペンを上げた状態で絶対座標に移動する　　　　単位[mm]　例 m-20,50.5
 9,  // <f><f>  ペンを下げた状態で絶対座標に移動する（書く）
 1,  // 　　　　　ペンを上げて原点位置に戻る
 1   // 　　　　　角度や座標の情報を表示
};

//受信コマンドを保持するバッファ
CommandBuffer cb(1000);

//完了しない、読みかけの受信コマンド
#define MAX_COMMAND_LENGTH 9
unsigned char command_reading_ontheway[MAX_COMMAND_LENGTH];
int command_reading_ontheway_length = 0;
int command_reading_complete_length = 0;
unsigned long last_receive_millis;  //最後に受信したタイミング(長すぎると破棄)
bool command_receivable = false;    //コマンドバイトは0xffの後にしかやってこない

void loop() {

  //コマンド受信処理
  if (Serial.available()) {
    if (millis() > last_receive_millis + 20) {
      if (command_reading_ontheway_length > 0) {
        //古いデータは破棄。バッファ満タン時はcommand_reading_onthewayが空のはずなので、正常時に破棄されない
        command_reading_ontheway_length = 0;
        Serial.print("data disposed");
      }
    }
    while (Serial.available() && cb.getFreeSize() >= MAX_COMMAND_LENGTH) {
      //1バイト読む
      unsigned char incoming_byte = Serial.read();
      last_receive_millis = millis();

      if (command_receivable && command_reading_ontheway_length == 0 &&
        incoming_byte >= 128 && incoming_byte < cmd_last) {

        //コマンド確定
        if (dataSize[incoming_byte - 128] == 1) {
          //1文字コマンドを追加
          cb.addCommand(incoming_byte);
#ifdef DEBUG
          Serial.println("New command added");
#endif
        } else {
          //追加のパラメーター待ち
          command_reading_ontheway[0] = incoming_byte;
          command_reading_ontheway_length = 1;
          command_reading_complete_length = dataSize[incoming_byte - 128];
        }
      } else if (command_reading_ontheway_length >= 1 &&
        command_reading_ontheway_length < command_reading_complete_length) {
        
        //パラメーターを追加
        command_reading_ontheway[command_reading_ontheway_length++] = incoming_byte;
        if (command_reading_ontheway_length == command_reading_complete_length) {
          //パラメーターが満たされたのでコマンドを追加
          cb.addCommand(&command_reading_ontheway[0], command_reading_complete_length);
          command_reading_ontheway_length = 0;
#ifdef DEBUG
          Serial.println("New command(param) added");
#endif
        }
      } else if (incoming_byte != 0xff) {
        //いずれにも合致しない場合は破棄
        command_reading_ontheway_length = 0;
        Serial.print("unknown command ");
        Serial.println(incoming_byte);
      }
      command_receivable = (incoming_byte == 0xff);
    }

    if (cb.getFreeSize() < MAX_COMMAND_LENGTH) {
      Serial.print("buffer full");
    }
  }

  if (DoIt()) {
    //DoItでサーボ制御した
    aligin_delay = millis() + 3000UL;

  } else if (cb.getCount() > 0) {
    //新しいコマンドを処理

    unsigned char cmd = cb.peekCommand();
#ifdef DEBUG
    Serial.print("Process command ");
    Serial.println(cmd);
#endif
    
    float f1, f2;
    switch ((int)cmd) {
      case cmd_move_x:   // <f>　　　ペンをX方向に相対移動する 単位[mm]  例 x-1.5
        cb.readCommand(&cmd, &f1);
        MoveTo(current_x + f1, current_y);
        break;
      case cmd_move_y:   // <f>　　　ペンをY方向に相対移動する 単位[mm]
        cb.readCommand(&cmd, &f1);
        MoveTo(current_x, current_y + f1);
        break;
      case cmd_angle_l:  // <f>　　　左サーボを相対回転する　　単位[度]
        cb.readCommand(&cmd, &f1);
        TurnTo(leftServo.getCurrentAngle() + f1, rightServo.getCurrentAngle());
        break;
      case cmd_angle_r:  // <f>　　　右サーボを相対回転する　　単位[度]
        cb.readCommand(&cmd, &f1);
        TurnTo(leftServo.getCurrentAngle(), rightServo.getCurrentAngle() + f1);
        break;
      case cmd_angle_u:  // <f>　　　上下サーボを相対回転する　単位[度]
        cb.readCommand(&cmd, &f1);
        updownServo.setAngle(updownServo.getCurrentAngle() + f1);
        break;
      case cmd_move_to_near:   //  　　　　ペンの位置を近距離側へ移動(snコマンド用)
        cb.readCommand(&cmd);
        MoveTo(0.0F, 35.0F);
        break;
      case cmd_set_near_angle: // <f>　　 上下サーボの近距離側のペンの書く高さを絶対角度で設定する　単位[度]
        cb.readCommand(&cmd, &f1);
        if (f1 > 0.0F) {
          updownWriteAngleAtNear = f1;
          updownOffAngleAtNear = f1 + 10.0F;
        }
        break;
      case cmd_move_to_far:    // 　　　　 ペンの位置を遠距離側へ移動(sfコマンド用)
        cb.readCommand(&cmd);
        MoveTo(0.0F, 110.0F);
        break;
      case cmd_set_far_angle:  // <f>　　 上下サーボの遠距離側のペンの書く高さを絶対角度で設定する　単位[度]
        cb.readCommand(&cmd, &f1);
        if (f1 > 0.0F) {
          updownWriteAngleAtFar = f1;
          updownOffAngleAtFar = f1 + 6.0F;
        }
        break;
      case cmd_pen_up:         // 　　　　 ペンを上げる
        cb.readCommand(&cmd);
        if (calcCurrentPosition()) {
          pen_status = false;
          updownServo.setAngle(calcUpdownAngle(current_y));
          setError(false);
        } else {
          setError(true);
        }
        break;
      case cmd_pen_down:       // 　　　　 ペンを下げる（書くポジション）
        cb.readCommand(&cmd);
        if (calcCurrentPosition()) {
          pen_status = true;
          updownServo.setAngle(calcUpdownAngle(current_y));
          setError(false);
        } else {
          setError(true);
        }
        break;
      case cmd_move_to:       // <f><f>  ペンを上げた状態で絶対座標に移動する　　　　単位[mm]　例 m-20,50.5
        cb.readCommand(&cmd, &f1, &f2);
        if (pen_status) {
          //ペンを上げる
          if (calcCurrentPosition()) {
            setError(false);
            pen_status = false;
            updownServo.setAngle(calcUpdownAngle(current_y));
          } else {
            setError(true);
          }        
        }
        //移動指示
        MoveTo(f1, f2);
        break;
      case cmd_line_to:       // <f><f>  ペンを下げた状態で絶対座標に移動する（書く）
        cb.readCommand(&cmd, &f1, &f2);
        if (!pen_status) {
          //ペンを下げる
          if (calcCurrentPosition()) {
            setError(false);
            pen_status = true;
            updownServo.setAngle(calcUpdownAngle(current_y));
          } else {
            setError(true);
          }        
        }
        //移動指示
        MoveTo(f1, f2);
        break;
      case cmd_align:         // 　　　　　ペンを上げて原点位置に戻る
        cb.readCommand(&cmd);
        if (pen_status) {
          //ペンを上げる
          if (calcCurrentPosition()) {
            setError(false);
            pen_status = false;
            updownServo.setAngle(calcUpdownAngle(current_y));
          } else {
            setError(true);
          }        
        }
        //移動指示
        TurnTo(leftDefaultAngle, rightDefaultAngle);
        break;
      case cmd_info:           // 　　　　　角度や座標の情報を表示
        cb.readCommand(&cmd);
        showStatus();
        break;
    }

  } else {
    //何もすることがなくなった

    if (status == MOVING) { //ALIGINの場合はDoIt()内で事前に処理済
      setStatus(IDLING);
      show_information = true;
      show_delay = millis() + 500UL;
    }

    //自動原点復帰　ブラウザ側の制御が面倒なのでやめ
    //if (status != ORIGIN && millis() > aligin_delay) {
    //  cb.addCommand(cmd_align);
    //}
  }
  
  updateError();
}

void showStatus() {
  Serial.print("{\"l\":");
  Serial.print(leftServo.getCurrentAngle());  
  Serial.print(",\"r\":");
  Serial.print(rightServo.getCurrentAngle());  
  Serial.print(",\"u\":");
  Serial.print(updownServo.getCurrentAngle());  
  Serial.print(",\"x\":");
  Serial.print(current_x);
  Serial.print(",\"y\":");
  Serial.print(current_y);
  if (pen_status)
    Serial.println(",\"pen\":1}");
  else
    Serial.println(",\"pen\":0}");
  show_information = false;
}
