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
bool json_mode = false;         //情報表示をjson形式で表示するか？

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
  
      //原点でなかったらアイドリング表示にする
      if (status != ORIGIN) {
        setStatus(IDLING);
        //showStatus();
        show_information = true;
        show_delay = millis() + 1000UL;
      } else {
        Serial.print("align");
      }
  
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
    } else if (status == MOVING) {
      strip.setPixelColor(0, COLOR_MOVING);
    } else {
      strip.setPixelColor(0, COLOR_IDLING);
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

/**
 * シリアルポートのコマンド一覧
 * <n> 整数(int)
 * <f> 実数(float)
 * 
 * 受信コマンド
 * x<f>　　　ペンをX方向に相対移動する 単位[mm]  例 x-1.5
 * y<f>　　　ペンをY方向に相対移動する 単位[mm]
 * l<f>　　　左サーボを相対回転する　　単位[度]
 * r<f>　　　右サーボを相対回転する　　単位[度]
 * u<f>　　　上下サーボを相対回転する　単位[度]
 * mn　　　　ペンの位置を近距離側へ移動(snコマンド用)
 * sn　　　　上下サーボ現在の角度を近距離側のペンの書く高さに設定する
 * sn<f>　　 上下サーボの近距離側のペンの書く高さを絶対角度で設定する　単位[度]
 * mf　　　　ペンの位置を遠距離側へ移動(sfコマンド用)
 * sf　　　　上下サーボ現在の角度を遠距離側のペンの書く高さに設定する
 * sf<f>　　 上下サーボの遠距離側のペンの書く高さを絶対角度で設定する　単位[度]
 * up　　　　ペンを上げる
 * dn　　　　ペンを下げる（書くポジション）
 * M<f>,<f>  ペンを上げた状態で絶対座標に移動する　　　　単位[mm]　例 m-20,50.5
 * L<f>,<f>  ペンを下げた状態で絶対座標に移動する（書く）
 * D<f>,<f>  何もしない（ダミー）
 * N<n>　　　合計n個のM,LまたはDコマンドのバッファ準備要求 nの有効範囲は1-100
 * g　　　　　ペンを上げて原点位置に戻る
 * i　　　　　角度や座標の情報を表示
 * j　　　　　情報表示形式をjsonにする
 * 
 * 送信コマンド
 * start　　nコマンド後、n個のmまたはlコマンドを受信したのち、startを発して描画を開始する
 * end　　　start後、すべてのmまたはlコマンドを処理したらendを返す
 * align　　原点に戻ったときに出力
 */

//座標列を読み込んで処理するためのバッファ
char  buf_c[100]; // 'M' または 'L'
float buf_x[100];
float buf_y[100];
int buf_count = 0;
int buf_index = 0;
bool processing = false; //処理中はtrue

void loop() {

  if (!DoIt()) {
    if (processing) {

      if (buf_index >= buf_count) {
        //バッファにあった処理が終了

        processing = false;
        buf_count = 0;
        buf_index = 0;

        Serial.println("end");

      } else {
        if (buf_c[buf_index] != 'D') { //ダミーはスキップ
          //次のバッファにある指示を実行
  
          //ペンの上下動
          if (buf_index == 0 || (buf_c[buf_index - 1] != buf_c[buf_index])) {
            pen_status = (buf_c[buf_index] == 'L');
            
            if (calcCurrentPosition()) {
              setError(false);
              updownServo.setAngle(calcUpdownAngle(current_y));
            } else {
              setError(true);
            }        
          }
  
          //移動指示
          MoveTo(buf_x[buf_index], buf_y[buf_index]);
        }
        buf_index++;
      }
    }
    else if (Serial.available()) {
      String s = Serial.readStringUntil('\n');

      if (s == "mn") {
        MoveTo(0.0F, 35.0F);
      }
      else if (s == "mf") {
        MoveTo(0.0F, 110.0F);
      }
      else if (s == "sn") {
        updownWriteAngleAtNear = updownServo.getCurrentAngle();
        updownOffAngleAtNear = updownWriteAngleAtNear + 5.0F;
        pen_status = true;
      }
      else if (s == "sf") {
        updownWriteAngleAtFar = updownServo.getCurrentAngle();
        updownOffAngleAtFar = updownWriteAngleAtFar + 3.0F;
        pen_status = true;
      }
      else if (s.length() > 2 && s[0] == 's' && s[1] == 'n') {
        float f = atof(s.c_str() + 2);
        if (f > 0.0F) {
          updownWriteAngleAtNear = f;
          updownOffAngleAtNear = f + 10.0F;
        }
      }
      else if (s.length() > 2 && s[0] == 's' && s[1] == 'f') {
        float f = atof(s.c_str() + 2);
        if (f > 0.0F) {
          updownWriteAngleAtFar = f;
          updownOffAngleAtFar = f + 6.0F;
        }
      }
      else if (s == "up") {
        if (calcCurrentPosition()) {
          pen_status = false;
          updownServo.setAngle(calcUpdownAngle(current_y));
          setError(false);
        } else {
          setError(true);
        }
      }
      else if (s == "dn") {
        if (calcCurrentPosition()) {
          pen_status = true;
          updownServo.setAngle(calcUpdownAngle(current_y));
          setError(false);
        } else {
          setError(true);
        }
      }
      else if (s[0] == 'M' || s[0] == 'L' || s[0] == 'D') {
        int i = s.indexOf(',');
        if (i > 1) {
          float x = atof(s.c_str() + 1);
          float y = atof(s.c_str() + i + 1);
          if (buf_index < buf_count) {
            //バッファにを準備していた場合

            buf_c[buf_index] = s[0];
            buf_x[buf_index] = x;
            buf_y[buf_index] = y;

            if (++buf_index == buf_count) {
              //バッファがいっぱいになったので処理を開始

              Serial.println("start");
              buf_index = 0;
              processing = true;
            }
          } else if (buf_count == 0 && s[0] != 'D') {
              //単発処理の場合

            if (calcCurrentPosition()) {
              setError(false);

              //ペンの上げ下げ
              pen_status = (s[0] == 'L');
              updownServo.setAngle(calcUpdownAngle(current_y));

              //移動指示
              MoveTo(x, y);
            } else {
              setError(true);
            }
          }
        }
      } else if (s[0] == 'N') {
        int n = atoi(s.c_str() + 1);
        if (n > 0 && n <= 100) {
          buf_count = n;
          buf_index = 0;
          processing = false;
        }
      } else if (s[0] == 'x') {
        float f = atof(s.c_str() + 1);
        MoveTo(current_x + f, current_y);
      }
      else if (s[0] == 'y') {
        float f = atof(s.c_str() + 1);
        MoveTo(current_x, current_y + f);
      }
      else if (s[0] == 'l') {
        float f = atof(s.c_str() + 1);
        TurnTo(leftServo.getCurrentAngle() + f, rightServo.getCurrentAngle());
      }
      else if (s[0] == 'r') {
        float f = atof(s.c_str() + 1);
        TurnTo(leftServo.getCurrentAngle(), rightServo.getCurrentAngle() + f);
      }
      else if (s[0] == 'u') {
        float f = atof(s.c_str() + 1);
        updownServo.setAngle(updownServo.getCurrentAngle() + f);
      }
      else if (s == "g") {
        if (calcCurrentPosition()) {
          pen_status = false;
          updownServo.setAngle(calcUpdownAngle(current_y));
          setError(false);
        } else {
          setError(true);
        }
        TurnTo(leftDefaultAngle, rightDefaultAngle);
      }
      else if (s == "i") {
        showStatus();
      }
      else if (s == "j") {
        json_mode = true;
      }
    } else {
      if (show_information && millis() >= show_delay) {
        showStatus();
      }
    }
  }

  updateError();
  //delay(1000);
}

void showStatus() {
  if (!json_mode) {
    Serial.print("Angles L:");
    Serial.print(leftServo.getCurrentAngle());  
    Serial.print("  R:");
    Serial.print(rightServo.getCurrentAngle());  
    Serial.print("  U:");
    Serial.println(updownServo.getCurrentAngle());  
  
    if (!current_position) {
      Serial.println("<Posiotion not calculated>");
      setError(true);
    } else {    
      setError(false);
      Serial.print("Position (");
      Serial.print(current_x);
      Serial.print(",");
      Serial.print(current_y);
      Serial.println(pen_status ? ") WRITE" : ") OFF");
    }
  } else {
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
    Serial.print(",\"pen\":");
    Serial.print(pen_status ? "\"1\"" : "\"0\"");
    Serial.println("}");
  }
  show_information = false;
}
