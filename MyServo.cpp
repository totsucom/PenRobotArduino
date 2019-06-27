#include <Arduino.h>
#include "MyServo.h"

/**
 * int gpio_pin　
 * 　サーボに使用するGPIOピン番号
 * float *pAngles, int *pPulses, int count 
 * 　サーボ角度(度)に対するパルス幅(マイクロ秒)の組み合わせ
 * 　サーボ角度の配列は昇順に並び変えられていること
 * float angle_per_ms
 * 　サーボ速度。1ミリ秒あたりの移動角度を指定
 * float default_angle 
 * 　サーボの初期角度
 * float min_angle, float max_angle
 * 　サーボが移動可能な最小、最大角度
 * int min_pulse, int max_pulse (オプション)
 * 　サーボが移動可能な最小、最大パルス幅
 * ※min_pulseがセットされた場合、getPulse(min_angle)と比較してパルスの大きい方(動作範囲が狭くなる方)を採用する
 * ※max_pulseがセットされた場合、getPulse(max_angle)と比較してパルスの小さい方(動作範囲が狭くなる方)を採用する
 */
MyServo::MyServo(int gpio_pin,
  float *pAngles, int *pPulses, int count,
  float angle_per_ms,
  float default_angle, float min_angle, float max_angle,
  int min_pulse = -1000000, int max_pulse = 1000000)
{
  if (count < 2) {
#ifdef DEBUG
    Serial.println("MyServo::MyServo() Pulse data array too short.");
#endif
    return;
  }
  
  m_pAngles = pAngles;
  m_pPulses = pPulses;
  m_nCount = count;
  m_fAnglePerMs = angle_per_ms;
  m_ulStartMillis = m_ulCompleteMillis = 0;
  
  int pulse = calcPulse(min_angle);
  m_nMinPulse = min_pulse > pulse ? min_pulse : pulse;

  pulse = calcPulse(max_angle);
  m_nMaxPulse = max_pulse < pulse ? max_pulse : pulse;

  pulse = calcPulse(default_angle);

  if (m_nMinPulse <= 0 || m_nMinPulse >= 3000 || m_nMaxPulse <= 0 || m_nMaxPulse >= 3000 || m_nMinPulse >= m_nMaxPulse || pulse < m_nMinPulse || pulse > m_nMaxPulse) {
#ifdef DEBUG
    Serial.println("MyServo::MyServo() Can not initialize.");
#endif
    return;
  }

  m_servo.attach(gpio_pin, m_nMinPulse, m_nMaxPulse);
  m_servo.write(pulse);
  m_fCurAngle = default_angle;
  m_nCurPulse = pulse;
  
#ifdef DEBUG
    Serial.print("MyServo gpio=");
    Serial.print(gpio_pin);
    Serial.print(" Range=");
    Serial.print(m_nMinPulse);
    Serial.print("..");
    Serial.print(m_nMaxPulse);
    Serial.print(" Current=");
    Serial.println(m_nCurPulse);
#endif
}

int MyServo::getMinPulse()
{
  return m_nMinPulse;
}

int MyServo::getMaxPulse()
{
  return m_nMaxPulse;
}

int MyServo::getCurrentPulse()
{
  return m_nCurPulse;
}
    
float MyServo::getCurrentAngle()
{
  return m_fCurAngle;
}

/**
 * サーボ動作が完了しているかを返す
 * ただし、電源投入直後のデフォルト角度への移動時間はこの関数で計算できない（この関数は常にfalseを返す）ので、
 * ユーザー側で十分な待ち時間（サーボが180度動ける時間など）をとること
 */
bool MyServo::isRunning()
{
  if (m_ulStartMillis == 0 && m_ulCompleteMillis == 0) {
    return false;
  }
  unsigned long now = millis();
  if (now > m_ulCompleteMillis || now < m_ulStartMillis) {
    m_ulStartMillis = m_ulCompleteMillis = 0;
    return false;
  }
  else {
    return true;
  }
}

/**
 * 低レベル直接角度(度)指定
 * 正確に動かしたいなら事前にisRunning()がfalseであることを確認したほうがいい
 */
bool MyServo::setAngle(float angle)
{
  int pulse = calcPulse(angle);
  if (pulse >= m_nMinPulse && pulse <= m_nMaxPulse) {
    m_servo.write(pulse);

    //動作終了時間を計算
    float action_ms = (angle - m_fCurAngle) / m_fAnglePerMs * 1.2F + 10.0F; //動作完了に 20% + 10[ms] の余裕を持たせている(ﾃｷﾄ-)
    if (action_ms < 0) action_ms = -action_ms;
    m_ulStartMillis = millis();
    m_ulCompleteMillis = m_ulStartMillis + action_ms;

    //現在角度は即座に更新(実際は違うけど)
    m_fCurAngle = angle;
    m_nCurPulse = pulse;
    return true;
  } else {
    //動作範囲外
    return false;
  }
}

/**
 * 低レベル直接パルス幅(マイクロ秒)指定
 * 正確に動かしたいなら事前にisRunning()がfalseであることを確認したほうがいい
 */
bool MyServo::setPulse(int pulse)
{
  if (pulse >= m_nMinPulse && pulse <= m_nMaxPulse) {
    m_servo.write(pulse);
    float angle = calcAngle(pulse);

    //動作終了時間を計算
    float action_ms = (angle - m_fCurAngle) / m_fAnglePerMs * 1.2F + 10.0F; //動作完了に 20% + 10[ms] の余裕を持たせている(ﾃｷﾄ-)
    if (action_ms < 0) action_ms = -action_ms;
    m_ulStartMillis = millis();
    m_ulCompleteMillis = m_ulStartMillis + action_ms;

    //現在角度は即座に更新(実際は違うけど)
    m_fCurAngle = angle;
    m_nCurPulse = pulse;
    return true;
  } else {
    //動作範囲外
    return false;
  }
}
 

/**
 * 計算
 * angle(度)に該当するパルス幅(マイクロ秒)を返す（動作に影響を与えない）
 * 線形補間を行う。コンストラクタで与えた配列外の角度でも線形補間を行う
 */
int MyServo::calcPulse(float angle)
{
  if (angle <= *m_pAngles) {
    float pulse0 = (float)*m_pPulses;
    float pulse1 = (float)*(m_pPulses + 1);
    float angle0 = *m_pAngles;
    float angle1 = *(m_pAngles + 1);
    return pulse0 - (pulse1 - pulse0) / (angle1 - angle0) * (angle0 - angle);
  } else if (angle >= *(m_pAngles + m_nCount - 1)) {
    float pulse0 = (float)*(m_pPulses + m_nCount - 2);
    float pulse1 = (float)*(m_pPulses + m_nCount - 1);
    float angle0 = *(m_pAngles + m_nCount - 2);
    float angle1 = *(m_pAngles + m_nCount - 1);
    return (pulse1 - pulse0) / (angle1 - angle0) * (angle - angle1) + pulse1;
  } else {
    float angle1 = *m_pAngles;
    for (int i = 1; i < m_nCount; i++) {
      float angle0 = angle1;
      angle1 = *(m_pAngles + i);
      if (angle >= angle0 && angle <= angle1) {
        float pulse0 = (float)*(m_pPulses + i - 1);
        float pulse1 = (float)*(m_pPulses + i);
        return (pulse1 - pulse0) / (angle1 - angle0) * (angle - angle0) + pulse0;
      }
    }
  }
  //ありえない
  return -1;
}

/**
 * 計算
 * pulse(マイクロ秒)に該当する角度(度)を返す（動作に影響を与えない）
 * 線形補間を行う。コンストラクタで与えた配列外のパルスでも線形補間を行う
 */
float MyServo::calcAngle(int pulse)
{
  if (pulse <= *m_pPulses) {
    float pulse0 = (float)*m_pPulses;
    float pulse1 = (float)*(m_pPulses + 1);
    float angle0 = *m_pAngles;
    float angle1 = *(m_pAngles + 1);
    return angle0 - (angle1 - angle0) / (pulse1 - pulse0) * (pulse0 - pulse);
  } else if (pulse >= *(m_pPulses + m_nCount - 1)) {
    float pulse0 = (float)*(m_pPulses + m_nCount - 2);
    float pulse1 = (float)*(m_pPulses + m_nCount - 1);
    float angle0 = *(m_pAngles + m_nCount - 2);
    float angle1 = *(m_pAngles + m_nCount - 1);
    return (angle1 - angle0) / (pulse1 - pulse0) * (pulse - pulse1) + angle1;
  } else {
    float pulse1 = *m_pPulses;
    for (int i = 1; i < m_nCount; i++) {
      float pulse0 = pulse1;
      pulse1 = *(m_pPulses + i);
      if (pulse >= pulse0 && pulse <= pulse1) {
        float angle0 = (float)*(m_pAngles + i - 1);
        float angle1 = (float)*(m_pAngles + i);
        return (angle1 - angle0) / (pulse1 - pulse0) * (pulse - pulse0) + angle0;
      }
    }
  }
  //ありえない
  return -1;
}
