#ifndef MYSERVO_H_INCLUDE
#define MYSERVO_H_INCLUDE

#include <Servo.h>

/* サーボの角度指定を容易にするクラス */

class MyServo {
  public:
    MyServo(int gpio_pin, float *pAngles, int *pPulses, int count, float angle_per_ms, float default_angle, float min_angle, float max_angle, int min_pulse = -1000000, int max_pulse = 1000000);

    //プロパティ
    int getMinPulse();
    int getMaxPulse();
    int getCurrentPulse();
    float getCurrentAngle();
    bool isRunning();

    //直接操作
    bool setAngle(float angle);
    bool setPulse(int pulse);

    //計算
    int calcPulse(float angle);
    float calcAngle(int pulse);

  private:
    Servo m_servo;

    //角度補正データ
    float *m_pAngles;
    int *m_pPulses;
    int m_nCount;

    //プロパティ
    float m_fAnglePerMs;
    int m_nMinPulse, m_nMaxPulse;

    float m_fCurAngle;
    int m_nCurPulse;
    
    unsigned long m_ulStartMillis;    //角度変更指定時のmillis()値
    unsigned long m_ulCompleteMillis; //角度変更完了予定時のmillis()値　動作終了後、両方0になる
    
};

#endif
