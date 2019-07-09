#include <Arduino.h>
#include "PointXY.h"

//#define DEBUG  //デバッグモード

PointXY::PointXY(float x, float y)
{
  this->x = x;
  this->y = y;
}

/*PointXY* PointXY::clone()
{
  return new PointXY(x, y);
}*/

String PointXY::toString()
{
  String s = "(";
  s += String(x);
  s += ",";
  s += String(y);
  s += ")";
  return s;
}

void PointXY::set(float nx, float ny)
{
  x = nx;
  y = ny;
}

/*void PointXY::set(PointXY *p)
{
  x = p->x;
  y = p->y;
}*/

void PointXY::add(float dx, float dy)
{
  x += dx;
  y += dy;
}

void PointXY::mul(float sx, float sy)
{
  x *= sx;
  y *= sy;
}

void PointXY::mul(float scale)
{
  x *= scale;
  y *= scale;
}

float PointXY::length() const
{
  return (float)sqrt( x * x + y * y);
}

void PointXY::normalize()
{
  float l = length();
#ifdef DEBUG
  if (l < 0.000001) {
    Serial.println("PointXY::Normalize() Divide by zero");
    x = y = 0.0F;
    return;
  }
#endif
  x /= l;
  y /= l;
}

/**
 * 原点中心の回転
 */
/*PointXY* PointXY::rotate(float deg, bool clockwise)
{
  float rad = deg / 180.0F * (float)PI;
  if (!clockwise) rad = -rad;
  float sin_t = (float)sin(rad);
  float cos_t = (float)cos(rad);
  return new PointXY(
        x * cos_t - y * sin_t,
        x * sin_t + y * cos_t
        );
}*/
void PointXY::rotate(float deg, bool clockwise)
{
  float rad = deg / 180.0F * (float)PI;
  if (!clockwise) rad = -rad;
  float sin_t = (float)sin(rad);
  float cos_t = (float)cos(rad);
  float xx = x * cos_t - y * sin_t;
  float yy = x * sin_t + y * cos_t;
  x = xx;
  y = yy;
}

/**
 * 指定座標中心の回転
 */
/*PointXY* PointXY::rotate(float center_x, float center_y, float deg, bool clockwise)
{
  float rad = deg / 180.0F * (float)PI;
  if (!clockwise) rad = -rad;
  float sin_t = (float)sin(rad);
  float cos_t = (float)cos(rad);
  return new PointXY(
      (x - center_x) * cos_t - (y - center_y) * sin_t + center_x,
      (x - center_x) * sin_t + (y - center_y) * cos_t + center_y
      );
}*/
void PointXY::rotate(float center_x, float center_y, float deg, bool clockwise)
{
  float rad = deg / 180.0F * (float)PI;
  if (!clockwise) rad = -rad;
  float sin_t = (float)sin(rad);
  float cos_t = (float)cos(rad);
  float xx = (x - center_x) * cos_t - (y - center_y) * sin_t + center_x;
  float yy = (x - center_x) * sin_t + (y - center_y) * cos_t + center_y;
  x = xx;
  y = yy;
}

/**
 * 内積。クラスのベクトルとベクトルｂのなす角が９０で０、９０を超えると負の数を返します
 */
float PointXY::innerProduct(const PointXY* b)
{
  return x * b->x + y * b->y;
}

/** 
 * 外積。ベクトルｂがこのクラスのベクトルの時計回り側にあると返り値はプラスになります
 */
float PointXY::outerProduct(const PointXY* b)
{
  return x * b->y - y * b->x;
}

/**        
 * 0.0 <= angle <= 180.0の値を返します
 */
/*float PointXY::angle(const PointXY* b)
{
  float rad = (float)acos(innerProduct(b) / (length() * b->length()));
  return rad * 180.0F / (float)PI;
}*/
float PointXY::angle(float ax, float ay, float bx, float by)
{
  float rad = (float)acos((ax * bx + ay * by) / ((float)sqrt( ax * ax + ay * ay) * (float)sqrt( bx * bx + by * by)));
  return rad * 180.0F / (float)PI;
}

/**
 * -180.0 < angle <= 180.0の値を返します
 * ベクトルbが反時計周りにあるとき、値は負の数を返します
 */
/*float PointXY::signedAngle(const PointXY* b)
{
  float rad = (float)acos(innerProduct(b) / (length() * b->length()));
  if (outerProduct(b) < 0.0F) rad = -rad;
  return rad * 180.0F / (float)PI;
}*/

float PointXY::signedAngle(float ax, float ay, float bx, float by)
{
  float rad = (float)acos((ax * bx + ay * by) / ((float)sqrt( ax * ax + ay * ay) * (float)sqrt( bx * bx + by * by)));
  if ((ax * by - ay * bx) < 0.0F) rad = -rad;
  return rad * 180.0F / (float)PI;
}

/**
 * 円と円の交点を計算する
 * 成功した場合はtrueを返し、p1,p2に結果をセットする
 */
bool PointXY::circleCrossPoints(float ax, float ay, float ar, float bx, float by, float br, PointXY *p1, PointXY *p2)
{
  //b座標を原点として計算
  float xx = ax - bx;
  float yy = ay - by;

  float w = xx * xx + yy * yy;
  if (w < 0.00001F) {
#ifdef DEBUG
    Serial.println("PointXY::circleCrossPoints() - devide by zero");
#endif
    return false;
  }

  float a = (w + br * br - ar * ar) / 2.0F;
  float v = w * br * br - a * a;
  if (v < 0.0F) {
#ifdef DEBUG
    Serial.println("PointXY::circleCrossPoints() - sqrt error");
#endif
    return false;
  }
  v = (float)sqrt(v);

  p1->x = (a * xx + yy * v) / w + bx;
  p1->y = (a * yy - xx * v) / w + by;
  p2->x = (a * xx - yy * v) / w + bx;
  p2->y = (a * yy + xx * v) / w + by;
  return true;
}
