#include <Arduino.h>
#include "PointXY.h"

PointXY::PointXY(float x, float y)
{
  this->x = x;
  this->y = y;
}

PointXY* PointXY::clone()
{
  return new PointXY(x, y);
}

String PointXY::toString()
{
  String s = "(";
  s += String(x);
  s += ",";
  s += String(y);
  s += ")";
  return s;
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
PointXY* PointXY::rotate(float deg, bool clockwise)
{
  float rad = deg / 180.0F * (float)PI;
  if (!clockwise) rad = -rad;
  float sin_t = (float)sin(rad);
  float cos_t = (float)cos(rad);
  return new PointXY(
        x * cos_t - y * sin_t,
        x * sin_t + y * cos_t
        );
}

/**
 * 指定座標中心の回転
 */
PointXY* PointXY::rotate(float center_x, float center_y, float deg, bool clockwise)
{
  float rad = deg / 180.0F * (float)PI;
  if (!clockwise) rad = -rad;
  float sin_t = (float)sin(rad);
  float cos_t = (float)cos(rad);
  return new PointXY(
      (x - center_x) * cos_t - (y - center_y) * sin_t + center_x,
      (x - center_x) * sin_t + (y - center_y) * cos_t + center_y
      );
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
float PointXY::angle(const PointXY* b)
{
  float rad = (float)acos(innerProduct(b) / (length() * b->length()));
  return rad * 180.0F / (float)PI;
}

/**
 * -180.0 < angle <= 180.0の値を返します
 * ベクトルbが反時計周りにあるとき、値は負の数を返します
 */
float PointXY::signedAngle(const PointXY* b)
{
  float rad = (float)acos(innerProduct(b) / (length() * b->length()));
  if (outerProduct(b) < 0.0F) rad = -rad;
  return rad * 180.0F / (float)PI;
}
