#include <Arduino.h>
#include "Motion.h"

//#define DEBUG //デバッグモード

#define DISTANCE_STEP 1.0F  //[mm]
#define ANGLE_STEP 5.0F     //[deg]

Move::Move()
{
    cur_step = 1;
    total_step = 0;
}

int Move::set(float cur_x, float cur_y, float to_x, float to_y)
{
    x1 = cur_x;
    y1 = cur_y;
    x2 = to_x;
    y2 = to_y;

    float dx = x2 - x1;
    float dy = y2 - y1;

    if (dx == 0.0F && dy == 0.0F) {
        cur_step = 1;
        total_step = 0;
        return 0;
    }

    float distance = (float)sqrt(dx * dx + dy * dy);

    total_step = (int)(distance / DISTANCE_STEP) + 1;
    cur_step = 0;
    return total_step;
}

/**
 * 返り値がtrueの場合x,yは有効
 * 逆に返り値がfalseの場合は目的座標に到着済みという意味
 */
bool Move::calc(float *x, float *y)
{
    if (cur_step >= total_step) return false;
    cur_step++;

#ifdef DEBUG
    Serial.print("Move::calc ");
    Serial.print(cur_step);
    Serial.print("/");
    Serial.println(total_step);
#endif    
    float ratio = (float)cur_step / (float)total_step;
    *x = (x2 - x1) * ratio + x1;
    *y = (y2 - y1) * ratio + y1;
    return true;
}

bool Move::isRunning()
{
/*    Serial.print("Move::isRunning ");
    Serial.print(cur_step);
    Serial.print("/");
    Serial.print(total_step);
    Serial.print(" ");
    Serial.println((bool)(cur_step < total_step));
*/
    return cur_step < total_step; // <= ではない
}

void Move::stop()
{
#ifdef DEBUG
    Serial.println("Move::stop");
#endif
    cur_step = 1;
    total_step = 0;
}

Turn::Turn()
{
    cur_step = 1;
    total_step = 0;
}

int Turn::set(float cur_ang1, float to_ang1, float cur_ang2, float to_ang2)
{
    a1 = cur_ang1;
    a2 = to_ang1;
    b1 = cur_ang2;
    b2 = to_ang2;

    float da = a2 - a1;
    if (da < 0) da = -da;

    float db = b2 - b1;
    if (db < 0) db = -db;

    if (da == 0.0F && db == 0.0F) {
        cur_step = 1;
        total_step = 0;
        return 0;
    }

    int ta = (int)(da / ANGLE_STEP) + 1;
    int tb = (int)(db / ANGLE_STEP) + 1;

    total_step = (ta >= tb) ? ta : tb;
    cur_step = 0;
    return total_step;
}

/**
 * 返り値がtrueの場合ang1,ang2は有効
 * 逆に返り値がfalseの場合は目的座標に到着済みという意味
 */
bool Turn::calc(float *ang1, float *ang2)
{
    if (cur_step >= total_step) return false;
    cur_step++;

#ifdef DEBUG
    Serial.print("Turn::calc ");
    Serial.print(cur_step);
    Serial.print("/");
    Serial.println(total_step);
#endif

    float ratio = (float)cur_step / (float)total_step;
    *ang1 = (a2 - a1) * ratio + a1;
    *ang2 = (b2 - b1) * ratio + b1;
    return true;
}

bool Turn::isRunning()
{
/*    Serial.print("Turn::isRunning ");
    Serial.print(cur_step);
    Serial.print("/");
    Serial.print(total_step);
    Serial.print(" ");
    Serial.println((bool)(cur_step < total_step));
*/
    return cur_step < total_step; // <= ではない
}

void Turn::stop() 
{
#ifdef DEBUG
    Serial.println("Turn::stop");
#endif
    cur_step = 1;
    total_step = 0;
}
