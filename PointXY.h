#ifndef POINTXY_H_INCLUDE
#define POINTXY_H_INCLUDE

/* ２D座標を計算するクラス */

/*
 c#やVB.NETだとクラス演算やクラスを関数の返り値としたら超絶便利だったんだけど
 c++だったら new delete に気を使わないといけなくなって、使いづらくなりました。
 add() mul() みたいに自身の値を更新するタイプのほうが使いやすい
 2019/7/1 maeoka メモ
*/

class PointXY {
  public:
    float x, y;
    PointXY(const float x = 0.0F, const float y = 0.0F);
    //PointXY* clone();
    String toString();
    void set(float nx, float ny);
    //void set(PointXY *p);
    void add(float dx, float dy);
    void mul(float sx, float sy);
    void mul(float scale);
    float length() const;
    void normalize();
    //PointXY* rotate(float deg, bool clockwise = true);
    void rotate(float deg, bool clockwise);
    //PointXY* rotate(float center_x, float center_y, float deg, bool clockwise = true);
    void rotate(float center_x, float center_y, float deg, bool clockwise = true);
    float innerProduct(const PointXY* b);
    float outerProduct(const PointXY* b);
    //float angle(const PointXY* b);
    //float signedAngle(const PointXY* b);

    static float angle(float ax, float ay, float bx, float by);
    static float signedAngle(float ax, float ay, float bx, float by);

    //もともとCircleクラスだったけど、これしか使わないのでここに持ってきた
    static bool circleCrossPoints(float ax, float ay, float ar, float bx, float by, float br, PointXY *p1, PointXY *p2);

/*
    //代入
    PointXY& operator =(const PointXY& p) {
      x = p.x;
      y = p.y;
      return *this;
    }
    //比較
    bool operator ==(const PointXY &p) const {
      return (x == p.x) && (y == p.y);
    }
    //演算
    PointXY operator +(const PointXY &p) const {
      return PointXY(x + p.x, y + p.y);
    }
    PointXY operator -(const PointXY &p) const {
      return PointXY(x - p.x, y - p.y);
    }
    PointXY operator *(float scale) const {
      return PointXY(x * scale, y * scale);
    }
    PointXY operator /(float scale) const {
      return PointXY(x / scale, y / scale);
    }
    PointXY& operator *=(float scale) {
        x *= scale;
        y *= scale;
        return (*this);
    }
    PointXY& operator /=(float scale) {
        x /= scale;
        y /= scale;
        return (*this);
    }
*/
};

#endif
