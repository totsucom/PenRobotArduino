#ifndef POINTXY_H_INCLUDE
#define POINTXY_H_INCLUDE

class PointXY {
  public:
    float x, y;
    PointXY(const float x = 0.0F, const float y = 0.0F);
    PointXY* clone();
    String toString();
    float length() const;
    void normalize();
    PointXY* rotate(float deg, bool clockwise = true);
    PointXY* rotate(float center_x, float center_y, float deg, bool clockwise = true);
    float innerProduct(const PointXY* b);
    float outerProduct(const PointXY* b);
    float angle(const PointXY* b);
    float signedAngle(const PointXY* b);

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




};

#endif
