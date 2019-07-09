#ifndef MOTION_H_INCLUDE
#define MOTION_H_INCLUDE

/* 座標移動やサーボの角度変更を管理するクラス */

class Move {
    private:
        float x1, y1, x2, y2;
        int total_step;
        int cur_step;

    public:
        Move();
        int set(float cur_x, float cur_y, float to_x, float to_y);
        bool calc(float *x, float *y);
        bool isRunning();
        void stop();
};

class Turn {
    private:
        float a1, a2, b1, b2;
        int total_step;
        int cur_step;

    public:
        Turn();
        int set(float cur_ang1, float to_ang1, float cur_ang2, float to_ang2);
        bool calc(float *ang1, float *ang2);
        bool isRunning();
        void stop();
};

#endif
