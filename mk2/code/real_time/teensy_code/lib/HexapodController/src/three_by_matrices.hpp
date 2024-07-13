#ifndef THREEBYTHREE_H
#define THREEBYTHREE_H
 
#include <stdint.h>
#include <stdbool.h>

class ThreeByThree {
    public:
        double values[3][3];
        void mult_left_three_by_three(const ThreeByThree& left);
        void mult_right_three_by_three(const ThreeByThree& right);
        void invert();
        double value(uint8_t row, uint8_t collumn);
    private:
};

class ThreeByOne {
    public:
        ThreeByOne();
        ThreeByOne(double value0, double value1, double value2);
        ThreeByOne(double values[3]);
        ThreeByOne(const ThreeByOne& orig);
        double values[3];
        void mult_three_by_three(const ThreeByThree& left);
        void rotateYaw(double yaw);
        void rotatePitch(double pitch);
        void rotateRoll(double roll);
        double magnitude();
        ThreeByOne add(const ThreeByOne& addend);
        void operator+=(const ThreeByOne& addend);
        ThreeByOne operator+(const ThreeByOne& addend);
        void operator/=(double divisor);
        ThreeByOne operator/(double divisor);
        void operator*=(double multiplier);
        ThreeByOne operator*(double multiplier);
        void operator-=(const ThreeByOne& subtrahend);
        ThreeByOne operator-(const ThreeByOne& subtrahend);
        _Bool operator>(ThreeByOne& right);
        _Bool operator<(ThreeByOne& right);
        _Bool operator!=(ThreeByOne& right);
        _Bool operator==(ThreeByOne& right);
        ThreeByOne unit_vector();
        void floorDivide(double divisor);
    private:
};

#endif
