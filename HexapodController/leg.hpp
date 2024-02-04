// #include <axis.hpp>
// #include <stdbool.h>
// #include <stdint.h>

// #ifndef LEG_H
// #define LEG_H


// #define NUM_AXES 3
// class Leg {
//     public:
//         double current_angles[NUM_AXES];
//         double current_cartesian[NUM_AXES];
//         _Bool rapid_move(double x,  double y, double z);
//         _Bool linear_move(double x,  double y, double z, double speed);
//     private:
//         double _length0;
//         double _length1;
//         double _length2;
//         _Bool move_axes();
//         _Bool inverse_kinematics(double x, double y, double z);
//         double _next_angles[NUM_AXES];
//         double _next_cartesian[NUM_AXES];
//         Axis axes[3];
// };

// #endif