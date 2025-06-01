#include "position.hpp"
#include <stdbool.h>
#include <Arduino.h>
#include <math.h>
#include "three_by_matrices.hpp"

void Position::set(double new_x, double new_y, double new_z, double new_roll, double new_pitch, double new_yaw) {
    x = new_x;
    y = new_y;
    z = new_z;
    roll = new_roll;
    pitch = new_pitch;
    yaw = new_yaw;
}

void Position::setPos(const Position& pos) {
    x = pos.x;
    y = pos.y;
    z = pos.z;
    roll = pos.roll;
    pitch = pos.pitch;
    yaw = pos.yaw;
}

void Position::scalarMult(double factor) {
    x = x * factor;
    y = y * factor;
    z = z * factor;
    roll = roll * factor;
    pitch = pitch * factor;
    yaw = yaw * factor;
}

Position Position::unitVector() {
    Position unit_vector;
    unit_vector = (*this);
    double magnitude = unit_vector.magnitude();
    if (fabs(magnitude) > .001) {
        unit_vector.scalarMult(1.0 / unit_vector.magnitude());
    }

    return unit_vector;
}

void Position::operator*=(const double& multiplier) {
    x = x * multiplier;
    y = y * multiplier;
    z = z * multiplier;
    roll = roll * multiplier;
    pitch = pitch * multiplier;
    yaw = yaw * multiplier;
}

Position Position::operator*(const double& multiplier) {
    Position product;
    product.set(x, y, z, roll, pitch, yaw);
    product *= multiplier;
    return product;
}

void Position::independentScalarMult(double factors[6]) {
    x = x * factors[0];
    y = y * factors[1];
    z = z * factors[2];
    roll = roll * factors[3];
    pitch = pitch * factors[4];
    yaw = yaw * factors[5];
}

void Position::operator-=(const Position& pos) {
    x = x - pos.x;
    y = y - pos.y;
    z = z - pos.z;
    roll = roll - pos.roll;
    pitch = pitch - pos.pitch;
    yaw = yaw - pos.yaw;
}

Position Position::operator-(const Position& subtrahend) {
    Position difference;
    difference.set(x, y, z, roll, pitch, yaw);
    difference -= subtrahend;
    return difference;
}

void Position::operator+=(const Position& pos) {
    x = x + pos.x;
    y = y + pos.y;
    z = z + pos.z;
    roll = roll + pos.roll;
    pitch = pitch + pos.pitch;
    yaw = yaw + pos.yaw;
}

Position Position::operator+(const Position& addend) {
    Position sum;
    sum.set(x, y, z, roll, pitch, yaw);
    sum += addend;
    return sum;
}

double Position::magnitude() {
    double orientation_magnitude = sqrt(roll*roll + pitch*pitch + yaw*yaw);
    double cartesian_magnitude = sqrt(x*x + y*y + z*z);
    return sqrt(orientation_magnitude*orientation_magnitude + cartesian_magnitude*cartesian_magnitude);
}

double Position::scaledMagnitude() {
    double orientation_magnitude = sqrt(roll*roll + pitch*pitch + yaw*yaw) * ROTATION_MAGNITUDE_SCALE;
    double cartesian_magnitude = sqrt(x*x + y*y + z*z);
    return sqrt(orientation_magnitude*orientation_magnitude + cartesian_magnitude*cartesian_magnitude);
}


_Bool Position::equals(const Position& pos) {
    if (fabs(x - pos.x) > 0.003)
        return false;
    if (fabs(y - pos.y) > 0.003)
        return false;
    if (fabs(z - pos.z) > 0.003)
        return false;
    if (fabs(roll - pos.roll) > 0.003)
        return false;
    if (fabs(pitch - pos.pitch) > 0.003)
        return false;
    if (fabs(yaw - pos.yaw) > 0.003)
        return false;
    return true;
}

//get a position opbject from a command string
Position getPosFromCommand(String command) {
  Position position;
  double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw= 0;
  x = command.substring(command.indexOf('x') + 1).toFloat();
  y = command.substring(command.indexOf('y') + 1).toFloat();
  z = command.substring(command.indexOf('z') + 1).toFloat();
  roll = command.substring(command.indexOf('R') + 1).toFloat();
  pitch = command.substring(command.indexOf('P') + 1).toFloat();
  yaw = command.substring(command.indexOf('W') + 1).toFloat();
  position.set(x, y, z, roll, pitch, yaw);
  return position;
}

ThreeByOne Position::coord() {
    return ThreeByOne(x, y, z);
}

_Bool Position::operator==(const Position& pos) {
    if (fabs(x - pos.x) > 0.003)
        return false;
    if (fabs(y - pos.y) > 0.003)
        return false;
    if (fabs(z - pos.z) > 0.003)
        return false;
    if (fabs(roll - pos.roll) > 0.003)
        return false;
    if (fabs(pitch - pos.pitch) > 0.003)
        return false;
    if (fabs(yaw - pos.yaw) > 0.003)
        return false;
    return true;
}

_Bool Position::operator!=(const Position& pos) {
    return !((*this) == pos);
}

_Bool Position::operator<(const Position& pos) {
    if (x - pos.x > 0.0)
        return false;
    if (y - pos.y > 0.0)
        return false;
    if (z - pos.z > 0.0)
        return false;
    if (roll - pos.roll > 0.0)
        return false;
    if (pitch - pos.pitch > 0.0)
        return false;
    if (yaw - pos.yaw > 0.0)
        return false;
    return true;
}

_Bool Position::operator>(const Position& pos) {
    if (x - pos.x < 0.0)
        return false;
    if (y - pos.y < 0.0)
        return false;
    if (z - pos.z < 0.0)
        return false;
    if (roll - pos.roll < 0.0)
        return false;
    if (pitch - pos.pitch < 0.0)
        return false;
    if (yaw - pos.yaw < 0.0)
        return false;
    return true;
}

_Bool Position::operator<=(const Position& pos) {
    return (!(*this > pos));
}

_Bool Position::operator>=(const Position& pos) {
    return (!(*this < pos));
}

Position Position::operator=(const Position& pos) {
    x = pos.x;
    y = pos.y;
    z = pos.z;
    roll = pos.roll;
    pitch = pos.pitch;
    yaw = pos.yaw;
    return *this;
}

void Position::clear() {
    x = 0.00;
    y = 0.00;
    z = 0.00;
    roll = 0.00;
    pitch = 0.00;
    yaw = 0.00;
}

void Position::usbSerialize() {
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(" y: ");
    Serial.print(y);
    Serial.print(" z: ");
    Serial.print(z);
    Serial.print(" R: ");
    Serial.print(roll);
    Serial.print(" P: ");
    Serial.print(pitch);
    Serial.print(" W: ");
    Serial.println(yaw);
}
