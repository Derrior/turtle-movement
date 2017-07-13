
#include "geometry.h"
#include <cmath>


int sign(double a) {
    if (a > 0) {
        return 1;
    } else if (a < 0) {
        return -1;
    }
    return 0;
}


double getAngle(const Point& v1, const Point& v2) {
    return atan2(v1.x * v2.z - v1.z * v2.x, v1.x * v2.x + v1.z * v2.z);
}

double Point::length() const {
    return sqrt(x * x + z * z);
}

double Point::distance(const Point &v) const {
    return hypot(x - v.x, z - v.z);
}

Point Point::IncreaseLength(double modifier) const {
    Point v(*this);
    double len = length();
    double multiplier = (len + modifier) / len;
    v.x *= multiplier;
    v.y *= multiplier;
    v.z *= multiplier;
    return v;
}

Point Point::Resize(double new_length) const {
    Point v(*this);
    double len = v.length();
    double multiplier = (new_length) / len;
    v.x *= multiplier;
    v.y *= multiplier;
    v.z *= multiplier;
    return v;
}

Point Point::Rotate(double angle) const {
    Point result;
    angle = -angle;
    result.x = x * cos(angle) + z * sin(angle);
    result.y = y;
    result.z = -x * sin(angle) + z * cos(angle);
    return result;
}

Point Point::operator + (const Point &u) const {
    return Point(x + u.x, y + u.y, z + u.z);
}
Point &Point::operator += (const Point &u) {
    x += u.x;
    y += u.y;
    z += u.z;
    return *this;
}

Line::Line(Point u, Point v) {
    a = u.x - v.x;
    b = v.z - u.z;
    c = -a * u.x - b * u.z;
}

double Line::distance(Point p) const {
    return fabs(a * p.x + b * p.z + c) / hypot(a, b);
}
