#ifndef TURTLE_MOVEMENT_GEOMETRY_H
#define TURTLE_MOVEMENT_GEOMETRY_H

#include "geometry_msgs/Point.h"
#include <cmath>

struct Point {
    double x, y, z;

    Point() {
        x = y = z = 0;
    }

    Point(geometry_msgs::Point a):
        x(a.x),
        y(a.y),
        z(a.z)
    {}

    Point(double x_, double y_, double z_):
        x(x_), y(y_), z(z_)
    {}

    Point(const Point &a, const Point &b):
        x(b.x - a.x), y(b.y - a.y), z(b.z - a.z)
    {}

    double length() const;

    double distance(const Point &v) const;

    Point IncreaseLength(double modifier) const;
    Point Resize(double new_length) const;
    Point Rotate(double angle) const;

    Point operator + (const Point &other) const;
    Point& operator += (const Point &other);
};

int sign(double a);

double getAngle(const Point& v1, const Point& v2);

struct Line {
    double a, b, c;
    Line(Point u, Point v);

    double distance(Point p) const;
};
#endif // TURTLE_MOVEMENT_GEOMETRY_H
