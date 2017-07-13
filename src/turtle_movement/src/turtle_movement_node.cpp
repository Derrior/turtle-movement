#include "tf/transform_broadcaster.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "nav_msgs/Odometry.h"

#include "rosgraph_msgs/Clock.h"

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace std;

const double EPS = 1e-2;
const double MAX_LINEAR_SPEED = 0.5;
const double MAX_ANGULAR_SPEED = 0.2;
const double ROTATION_CONST = 0.5;
const double TURTLE_RADIUS = 0.4;

double current_time = 0;

struct Turtle;
struct CameraMsgCallback;
struct DestinationPointCallback;
struct LocationCallback;

struct Command {
    double duration;
    geometry_msgs::Twist command;
};


struct Turtle {
    sensor_msgs::PointCloud2 camera_image;
    Point target_point, location, orientation;
    Point in_between_target;

    double camera_moment;
    double prev_time, curr_time;
    double current_speed;

    vector<Command> dummy_commands;

    Turtle() {
        orientation.x = 1;
    }

    void SetTime(double time) {
        prev_time = curr_time;
        curr_time = time;
        if ((int)curr_time > (int)prev_time) {
            cerr << "LOC: " << location.x << ' ' << location.z << " | ORI: " << orientation.x << ' ' << orientation.z << '\n';
            if (curr_time - camera_moment > 1) {
                cerr << "No camera\n";
                camera_image.height = 0;
            }
        }
    }

    Point getCameraPixelCoords(int i, int j) {
        int offset = i * camera_image.row_step + j * camera_image.point_step;
        float x, y, z;
        memcpy(&x, camera_image.data.data() + offset, 4);
        memcpy(&y, camera_image.data.data() + offset + 4, 4);
        memcpy(&z, camera_image.data.data() + offset + 8, 4);
        double angle = getAngle(Point(1, 0, 0), orientation);
        Point relative(x, y, z);
        return relative.Rotate(angle) + location;
    }

    bool WorthToAnalyse(const Point &p, const Point &lowest) {
        // cerr << p.x << ' ' << p.y << ' ' << p.z << '\n';
        return (p.y > EPS) && (p.y - lowest.y > EPS) && !isnan(p.x) && !isnan(p.z);
    }

    double getTurningAngle(Point pointToTurn) {
        double ret = getAngle(orientation, Point(location, pointToTurn));
        return ret;
    }

    void LastDummyCommand(geometry_msgs::Twist &resultVelocity) {
        resultVelocity = dummy_commands.back().command;
        dummy_commands.back().duration -= curr_time - prev_time;
        if (dummy_commands.back().duration <= 0) {
            dummy_commands.pop_back();
        }
    }
    bool checkObstacles() {
        if (camera_image.height == 0) {
            return false;
        }
        Line trajectory(location, in_between_target);
        bool has_an_obstacle = false;
        Point to_in_between = Point(location, in_between_target);
        Point lowest = getCameraPixelCoords(camera_image.height - 1, camera_image.width / 2);
        for (size_t i = 0; i + 10 < camera_image.height; i += 2) {
            for (size_t j = 0; j < camera_image.width; j += 2) {
                Point currentPoint = getCameraPixelCoords(i, j);
                if (WorthToAnalyse(currentPoint, lowest)) {
                    double distance_to_pixel = lowest.distance(currentPoint);
                    double distance_to_finish = lowest.distance(in_between_target);
                    if (trajectory.distance(currentPoint) < TURTLE_RADIUS &&
                            distance_to_pixel < distance_to_finish + TURTLE_RADIUS) {
                        has_an_obstacle = true;
                        to_in_between = to_in_between.Resize(distance_to_pixel);
                    }
                }
            }
        }
        if (has_an_obstacle) {
            ROS_DEBUG("spotted an obstacle, distance: %f", to_in_between.length());
            to_in_between = to_in_between.IncreaseLength(-TURTLE_RADIUS);
            current_speed = 0;
        }
        if (has_an_obstacle && to_in_between.length() < 1) {
            to_in_between = to_in_between.Rotate(ROTATION_CONST).Resize(5);
            dummy_commands.push_back(Command());
            dummy_commands.back().command.linear.x = -0.1;
            dummy_commands.back().duration = 0.1;
            ROS_DEBUG("rotated");
            ROS_DEBUG("Loc: %f %f | Dir: %f %f", location.x, location.z,
                                        to_in_between.x, to_in_between.z);
        }
        in_between_target = to_in_between + location;
        return has_an_obstacle;
    }

    bool MoveToCurrentTarget(geometry_msgs::Twist &resultVelocity) {
        if (camera_image.height == 0) {
            resultVelocity.linear.x = resultVelocity.angular.z = 0;
            return true;
        } else if (!dummy_commands.empty()) {
            LastDummyCommand(resultVelocity);
            return true;
        }
        Point direction = Point(location, in_between_target);
        double angle = getAngle(orientation, direction);
        double distance = direction.length();
        if (distance < TURTLE_RADIUS) {
            return false;
        }
        resultVelocity.linear.x = current_speed;
        resultVelocity.angular.z = min(MAX_ANGULAR_SPEED, fabs(angle)) * sign(angle);
        if (2 * fabs(angle) / MAX_ANGULAR_SPEED > distance / MAX_LINEAR_SPEED) {
            resultVelocity.linear.x = 0;
        } else if (distance < 2 * MAX_LINEAR_SPEED) {
            resultVelocity.linear.x = distance / 2;
        } else {
            resultVelocity.linear.x += 0.2;
        }
        resultVelocity.linear.x = min(MAX_LINEAR_SPEED, max(0., resultVelocity.linear.x));
        current_speed = resultVelocity.linear.x;
        return true;
    }


    void ChooseCurrentTarget() {
        if (camera_image.height == 0) {
            return;
        }
        double angle = getTurningAngle(target_point);
        double distance = Point(location, target_point).length();
        if (distance < TURTLE_RADIUS) {
            return;
        }
        if (fabs(angle) > 3 * MAX_ANGULAR_SPEED) {
            dummy_commands.push_back(Command());
            dummy_commands.back().command.angular.z = sign(angle) * MAX_ANGULAR_SPEED;
            dummy_commands.back().duration = 0.1;
            return;
        }

        Point direction(location, target_point);
        if (direction.length() > 5) {
            direction = direction.Resize(5);
        }
        in_between_target = location + direction;
        bool has_an_obstacle = checkObstacles();
    }
};

// Callbacks

struct CameraMsgCallback {
    Turtle &turtle;

    CameraMsgCallback(Turtle &turtle_):
        turtle(turtle_)
    {}

    void operator()(sensor_msgs::PointCloud2::ConstPtr message) {
        turtle.camera_image = *message;
        turtle.camera_moment = current_time;
    }
};

struct DestinationPointCallback {
    Turtle &turtle;

    DestinationPointCallback(Turtle &turtle_):
        turtle(turtle_) {
        turtle.target_point.x = 40;
        turtle.target_point.z = 40;
    }

    void operator()(geometry_msgs::Point::ConstPtr message) {
         turtle.target_point = *message;
         turtle.in_between_target = turtle.location;
         turtle.dummy_commands.clear();
    }
};

void TimeCallback(rosgraph_msgs::Clock::ConstPtr message) {
    current_time = message->clock.toSec();
}

struct LocationCallback {
    Turtle &turtle;

    LocationCallback(Turtle &turtle_):
        turtle(turtle_)
    {}

    void operator()(nav_msgs::Odometry::ConstPtr message) {
        Point new_position = message->pose.pose.position;
        swap(new_position.y, new_position.z);
        turtle.location = new_position;
        turtle.orientation = Point(1, 0, 0).Rotate(tf::getYaw(message->pose.pose.orientation));
        turtle.curr_time = (double)clock() / CLOCKS_PER_SEC;
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_movement_node");
    ros::NodeHandle n;

    Turtle turtle;
    CameraMsgCallback camera_callback(turtle);
    DestinationPointCallback target_callback(turtle);
    LocationCallback location_callback(turtle);

    ros::Publisher turtleVelocityPub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    ros::Subscriber turtleCameraRec = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, camera_callback);
    ros::Subscriber targetPoint = n.subscribe<geometry_msgs::Point>("target", 1, target_callback);
    ros::Subscriber currentLocation = n.subscribe<nav_msgs::Odometry>("/odom", 1000, location_callback);
    ros::Subscriber timeSubscriber = n.subscribe<rosgraph_msgs::Clock>("/clock", 1, TimeCallback);

    cerr << "end subscription\n";
    geometry_msgs::Twist velocityMsg;
    while (ros::ok()) {
        turtle.SetTime(current_time);
        if (!turtle.MoveToCurrentTarget(velocityMsg)) {
            turtle.ChooseCurrentTarget();
            turtle.MoveToCurrentTarget(velocityMsg);
        }
        turtleVelocityPub.publish(velocityMsg);
        ros::spinOnce();
        current_time = (double)clock() / CLOCKS_PER_SEC;
    }
}
