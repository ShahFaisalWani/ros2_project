#ifndef ROS2_CONTROL_DEMO_EXAMPLE_2_WHEEL_HPP
#define ROS2_CONTROL_DEMO_EXAMPLE_2_WHEEL_HPP

#include <string>

class Wheel
{
public:
    void setup(const std::string &wheel_name, int counts_per_rev);
    double calcEncAngle();  // Calculate the wheel's angle based on encoder counts
    void processEncoder(int a, int b);  // Process quadrature encoder signals

    std::string name;
    double pos = 0.0;  // Wheel position in radians
    double vel = 0.0;  // Wheel velocity in rad/s
    double cmd = 0.0;  // Commanded velocity

private:
    int enc = 0;  // Encoder count
    double rads_per_count;  // Radians per encoder count

    // Previous states of the quadrature encoder channels A and B
    int prev_a = 0;
    int prev_b = 0;
};

#endif // ROS2_CONTROL_DEMO_EXAMPLE_2_WHEEL_HPP
