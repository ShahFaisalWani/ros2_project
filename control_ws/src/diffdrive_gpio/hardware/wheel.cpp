#include "diffdrive_gpio/wheel.hpp"
#include <cmath>

void Wheel::setup(const std::string &wheel_name, int counts_per_rev)
{
    name = wheel_name;
    rads_per_count = 2 * M_PI / counts_per_rev;  // Convert encoder counts to radians
}

double Wheel::calcEncAngle()
{
    // Calculate the wheel's current position in radians based on encoder count
    return enc * rads_per_count;
}

void Wheel::processEncoder(int a, int b)
{
    // Quadrature encoder logic to update the encoder count based on A and B signals
    int transition = (prev_a << 1) | prev_b;  // Previous state (2 bits)
    int new_transition = (a << 1) | b;        // Current state (2 bits)

    // Check quadrature state transitions to increment or decrement the encoder count
    if ((transition == 0b00 && new_transition == 0b01) ||
        (transition == 0b01 && new_transition == 0b11) ||
        (transition == 0b11 && new_transition == 0b10) ||
        (transition == 0b10 && new_transition == 0b00))
    {
        enc++;  // Clockwise rotation
    }
    else if ((transition == 0b00 && new_transition == 0b10) ||
             (transition == 0b10 && new_transition == 0b11) ||
             (transition == 0b11 && new_transition == 0b01) ||
             (transition == 0b01 && new_transition == 0b00))
    {
        enc--;  // Counterclockwise rotation
    }

    // Update previous A and B states for next transition
    prev_a = a;
    prev_b = b;
}
