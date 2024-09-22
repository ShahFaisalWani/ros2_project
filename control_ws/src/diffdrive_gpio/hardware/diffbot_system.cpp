#include "diffdrive_gpio/diffbot_system.hpp"
#include <pigpio.h> 
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace diffdrive_gpio
{

hardware_interface::CallbackReturn DiffDriveGPIOHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];

    // Encoder pins
    cfg_.gpio_left_encoder_a = std::stoi(info_.hardware_parameters["gpio_left_encoder_a"]);
    cfg_.gpio_left_encoder_b = std::stoi(info_.hardware_parameters["gpio_left_encoder_b"]);
    cfg_.gpio_right_encoder_a = std::stoi(info_.hardware_parameters["gpio_right_encoder_a"]);
    cfg_.gpio_right_encoder_b = std::stoi(info_.hardware_parameters["gpio_right_encoder_b"]);

    // Motor pins
    cfg_.gpio_left_motor_1 = std::stoi(info_.hardware_parameters["gpio_left_motor_1"]);
    cfg_.gpio_left_motor_2 = std::stoi(info_.hardware_parameters["gpio_left_motor_2"]);
    cfg_.gpio_right_motor_1 = std::stoi(info_.hardware_parameters["gpio_right_motor_1"]);
    cfg_.gpio_right_motor_2 = std::stoi(info_.hardware_parameters["gpio_right_motor_2"]);
    cfg_.gpio_left_enable = std::stoi(info_.hardware_parameters["gpio_left_enable"]);
    cfg_.gpio_right_enable = std::stoi(info_.hardware_parameters["gpio_right_enable"]);

    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

    return hardware_interface::CallbackReturn::SUCCESS;
}

// Configure function for lifecycle state
hardware_interface::CallbackReturn DiffDriveGPIOHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (gpioInitialise() < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveGPIOHardware"), "Failed to initialize pigpio.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveGPIOHardware"), "Successfully initialized pigpio.");

    gpioSetMode(cfg_.gpio_left_encoder_a, PI_INPUT);
    gpioSetMode(cfg_.gpio_left_encoder_b, PI_INPUT);
    gpioSetMode(cfg_.gpio_right_encoder_a, PI_INPUT);
    gpioSetMode(cfg_.gpio_right_encoder_b, PI_INPUT);

    gpioSetMode(cfg_.gpio_left_motor_1, PI_OUTPUT);
    gpioSetMode(cfg_.gpio_left_motor_2, PI_OUTPUT);
    gpioSetMode(cfg_.gpio_right_motor_1, PI_OUTPUT);
    gpioSetMode(cfg_.gpio_right_motor_2, PI_OUTPUT);
    gpioSetMode(cfg_.gpio_left_enable, PI_OUTPUT);
    gpioSetMode(cfg_.gpio_right_enable, PI_OUTPUT);

    return hardware_interface::CallbackReturn::SUCCESS;
}

// Cleanup function for lifecycle state
hardware_interface::CallbackReturn DiffDriveGPIOHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    gpioTerminate();  // Safely terminate GPIO
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveGPIOHardware"), "GPIO cleaned up.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// Activation function
hardware_interface::CallbackReturn DiffDriveGPIOHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveGPIOHardware"), "Activating DiffDriveGPIOHardware...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// Deactivation function
hardware_interface::CallbackReturn DiffDriveGPIOHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveGPIOHardware"), "Deactivating DiffDriveGPIOHardware...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveGPIOHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveGPIOHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));
    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

    return command_interfaces;
}

hardware_interface::return_type DiffDriveGPIOHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    double delta_seconds = period.seconds();

    // Read encoder values for both wheels
    int left_encoder_a = gpioRead(cfg_.gpio_left_encoder_a);
    int left_encoder_b = gpioRead(cfg_.gpio_left_encoder_b);
    int right_encoder_a = gpioRead(cfg_.gpio_right_encoder_a);
    int right_encoder_b = gpioRead(cfg_.gpio_right_encoder_b);

    // Process encoder inputs to update wheel encoder counts
    wheel_l_.processEncoder(left_encoder_a, left_encoder_b);
    wheel_r_.processEncoder(right_encoder_a, right_encoder_b);

    // Calculate new positions in radians
    double left_position_prev = wheel_l_.pos;
    wheel_l_.pos = wheel_l_.calcEncAngle();

    double right_position_prev = wheel_r_.pos;
    wheel_r_.pos = wheel_r_.calcEncAngle();

    // Calculate velocities (change in position over time)
    wheel_l_.vel = (wheel_l_.pos - left_position_prev) / delta_seconds;
    wheel_r_.vel = (wheel_r_.pos - right_position_prev) / delta_seconds;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveGPIOHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Set motor control signals based on commanded velocities
    double left_cmd = wheel_l_.cmd;
    double right_cmd = wheel_r_.cmd;

    // Control left motor direction and speed
    if (left_cmd >= 0) {
        gpioWrite(cfg_.gpio_left_motor_1, 1);
        gpioWrite(cfg_.gpio_left_motor_2, 0);
    } else {
        gpioWrite(cfg_.gpio_left_motor_1, 0);
        gpioWrite(cfg_.gpio_left_motor_2, 1);
    }
    gpioPWM(cfg_.gpio_left_enable, std::min(std::abs(left_cmd * 255), 255.0));

    // Control right motor direction and speed
    if (right_cmd >= 0) {
        gpioWrite(cfg_.gpio_right_motor_1, 1);
        gpioWrite(cfg_.gpio_right_motor_2, 0);
    } else {
        gpioWrite(cfg_.gpio_right_motor_1, 0);
        gpioWrite(cfg_.gpio_right_motor_2, 1);
    }
    gpioPWM(cfg_.gpio_right_enable, std::min(std::abs(right_cmd * 255), 255.0));

    return hardware_interface::return_type::OK;
}

} // namespace diffdrive_gpio

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_gpio::DiffDriveGPIOHardware, hardware_interface::SystemInterface)
