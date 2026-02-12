#include <sstream>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

class RFBeaconPublisher : public rclcpp::Node {
public:
    explicit RFBeaconPublisher(const rclcpp::NodeOptions& options)
        : Node("rf_beacon_publisher_node", options), generator_(std::random_device{}()) {

        // Topic to publish RF signal strength
        std::string publisher_topic_ = this->declare_parameter("publisher_topic", "rf_signal");

        // Gazebo ModelStates subscription topic
        std::string subscription_topic_ = this->declare_parameter("subscription_topic", "model_states");

        // RF model parameters
        path_loss_ = this->declare_parameter("path_loss", 3.0);
        min_range_ = this->declare_parameter("min_range", 0.1);
        max_range_ = this->declare_parameter("max_range", 10.0);
        tx_power_ = this->declare_parameter("tx_power", 4.0);
        noise_mean_ = this->declare_parameter("noise_mean", 0.0);
        noise_std_ = this->declare_parameter("noise_std", 2.0);

        // Publisher for RF signal data
        rf_signal_pub_ = this->create_publisher<std_msgs::msg::String>(publisher_topic_, rclcpp::SensorDataQoS().reliable());

        // Subscribe to Gazebo's /model_states
        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            subscription_topic_, rclcpp::SensorDataQoS(),
            std::bind(&RFBeaconPublisher::model_states_callback, this, std::placeholders::_1));

        // Publish rate (Hz)
        double publish_rate = this->declare_parameter("publish_rate", 10.0);
        timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::duration<double>(1.0 / publish_rate)),
            std::bind(&RFBeaconPublisher::publish_rf_signal, this));
    }

private:
    void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == this->get_name()) {
                beacon_position_ = msg->pose[i].position;
                beacon_position_initialized_ = true;
            } else if (msg->name[i] == "create3") {
                robot_position_ = msg->pose[i].position;
            }
        }
    }

    void publish_rf_signal() {
        if (!beacon_position_initialized_) {
            RCLCPP_WARN(this->get_logger(), "Beacon position not initialized yet.");
            return;
        }

        // Compute signal strength
        double signal_strength = calculate_signal_strength(
            robot_position_.x, robot_position_.y, robot_position_.z,
            beacon_position_.x, beacon_position_.y, beacon_position_.z);

        // Apply Gaussian noise
        signal_strength += generate_gaussian_noise();

        // Publish RF signal data
        auto msg = std::make_shared<std_msgs::msg::String>();
        std::ostringstream oss;
        oss << "Beacon=" << this->get_name() << ";SignalStrength=" << signal_strength;
        msg->data = oss.str();

        rf_signal_pub_->publish(*msg);
    }

    double calculate_signal_strength(double robot_x, double robot_y, double robot_z,
                                     double beacon_x, double beacon_y, double beacon_z) {
        double dx = robot_x - beacon_x;
        double dy = robot_y - beacon_y;
        double dz = robot_z - beacon_z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Avoid log(0) errors
        distance = std::max(distance, min_range_);

        // Compute RSSI
        double rssi = tx_power_ - 10 * path_loss_ * std::log10(distance);

        // Simulate no signal beyond max range
        if (distance > max_range_) {
            rssi = -100;
        }

        return rssi;
    }

    double generate_gaussian_noise() {
        std::normal_distribution<double> distribution(noise_mean_, noise_std_);
        return distribution(generator_);
    }

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rf_signal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Point beacon_position_;
    geometry_msgs::msg::Point robot_position_;

    bool beacon_position_initialized_ = false;
    double min_range_;
    double max_range_;
    double tx_power_;
    double path_loss_;
    double noise_mean_;
    double noise_std_;

    std::default_random_engine generator_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RFBeaconPublisher>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
