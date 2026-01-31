/**
 * @file   ImuSimulator.cpp
 * @brief  Tool to simulate imu data, ref:
 * https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model.
 * @author Rick Liu
 */

// std, eigen and boost
#include <boost/filesystem.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <ctime>
#include <fstream>
#include <set>

// ROS 2
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

#include "allan_variance_ros2/yaml_parsers.hpp"

using Vec3d = Eigen::Vector3d;

Vec3d RandomNormalDistributionVector(double sigma) {
  static boost::mt19937 rng;
  static boost::normal_distribution<> nd(0, 1);
  return {sigma * nd(rng), sigma * nd(rng), sigma * nd(rng)};
}

template <typename S, typename T> void FillROSVector3d(const S &from, T &to) {
  to.x = from.x();
  to.y = from.y();
  to.z = from.z();
}

class ImuSimulator {
public:
  ImuSimulator(std::string config_file, std::string output_path) {
    auto yaml_config = loadYamlFile(config_file);

    get(yaml_config, "accelerometer_noise_density",
        accelerometer_noise_density_);
    get(yaml_config, "accelerometer_random_walk", accelerometer_random_walk_);
    get(yaml_config, "accelerometer_bias_init", accelerometer_bias_init_);

    get(yaml_config, "gyroscope_noise_density", gyroscope_noise_density_);
    get(yaml_config, "gyroscope_random_walk", gyroscope_random_walk_);
    get(yaml_config, "gyroscope_bias_init", gyroscope_bias_init_);

    get(yaml_config, "rostopic", rostopic_);
    std::cout << "rostopic: " << rostopic_ << std::endl;
    get(yaml_config, "update_rate", update_rate_);
    std::cout << "update_rate: " << update_rate_ << std::endl;
    get(yaml_config, "sequence_time", sequence_time_);
    std::cout << "sequence_time: " << sequence_time_ << std::endl;

    rosbag2_storage::TopicMetadata imu_topic;
    imu_topic.name = rostopic_;
    imu_topic.type = "sensor_msgs/msg/Imu";
    imu_topic.serialization_format = "cdr";

    bag_output_.open(output_path);
    bag_output_.create_topic(imu_topic);
  }

  virtual ~ImuSimulator() { bag_output_.close(); }

  void run() {
    std::cout << "Generating IMU data ..." << std::endl;

    double dt = 1 / update_rate_;

    // clang-format off
    rclcpp::Time start_time(1, 0);
    Vec3d accelerometer_bias = Vec3d::Constant(accelerometer_bias_init_);
    Vec3d gyroscope_bias = Vec3d::Constant(gyroscope_bias_init_);
    Vec3d accelerometer_real = Vec3d::Zero();
    Vec3d gyroscope_real = Vec3d::Zero();

    auto serialization_ = std::make_unique<rclcpp::Serialization<sensor_msgs::msg::Imu>>();

    for (int64_t i = 0; i < sequence_time_ * update_rate_; ++i) {

      // Break if requested by user
      if (!rclcpp::ok()) {
        break;
      }

      // Reference: https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
      accelerometer_bias += RandomNormalDistributionVector(accelerometer_random_walk_) * sqrt(dt);
      gyroscope_bias += RandomNormalDistributionVector(gyroscope_random_walk_) * sqrt(dt);

      Vec3d acc_measure = accelerometer_real + accelerometer_bias + RandomNormalDistributionVector(accelerometer_noise_density_) / sqrt(dt);
      Vec3d gyro_measure = gyroscope_real + gyroscope_bias + RandomNormalDistributionVector(gyroscope_noise_density_) / sqrt(dt);

      sensor_msgs::msg::Imu msg;
      msg.header.stamp = start_time + rclcpp::Duration(1, 0) * (i / update_rate_);
      FillROSVector3d(acc_measure, msg.linear_acceleration);
      FillROSVector3d(gyro_measure, msg.angular_velocity);

      auto serialized_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      serialized_msg->send_timestamp = rclcpp::Time(msg.header.stamp).nanoseconds();
      serialized_msg->topic_name = rostopic_;
      
      rclcpp::SerializedMessage serialized_data;
      serialization_->serialize_message(&msg, &serialized_data);
      
      serialized_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
      *serialized_msg->serialized_data = serialized_data.release_rcl_serialized_message();

      bag_output_.write(serialized_msg);
    }
    // clang-format on

    std::cout << "Finished generating data. " << std::endl;
  }

private:
  // ROS 2
  rosbag2_cpp::Writer bag_output_;

private:
  double accelerometer_noise_density_;
  double accelerometer_random_walk_;
  double accelerometer_bias_init_;

  double gyroscope_noise_density_;
  double gyroscope_random_walk_;
  double gyroscope_bias_init_;

  std::string rostopic_;
  double update_rate_;

  double sequence_time_;
};

int main(int argc, char **argv) {
  std::string rosbag_folder;
  std::string config_file;

  if (argc >= 2) {
    rosbag_folder = argv[1];
    config_file = argv[2];
    std::cout << "Bag filename = " << rosbag_folder << std::endl;
    std::cout << "Config File = " << config_file << std::endl;
  } else {
    std::cerr << "Usage: ./imu_simulator /path/to/output/bag_folder /path/to/simulation/config_filename" << std::endl;
    return 1;
  }

  auto start = std::clock();

  rclcpp::init(argc, argv);
  ImuSimulator simulator(config_file, rosbag_folder);
  std::cout << "IMU simulator constructed" << std::endl;
  simulator.run();

  double durationTime = (std::clock() - start) / (double)CLOCKS_PER_SEC;
  std::cout << "Total computation time: " << durationTime << " s" << std::endl;

  rclcpp::shutdown();
  return 0;
}
