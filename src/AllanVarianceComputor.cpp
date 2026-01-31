
/**
 * @file   AllanVarianceComputor.cpp
 * @brief  Implementation of the AllanVarianceComputor class.
 * @author Russell Buchanan
 */

#include "allan_variance_ros2/AllanVarianceComputor.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/rclcpp.hpp>

namespace allan_variance_ros2 {

AllanVarianceComputor::AllanVarianceComputor(std::string config_file, std::string output_path)
    : firstMsg_(true), overlap_(0.0) {
  YAML::Node node = loadYamlFile(config_file);
  
  get(node, "imu_topic", input_topic_);
  std::cout << "imu_topic: " << input_topic_ << std::endl;
  get(node, "imu_rate", imu_rate_);
  std::cout << "imu_rate: " << imu_rate_ << std::endl;
  get(node, "measure_rate", measure_rate_);
  std::cout << "measure_rate: " << measure_rate_ << std::endl;
  get(node, "sequence_time", sequence_time_);
  std::cout << "sequence_time: " << sequence_time_ << std::endl;

  imu_skip_ = std::max(int(imu_rate_ / measure_rate_), 1);

  imu_output_file_ = output_path + "/" + "allan_variance" + ".csv";
}

// write_imu_only assumes batch optimization and thanamespace allan_variance_rost an optimization run had already happened
void AllanVarianceComputor::run(std::string bag_path) {
  std::cout << "Processing " << bag_path << " ..." << std::endl;

  av_output_ = std::ofstream(imu_output_file_.c_str(), std::ofstream::out);

  int imu_counter = 0;

  try {
    rosbag2_cpp::Reader reader;
    reader.open(bag_path);

    // Loop through data
    time_t start = clock();
    rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
    while (rclcpp::ok() && reader.has_next()) {
      // Fill IMU buffer
      const auto& msg = reader.read_next();
      // if (msg -> topic_name != input_topic_ || ) continue;
      sensor_msgs::msg::Imu imu_msg;
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      serialization.deserialize_message(&serialized_msg, &imu_msg);

      tCurrNanoSeconds_ = rclcpp::Time(imu_msg.header.stamp).nanoseconds();

      imu_counter++;

      // Subsample IMU measurements
      if (imu_counter % imu_skip_ != 0 || imu_counter / imu_rate_ > sequence_time_) {
        continue;
      }

      if (difftime(clock(), start) / CLOCKS_PER_SEC >= 2.0) {
        std::cout << imu_counter / imu_rate_ << " / " << sequence_time_ << " seconds loaded" << std::endl;
        start = clock();
      }

      if (firstMsg_) {
        firstMsg_ = false;
        firstTime_ = tCurrNanoSeconds_;
        lastImuTime_ = tCurrNanoSeconds_;
      }

      if (tCurrNanoSeconds_ < lastImuTime_) {
        skipped_imu_++;
        std::cerr << "IMU out of order. Current(ns): "
                          << tCurrNanoSeconds_ - firstTime_ << " Last(ns): "
                          << lastImuTime_ - firstTime_ << " (" << skipped_imu_ << " dropped)" << std::endl;
        continue;
      }
      lastImuTime_ = tCurrNanoSeconds_;

      ImuMeasurement input;
      input.t = rclcpp::Time(imu_msg.header.stamp).nanoseconds();
      input.I_a_WI = Eigen::Vector3d(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
                                      imu_msg.linear_acceleration.z);
      input.I_w_WI =
          Eigen::Vector3d(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);

      imuBuffer_.push_back(input);
    }

    reader.close();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  std::cout << "Finished collecting data. " << imuBuffer_.size() << " measurements" << std::endl;

  // Compute Allan Variance here
  if(!imuBuffer_.empty()) {
    allanVariance();
  } else {
    std::cerr << "No IMU messages to process, is your topic right?" << std::endl;
  }
}

void AllanVarianceComputor::closeOutputs() { av_output_.close(); }

void AllanVarianceComputor::allanVariance() {

  std::mutex mtx;
  bool stop_early = false;
  std::map<int,std::vector<std::vector<double>>> averages_map;

  // Range we will sample from (0.1s to 1000s)
  int period_min = 1;
  int period_max = 10000;

  // Overlapping method
  #pragma omp parallel for
  for (int period = period_min; period < period_max; period++) {

    if (!rclcpp::ok() || stop_early) {
      stop_early = true;
      continue;
    }

    std::vector<std::vector<double>> averages;
    double period_time = period * 0.1; // Sampling periods from 0.1s to 1000s

    int max_bin_size = period_time * measure_rate_;
    int overlap = floor(max_bin_size * overlap_);

    std::vector<double> current_average = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Compute Averages
    for (int j = 0; j < ((int)imuBuffer_.size() - max_bin_size); j += (max_bin_size - overlap)) {
      // get average for current bin
      for (int m = 0; m < max_bin_size; m++) {
        // std::cout << "j + m: " << j + m << std::endl;
        // // Acceleration
        current_average[0] += imuBuffer_[j + m].I_a_WI[0];
        current_average[1] += imuBuffer_[j + m].I_a_WI[1];
        current_average[2] += imuBuffer_[j + m].I_a_WI[2];

        // Gyro - assumes measurements in radians and convert to degrees
        current_average[3] += imuBuffer_[j + m].I_w_WI[0] * 180 / M_PI;
        current_average[4] += imuBuffer_[j + m].I_w_WI[1] * 180 / M_PI;
        current_average[5] += imuBuffer_[j + m].I_w_WI[2] * 180 / M_PI;
      }

      current_average[0] /= max_bin_size;
      current_average[1] /= max_bin_size;
      current_average[2] /= max_bin_size;
      current_average[3] /= max_bin_size;
      current_average[4] /= max_bin_size;
      current_average[5] /= max_bin_size;

      averages.push_back(current_average);
      current_average = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }


    {
      std::lock_guard<std::mutex> lck(mtx);
      int num_averages = averages.size();
      std::cout << "Computed " << num_averages << " averages for period " << period_time
                      << " (" << (10000 - averages_map.size()) << " left)" << std::endl;
      averages_map.insert({period, averages});
    }
  }

  if(!rclcpp::ok() || stop_early) {
    std::cout << "Stop requested, stopping calculation!" << std::endl;
    return;
  }


  std::vector<std::vector<double>> allan_variances;
  for (int period = period_min; period < period_max; period++) {

    std::vector<std::vector<double>> averages = averages_map.at(period);
    double period_time = period * 0.1; // Sampling periods from 0.1s to 1000s
    int num_averages = averages.size();
    std::cout << "Computed " << num_averages << " bins for sampling period " << period_time << " out of "
                                << imuBuffer_.size() << " measurements." << std::endl;

    // Compute Allan Variance
    std::vector<double> allan_variance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int k = 0; k < num_averages - 1; k++) {
      allan_variance[0] += std::pow(averages[k + 1][0] - averages[k][0], 2);
      allan_variance[1] += std::pow(averages[k + 1][1] - averages[k][1], 2);
      allan_variance[2] += std::pow(averages[k + 1][2] - averages[k][2], 2);
      allan_variance[3] += std::pow(averages[k + 1][3] - averages[k][3], 2);
      allan_variance[4] += std::pow(averages[k + 1][4] - averages[k][4], 2);
      allan_variance[5] += std::pow(averages[k + 1][5] - averages[k][5], 2);
    }
    std::vector<double> avar = {
        allan_variance[0] / (2 * (num_averages - 1)),
        allan_variance[1] / (2 * (num_averages - 1)),
        allan_variance[2] / (2 * (num_averages - 1)),
        allan_variance[3] / (2 * (num_averages - 1)),
        allan_variance[4] / (2 * (num_averages - 1)),
        allan_variance[5] / (2 * (num_averages - 1))};

    std::vector<double> allan_deviation = {std::sqrt(avar[0]), std::sqrt(avar[1]), std::sqrt(avar[2]),
                                           std::sqrt(avar[3]), std::sqrt(avar[4]), std::sqrt(avar[5])};

    writeAllanDeviation(allan_deviation, period_time);

    allan_variances.push_back(avar);

  }
}

void AllanVarianceComputor::writeAllanDeviation(std::vector<double> variance, double period) {
  aVRecorder_.period = period;
  aVRecorder_.accX = variance[0];
  aVRecorder_.accY = variance[1];
  aVRecorder_.accZ = variance[2];
  aVRecorder_.gyroX = variance[3];
  aVRecorder_.gyroY = variance[4];
  aVRecorder_.gyroZ = variance[5];
  aVRecorder_.writeOnFile(av_output_);
}

}  // namespace allan_variance_ros2
