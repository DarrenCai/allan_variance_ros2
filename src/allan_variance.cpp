/**
 * @file   allan_variance.cpp
 * @brief  Tool to compute Allan Variance and Deviation from rosbag2.
 * @author DarrenCai
 */

// std, eigen and boost
#include <boost/filesystem.hpp>
#include <ctime>
#include <fstream>
#include <set>

#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "argparse.hpp"

// allan_variance_ros2
#include "allan_variance_ros2/AllanVarianceComputor.hpp"

using namespace std;

struct bag_file {
  string path, name;
  bool operator< (const bag_file& rhs) const {
    return name < rhs.name;
  }
};

int main(int argc, char** argv) {
  argparse::ArgumentParser args("allan_variance");
  args.add_argument("config_file");
  args.add_argument("ros2_bags_folder").default_value("bags");
  args.add_argument("-o", "--out").help("output path");

  try {
      args.parse_args(argc, argv);
  } catch (const exception& e) {
      cerr << args;
      return 1;
  }
  
  string config_file = args.get<string>("config_file");
  string bags_folder = args.get<string>("ros2_bags_folder");
  string output_path = bags_folder;

  if (const auto& op = args.present("-o")) {
    output_path = *op;
  }
  
  cout << "config file: " << config_file << endl;
  cout << "ros2 bag folder: " << bags_folder << endl;
  cout << "output path: " << output_path << endl;

  rclcpp::init(argc, argv);

  namespace fs = boost::filesystem;
  fs::path path = fs::path(bags_folder);

  rosbag2_cpp::Reader reader;
  set<bag_file> bag_files_sorted;
  auto func = [&reader, &bag_files_sorted](const fs::path& fp) {
      try {
          reader.open(fp.string());
          bag_files_sorted.insert({fp.string(), fp.stem().string()});
      } catch (const exception& e) {
          // do nothing
      }
      reader.close();
  };
  for (const auto& entry: fs::directory_iterator(path)) {
    if (fs::is_directory(entry.status())) {
      for (const auto& e: fs::directory_iterator(entry.path())) if (fs::is_regular_file(e.status())) func(e.path());
    } else if (fs::is_regular_file(entry.status())) func(entry.path());
  }
  cout << "Bag filenames count: " << bag_files_sorted.size() << endl;
  for (const auto& bag_file: bag_files_sorted)
    cout << "Bag filenames count: " << bag_file.path << endl;

  clock_t start = clock();

  allan_variance_ros2::AllanVarianceComputor computor(config_file, output_path);
  cout << "Batch computor constructed" << endl;
  for (const auto& bag_file : bag_files_sorted) computor.run(bag_file.path);

  double durationTime = (clock() - start) / (double)CLOCKS_PER_SEC;
  cout << "Total computation time: " << durationTime << " s" << endl;
  cout << "Data written to allan_variance.csv" << endl;

  rclcpp::shutdown();
  return 0;
}
