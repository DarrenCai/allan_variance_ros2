ROS 2 version of [Allan Variance ROS](https://github.com/ori-drs/allan_variance_ros)

# How to build
```bash
git clone https://github.com/DarrenCai/allan_variance_ros2.git
cd path_to_allan_variance_ros2_ws
colcon build
```

# How to use
## setup
```bash
source path_to_allan_variance_ros2_ws/install/setup.bash
```
## Run the Allan Variance computation tool (example config files provided):
```bash
ros2 run allan_variance_ros2 allan_variance config/your_config.yaml your_bags_folder [-o your_output_folder]
```
## This will compute the Allan Deviation for the IMU and generate a CSV. The next step is to visualize the plots and get parameters. 
```bash
ros2 run allan_variance_ros2 analysis.py --data your_path_to_allan_variance.csv [--config config/your_config.yaml] [--output your_path_to_output_imu.yaml]
```