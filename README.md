# Autonomous-Drones
List of open-source algorithms and resources for UAVs

## Perception

| Algorithm   | Institution    | Background                                                                                                           | ROS           | Link                                                 |
|-------------|----------------|----------------------------------------------------------------------------------------------------------------------|---------------|------------------------------------------------------|
| voxblox     | ETH            | voxel-based mapping                                                                                                  | :heavy_check_mark:   | https://github.com/ethz-asl/voxblox                  |
| maplab      | ETH            | visual inertial mapping                                                                                              | :heavy_check_mark:    | https://github.com/ethz-asl/mav_voxblox_planning     |
| orb-slam2   |                | sparse 3D reconstruction                                                                                             | :heavy_check_mark:    | https://github.com/raulmur/ORB_SLAM2                 |
| open_vins   | U. of Delaware | EKF fuses inertial info with sparse visual features                                                                  | :heavy_check_mark: | https://github.com/rpng/open_vins                    |
| SVO 2.0     | ETH            | semi-direct paradigm to estimate pose from pixel intensities and features                                            | :heavy_check_mark:  | http://rpg.ifi.uzh.ch/svo2.html                      |
| DSO         | TUM            | direct sparse odometry                                                                                               |               | https://github.com/JakobEngel/dso/                   |
| XIVO        | UCLA           | inertial-aided visual odometry                                                                                       |               | https://feixh.github.io/projects/xivo/               |
| VINS-Fusion | HKUST          | An optimization-based multi-sensor state estimator                                                                   |               | https://github.com/HKUST-Aerial-Robotics/VINS-Fusion |
| Kimera-VIO  | MIT            | real-time metric-semantic SLAM and VIO                                                                               | :heavy_check_mark:           | https://github.com/MIT-SPARK/Kimera                  |
| tagSLAM     | UPenn          | tagSLAM with apriltags                                                                                               | :heavy_check_mark:           | https://github.com/berndpfrommer/tagslam             |
| LARVIO      |                | A lightweight, accurate and robust monocular visual inertial odometry based on Multi-State Constraint Kalman Filter. | :heavy_check_mark:       | https://github.com/PetWorm/LARVIO                    |
| R-VIO       |                | based on robocentric sliding-window filtering-based VIO framework                                                    |               | https://github.com/rpng/R-VIO                        |
| nanomap     | MIT            | fast, uncertainty-aware proximity queries with lazy search of local 3D data                                          |               | https://github.com/peteflorence/nanomap_ros          |
| MSCKF_VIO   | UPenn          | package is a stereo version of MSCKF                                                                                 |               | https://github.com/KumarRobotics/msckf_vio           |
| VINS_mono   | HKUST          | Robust and Versatile Monocular Visual-Inertial State Estimator                                                       |               | https://github.com/HKUST-Aerial-Robotics/VINS-Mono   |


## Navigation
| mav_trajectory_generation         | ETH          | creates polynomial path                                                                           | :heavy_check_mark: | https://github.com/ethz-asl/mav_trajectory_generation                                                           |
|-----------------------------------|--------------|---------------------------------------------------------------------------------------------------|--------------------|-----------------------------------------------------------------------------------------------------------------|
| mav_voxblox_planning              | ETH          | planning tool using voxblox (RRT*, etc.)                                                          | :heavy_check_mark: | https://github.com/ethz-asl/mav_voxblox_planning                                                                |
| pulp-dronet                       | ETH          | deep learning visual navigation                                                                   |                    | https://github.com/pulp-platform/pulp-dronet                                                                    |
| Ewok: real-time traj replanning   | TUM          | replanning of global traj, needs prior map                                                        |                    | https://github.com/VladyslavUsenko/ewok                                                                         |
| Deep RL with Transfer Learning    | Georgia Tech | end-to-end navigation trained from simulation                                                     |                    | https://github.com/aqeelanwar/DRLwithTL_real                                                                    |
| NVIDIA redtail project            |              | Autonomous navigation for drones                                                                  |                    | https://github.com/NVIDIA-AI-IOT/redtail                                                                        |
| Fast-Planner                      | HKUST        | robust and efficient trajectory planner for quads                                                 | :heavy_check_mark: | https://github.com/HKUST-Aerial-Robotics/Fast-Planner                                                           |
| open_street_map                   |              | Experimental packages for ROS access to Open Street Map information                               | :heavy_check_mark: | https://github.com/ros-geographic-info/open_street_map                                                          |
| spatio-temporal semantic corridor | HKUST        | Safe Trajectory Generation For Complex Urban Environments Using Spatio-temporal Semantic Corridor |                    | https://github.com/HKUST-Aerial-Robotics/spatiotemporal_semantic_corridor                                       |
| EVDodgeNet                        | ETH          | obstacle avoidance with event cameras                                                             |                    | https://github.com/prgumd/EVDodgeNet                                                                            |
| aeplanner                         | KTH          | unknown environment exploration based on octomap                                                  |                    | https://github.com/mseln/aeplanner                                                                              |
| nvbplanner                        | ETH          | unknown environment exploration                                                                   |                    | https://github.com/ethz-asl/nbvplanner                                                                          |
| Deep-drone acrobatics             | ETH          | fly complex maneuvers with multi-layer perceptron                                                 |                    | https://github.com/uzh-rpg/deep_drone_acrobatics                                                                |
| mav_control_rw                    | ETH          | trajectory tracking with MPC                                                                      |                    | https://github.com/ethz-asl/mav_control_rw                                                                      |
| rpg_quadrotor_control             | ETH          | complete framework for flying quadrotors                                                          |                    | https://github.com/uzh-rpg/rpg_quadrotor_control                                                                |
| flight controller                 | HKUST        | high level controller compatible with DJI N3 flight controller                                    |                    | https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment/onboard_computer/controller/n3ctrl |



