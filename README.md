# Autonomous-Drones
List of open-source algorithms and resources for UAVs

## Perception

| Algorithm   | Institution    | Background                                                                                                           | ROS          |
|-------------|----------------|----------------------------------------------------------------------------------------------------------------------|---------------|------------------------------------------------------|
| [voxblox](https://github.com/ethz-asl/voxblox)     | ETH            | voxel-based mapping                                                                                                  | :heavy_check_mark:     |
| [maplab](https://github.com/ethz-asl/mav_voxblox_planning)      | ETH            | visual inertial mapping                                                                                              | :heavy_check_mark:       |
| [orb-slam2](https://github.com/raulmur/ORB_SLAM2)   |                | sparse 3D reconstruction                                                                                             | :heavy_check_mark:                  |
| [open_vins](https://github.com/rpng/open_vins)   | U. of Delaware | EKF fuses inertial info with sparse visual features                                                                  | :heavy_check_mark: | https://github.com/rpng/open_vins                    |
| [SVO 2.0](http://rpg.ifi.uzh.ch/svo2.html)     | ETH            | semi-direct paradigm to estimate pose from pixel intensities and features                                            | :heavy_check_mark:  | http://rpg.ifi.uzh.ch/svo2.html                      |
| [DSO](https://github.com/JakobEngel/dso/)         | TUM            | direct sparse odometry                                                                                               |               | https://github.com/JakobEngel/dso/                   |
| [XIVO](https://feixh.github.io/projects/xivo/)        | UCLA           | inertial-aided visual odometry                                                                                       |               |
| [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | HKUST          | An optimization-based multi-sensor state estimator                                                                   |               |
| [Kimera-VIO](https://github.com/MIT-SPARK/Kimera)  | MIT            | real-time metric-semantic SLAM and VIO                                                                               | :heavy_check_mark:                  |
| [tagSLAM](https://github.com/berndpfrommer/tagslam)     | UPenn          | tagSLAM with apriltags                                                                                               | :heavy_check_mark:                   |
| [LARVIO](https://github.com/PetWorm/LARVIO)      |                | A lightweight, accurate and robust monocular visual inertial odometry based on Multi-State Constraint Kalman Filter. | :heavy_check_mark:                     |
| [R-VIO](https://github.com/rpng/R-VIO)    |                | based on robocentric sliding-window filtering-based VIO framework                                                    |                                  |
| [nanomap](https://github.com/peteflorence/nanomap_ros)  | MIT            | fast, uncertainty-aware proximity queries with lazy search of local 3D data                                          |                 |
| [MSCKF_VIO](https://github.com/KumarRobotics/msckf_vio)  | UPenn          | package is a stereo version of MSCKF                                                                                 |                   |
| [VINS_mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)   | HKUST          | Robust and Versatile Monocular Visual-Inertial State Estimator                                                       |    |


## Navigation
| Algorithm   | Institution    | Background                                                                                                           | ROS           | Link                                                 |
|-------------|----------------|----------------------------------------------------------------------------------------------------------------------|---------------|------------------------------------------------------|
| mav_trajectory_generation         | ETH          | creates polynomial path                                                                           | :heavy_check_mark: | https://github.com/ethz-asl/mav_trajectory_generation                |        
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



## Autonomy

| Algorithm                               | Institution  | Background                                                                              | ROS                | Link                                                               |
|-----------------------------------------|--------------|-----------------------------------------------------------------------------------------|--------------------|--------------------------------------------------------------------|
| sim2real_drone_racing                   | ETH          | deep learning Sim2Real Drone racing                                                     | :heavy_check_mark: | https://github.com/uzh-rpg/sim2real_drone_racing                   |
| dronet: learning to fly                 | ETH          | deep learning trained from cars to predict steering angle, collision prob               | :heavy_check_mark: | https://github.com/uzh-rpg/rpg_public_dronet                       |
| Deep RL w Airsim                        |              | allows RL with Airsim                                                                   |                    | https://github.com/guillemhub/DRLDBackEnd                          |
| Autonomous UAV swarms                   |              |                                                                                         |                    | https://github.com/AlexJinlei/Autonomous_UAVs_Swarm_Mission        |
| autonomous-drone                        |              | enable autonomous drone delivery w Aero RTF and PX4                                     | :heavy_check_mark: | https://github.com/szebedy/autonomous-drone                        |
| PEDRA                                   | Georgia Tech | RL for drones with unreal engine                                                        |                    | https://github.com/aqeelanwar/PEDRA                                |
| Apollo Autonomous Driving               | Apollo       | Full autonomous driving stack                                                           |                    | https://github.com/ApolloAuto/apollo                               |
| ROS_behavior_tree                       |              | Behavior trees for autonomy                                                             |                    | https://github.com/miccol/ROS-Behavior-Tree                        |
| planning in ROS                         |              | generic method for task planning                                                        | :heavy_check_mark: | https://github.com/KCL-Planning/ROSPlan                            |
| HKUST Aerial Robotics                   | HKUST        | a complete and robust system for aggressive flight in complex environment               |                    | https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan       |
| EDUM Planner                            | HKUST        | decision-making for automated driving using guided branching                            |                    | https://github.com/HKUST-Aerial-Robotics/eudm_planner              |
| autoware.ai                             |              | self-driving vehicles                                                                   |                    | https://github.com/autowarefoundation/autoware.ai                  |
| PX4 generalized intelligence            |              | low-level autonomy for PX4                                                              |                    | https://github.com/generalized-intelligence/GAAS                   |
| PX4 avoidance                           |              |                                                                                         |                    | https://github.com/PX4/avoidance                                   |
| Uncertainty estimation in deep learning | ETH          | can quantify uncertainty on existing neural networks                                    |                    | https://github.com/mattiasegu/uncertainty_estimation_deep_learning |
| drif                                    | Cornell      | Map Natural Language Instructions to Physical Quadcopter Control using Simulated Flight |                    | https://github.com/VerifiableRobotics/slugs                        |
| slugs                                   |              | slugs - SmalL bUt Complete GROne Synthesizer                                            |                    |                                                                    |
| LTL_stack                               |              | ROS Packages for running correct-by-construction controllers with ROS                   | :heavy_check_mark: | https://github.com/VerifiableRobotics/LTL_stack                    |

## Controls

| Algorithm                    | Institution | Background                                                                                                               | ROS | Link                                                       |
|------------------------------|-------------|--------------------------------------------------------------------------------------------------------------------------|-----|------------------------------------------------------------|
| lbmpc_ipm                    |             |  Learning-Based Model Predictive Control (LBMPC) that uses the LBmpcIPM solver                                           |     | https://github.com/mhkabir/LBMPC                           |
| neural_mpc                   | Berkeley    | Model Predictive Control with one-step feedforward neural network dynamics model from Model-based Reinforcement Learning |     | https://github.com/aravindsrinivas/neural-mpc              |
| Control Toolbox              | ETH         | efficient C++ library for control, estimation, optimization and motion planning in robotics                              |     | https://github.com/ethz-adrl/control-toolbox               |
| PythonLinearNonlinearControl |             | library implementing the linear and nonlinear control theories in python                                                 |     | https://github.com/Shunichi09/PythonLinearNonlinearControl |
| rpg_mpc                      | ETH         | Model Predictive Control for Quadrotors with extension to Perception-Aware MPC                                           |     |                  https://github.com/uzh-rpg/rpg_mpc                                          |
| rpg_quadrotor_control        | ETH         | alternative to PX4 that works with RotorS                                                                                |     | https://github.com/uzh-rpg/rpg_quadrotor_control           |
| gymFC                        |             | flight control tuning framework with a focus in attitude control                                                         |     | https://github.com/wil3/gymfc                              |
| ACADO toolkit                |             | MPC toolkit that takes care of the implementation                                                                        |     | http://acado.github.io/index.html                          |
| MPC ETH                      | ETH         | also has PX4 implementation (claim badly hacked though)                                                                  |     | https://github.com/ethz-asl/mav_control_rw                 |
| mavros_controller            | PX4         | trajectory tracking based on geometric control                                                                           |     | https://github.com/Jaeyoung-Lim/mavros_controllers         |
| DDC-MPC                      |             | ETH MPC                                                                                                                  |     | https://github.com/uzh-rpg/data_driven_mpc                 |


## Tools

| Algorithm                  | Institution | Background                                                        | ROS | Link                                                                           |
|----------------------------|-------------|-------------------------------------------------------------------|-----|--------------------------------------------------------------------------------|
| PythonRobotics             |             | great overview of robotics                                        |     | https://github.com/AtsushiSakai/PythonRobotics                                 |
| Awesome-robotic-tooling    |             | important tools for robotic programming                           |     | https://github.com/Ly0n/awesome-robotic-tooling                                |
| aweomse-dronecraft         |             | everything about drones                                           |     | https://github.com/Zarkopafilis/awesome-dronecraft                             |
| resilience-engineering     |             | How to make safe systems?                                         |     | https://github.com/lorin/resilience-engineering                                |
| Trajectory Prediction      |             | resources for predicting environment like movement of pedestrians |     | https://github.com/jiachenli94/Awesome-Interaction-aware-Trajectory-Prediction |
| hidden markov model        |             | models lane switching, might be interesting                       |     | https://github.com/chauvinSimon/hmm_for_autonomous_driving                     |
| modelling agents w prob    |             | MPD, POMPD, etc.                                                  |     | https://agentmodels.org/                                                       |
| hierarchical state machine | ROS         | Develop robotic tasks through graphical user interface            |     | http://wiki.ros.org/smach/Tutorials                                            |
| Zurich simulator           |             | good for RL                                                       |     | https://github.com/uzh-rpg/flightmare                                          |
