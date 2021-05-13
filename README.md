# Autonomous UAVs
List of open-source algorithms and resources for autonomous drones. The list is a work in progress, so some information may be wrong and lots of useful resources are still missing.

## Perception

| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
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
| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
| [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation)        | ETH          | creates polynomial path                                                                           | :heavy_check_mark:                 |        
| [mav_voxblox_planning](https://github.com/ethz-asl/mav_voxblox_planning)        | ETH          | planning tool using voxblox (RRT*, etc.)                                                          | :heavy_check_mark:                                                                 |
| [pulp-dronet](https://github.com/pulp-platform/pulp-dronet)                       | ETH          | deep learning visual navigation                                                                   |                                                                                       |
| [Ewok: real-time traj replanning](https://github.com/VladyslavUsenko/ewok)   | TUM          | replanning of global traj, needs prior map                                                        |                                                                                       |
| [Deep RL with Transfer Learning](https://github.com/aqeelanwar/DRLwithTL_real)    | Georgia Tech | end-to-end navigation trained from simulation                                                     |                                                                                    |
| [NVIDIA redtail project](https://github.com/NVIDIA-AI-IOT/redtail)            |              | Autonomous navigation for drones                                                                  |                                                                                         |
| [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)  | HKUST        | robust and efficient trajectory planner for quads                                                 | :heavy_check_mark:                                                           |
| [ego-planner swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) | Zhejiang University |  Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments
| [spatio-temporal semantic corridor](https://github.com/HKUST-Aerial-Robotics/spatiotemporal_semantic_corridor) | HKUST        | Safe Trajectory Generation For Complex Urban Environments Using Spatio-temporal Semantic Corridor |                                                          |
| [EVDodgeNet](https://github.com/prgumd/EVDodgeNet)                        | ETH          | obstacle avoidance with event cameras                                                             |                                                                                             |
| [aeplanner](https://github.com/mseln/aeplanner)                         | KTH          | unknown environment exploration based on octomap                                                  |                                                                                             |
| [nvbplanner](https://github.com/ethz-asl/nbvplanner)                        | ETH          | unknown environment exploration                                                                   |                                                                                        |
| [HKUST Aerial Robotics](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan)                   | HKUST        | a complete and robust system for aggressive flight in complex environment               |                          |
| [PX4 generalized intelligence](https://github.com/generalized-intelligence/GAAS)            |              | low-level autonomy for PX4                                                              |                                      |
| [PX4 avoidance](https://github.com/PX4/avoidance)                           |              |        low-level autonomy for PX4           |
| [sim2real_drone_racing](https://github.com/uzh-rpg/sim2real_drone_racing)                   | ETH          | deep learning Sim2Real Drone racing                                                     | :heavy_check_mark:                  |
| [waypoint_navigator](https://github.com/ethz-asl/waypoint_navigator) | ETH | high-level waypoint-following for micro aerial vehicles | :heavy_check_mark: |
| [autonomousmavs](https://github.com/IntelLabs/autonomousmavs) | | navigation in cluttered environment | :heavy_check_mark: |

## Strategic Decision-Making

| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
| [Apollo Autonomous Driving](https://github.com/ApolloAuto/apollo)               | Apollo       | Full autonomous driving stack                                                           |                                                  |
| [ROS_behavior_tree](https://github.com/miccol/ROS-Behavior-Tree)                       |              | Behavior trees for autonomy                                                             |                                            |
| [planning in ROS](https://github.com/KCL-Planning/ROSPlan)                         |              | generic method for task planning                                                        | :heavy_check_mark:                            |
| [EDUM Planner](https://github.com/HKUST-Aerial-Robotics/eudm_planner)                            | HKUST        | decision-making for automated driving using guided branching                            |                                  |
| [autoware.ai](https://github.com/autowarefoundation/autoware.ai)                             |              | self-driving vehicles                                                                   |                                     |
| [dronet: learning to fly](https://github.com/uzh-rpg/rpg_public_dronet)                 | ETH          | deep learning trained from cars to predict steering angle, collision prob               | :heavy_check_mark: | 
| [Deep RL w Airsim](https://github.com/guillemhub/DRLDBackEnd)                        |              | allows RL with Airsim                                                                   |                                             |
| [Autonomous UAV swarms](https://github.com/AlexJinlei/Autonomous_UAVs_Swarm_Mission)                   |              |                                                                                         |                           |
| [autonomous-drone](https://github.com/szebedy/autonomous-drone)                        |              | enable autonomous drone delivery w Aero RTF and PX4                                     | :heavy_check_mark:           |
| [PEDRA](https://github.com/aqeelanwar/PEDRA)                                   | Georgia Tech | RL for drones with unreal engine                                                        |                                                   |
| [drif](https://github.com/VerifiableRobotics/slugs)                                    | Cornell      | Map Natural Language Instructions to Physical Quadcopter Control using Simulated Flight |                                     |
| [slugs](https://github.com/VerifiableRobotics/slugs)                                   |              | slugs - SmalL bUt Complete GROne Synthesizer                                            |                    |                                                                    |
| [LTL_stack](https://github.com/VerifiableRobotics/LTL_stack)                               |              | ROS Packages for running correct-by-construction controllers with ROS                   | :heavy_check_mark:                    | 
| [multidrone_planning](https://github.com/grvcTeam/multidrone_planning)  | | cooperative planning and mission execution in autonomous cinematography with multiple drones| :heavy_check_mark:|


## Multi-Agent

| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
| [Robofleet](https://github.com/ut-amrl/robofleet) | UT Austin | Web-based multi-robot control and visualization for ROS | :heavy_check_mark:  |


## Controls

| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
| [lbmpc_ipm](https://github.com/mhkabir/LBMPC)                    |             |  Learning-Based Model Predictive Control (LBMPC) that uses the LBmpcIPM solver                                           |                                |
| [neural_mpc](https://github.com/aravindsrinivas/neural-mpc)                   | Berkeley    | Model Predictive Control with one-step feedforward neural network dynamics model from Model-based Reinforcement Learning |                   |
| [Control Toolbox](https://github.com/ethz-adrl/control-toolbox)              | ETH         | efficient C++ library for control, estimation, optimization and motion planning in robotics                              |                    |
| [PythonLinearNonlinearControl](https://github.com/Shunichi09/PythonLinearNonlinearControl) |             | library implementing the linear and nonlinear control theories in python                                                 |      |
| [rpg_mpc](https://github.com/uzh-rpg/rpg_mpc)                      | ETH         | Model Predictive Control for Quadrotors with extension to Perception-Aware MPC                                           |                                              |
| [rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control)        | ETH         | alternative to PX4 that works with RotorS                                                                                |                |
| [gymFC](https://github.com/wil3/gymfc)                        |             | flight control tuning framework with a focus in attitude control                                                         |                                   |
| [ACADO toolkit](http://acado.github.io/index.html)                |             | MPC toolkit that takes care of the implementation                                                                        |                               |
| [MPC ETH](https://github.com/ethz-asl/mav_control_rw)                      | ETH         | also has PX4 implementation (claim badly hacked though)                                                                  |                     |
| [mavros_controller](https://github.com/Jaeyoung-Lim/mavros_controllers)            | PX4         | trajectory tracking based on geometric control                                                                           |              |
| [DDC-MPC](https://github.com/uzh-rpg/data_driven_mpc)                      |      ETH       | Data-Driven MPC for Quadrotors
| [Deep-drone acrobatics](https://github.com/uzh-rpg/deep_drone_acrobatics)             | ETH          | fly complex maneuvers with multi-layer perceptron                                                 |                                                                                   |
| [mav_control_rw](https://github.com/ethz-asl/mav_control_rw)                    | ETH          | trajectory tracking with MPC                                                                      |                                                                                         |
| [rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control)             | ETH          | complete framework for flying quadrotors                                                          |                                                                            |
| [flight controller](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment/onboard_computer/controller/n3ctrl)                 | HKUST        | high level controller compatible with DJI N3 flight controller                                    |                     |
| [mavros_trajectory_tracking](https://github.com/mzahana/mavros_trajectory_tracking) | | combines mav_trajectory_generation and waypoint_navigator with mavros_controller |:heavy_check_mark: | 



## Useful Tools and Resources

| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
| [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)             |             | great overview of robotics   |                               
| [Awesome-robotic-tooling](https://github.com/Ly0n/awesome-robotic-tooling)    |             | important tools for robotic programming |
| [awesome-dronecraft](https://github.com/Zarkopafilis/awesome-dronecraft)         |             | everything about drones                                           |                                 |
| [resilience-engineering](https://github.com/lorin/resilience-engineering)     |             | How to make safe systems?                                         |                                 |
| [Trajectory Prediction](https://github.com/jiachenli94/Awesome-Interaction-aware-Trajectory-Prediction)      |             | resources for predicting environment like movement of pedestrians |  |
| [hidden markov model](https://github.com/chauvinSimon/hmm_for_autonomous_driving)        |             | models lane switching, might be interesting                       |                      |
| [modelling agents w prob](https://agentmodels.org/)    |             | MPD, POMPD, etc.                                                  |                                   |
| [hierarchical state machine](http://wiki.ros.org/smach/Tutorials) |          | Develop robotic tasks through graphical user interface            |                                             |
| [Uncertainty estimation in deep learning](https://github.com/mattiasegu/uncertainty_estimation_deep_learning) | ETH          | can quantify uncertainty on existing neural networks                                    |                   |
| [Flightmare simulator](https://github.com/uzh-rpg/flightmare)           |             | Flightmare is composed of two main components: a configurable rendering engine built on Unity and a flexible physics engine for dynamics simulation.                                        | |
| [US-manufactured drones](https://dronelife.com/2021/05/13/u-s-based-drone-manufacturers-check-out-our-updated-list/) | | A list of drones manufactured in the US.

## Labs/Organizations to follow
| Lab Website | Git | Where       |
|:-------------|-------------|-------------|
| [Robotics & Perception Group](http://rpg.ifi.uzh.ch/) | [Link](https://github.com/uzh-rpg) | Zurich, Switzerland |
| [GRASP Lab](https://www.kumarrobotics.org/) | [Link](https://github.com/KumarRobotics) | Philadelphia, USA
| [ZJU FAST Lab](http://zju-fast.com/) | [Link](https://github.com/ZJU-FAST-Lab) | Hangzhou, China |

