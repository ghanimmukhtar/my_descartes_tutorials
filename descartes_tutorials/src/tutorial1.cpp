// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>

#include <baxter_core_msgs/JointCommand.h>

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

std::vector<double> joints_values(19);
std::vector <std::string> variable_names(19);

void jocommCallback(sensor_msgs::JointState jo_state)
{
    //simulation
    for(int i = 0; i < jo_state.name.size(); i++){
        variable_names[i] = jo_state.name[i];
        joints_values[i] = jo_state.position[i];
        //std::cout << jo_state.position[i] << std::endl;
    }
}

/**
 * Generates an completely defined (zero-tolerance) cartesian point from a pose
 */
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);
/**
 * Generates a cartesian point with free rotation about the Z axis of the EFF frame
 */
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint2(const Eigen::Affine3d& pose);
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(double x, double y, double z, double rx, double ry, double rz);

/**
 * Translates a descartes trajectory to a ROS joint trajectory
 */
trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory, const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names, double time_delay);

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "descartes_tutorial");
  ros::NodeHandle nh;

  ros::Subscriber sub_jointmsg;
  sub_jointmsg = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
  ros::Publisher pub_msg;
  pub_msg = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",1);
  // Required for communication with moveit components
  ros::AsyncSpinner spinner (1);
  spinner.start();

  ros::Publisher planning_scene_publish = nh.advertise<moveit_msgs::PlanningScene>("planning_scene",1);
  //collision objects
  moveit_msgs::AttachedCollisionObject attached_object, attached_object2;
  attached_object.link_name = "base"; attached_object2.link_name = "base";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "base"; attached_object2.object.header.frame_id = "base";
  /* The id of the object */
  attached_object.object.id = "box"; attached_object2.object.id = "second_box";
  /* A default pose */
  geometry_msgs::Pose pose; geometry_msgs::Pose pose_2;
  pose.orientation.w = 1.0;   pose.position.x =  0.6;    pose.position.y =  0.5;    pose.position.z =  0.5;
  pose_2.orientation.w = 1.0;   pose_2.position.x =  0.5;    pose_2.position.y =  -0.2;    pose_2.position.z =  0.2;
  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;    primitive.dimensions[1] = 0.4;    primitive.dimensions[2] = 0.1;
  attached_object.object.primitives.push_back(primitive); //attached_object2.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose); //attached_object2.object.primitive_poses.push_back(pose_2);
  attached_object.object.operation = attached_object.object.ADD; //attached_object2.object.operation = attached_object2.object.ADD;
  ROS_INFO("Adding two objects into the world at the locations specified by poses.");
  moveit_msgs::PlanningScene planning_scene_2;
  planning_scene_2.is_diff = true;
  planning_scene_2.world.collision_objects.push_back(attached_object.object);
  planning_scene_publish.publish(planning_scene_2);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene my_plan_scene(robot_model);
  my_plan_scene.setPlanningSceneDiffMsg(planning_scene_2);

  //moveit::planning_interface::MoveGroup::Plan my_plan;
  //create a move group that will be used to plan the motion later, in this case it is the right arm of the baxter robot
  //moveit::planning_interface::MoveGroup group("left_arm");
  moveit_msgs::PlanningSceneWorld my_world;
  my_world.octomap.octomap;
  collision_detection::CollisionWorldConstPtr my_collision_world;
  //EigenSTL::vector_Affine3d my_object_pose = my_collision_world->getWorld()->getObject(my_collision_world->getWorld()->getObjectIds()[0])->shape_poses_;
  //std::cout << "-------- look at object pose: -----------" << my_object_pose.data()->matrix() << std::endl;

  // 1. Define sequence of points
  /*TrajectoryVec points;
  for (unsigned int i = 0; i < 2; ++i)
  {
    //double x = 0.58 - i * 0.058, y = 0.183 + i * 0.0617, z = 0.2, rx = 0.0, ry = M_PI, rz = 0.0;
      double x = 0.0 + i * 0.8, y = 0.8 - i * 0.8, z = 0.2, rx = 0.0, ry = M_PI, rz = 0.0;
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(x,y,z,rx,ry,rz);
    points.push_back(pt);
  }

  for (unsigned int i = 0; i < 11; ++i)
  {
    double x = 0.8, y = 0.80 - i * 0.08, z = 0.2, rx = 0.0, ry = M_PI/2, rz = 0.0;
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(x,y,z,rx,ry,rz);
    points.push_back(pt);
  }

  /*
  for (unsigned int i = 0; i < 5; ++i)
  {
    Eigen::Affine3d pose;
    pose = Eigen::Translation3d(0.0, 0.9 + 0.04 * i, .5);
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points.push_back(pt);
  }
  */
  // 1. Create a robot model and initialize it
  descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);

  //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  //robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();


  // Name of description on parameter server. Typically just "robot_description".
  const std::string robot_description = "robot_description";

  // name of the kinematic group you defined when running MoveitSetupAssistant
  const std::string group_name = "left_arm";

  // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
  const std::string world_frame = "/base";

  // tool center point frame (name of link associated with tool)
  const std::string tcp_frame = "left_gripper";


  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }

  // 2. Define sequence of points
  //first get current position of end effector
  Eigen::Affine3d my_pose;
  Eigen::Vector3d current_position;
  const std::vector<double> left_arm_joint_values = {joints_values[5], joints_values[6], joints_values[3], joints_values[4], joints_values[7], joints_values[8], joints_values[9]};
  bool success = model->getFK(left_arm_joint_values,my_pose);
  if (success)
      current_position = my_pose.translation();
  //std::cout << "robot pose is: \n" << my_pose.translation() << std::endl;
  model->setCheckCollisions(true);

  TrajectoryVec points;
  for (unsigned int i = 0; i < 2; ++i)
  {
      descartes_core::TrajectoryPtPtr pt;
    //double x = 0.58 - i * 0.058, y = 0.183 + i * 0.0617, z = 0.2, rx = 0.0, ry = M_PI, rz = 0.0;
      //double x = current_position(0), y = current_position(1), z = current_position(2), rx = 0.0, ry = M_PI, rz = 0.0;
      if (i > 0){
          double x = atof(argv[1]), y = atof(argv[2]), z = atof(argv[3]),
          rx = atof(argv[4]), ry = atof(argv[5]), rz = atof(argv[6]);
          pt = makeTolerancedCartesianPoint(x,y,z,rx,ry,rz);
      }
      else pt = makeTolerancedCartesianPoint2(my_pose);

    points.push_back(pt);
  }
  //model->check_collisions_
  // 3. Create a planner and initialize it with our robot model
  descartes_planner::DensePlanner planner;
  planner.initialize(model);

  // 4. Feed the trajectory to the planner
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -2;
  }

  TrajectoryVec result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -3;
  }

  // 5. Translate the result into a type that ROS understands
  // Get Joint Names
  std::vector<std::string> names;
  names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
  //nh.getParam("controller_joint_names", names);
  // Generate a ROS joint trajectory with the result path, robot model, given joint names,
  // a certain time delta between each trajectory point
  trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 1.0);

  //usleep(11e6);
  //execute the trajectory by publishing joints commands
  /*for(int i = 0; i < joint_solution.points.size(); i++){
      trajectory_msgs::JointTrajectoryPoint my_point = joint_solution.points[i];
      baxter_core_msgs::JointCommand command_msg;
      command_msg.mode = command_msg.POSITION_MODE;
      command_msg.names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
      command_msg.command = {0, 0, 0, 0, 0, 0, 0};
      pub_msg.publish(command_msg);
      usleep(2e6);
      //std::cout << "********* points for left arm joints are: " << joint_solution.points[i] << "*************************" << std::endl;
      for(int j = 0; j < joint_solution.joint_names.size(); j++)
          std::cout << "********* names for left arm joints are: " << joint_solution.joint_names[j] << "*************************" << std::endl;
      //std::cout << "********* joint no: " << i << " in left arm first point should be: " << joint_solution.points[0][i] << "**********************" << std::endl;
  }

  ROS_ERROR("************** I am here try to execute ***********************");
  moveit_msgs::RobotState start_robot_state;
  robot_state::RobotStatePtr current_state = group.getCurrentState();
  usleep(1e-6);
  current_state->setVariablePositions(variable_names,joints_values);
  std::cout << "************* left arm ******************" << std::endl;

  std::cout << "first joint value is: " << *current_state->getJointPositions("left_s0") << std::endl;
  std::cout << "second joint value is: " << *current_state->getJointPositions("left_s1") << std::endl;
  std::cout << "third joint value is: " << *current_state->getJointPositions("left_e0") << std::endl;
  std::cout << "fourth joint value is: " << *current_state->getJointPositions("left_e1") << std::endl;
  std::cout << "fifth joint value is: " << *current_state->getJointPositions("left_w0") << std::endl;
  std::cout << "sixth joint value is: " << *current_state->getJointPositions("left_w1") << std::endl;
  std::cout << "seventh joint value is: " << *current_state->getJointPositions("left_w2") << std::endl;

  std::cout << "************* right arm ******************" << std::endl;

  std::cout << "first joint value is: " << *current_state->getJointPositions("right_s0") << std::endl;
  std::cout << "second joint value is: " << *current_state->getJointPositions("right_s1") << std::endl;
  std::cout << "third joint value is: " << *current_state->getJointPositions("right_e0") << std::endl;
  std::cout << "fourth joint value is: " << *current_state->getJointPositions("right_e1") << std::endl;
  std::cout << "fifth joint value is: " << *current_state->getJointPositions("right_w0") << std::endl;
  std::cout << "sixth joint value is: " << *current_state->getJointPositions("right_w1") << std::endl;
  std::cout << "seventh joint value is: " << *current_state->getJointPositions("right_w2") << std::endl;
  robot_state::robotStateToRobotStateMsg(*current_state,start_robot_state);
  my_plan.start_state_ = start_robot_state;
  my_plan.trajectory_.joint_trajectory = joint_solution;
  group.execute(my_plan);
  // 6. Send the ROS trajectory to the robot for execution
  for (int i = 0; i < variable_names.size(); i++){
      std::cout << "joint no: " << i << " name is: " << variable_names[i] << std::endl;
      std::cout << "joint no: " << i << " value is: " << joints_values[i] << std::endl;
  }*/
  //executeTrajectory(joint_solution);
  //joint_solution.header.stamp = ros::Time::now();
  executeTrajectory(joint_solution);
  if (!executeTrajectory(joint_solution))
  {
    ROS_ERROR("Could not execute trajectory!");
    return -4;
  }

  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
}

//descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(double x, double y, double z, double rx, double ry, double rz)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  //return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
    return TrajectoryPtPtr( new AxialSymmetricPt(x,y,z,rx,ry,rz, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint2(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
}

trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     double time_delay)
{
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = "world_frame";
  result.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);

    /*for (int i = 0; i < joints.size(); i++){
        std::cout << "joint: " << i << " value is " << joints[i] << std::endl;
    }
    std::cout << "******************************************" << std::endl;*/
    //std::cout << "joints size is: " << joints.size() << std::endl;
    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    /*pt.positions.push_back(joints_values[14]); pt.positions.push_back(joints_values[15]);
    pt.positions.push_back(joints_values[12]); pt.positions.push_back(joints_values[13]);
    pt.positions.push_back(joints_values[16]); pt.positions.push_back(joints_values[17]);
    pt.positions.push_back(joints_values[18]);*/

    //for real/simulated baxter
    for(int i = 0; i < joints.size(); i++)
        pt.positions.push_back(joints[i]);

    //for standalone execution
    //for(int i = 0; i < 10; i++)
      //  pt.positions.push_back(0);
    //for(int i = 0; i < joints.size(); i++)
      //  pt.positions.push_back(joints[i]);

    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    //for real/simulated baxter
    pt.velocities.resize(7, 0.0);
    pt.accelerations.resize(7, 0.0);
    pt.effort.resize(7, 0.0);

    //for standalone execution
    //pt.velocities.resize(14, 0.0);
    //pt.accelerations.resize(14, 0.0);
    //pt.effort.resize(14, 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    result.points.push_back(pt);
    //executeTrajectory(result);
    //executeTrajectory(result);
  }

  return result;
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
    //for real/simulated baxter
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/left/follow_joint_trajectory", true);
    //for standalone execution
  //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  //goal.trajectory.joint_names = trajectory.joint_names;
  goal.trajectory = trajectory;
  //goal.path_tolerance = 5;
  goal.goal_time_tolerance = ros::Duration(1.0);
  ac.sendGoal(goal);

  /*for (int i = 0; i < trajectory.points.size(); i++){
      //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/left/follow_joint_trajectory", true);
      //ac.waitForServer(ros::Duration(2.0));
      ac.cancelAllGoals();
      goal.trajectory.header.stamp = trajectory.header.stamp;
      goal.trajectory.header.frame_id = trajectory.header.frame_id;
      goal.trajectory.points.push_back(trajectory.points[i]);
      ac.sendGoal(goal);
      ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5));
  }*/
  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  } else {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}
