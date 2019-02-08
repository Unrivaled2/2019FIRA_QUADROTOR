#include "rpg_quadrotor_integration_test/rpg_quadrotor_integration_test.h"

#include <gtest/gtest.h>
#include <vector>

#include <iostream>
#include <fstream>
#include <autopilot/autopilot_states.h>
#include <Eigen/Dense>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <trajectory_generation_helper/circle_trajectory_helper.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>

namespace rpg_quadrotor_integration_test
{

QuadrotorIntegrationTest::QuadrotorIntegrationTest() :
    executing_trajectory_(false),first_record(true),sum_position_error_squared_(
        0.0), max_position_error_(0.0), sum_thrust_direction_error_squared_(
        0.0), max_thrust_direction_error_(0.0),sum_angular_velocity_error_squared_(0.0),
    max_angular_velocity_error_(0.0),sum_linear_velocity_error_squared_(0.0),
    max_linear_velocity_error_(0.0),max_linear_velocity_(0.0),max_angluar_velocity_(0.0),
    max_x_linear_velocity_(0.0),max_y_linear_velocity_(0.0),last_record(0.0),curr_record(0.0)
    ,total_record(0.0),curr_pos_error(0.0),max_pos_error(0.0),sum(0.0)
{
  ros::NodeHandle nh;

  arm_pub_ = nh.advertise<std_msgs::Bool>("bridge/arm", 1);

  //kExecLoopRate 代表每秒经过的点的个数 这里kExecLoopRate=50 每0.02秒经过一个点
  //计时器可以每间隔0.02秒调用一次回调函数，目前存在的问题是只调用了一次，
  //且最后一个参数含义查询应是oneshot=False 
  measure_tracking_timer_ = nh_.createTimer(
      ros::Duration(1.0 / kExecLoopRate_),
      &QuadrotorIntegrationTest::measureTracking,this);
  
  //add a subscriber to call the callback function measureTraking//
  //这里重新写一个接收器和回调函数进行回调函数调用,证明没什么用
  ros::Subscriber sub = nh.subscribe("autopilot/feedback",15,&QuadrotorIntegrationTest::autopilot_feedback,this);
  //cug add
}
QuadrotorIntegrationTest::~QuadrotorIntegrationTest()
{
}

void QuadrotorIntegrationTest::autopilot_feedback(const quadrotor_msgs::AutopilotFeedback::ConstPtr& msg)
{
  const double time = autopilot_helper_.getCurrentTrajectoryExecutionLeftDuration().toSec();
 if (executing_trajectory_ && time != 0.0)
 {    
    std::fstream f("/home/cug/circle_data/points_338/summary.txt",std::ios::out);
    std::fstream f_p("/home/cug/circle_data/points_338/position_data.txt",std::ios::app);
    std::fstream f_a_e("/home/cug/circle_data/points_338/angular_velocity_data.txt",std::ios::app);
    std::fstream f_l_e("/home/cug/circle_data/points_338/linear_velocity_data.txt",std::ios::app);
   	 // Position error
    const double position_error =
        autopilot_helper_.getCurrentPositionError().norm();
    sum_position_error_squared_ += pow(position_error, 2.0);
    if (position_error > max_position_error_)
    {
      max_position_error_ = position_error;
    }

    /*
     get reference and estimate position 
    */
     
     Eigen::Vector3d estimate_position = autopilot_helper_.getCurrentPositionEstimate();
     Eigen::Vector3d refer_position = autopilot_helper_.getCurrentReferencePosition();
     f_p<<"position estimate:  "<<estimate_position(0)<<"  "<<estimate_position(1)<<"  "
                               <<estimate_position(2)<<std::endl;
     f_p<<"position reference: "<<refer_position(0)<<"  "<<refer_position(1)<<"  "
				<<refer_position(2)<<std::endl<<std::endl;
     f_p.close();
   
    /*compute the angular_velocity_error 
      add it to the sum 
      update the max error*/
   
    quadrotor_msgs::AutopilotFeedback autopilot_feedback = *msg;
    Eigen::Vector3d refer_angular = quadrotor_common::geometryToEigen		(autopilot_feedback.reference_state.velocity.angular);
    Eigen::Vector3d estimate_angular = quadrotor_common::geometryToEigen 
(autopilot_feedback.state_estimate.twist.twist.angular);
    const double angular_velocity_error = (estimate_angular - refer_angular).norm();
    sum_angular_velocity_error_squared_ += pow(angular_velocity_error,2.0);
    if( max_angular_velocity_error_ < angular_velocity_error)
    {
	max_angular_velocity_error_ = angular_velocity_error;
    };
    f_a_e<<"angular velocity estimate:  "<<estimate_angular(0)<<"  "<<estimate_angular(1)<<"  "
       <<estimate_angular(2)<<std::endl;
    f_a_e<<"angular velocity reference: "<<refer_angular(0)<<"  "<<refer_angular(1)<<"  "
       <<refer_angular(2)<<std::endl<<std::endl;
    f_a_e.close();
  
    /*compute the linear_velocity_error
      add it to the sum
      update the max error
    */
    Eigen::Vector3d estimate_velocity = autopilot_helper_.getCurrentVelocityEstimate();
    Eigen::Vector3d reference_velocity = autopilot_helper_.getCurrentReferenceVelocity();
    const double linear_velocity_error = autopilot_helper_.getCurrentVelocityError().norm();
    sum_linear_velocity_error_squared_ += pow(linear_velocity_error,2.0);
    if(linear_velocity_error > max_linear_velocity_error_)
    {
    	max_linear_velocity_error_ = linear_velocity_error;
    };

    f_l_e<<"linear velocity estimate:  "<<estimate_velocity(0)<<"  "<<estimate_velocity(1)<<"  "
       <<estimate_velocity(2)<<std::endl;
    f_l_e<<"linear velocity reference: "<<reference_velocity(0)<<"  "<<reference_velocity(1)<<"  "
       <<reference_velocity(2)<<std::endl<<std::endl;
    f_l_e.close();


    f<<"sum_position_error_squared_: "<<sum_position_error_squared_<<std::endl
     <<"max_position_error_: "<<max_position_error_<<std::endl;
    f<<"sum_angular_velocity_error_squared_: "<< sum_angular_velocity_error_squared_<<std::endl;
    f<<"max_angular_velocity_error_: "<<max_angular_velocity_error_<<std::endl;
    f<<"sum_linear_velocity_error_squared_: "<<sum_linear_velocity_error_squared_<<std::endl;
    f<<"max_linear_velocity_error_: "<<max_linear_velocity_error_<<std::endl;
    double  max_linear_velocity = autopilot_helper_.getCurrentVelocityEstimate().norm();
    if(max_linear_velocity > max_linear_velocity_)
    {
	max_linear_velocity_ = max_linear_velocity;
    }
    double max_x_linear_velocity = autopilot_helper_.getCurrentVelocityEstimate()(0);
    if(max_x_linear_velocity > max_x_linear_velocity_)
    {
	max_x_linear_velocity_ = max_x_linear_velocity;
    };
    double max_y_linear_velocity = autopilot_helper_.getCurrentVelocityEstimate()(1);
    if(max_y_linear_velocity > max_y_linear_velocity_)
    {
	max_y_linear_velocity_ = max_y_linear_velocity;
    };
 
    double max_angluar_velocity = estimate_angular.norm();
    if(max_angluar_velocity > max_angluar_velocity_)
    {
	max_angluar_velocity_ = max_angluar_velocity;
    };
   
    /*************test for get duration************************/
    std::fstream f_o("/home/cug/duration.txt",std::ios::app); 
    if(first_record)
    {
	total_record = 0.0;
	last_record = autopilot_helper_.getCurrentTrajectoryExecutionLeftDuration().toSec();
        f_o<<total_record<<std::endl;
	first_record = false;
    }else
    {
	curr_record = autopilot_helper_.getCurrentTrajectoryExecutionLeftDuration().toSec();
	total_record += last_record - curr_record;
	last_record = curr_record;
	f_o<<total_record<<std::endl;
    };        
    f_o.close();
    /********************test for duration********************/
    /*******************test for pose*************************/   
    std::fstream f_pose("/home/cug/pose.txt",std::ios::app); 
    Eigen::Quaterniond quaterniond = autopilot_helper_.getCurrentOrientationEstimate();
    double w = quaterniond.w();
    double x = quaterniond.x();
    double y = quaterniond.y();
    double z = quaterniond.z();
    double roll = atan((2*(w*x + y*z))/(1-2*(pow(x,2)+pow(y,2))))*180/M_PI;
    double pitch = asin(2*(w*y-z*x))*180/M_PI;
    double yaw = atan((2*(w*z + x*y))/(1-2*(pow(y,2)+pow(z,2))))*180/M_PI;
    f_pose<<yaw<<std::endl<<roll<<std::endl<<pitch<<std::endl;
    f_pose.close();
    std::fstream f_pos("/home/cug/position.txt",std::ios::app); 
    f_pos<<estimate_position(0)<<std::endl<<estimate_position(1)<<std::endl;
    f_pos.close();
   /*********************test for pose************************/
    f<<"max_linear_velocity_: "<<max_linear_velocity_<<std::endl;
    f<<"max_x_linear_velocity_: "<<max_x_linear_velocity_<<"  "<<"论文数据: "<<4.00<<std::endl;
    f<<"max_y_linear_velocity_: "<<max_y_linear_velocity_<<"  "<<"论文数据: "<<4.00<<std::endl;
    f<<"max_angluar_velocity_: "<<max_angluar_velocity_<<"  "<<"论文数据: "<<1.48<<"(存疑)"<<std::endl;
    f.close();
    
    /*********************之前由于getReference...()的问题，得到的误差是错误的***************************/
    double curr_quad_sum = sqrt(pow(estimate_position(0),2) + pow(estimate_position(1),2));
    double error = fabs(curr_quad_sum - 1.8);
    curr_pos_error += error;
    if(error > max_pos_error)
    {
	max_pos_error = error;
    };
    sum = sum + 1.0;
    const double average_error = curr_pos_error/sum;
    std::fstream f_error("/home/cug/postionError_points.txt",std::ios::app);
    if((curr_quad_sum - 3.24)<0)
    {
	f_error<<error<<"内"<<std::endl;
    }else
    {
    f_error<<error<<"外"<<std::endl;
    };
    f_error.close();
    std::fstream f_sum("/home/cug/summary.txt",std::ios::out);
    f_sum<<"sum_pos_error: "<<curr_pos_error<<std::endl;
    f_sum<<"max_pos_error: "<<max_pos_error<<std::endl;
    f_sum<<"average_pos_error: "<<average_error<<std::endl;
    f_pos.close();
    f_sum.close();
    /*******************************得到正确的max_pos_error sum_pos_error*****************************/
    /************汇总时间，位置，姿态***************************/
    std::fstream os("/home/cug/points_sum.txt",std::ios::app);
    os<<total_record<<","<<estimate_position(0)<<","<<estimate_position(1)<<","<<estimate_position(2)<<","<<yaw<<","<<roll<<","<<pitch
    <<","<<error<<std::endl;
    os.close();
  }
};


void QuadrotorIntegrationTest::measureTracking(const ros::TimerEvent& time)
{
  if (executing_trajectory_)
  {
    // Position error
    const double position_error =
        autopilot_helper_.getCurrentPositionError().norm();
    sum_position_error_squared_ += pow(position_error, 2.0);
    if (position_error > max_position_error_)
    {
      max_position_error_ = position_error;
    }

    // Thrust direction error
    const Eigen::Vector3d ref_thrust_direction =
        autopilot_helper_.getCurrentReferenceOrientation()
            * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d thrust_direction =
        autopilot_helper_.getCurrentOrientationEstimate()
            * Eigen::Vector3d::UnitZ();

    const double thrust_direction_error = acos(
        ref_thrust_direction.dot(thrust_direction));
    sum_thrust_direction_error_squared_ += pow(thrust_direction_error, 2.0);
    if (thrust_direction_error > max_thrust_direction_error_)
    {
      max_thrust_direction_error_ = thrust_direction_error;
    }
  }
}



//#############################为测试不同的点数对于生成圆轨迹的影响，直接改代码编译有问题，所以直接把
//#############################生成圆的代码移了过来#########################################
quadrotor_common::Trajectory QuadrotorIntegrationTest::computeHorizontalCircleTrajectory(
    const Eigen::Vector3d center, const double radius, const double speed,
    const double phi_start, const double phi_end,
    const double sampling_frequency,int points_num)
{
  /*
   * We use a coordinate system with x to the front, y to the left, and z up.
   * When setting phi_start = 0.0 the start point will be at
   * (radius, 0, 0) + center
   * If phi_end > phi_start the trajectory is going counter clock wise
   * otherwise it is going clockwise
   */
  quadrotor_common::Trajectory trajectory;
  //设定轨迹类型，剩余四种
  trajectory.trajectory_type =
      quadrotor_common::Trajectory::TrajectoryType::GENERAL;
  //phi_total就是转过的角度
  const double phi_total = phi_end - phi_start;
  //通过比较phi_start和phi_end的大小决定了顺时针还是逆时针
  const double direction = phi_total / fabs(phi_total);
  //计算角速度，公式是w = v/r
  const double omega = direction * fabs(speed / radius);
  //sampling_frequency = 50代表每秒经过的点的个数
  const double angle_step = fabs(2*M_PI/points_num);

  for (double d_phi = 0.0; d_phi <= fabs(phi_total); d_phi += angle_step)
  {
    const double phi = phi_start + direction * d_phi;
    const double cos_phi = cos(phi);
    const double sin_phi = sin(phi);
    //点的成员有pos,v,a,jerk,snap,time_from_start
    quadrotor_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(fabs(d_phi/omega));
    //因为是二维的平面，POS=(radius * [sin(omega * t),cos(omega * t) ,0] + center
    //不断对该公式求导计算v,a,jerk,snap
    point.position = radius * Eigen::Vector3d(cos_phi, sin_phi, 0.0) + center;
    point.velocity = radius * omega * Eigen::Vector3d(-sin_phi, cos_phi, 0.0);
    point.acceleration = radius * pow(omega, 2.0)
        * Eigen::Vector3d(-cos_phi, -sin_phi, 0.0);
    point.jerk = radius * pow(omega, 3.0)
        * Eigen::Vector3d(sin_phi, -cos_phi, 0.0);
    point.snap = radius * pow(omega, 4.0)
        * Eigen::Vector3d(cos_phi, sin_phi, 0.0);
    trajectory.points.push_back(point);
  }

  // Add last point at phi_end
  /*const double phi = phi_start + phi_total;
  const double cos_phi = cos(phi);
  const double sin_phi = sin(phi);
  quadrotor_common::TrajectoryPoint point;
  point.time_from_start = ros::Duration(fabs(phi_total / omega));
  point.position = radius * Eigen::Vector3d(cos_phi, sin_phi, 0.0) + center;
  point.velocity = radius * omega * Eigen::Vector3d(-sin_phi, cos_phi, 0.0);
  point.acceleration = radius * pow(omega, 2.0)
      * Eigen::Vector3d(-cos_phi, -sin_phi, 0.0);
  point.jerk = radius * pow(omega, 3.0)
      * Eigen::Vector3d(sin_phi, -cos_phi, 0.0);
  point.snap = radius * pow(omega, 4.0)
      * Eigen::Vector3d(cos_phi, sin_phi, 0.0);

  trajectory.points.push_back(point);*/

  return trajectory;
}


/************************************************************************************************/

void QuadrotorIntegrationTest::run()
{
  ros::Rate command_rate(kExecLoopRate_);

  // Make sure everything is up and running
  // Wait for Autopilot feedback with assert
  ASSERT_TRUE(autopilot_helper_.waitForAutopilotFeedback(10.0, kExecLoopRate_))
      << "Did not receive autopilot feedback within 10 seconds.";

  ros::Duration(3.0).sleep();

  // Arm bridge
  std_msgs::Bool arm_msg;
  arm_msg.data = true;
  arm_pub_.publish(arm_msg);

  ///////////////
  // Check off command
  ///////////////

  // Takeoff
  autopilot_helper_.sendStart();

  // Wait for autopilot to go to start
  EXPECT_TRUE(autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::START, 0.5,
          kExecLoopRate_))
      << "Autopilot did not switch to start after sending start command.";

  // Abort start and send off
  autopilot_helper_.sendOff();

  // Wait for autopilot to go to off
  EXPECT_TRUE(
      autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::OFF, 0.1,
          kExecLoopRate_))
      << "Autopilot could not be forced to off during take off.";

  ///////////////
  // Check take off
  ///////////////

  // Takeoff for real
  autopilot_helper_.sendStart();

  // Wait for autopilot to go to hover 因为速度检查的移动删除了，这一部分的悬停也取消
  //EXPECT_TRUE(
  //    autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 10.0,
  //        kExecLoopRate_))
  //    << "Autopilot did not switch to hover after take off.";
  //
  //executing_trajectory_ = true; // Start measuring errors
 
  ///////////////
  // Check velocity control
  ///////////////

  // Send velocity commands 这一部分检查速度控制，通过定点移动至(1.0, -0.8, 0.3)
  /*const Eigen::Vector3d vel_cmd = Eigen::Vector3d(1.0, -0.8, 0.3);
  const double heading_rate_cmd = -1.0;

  ros::Time start_sending_vel_cmds = ros::Time::now();
  while (ros::ok())
  {
    autopilot_helper_.sendVelocityCommand(vel_cmd, heading_rate_cmd);
    if ((ros::Time::now() - start_sending_vel_cmds) > ros::Duration(2.0))
    {
      EXPECT_TRUE(
          (autopilot_helper_.getCurrentAutopilotState()
              == autopilot::States::VELOCITY_CONTROL))
          << "Autopilot did not switch to velocity control correctly.";
      break;
    }
    ros::spinOnce();
    command_rate.sleep();
  }*/

  // Wait for autopilot to go back to hover 将速度检查和定点移动之间的悬停取消
  EXPECT_TRUE(
      autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 10.0,
          kExecLoopRate_))
      << "Autopilot did not switch back to hover correctly.";

  ///////////////
  // Check go to pose
  ///////////////

  // Send pose command  直接移动到轨迹开始的地方
  const Eigen::Vector3d position_cmd = Eigen::Vector3d(0,0,1.0);
  const double heading_cmd = 0.0;

  autopilot_helper_.sendPoseCommand(position_cmd, heading_cmd);

  // Wait for autopilot to go to got to pose state
  EXPECT_TRUE(
      autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::TRAJECTORY_CONTROL, 2.0,
          kExecLoopRate_))
      << "Autopilot did not switch to trajectory control because of go to pose action correctly.";

  // Wait for autopilot to go back to hover
  EXPECT_TRUE(
      autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 10.0,
          kExecLoopRate_))
      << "Autopilot did not switch back to hover correctly.";

  // Check if we are at the requested pose
  EXPECT_TRUE(
      (autopilot_helper_.getCurrentReferenceState().position -
          position_cmd).norm() < 0.01)
      << "Go to pose action did not end up at the right position.";

  EXPECT_TRUE(autopilot_helper_.getCurrentReferenceHeading() -
      heading_cmd < 0.01)
      << "Go to pose action did not end up at the right heading.";
  //cug增加一个直接到达双扭曲线起始点的位置
  const Eigen::Vector3d position_start = Eigen::Vector3d(1.8,0.0,1.0);
  autopilot_helper_.sendPoseCommand(position_start, heading_cmd);
 EXPECT_TRUE(
      autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::TRAJECTORY_CONTROL, 2.0,
          kExecLoopRate_))
      << "Autopilot did not switch to trajectory control because of go to pose action correctly.";

  // Wait for autopilot to go back to hover
  EXPECT_TRUE(
      autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 10.0,
          kExecLoopRate_))
      << "Autopilot did not switch back to hover correctly.";

  // Check if we are at the requested pose
  EXPECT_TRUE(
      (autopilot_helper_.getCurrentReferenceState().position -
          position_cmd).norm() < 0.01)
      << "Go to pose action did not end up at the right position.";

  EXPECT_TRUE(autopilot_helper_.getCurrentReferenceHeading() -
      heading_cmd < 0.01)
      << "Go to pose action did not end up at the right heading."; 
  //cug增加一个直接到达双扭曲线起始点的位置


  ///////////////
  // Check sending reference states
  ///////////////

  // Generate trajectory, sample it and send it as reference states
  const double max_vel = 2.0;
  const double max_thrust = 15.0;
  const double max_roll_pitch_rate = 0.5;

  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = position_cmd;
  start_state.heading = 0.0;
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = Eigen::Vector3d(0.0, 0.0, 1.0);//改成差1米观察
  end_state.heading = M_PI;
  /* 这里将参考轨迹测试注释
  quadrotor_common::Trajectory manual_traj =
      trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start_state, end_state, 5, max_vel, max_thrust, max_roll_pitch_rate,
          kExecLoopRate_);

  trajectory_generation_helper::heading::addConstantHeadingRate(
      start_state.heading, end_state.heading, &manual_traj);

  bool autopilot_was_in_reference_control_mode = false;
  while (ros::ok() && !manual_traj.points.empty())
  {
    autopilot_helper_.sendReferenceState(manual_traj.points.front());
    manual_traj.points.pop_front();
    ros::spinOnce();
    if (!autopilot_was_in_reference_control_mode
        && autopilot_helper_.getCurrentAutopilotState()
            == autopilot::States::REFERENCE_CONTROL)
    {
      autopilot_was_in_reference_control_mode = true;
    }
    command_rate.sleep();
  }

  EXPECT_TRUE(
      autopilot_was_in_reference_control_mode)
      << "Autopilot did not switch to reference control correctly.";

  // Wait for autopilot to go back to hover
  */
  EXPECT_TRUE(
      autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 2.0,
          kExecLoopRate_))
      << "Autopilot did not switch back to hover correctly.";

  ///////////////
  // Check trajectory control
  ///////////////

  // Generate trajectories and send them as complete trajectories
  // One polynomial to enter a ring and a ring to check execution of
  // consecutive trajectories

  // Ring trajectory with enter segment
  std::vector<Eigen::Vector3d> way_points;//把所有的高度变成一米,飞行顺序是从第一个点到最后一个点
  way_points.push_back(Eigen::Vector3d(-1.5, 0.0, 1.0));
  way_points.push_back(Eigen::Vector3d(0.0,-1.5,1.0));
  way_points.push_back(Eigen::Vector3d(1.5,0.0,1.0));
  way_points.push_back(Eigen::Vector3d(0.0,1.5,1.0));
  //way_points.push_back(Eigen::Vector3d(1.5, 1.5, 1.0));

  //ring_segment_times代表将轨迹划分之后，对每一小段分配的时间
  Eigen::VectorXd initial_ring_segment_times = Eigen::VectorXd::Ones(
      int(way_points.size()));
  polynomial_trajectories::PolynomialTrajectorySettings ring_trajectory_settings;
  //ring_trajectory_settings包含了轨迹点，continuity_order指的是要求的连续性阶数，即几阶连续
  //polynomial_order指的是轨迹的阶数
  //minimization_weights的初始化指的是进行前面系数的初始化
  ring_trajectory_settings.continuity_order = 4;
  Eigen::VectorXd minimization_weights(5);
  minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
  ring_trajectory_settings.minimization_weights = minimization_weights;
  ring_trajectory_settings.polynomial_order = 11;
  ring_trajectory_settings.way_points = way_points;
  quadrotor_common::Trajectory ring_traj =
  trajectory_generation_helper::polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement( 
//max_vel,max_thrust,max_roll_pitch_rate定义为常量，暂且不管
//kExecLoopRate_ = 50.0;代表每秒经过的路径点的个数
          initial_ring_segment_times, ring_trajectory_settings, max_vel,
          max_thrust, max_roll_pitch_rate, kExecLoopRate_);

  polynomial_trajectories::PolynomialTrajectorySettings enter_trajectory_settings =
      ring_trajectory_settings;
  enter_trajectory_settings.way_points.clear();

//cug  对圆形轨迹进行测试××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
  Eigen::Vector3d center_circle = Eigen::Vector3d(0.0,0.0,1.0);
   quadrotor_common::Trajectory circle_test = QuadrotorIntegrationTest::computeHorizontalCircleTrajectory(  
 center_circle,1.8,1.4,0.0,6*M_PI,kExecLoopRate_,13);
/***********************************************************************************************/

  end_state.position = Eigen::Vector3d(0.0, 0.0, 1.0);//改成差1米观察
  end_state.heading = M_PI;
  quadrotor_common::TrajectoryPoint end_center;
  end_center.position = Eigen::Vector3d(2.1213,0.0,1.0);
  end_center.heading = M_PI;
  //直接将lemniscate的生成代码放进来××××××××××××××××××××××××
//#####################################################
//### x = 2*cos(sqrt(2)*t)                           ##
//### Y = 2*sin(sqrt(2)*t)cos(sqrt(2)*t)             ##
//#####################################################
float end = 19.8;
float times = 1502;
float start = 0.0;
float time_step = (end - start)/times;
int i = 0;
Eigen::Vector3d center = Eigen::Vector3d(0.0,0.0,1.0); 

quadrotor_common::Trajectory trajectory;
trajectory.trajectory_type = 
	quadrotor_common::Trajectory::TrajectoryType::GENERAL;
executing_trajectory_ = true;
//点数338
while(i < (times-4))
{
	if(i==338)
		break;
	float time = start +1*i*time_step;
	float pos_x = 2*cos(sqrt(2)*time);
	float pos_y = 2*sin(sqrt(2)*time)*cos(sqrt(2)*time);
	float x = time;
	float x_v = -sqrt(8)*sin(sqrt(2)*x);
   	float x_a = -4*cos(sqrt(2)*x);
        float x_j =  sqrt(32)*sin(sqrt(2)*x);
        float x_s = 8*cos(sqrt(2)*x);
        float y_v = sqrt(8)*pow(cos(sqrt(2)*x),2)-sqrt(8)*pow(sin(sqrt(2)*x),2);
        float y_a = -16*cos(sqrt(2)*x)*sin(sqrt(2)*x);
        float y_j = sqrt(pow(2,9))*pow(sin(sqrt(2)*x),2)-sqrt(pow(2,9))*pow(cos(sqrt(2)*x),2);
        float y_s = 128*cos(sqrt(2)*x)*sin(sqrt(2)*x);
	quadrotor_common::TrajectoryPoint point;
	point.time_from_start = ros::Duration(time);
	point.position = Eigen::Vector3d(pos_x,pos_y,0.0)+center;
	point.velocity = Eigen::Vector3d(x_v,y_v,0.0);
	point.acceleration = Eigen::Vector3d(x_a,y_a,0.0);
	point.jerk = Eigen::Vector3d(x_j,y_j,0.0);
	point.snap = Eigen::Vector3d(x_s,y_s,0.0);
	trajectory.points.push_back(point);
	i++;
};


 
  //直接将lemniscate的生成代码放进来××××××××××××××××××××××××

//cug××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××  
//对圆形轨迹进行测试,生成轨迹后，使用enter_traj先移动到起点，然后sendTrajectory

//change the third parameter orign:ring_traj.points.front()
  quadrotor_common::Trajectory enter_traj =
      trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
          Eigen::VectorXd::Ones(1), end_state, end_center,
          enter_trajectory_settings, 1.03 * max_vel, 1.03 * max_thrust,
          max_roll_pitch_rate, kExecLoopRate_);

  trajectory_generation_helper::heading::addConstantHeadingRate(
      end_state.heading, 0.0, &enter_traj);
  //change the fourth parameter orign:&ring_traj
  trajectory_generation_helper::heading::addConstantHeadingRate(0.0, M_PI,
                                                                &trajectory);
  //enter_traj 用于将reference_trajectory的end_state 和 minimum_snap_trajectory的start_state之间衔接起来
  //autopilot_helper_.sendTrajectory(enter_traj);
  //autopilot_helper_.sendTrajectory(ring_traj);
  //双扭曲线中使用定点移动到达起始点,增加一个悬停操作
  autopilot_helper_.sendTrajectory(circle_test);
  // Check if autopilot goes to trajectory control state
  EXPECT_TRUE(
      autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::TRAJECTORY_CONTROL, 2.0,
          kExecLoopRate_))
      << "Autopilot did not switch back to hover correctly.";

  ///////////////
  // Check enforcing hover
  ///////////////

  //还有1.5秒的时候停止执行轨迹，进行强制悬停
  // Before trajectory finishes force autopilot to hover
  /*while (autopilot_helper_.getCurrentTrajectoryExecutionLeftDuration()
      > ros::Duration(0))*/
  /*
  while(autopilot_helper_.getCurrentTrajectoryExecutionLeftDuration()
      > ros::Duration(0.0))
  {
    std::fstream f("/home/cug/cug_4.txt",std::ios::app);
    Eigen::Vector3d velocity = autopilot_helper_.getCurrentReferenceVelocity();
    f<<"the x velocity is"<<velocity(0)<<std::endl;
    f.close();
    ros::spinOnce();
    command_rate.sleep();
  }
  */
  //注意按照轨迹运行的命令发送后会继续执行程序
  if(true)
  {
   ros::NodeHandle nh;
   ros::Subscriber sub = nh.subscribe("autopilot/feedback",15,&QuadrotorIntegrationTest::autopilot_feedback,this);
   ros::spin();
  };
  executing_trajectory_ = false; // Stop measuring errors

  //autopilot_helper_.sendForceHover();取消强制悬停

  // Wait for autopilot to go back to hover
  EXPECT_TRUE(
      autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 1.0,
          kExecLoopRate_))
      << "Autopilot did not switch to hover after being forced to hover.";
  ///////////////
  // Check landing
  ///////////////

  
  autopilot_helper_.sendLand();

  // Wait for autopilot to go to land
  EXPECT_TRUE(
      autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::LAND, 1.0,
          kExecLoopRate_))
      << "Autopilot did not switch to land after sending land command within "
          "timeout.";

  
  // Wait for autopilot to go to off
 EXPECT_TRUE(
      autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::OFF, 10.0,
          kExecLoopRate_))
      << "Autopilot did not switch to off after landing within timeout.";

  ///////////////
  // Check sending control commands
  ///////////////

  //ros::Duration(0.2).sleep();

  // Send control command to spin motors
  quadrotor_common::ControlCommand control_command;
  control_command.armed = true;
  control_command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
  control_command.collective_thrust = 3.0;

  ros::Time start_sending_cont_cmds = ros::Time::now();
  while (ros::ok())
  {
    autopilot_helper_.sendControlCommandInput(control_command);
    if ((ros::Time::now() - start_sending_cont_cmds) > ros::Duration(0.5))
    {
      EXPECT_TRUE(
          (autopilot_helper_.getCurrentAutopilotState()
              == autopilot::States::COMMAND_FEEDTHROUGH))
          << "Autopilot did not switch to command feedthrough correctly.";
      break;
    }
    ros::spinOnce();
    command_rate.sleep();
  }

  autopilot_helper_.sendOff();

  // Check tracking performance
  EXPECT_LT(max_position_error_, 0.15)
      << "Max position error (||est - ref||) from autopilot too large";
  EXPECT_LT(sum_position_error_squared_, 2.0)
      << "Sum of position errors (||est - ref||^2) squared over all received "
          "feedback messages from the autopilot too large";
  EXPECT_LT(max_thrust_direction_error_, 0.25)
      << "Max thrust direction error (acos(des.dot(est))) from autopilot "
          "too large";
  EXPECT_LT(sum_thrust_direction_error_squared_, 15.0)
      << "Sum of thrust direction (acos(des.dot(est))^2) squared over all "
          "received feedback messages from the autopilot too large";
}

TEST(QuadrotorIntegrationTest, AutopilotFunctionality)
{
  QuadrotorIntegrationTest rpg_quadrotor_integration_test;
  rpg_quadrotor_integration_test.run();
}

} // namespace rpg_quadrotor_integration_test

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rpg_quadrotor_integration_test");
  return RUN_ALL_TESTS();
}


