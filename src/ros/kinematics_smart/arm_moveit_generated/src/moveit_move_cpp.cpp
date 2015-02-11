/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


void print_kinematic_state(robot_state::RobotStatePtr kinematic_state, const robot_state::JointModelGroup* joint_model_group) {
    // Get joint names
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    // Get Joint Values
    std::vector<double> joint_values;

    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
}

std::vector<double> get_kinematic_state(robot_state::RobotStatePtr kinematic_state, const robot_state::JointModelGroup* joint_model_group) {
    // Get joint names
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    // Get Joint Values
    std::vector<double> joint_values;

    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    
    return joint_values;
}

robot_state::RobotState get_final_state(double x, double y, double z) {

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("Arm");
    
    print_kinematic_state(kinematic_state, joint_model_group);
    
    /* Check whether any joint is outside its joint limits */
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
    
    /* Enforce the joint limits for this state and check again*/
    kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    Eigen::VectorXd joints(6);
    joints << 0, 1.9722, -1.9722, 0, 0, 0;
  
    kinematic_state->setJointGroupPositions(joint_model_group, joints);
    Eigen::Affine3d end_effector_state;
    end_effector_state = kinematic_state->getGlobalLinkTransform("effector_tip_link");
    
    print_kinematic_state(kinematic_state, joint_model_group);
    
    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
    
    Eigen::Vector3d v;
    Eigen::Vector3d cobra_v;
    v << x, y, z;//0.1285793, 0, 0.469884;
    //This is the eef position for cobra pose
    cobra_v << 0.0285793, 0, 0.369884;
    
    /*end_effector_state.rotate(r);*/
        
    //this is checked to compile    
    Eigen::Affine3d frame_transform;
    frame_transform = kinematic_state->getFrameTransform("effector_tip_link");
    //The translation is currently according to the frame of the end effector. You need to transform it so that it is based on the model frame. I was thinking of using getFrameTransform.
    
    //Convert Affine to Matrix
    Eigen::Matrix3d frame_transform_Matrix;
    frame_transform_Matrix = frame_transform.linear();
    frame_transform_Matrix = frame_transform.matrix().topLeftCorner<3,3>();
    
    //Vector to transform eef position
    Eigen::Vector3d v_prime;
    Eigen::Vector3d cobra_v_prime;
    v_prime = frame_transform_Matrix.inverse()*v;
    cobra_v_prime = frame_transform_Matrix.inverse()*(-cobra_v);
    
    //Remove the cobra_pose component so I can specify 3D space directly
    end_effector_state.translate(cobra_v_prime);
    end_effector_state.translate(v_prime);
    
    std::cout << "v_prime is "<< v_prime << std::endl;
    
    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
    
    //Inverse Kinematics
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
   
    if(found_ik){
      ROS_INFO_STREAM("Found IK Solution");
    }
    else {
      ROS_INFO_STREAM("Did Not Find IK Solution");
    }
     
    ROS_INFO_STREAM("===================================");
    ROS_INFO_STREAM("***********FINAL STATE*************");
    ROS_INFO_STREAM("===================================");

    print_kinematic_state(kinematic_state, joint_model_group);

    return *kinematic_state;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_test");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  sleep(20.0);
  
  // BEGIN_TUTORIAL
  // 
  // Setup
  // ^^^^^
  // 
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("Arm");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.

  robot_state::RobotState start_state(*group.getCurrentState());

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = -0.541201002942;
  target_pose1.orientation.y = -0.20264044842;
  target_pose1.orientation.z = -0.383688659519;
  target_pose1.orientation.w = 0.720292534763;
  target_pose1.position.x = 0.166735663408;
  target_pose1.position.y = -0.0931762477561;
  target_pose1.position.z = 0.483288890133;
  //group.setPoseTarget(target_pose1);

  const robot_state::JointModelGroup *joint_model_group =
                start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, target_pose1);
  group.setStartState(start_state);

  //robot_state::RobotState new_state(*group.getCurrentState());
  //new_state = get_final_state(0.0285793, 0, 0.369884);
  //group.setStartState(new_state);
  group.setJointValueTarget(get_final_state(0.0285793, 0.05, 0.369884));
  
  //group.setRandomTarget();

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // Now that we have a plan we can visualize it in Rviz.  This is not
  // necessary because the group.plan() call we made above did this
  // automatically.  But explicitly publishing plans is useful in cases that we
  // want to visualize a previously created plan.
  if (1)
  {
    ROS_INFO("Visualizing plan 1 (again)");    
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);
  }
  


// END_TUTORIAL 

  ros::shutdown();  
  return 0;
}

