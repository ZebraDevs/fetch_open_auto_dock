/*
 * Copyright 2015, Fetch Robotics, Inc.
 * Author: Michael Ferguson, Griswald Brooks
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <fetch_auto_dock/auto_dock.h>

// STL Includes.
#include <math.h>
#include <algorithm>

// ROS Includes.
#include <angles/angles.h>

AutoDocking::AutoDocking() :
  dock_(nh_, "dock", boost::bind(&AutoDocking::dockCallback, this, _1), false),
  undock_(nh_, "undock", boost::bind(&AutoDocking::undockCallback, this, _1), false),
  charge_lockout_("charge_lockout", true),
  controller_(nh_),
  perception_(nh_),
  aborting_(false),
  num_of_retries_(NUM_OF_RETRIES_),
  cancel_docking_(true),
  charging_timeout_set_(false)
{
  // Load ros parameters
  ros::NodeHandle pnh("~");
  pnh.param("abort_distance",                    abort_distance_,                    0.40);
  pnh.param("abort_threshold",                   abort_threshold_,                   0.025);
  pnh.param("abort_angle",                       abort_angle_,                       5.0*(M_PI/180.0)),
  pnh.param("num_of_retries",                    NUM_OF_RETRIES_,                    5);
  pnh.param("dock_connector_clearance_distance", DOCK_CONNECTOR_CLEARANCE_DISTANCE_, 0.2);
  pnh.param("docked_distance_threshold",         DOCKED_DISTANCE_THRESHOLD_,         0.34);

  // Subscribe to robot state
  state_ = nh_.subscribe<fetch_driver_msgs::RobotState>("robot_state",
                                                        1,
                                                        boost::bind(&AutoDocking::stateCallback, this, _1));

  // Start action server thread
  dock_.start();
  undock_.start();
}

AutoDocking::~AutoDocking()
{
}

void AutoDocking::stateCallback(const fetch_driver_msgs::RobotStateConstPtr& state)
{
  // Check the voltage to see if we were connected to the dock.
  // Previously, this check was done by checking the supply_breaker.current
  // but this isn't the best check since there are conditions where the charger hasn't 
  // yet enabled charging or is in some error state.
  // The thing that really needs to be checked is if we have made sufficient physical
  // contact with the dock connect, which the voltage should tell us.
  // This should also help extend the life of the tires, as previously the robot would
  // continue to drive until the supply_breaker.current passed a threshold, which could
  // take several seconds.
  charging_ = (state->charger.supply_voltage > 30.0);
}

void AutoDocking::dockCallback(const fetch_auto_dock_msgs::DockGoalConstPtr& goal)
{
  fetch_auto_dock_msgs::DockFeedback feedback;
  fetch_auto_dock_msgs::DockResult result;

  // Reset flags.
  result.docked = false;
  aborting_ = false;
  charging_timeout_set_ = false;
  cancel_docking_ = false;

  // Not currently supporting move_base option
  if (goal->use_move_base)
  {
    dock_.setAborted(result);
    ROS_ERROR("Docking failure: use_move_base not yet supported");
    return;
  }

  // Object for controlling loop rate.
  ros::Rate r(50.0);

  // Start perception
  perception_.start(goal->dock_pose);

  // For timeout calculation
  initDockTimeout();

  // Get initial dock pose.
  geometry_msgs::PoseStamped dock_pose_base_link;
  while (!perception_.getPose(dock_pose_base_link, "base_link"))
  {
    // Wait for perception to get its first pose estimate.
    if (!continueDocking(result))
    {
      ROS_DEBUG_NAMED("autodock_dock_callback",
                    "Docking failed: Initial dock not found.");
      break;
    }
  }

  // Preorient the robot.
  double dock_yaw = angles::normalize_angle(tf::getYaw(dock_pose_base_link.pose.orientation));
  if (!std::isfinite(dock_yaw))
  {
    ROS_ERROR_STREAM_NAMED("auto_dock", "Dock yaw is invalid.");
    cancel_docking_ = true;
  }
  else if (ros::ok() && continueDocking(result))
  {
    // Set backup limit to be the initial dock range.
    backup_limit_ = sqrt(pow(dock_pose_base_link.pose.position.x, 2) + pow(dock_pose_base_link.pose.position.y, 2));
    // Shorten up the range a bit.
    backup_limit_ *= 0.9;

    // Preorient the robot towards the dock.
    while (!controller_.backup(0.0, -dock_yaw) && 
           continueDocking(result)             &&
           ros::ok()
           )
    {
      r.sleep();  // Sleep the rate control object.
    }
  }

  // Make sure controller is ready
  controller_.stop();


  // Control
  while (ros::ok() && continueDocking(result))
  {
    // Update perception
    if (perception_.getPose(feedback.dock_pose))
    {
      if (aborting_)
      {
        // Backup.
        executeBackupSequence(r);

        // Reset abort flag.
        aborting_ = false;

        // Decrement the number of retries.
        num_of_retries_--;
      }
      else
      {
        // Check to see if we are on target and compute yaw correction angle.
        if (isApproachBad(correction_angle_))
        {
          // Not on target, abort, abort, abort!
          controller_.stop();
          aborting_ = true;
        }
        else
        {
          // Update control
          controller_.approach(feedback.dock_pose);
          // Are we on the dock? Check charging timeout.
          checkDockChargingConditions();
        }
      }

      // Feedback is mainly for our debugging
      controller_.getCommand(feedback.command);
      dock_.publishFeedback(feedback);
    }

    // Sleep the rate control object.
    r.sleep();
  }

  // Make sure we stop things before we are done.
  controller_.stop();
  perception_.stop();
}

/**
 * @brief Method that checks success or failure of docking.
 * @param result Dock result message used to set the dock action server state.
 * @return True if we have neither succeeded nor failed to dock.
 */
bool AutoDocking::continueDocking(fetch_auto_dock_msgs::DockResult& result)
{
  // If charging, stop and return success.
  if (charging_)
  {
    result.docked = true;
    dock_.setSucceeded(result);
    ROS_DEBUG_NAMED("autodock_dock_callback",
                    "Docking success: Robot has docked");
    return false;
  }
  // Timeout on time or retries.
  else if (isDockingTimedOut() || cancel_docking_)
  {
    dock_.setAborted(result);
    ROS_WARN("Docking failed: timed out");
    return false;
  }
  // Something is stopping us.
  else if (dock_.isPreemptRequested())
  {
    dock_.setPreempted(result);
    ROS_WARN("Docking failure: preempted");
    return false;
  }

  return true;
}

/**
 * @brief Method to see if the robot seems to be docked but not charging.
 *        If the robot does seem to be docked and not charging, try will 
 *        timeout and set abort condition. 
 */
void AutoDocking::checkDockChargingConditions()
{
  // Grab the dock pose in the base_link so we can evaluate it wrt the robot.
  geometry_msgs::PoseStamped dock_pose_base_link;
  perception_.getPose(dock_pose_base_link, "base_link");

  // Are we close enough to be docked?
  if (dock_pose_base_link.pose.position.x < DOCKED_DISTANCE_THRESHOLD_)
  {
    if (!charging_timeout_set_)
    {
      deadline_not_charging_ = ros::Time::now() + ros::Duration(5.0);
      charging_timeout_set_ = true;
    }
    else if (ros::Time::now() > deadline_not_charging_)
    {
      // Reset charging timeout flag.
      charging_timeout_set_ = false;
      // Reset correction angle since it was probably fine.
      correction_angle_ = 0.0;
      // Reset since we don't seem to be charging.
      aborting_ = true;  
    }
  }
  else
  {
    // If we back up enough, reset the charging timeout flag.
    charging_timeout_set_ = false;
  }
}

/**
 * @brief Method sets the docking deadline and number of retries.
 */
void AutoDocking::initDockTimeout()
{
  deadline_docking_ = ros::Time::now() + ros::Duration(120.0);
  num_of_retries_ = NUM_OF_RETRIES_;
}

/**
 * @brief Method checks to see if we have run out of time or retries.
 * @return True if we are out of time or tries.
 */
bool AutoDocking::isDockingTimedOut()
{
  // Have we exceeded our deadline or tries?
  if (ros::Time::now() > deadline_docking_ || !num_of_retries_)
  {
    return true;
  }
  return false;
}

/**
 * @brief Method to back the robot up under the abort conditon.
 *        Once complete, the method resets the abort flag.
 *        Robot will backup slightly, straighten out, backup a good distance,
 *        and then reorient. Method is blocking.
 * @param r Rate object to control execution loop.
 */
void AutoDocking::executeBackupSequence(ros::Rate& r)
{
  // Disable charging for a second.
  lockoutCharger(1);

  // Get off of the dock. Try to straighten out.
  while (!controller_.backup(DOCK_CONNECTOR_CLEARANCE_DISTANCE_, correction_angle_))
  {
    if (isDockingTimedOut())
    {
      return;
    }
    r.sleep();  // Sleep the rate control object.
  }
  // Move to recovery pose.
  while (!controller_.backup(backupDistance(), 0.0))
  {
    if (isDockingTimedOut())
    {
      return;
    }
    r.sleep();
  }
}

/**
 * @brief Method to compute the distance the robot should backup when attemping a docking
 *        correction. Method uses a number of state variables in the class to compute
 *        distance. TODO(enhancement): Should these be parameterized instead? 
 * @return Distance for robot to backup in meters.
 */
double AutoDocking::backupDistance()
{
  // Initialized to 1.0 meter as our basic backup amount.
  double distance = 1.0;

  // Distance should be proportional to the amount of yaw correction.
  // The constants are purely arbitrary because they seemed good at the time.
  // distance *= 3.5*fabs(correction_angle_);
  distance *= 1.5*fabs(correction_angle_);
  // We should backup more the more times we try. This function should range from 1 to 2.
  // num_of_retries is initially equal to NUM_OF_RETRIES and decrements as the robot retries.
  double retry_constant = 2 - static_cast<float>(num_of_retries_)/NUM_OF_RETRIES_;
  retry_constant = std::max(1.0, std::min(2.0, retry_constant));
  distance *= retry_constant;

  // Cap the backup limit to 1 meter, just in case.
  backup_limit_ = std::min(1.0, backup_limit_);
  // Threshold distance.
  distance = std::max(0.2, std::min(backup_limit_, distance));

  return distance;
}

/**
 * @brief Method to check approach abort conditions. If we are close to the dock
 *        but the robot is too far off side-to-side or at a bad angle, it should
 *        abort. Method also returns through the parameter the orientation of the
 *        dock wrt the robot for use in correction behaviors.
 * @param dock_yaw Yaw angle of the dock wrt the robot in radians.
 * @return True if the robot should abort the approach.
 */
bool AutoDocking::isApproachBad(double & dock_yaw)
{
  // Grab the dock pose in the base_link so we can evaluate it wrt the robot.
  geometry_msgs::PoseStamped dock_pose_base_link;
  perception_.getPose(dock_pose_base_link, "base_link");

  dock_yaw = angles::normalize_angle(tf::getYaw(dock_pose_base_link.pose.orientation));

  // If we are close to the dock but not quite docked, check other approach parameters.
  if (dock_pose_base_link.pose.position.x < abort_distance_ &&
      dock_pose_base_link.pose.position.x > DOCKED_DISTANCE_THRESHOLD_
      )
  {
    // Check to see if we are too far side-to-side or at a bad angle.
    if (fabs(dock_pose_base_link.pose.position.y)              > abort_threshold_ ||
        fabs(dock_yaw)                                         > abort_angle_
       )
    {
      // Things are bad, abort.
      return true;
    }
  }
  // Everything is ok.
  return false;
}

bool AutoDocking::lockoutCharger(unsigned seconds)
{
  // Check if parameter is valid.
  if (seconds > 255)
  {
    return false;
  }

  fetch_driver_msgs::DisableChargingGoal lockout_goal;

  // Attempt to connect to the charger lockout server and lockout the charger.
  if (charge_lockout_.waitForServer(ros::Duration(1.0)))
  {
    lockout_goal.disable_duration = ros::Duration(seconds);
    charge_lockout_.sendGoal(lockout_goal);

    if (!charge_lockout_.waitForResult(ros::Duration(1.0)))
    {
      ROS_WARN_STREAM_NAMED("autodock_undock_callback", "Unable to lockout charger before undocking.");
      return false;
    }
  }
  else
  {
    ROS_WARN_STREAM_NAMED("autodock_undock_callback", "Charge lockout server could not be contacted.");
    return false;
  }

  return true;
}

void AutoDocking::undockCallback(const fetch_auto_dock_msgs::UndockGoalConstPtr& goal)
{


  // Disable the charger for just a little bit longer than the undock procedure might take.
  lockoutCharger(6); 

  fetch_auto_dock_msgs::UndockFeedback feedback;
  fetch_auto_dock_msgs::UndockResult result;
  result.undocked = false;

  // Distances to backup/turn
  double backup = DOCK_CONNECTOR_CLEARANCE_DISTANCE_;
  double turn = goal->rotate_in_place ? 3.1 : 0.0;

  // Make sure controller is ready
  controller_.stop();

  // Timeout for undocking
  ros::Time timeout = ros::Time::now() + ros::Duration(5.0);

  // Control
  ros::Rate r(50.0);
  while (ros::ok())
  {
    if (undock_.isPreemptRequested())
    {
      controller_.stop();
      undock_.setPreempted(result);
      ROS_WARN_NAMED("autodock_undock_callback", "Undocking preempted");
      return;
    }

    // Update control
    if (controller_.backup(backup, turn))
    {
      // Odom says we have undocked
      result.undocked = true;
      controller_.stop();
      undock_.setSucceeded(result);
      // Command the charger to turn back on.
      lockoutCharger(0);
      ROS_DEBUG_NAMED("autodock_undock_callback", "Robot has undocked");
      return;
    }

    // Feedback is mainly for our debugging
    undock_.publishFeedback(feedback);

    // Timeout
    if (ros::Time::now() > timeout)
    {
      controller_.stop();
      undock_.setAborted(result);
      // Command the charger to turn back on.
      lockoutCharger(0);
      ROS_WARN_NAMED("autodock_undock_callback", "Undocking failed: timed out");
      return;
    }

    r.sleep();
  }

  controller_.stop();
  undock_.setAborted(result);
  // Command the charger to turn back on.
  lockoutCharger(0);
  ROS_WARN_NAMED("autodock_undock_callback", "Undocking failed: ROS no longer OK");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "auto_dock");
  AutoDocking auto_dock;
  ros::spin();
  return 0;
}
