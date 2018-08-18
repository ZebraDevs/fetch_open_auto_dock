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

#ifndef FETCH_AUTO_DOCK_H
#define FETCH_AUTO_DOCK_H

// Custom Includes.
#include <fetch_auto_dock/controller.h>
#include <fetch_auto_dock/perception.h>

// ROS Includes.
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// Fetch Includes.
#include <fetch_driver_msgs/RobotState.h>
#include <fetch_driver_msgs/DisableChargingAction.h>
#include <fetch_auto_dock_msgs/DockAction.h>
#include <fetch_auto_dock_msgs/UndockAction.h>

class AutoDocking
{
  typedef actionlib::SimpleActionClient<fetch_driver_msgs::DisableChargingAction> charge_lockout_client_t;
  typedef actionlib::SimpleActionServer<fetch_auto_dock_msgs::DockAction> dock_server_t;
  typedef actionlib::SimpleActionServer<fetch_auto_dock_msgs::UndockAction> undock_server_t;

public:
  /**
   * @brief Autodocking constructor. Object builds action servers for docking and undocking.
   */
  AutoDocking();

  ~AutoDocking();

private:
  /**
   * @brief Method to update the robot charge state.
   * @param state Robot state message to extract charge state from.
   */
  void stateCallback(const fetch_driver_msgs::RobotStateConstPtr& state);

  /**
   * @brief Method to execute the docking behavior.
   * @param goal Initial pose estimate of the dock.
   */
  void dockCallback(const fetch_auto_dock_msgs::DockGoalConstPtr& goal);

  /**
   * @brief Method that checks success or failure of docking.
   * @param result Dock result message used to set the dock action server state.
   * @return True if we have neither succeeded nor failed to dock.
   */
  bool continueDocking(fetch_auto_dock_msgs::DockResult& result);

  /**
   * @brief Method to see if the robot seems to be docked but not charging.
   *        If the robot does seem to be docked and not charging, try will 
   *        timeout and set abort condition. 
   */
  void checkDockChargingConditions();

  /**
   * @brief Method to execute the undocking behavior.
   * @param goal Docking control action for rotating off of the goal.
   */
  void undockCallback(const fetch_auto_dock_msgs::UndockGoalConstPtr& goal);

  /**
   * @brief Method sets the docking deadline and number of retries.
   */
  void initDockTimeout();

  /**
   * @brief Method checks to see if we have run out of time or retries.
   * @return True if we are out of time or tries.
   */
  bool isDockingTimedOut();

  /**
   * @brief Method to back the robot up under the abort conditon.
   *        Once complete, the method resets the abort flag.
   *        Robot will backup slightly, straighten out, backup a good distance,
   *        and then reorient. Method is blocking.
   * @param r Rate object to control execution loop.
   */
  void executeBackupSequence(ros::Rate& r);

  /**
   * @brief Method to compute the distance the robot should backup when attemping a docking
   *        correction. Method uses a number of state variables in the class to compute
   *        distance. TODO(enhancement): Should these be parameterized instead? 
   * @return Distance for robot to backup in meters.
   */
  double backupDistance();

  /**
   * @brief Method to check approach abort conditions. If we are close to the dock
   *        but the robot is too far off side-to-side or at a bad angle, it should
   *        abort. Method also returns through the parameter the orientation of the
   *        dock wrt the robot for use in correction behaviors.
   * @param dock_yaw Yaw angle of the dock wrt the robot in radians.
   * @return True if the robot should abort the approach.
   */
  bool isApproachBad(double & dock_yaw);

  /**
   * @brief Method to disable the charger for a finite amount of time.
   * @param seconds Number of seconds to disable the charger for. Maximum 
   *                number of seconds is 255. Zero seconds enables the charger.
   * @return True if the number of seconds is valid and the lockout request was 
   *         successful.
   */
  bool lockoutCharger(unsigned seconds);

  // Configuration Constants.
  int NUM_OF_RETRIES_;                        // Number of times the robot gets to attempt
                                              // docking before failing.
  double DOCK_CONNECTOR_CLEARANCE_DISTANCE_;  // The amount to back off in order to clear the
                                              // dock connector.
  double DOCKED_DISTANCE_THRESHOLD_;          // Threshold distance that indicates that the
                                              // robot might be docked.
  // Nodes and servers.
  ros::NodeHandle nh_;
  dock_server_t dock_;                        // Action server to manage docking.
  undock_server_t undock_;                    // Action server to manage undocking.
  charge_lockout_client_t charge_lockout_;    // Action client to request charger lockouts.

  // Helper objects.
  BaseController controller_;  // Drives the robot during docking and undocking phases.
  DockPerception perception_;  // Used to detect dock pose.

  // Subscribe to robot_state, determine if charging
  ros::Subscriber state_;
  bool charging_;

  // Failure detection
  double abort_distance_;    // Distance below which to check abort criteria.
  double abort_threshold_;   // Y-offset that triggers abort.
  double abort_angle_;       // Angle offset that triggers abort.
  double correction_angle_;  // Yaw correction angle the robot should use to line up with the dock.
  double backup_limit_;      // Maximum distance the robot will backup when trying to retry.
                             // Based on range of initial dock pose estimate.
  bool   aborting_;          // If the robot realizes it won't be sucessful, it needs to abort.
  int    num_of_retries_;    // The number of times the robot gets to abort before failing.
                             // This variable will count down.
  bool cancel_docking_;      // Signal that docking has failed and the action server should abort the goal.
  ros::Time deadline_docking_;       // Time when the docking times out.
  ros::Time deadline_not_charging_;  // Time when robot gives up on the charge state and retries docking.
  bool charging_timeout_set_;        // Flag to indicate if the deadline_not_charging has been set.
};

#endif  // FETCH_AUTO_DOCK_H