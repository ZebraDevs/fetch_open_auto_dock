/*
 * Copyright 2015 Fetch Robotics Inc.
 * Author: Griswald Brooks
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

#ifndef FETCH_AUTO_DOCK_LINEAR_POSE_FILTER_H
#define FETCH_AUTO_DOCK_LINEAR_POSE_FILTER_H

// STL includes.
#include <vector>
#include <deque>

// ROS includes.
#include <geometry_msgs/Pose.h>

// Boost includes.
#include <boost/shared_ptr.hpp>

class LinearPoseFilter2D
{
public:
  /**
   * @brief Constructor for LinearPoseFilter2D.
   * @param b Filter coefficients for the input samples. 
   *          The number of coefficients is the order of the filter + 1.
   * @param a Filter coefficients for the output samples.
   *          Vector a should have the same number of elements as b, 
   *          with the first element being 1.
   */
  LinearPoseFilter2D(const std::vector<float>& b, const std::vector<float>& a);

  /**
   * @brief Method to set the filter coefficients.
   * @param b Filter coefficients for the input samples. 
   *          The number of coefficients is the order of the filter + 1.
   * @param a Filter coefficients for the output samples.
   *          Vector a should have the same number of elements as b, 
   *          with the first element being 1.
   */
  void setCoeff(const std::vector<float>& b, const std::vector<float>& a);

  /**
   * @brief Method to filter pose objects. Filter only filters position.x, position.y and the 
   *        yaw of the pose.
   * @param pose Input pose to be filtered.
   * @return Filtered pose.
   */
  geometry_msgs::Pose filter(const geometry_msgs::Pose& pose);

  /**
   * @brief Method to reset the filter to ensure that if the filter is being reused, it is not
   *        corrupted by old values. Sets sample histories to the origin.
   */
  void reset();

  /**
  * @brief Method to set the sample history of the filter to some state.
  *        All of the sample histories will be set to the parameterized values.
  * @param input_pose  The initial input pose that will be set for all previous inputs.
  * @param output_pose The initial output pose that will be set for all previous outputs.
  */
  void setFilterState(const geometry_msgs::Pose& input_pose, const geometry_msgs::Pose& output_pose);

  /**
  * @brief Method to set the sample history of the filter to some (set of) state(s).
  *                     new_poses[0] is the oldest sample and new_poses[new_poses.size() - 1]
  *                     is the newest sample.
  *                     If the number of poses is greater than the length of the filter memory, 
  *                     only the newest poses will be used.
  *                     If the number of poses is less than the length of the filter memory,
  *                     the poses will be copied to the most recent history and the original
  *                     samples will retain their original location in the buffer.
  *                     Example:
  *                     // New poses.
  *                     np = [1, 2, 3]
  *                     // Original poses.
  *                     p  = [9, 8, 7, 6, 5]
  *                     // Poses after setFilterState
  *                     p  = [9, 8, 1, 2, 3]
  * @param input_poses  Set of input poses to set the input state time history to.
  * @param output_poses Set of output poses to set the output state time history to.
  */
  void setFilterState(const std::vector<geometry_msgs::Pose>& input_poses,
                      const std::vector<geometry_msgs::Pose>& output_poses);

  /**
   * @brief Not yet implemented but need to have methods that take only input histories and then
   *        recreate the appropriate output histories. This is because the input history encodes the 
   *        entire filter history.
   */
  void setFilterState(const geometry_msgs::Pose& input_pose);
  void setFilterState(const std::vector<geometry_msgs::Pose>& input_poses);
private:
  /**
   * @brief Method to get origin pose.
   * @return Pose with the linear pose set to the origin and the quaternion set to 
   *         Roll, Pitch, Yaw set to zero.
   */
  geometry_msgs::Pose originPose();

  /**
   * @brief Method to return the unnormalized/wrapped-up yaw.
   *
   *        This method adds shortest angular distance between the pose and the reference
   *        yaw and returns it. 
   *        Example:
   *          pose = -178 degrees, reference_yaw = 177 degrees, return = 182 degrees.
   *          pose =  175 degrees, reference_yaw = -170 degrees, return = -185 degrees.
   * @param pose The pose to be unnormalized.
   * @param reference_yaw The yaw with which to reference and whose sign is respected.
   * @return Unnormalized yaw in radians.
   */
  float getUnnormalizedYaw(geometry_msgs::Pose pose, float reference_yaw);

  /**
   * @brief Method to get the most recent filtered unnormalized yaw. 
   *        If the filter has never produced an output before, the zero will be given.
   * @return Most recently filtered yaw.
   */
  float getNewestOutputYaw();

  std::deque<geometry_msgs::Pose> out_;   /// Vector of previous output poses.
  std::deque<geometry_msgs::Pose> in_;    /// Vector of previous input poses.
  std::deque<float> yaw_out_;             /// Vector of previous output yaws that aren't normalized.
  std::deque<float> yaw_in_;              /// Vector of previous input yaws that aren't normalized.
  std::vector<float> b_;                  /// Vector of input coefficients.
  std::vector<float> a_;                  /// Vector of output coefficients.
};

typedef boost::shared_ptr<LinearPoseFilter2D> LinearPoseFilter2DPtr;

#endif  // FETCH_AUTO_DOCK_LINEAR_POSE_FILTER_H
