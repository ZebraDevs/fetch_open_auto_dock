/*
 * Copyright 2015 Fetch Robotics Inc.
 * Author: Michael Ferguson, Sriramvarun Nidamarthy
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

#include <fetch_auto_dock/icp_2d.h>
#include <fetch_auto_dock/perception.h>

#include <angles/angles.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <list>
#include <queue>
#include <string>
#include <vector>

double getPoseDistance(const geometry_msgs::Pose a,
                       const geometry_msgs::Pose b)
{
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  return sqrt(dx*dx + dy*dy);
}

DockPerception::DockPerception(ros::NodeHandle& nh) :
  running_(false),
  tracking_frame_("odom"),
  found_dock_(false)
{
  ros::NodeHandle pnh("~");

  // Should we publish the debugging cloud
  if (!pnh.getParam("debug", debug_))
  {
    debug_ = false;
  }

  // Create coefficient vectors for a second order butterworth filter
  // with a cutoff frequency of 10 Hz assuming the loop is updated at 50 Hz
  // [b, a] = butter(order, cutoff_frequ/half_sampling_freq)
  // [b, a] = butter(2, 10/25)
  float b_arr[] = {0.20657,  0.41314, 0.20657};
  float a_arr[] = {1.00000, -0.36953, 0.19582};
  std::vector<float> b(b_arr, b_arr + sizeof(b_arr)/sizeof(float));
  std::vector<float> a(a_arr, a_arr + sizeof(a_arr)/sizeof(float));
  dock_pose_filter_.reset(new LinearPoseFilter2D(b, a));

  // Limit the average reprojection error of points onto
  // the ideal dock. This prevents the robot docking
  // with something that is very un-dock-like.
  if (!pnh.getParam("max_alignment_error", max_alignment_error_))
  {
    max_alignment_error_ = 0.01;
  }

  // Create ideal cloud
  // Front face is 300mm long
  for (double y = -0.15; y <= 0.15; y += 0.001)
  {
    geometry_msgs::Point p;
    p.x = p.z = 0.0;
    p.y = y;
    ideal_cloud_.push_back(p);
    front_cloud_.push_back(p);
  }
  // Each side is 100mm long, at 45 degree angle
  for (double x = 0.0; x < 0.05 /*0.0707106*/; x += 0.001)
  {
    geometry_msgs::Point p;
    p.x = x;
    p.y = 0.15 + x;
    p.z = 0.0;
    ideal_cloud_.push_back(p);
    p.y = -0.15 - x;
    ideal_cloud_.insert(ideal_cloud_.begin(), p);
  }

  // Debugging publishers first
  if (debug_)
  {
    debug_points_ = nh.advertise<sensor_msgs::PointCloud2>("dock_points", 10);
  }

  // Init base scan only after publishers are created
  scan_sub_ = nh.subscribe("base_scan", 1, &DockPerception::callback, this);

  ROS_INFO_NAMED("perception","Dock perception initialized");
}

bool DockPerception::start(const geometry_msgs::PoseStamped& pose)
{
  running_ = false;
  found_dock_ = false;
  dock_ = pose;
  running_ = true;
  return true;
}

bool DockPerception::stop()
{
  running_ = false;
  return true;
}

bool DockPerception::getPose(geometry_msgs::PoseStamped& pose, std::string frame)
{
  // All of this requires a lock on the dock_
  boost::mutex::scoped_lock lock(dock_mutex_);

  if (!found_dock_)
    return false;

  if (ros::Time::now() > dock_stamp_ + ros::Duration(0.35))
  {
    ROS_DEBUG_NAMED("dock_perception", "Dock pose timed out");
    return false;
  }

  // Check for a valid orientation.
  tf::Quaternion q;
  tf::quaternionMsgToTF(dock_.pose.orientation, q);
  if (!isValid(q))
  {
    ROS_ERROR_STREAM_NAMED("perception", 
                           "Dock orientation invalid.");
    ROS_DEBUG_STREAM_NAMED("perception",
                           "Quaternion magnitude is "
                           << q.length()
                           << " Quaternion is ["
                           << q.x() << ", " << q.y() << ", "
                           << q.z() << ", " << q.w() << "]"
                           );
    return false;
  }

  pose = dock_;

  if (frame != "")
  {
    // Transform to desired frame
    try
    {
      listener_.waitForTransform(frame,
                                 pose.header.frame_id,
                                 pose.header.stamp,
                                 ros::Duration(0.1));
      listener_.transformPose(frame, pose, pose);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Couldn't transform dock pose");
      return false;
    }
  }

  return found_dock_;
}

void DockPerception::callback(const sensor_msgs::LaserScanConstPtr& scan)
{
  // Be lazy about search
  if (!running_)
  {
    return;
  }

  // Make sure goal is valid (orientation != 0 0 0 0)
  if (dock_.header.frame_id == "" ||
      (dock_.pose.orientation.z == 0.0 && dock_.pose.orientation.w == 0.0))
  {
    // Lock the dock_
    boost::mutex::scoped_lock lock(dock_mutex_);

    // If goal is invalid, set to a point directly ahead of robot
    for (size_t i = scan->ranges.size()/2; i < scan->ranges.size(); i++)
    {
      if (std::isfinite(scan->ranges[i]))
      {
        double angle = scan->angle_min + i * scan->angle_increment;
        dock_.header = scan->header;
        dock_.pose.position.x = cos(angle) * scan->ranges[i];
        dock_.pose.position.y = sin(angle) * scan->ranges[i];
        dock_.pose.orientation.x = 1.0;
        dock_.pose.orientation.y = 0.0;
        dock_.pose.orientation.z = 0.0;
        dock_.pose.orientation.w = 0.0;
        ROS_DEBUG_NAMED("dock_perception", "Set initial pose to (%f, %f, %f)",
                        dock_.pose.position.x, dock_.pose.position.y,
                        icp_2d::thetaFromQuaternion(dock_.pose.orientation));
        break;
      }
    }
  }

  // Make sure goal is in the tracking frame
  if (dock_.header.frame_id != tracking_frame_)
  {
    boost::mutex::scoped_lock lock(dock_mutex_);
    try
    {
      listener_.waitForTransform(tracking_frame_,
                                 dock_.header.frame_id,
                                 dock_.header.stamp,
                                 ros::Duration(0.1));
      listener_.transformPose(tracking_frame_, dock_, dock_);
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Couldn't transform dock pose to tracking frame");
      return;
    }
    ROS_DEBUG_NAMED("dock_perception", "Transformed initial pose to (%f, %f, %f)",
                    dock_.pose.position.x, dock_.pose.position.y,
                    icp_2d::thetaFromQuaternion(dock_.pose.orientation));
  }

  // Cluster the laser scan
  laser_processor::ScanMask mask;
  laser_processor::ScanProcessor processor(*scan, mask);
  processor.splitConnected(0.04);  // TODO(enhancement) parameterize
  processor.removeLessThan(5);

  // Sort clusters based on distance to last dock
  std::priority_queue<DockCandidatePtr, std::vector<DockCandidatePtr>, CompareCandidates> candidates;
  for (std::list<laser_processor::SampleSet*>::iterator i = processor.getClusters().begin();
       i != processor.getClusters().end();
       i++)
  {
    DockCandidatePtr c = extract(*i);
    if (c && c->valid(found_dock_))
    {
      candidates.push(c);
    }
  }
  ROS_DEBUG_STREAM_NAMED("dock_perception", "Extracted " << candidates.size() << " clusters");

  // Extract ICP pose/fit on best clusters
  DockCandidatePtr best;
  geometry_msgs::Pose best_pose;
  while (!candidates.empty())
  {
    geometry_msgs::Pose pose = dock_.pose;
    double score = fit(candidates.top(), pose);
    if (score >= 0)
    {
      best = candidates.top();
      best_pose = pose;
      break;
    }
    else  // Let's see what's wrong with this point cloud.
    {
      if (debug_)
      {
        DockCandidatePtr not_best = candidates.top();

        // Create point cloud
        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp = scan->header.stamp;
        cloud.header.frame_id = tracking_frame_;
        cloud.width = cloud.height = 0;

        // Allocate space for points
        sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
        cloud_mod.setPointCloud2FieldsByString(1, "xyz");
        cloud_mod.resize(not_best->points.size());

        // Fill in points
        sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");
        for (size_t i = 0; i < not_best->points.size(); i++)
        {
          cloud_iter[0] = not_best->points[i].x;
          cloud_iter[1] = not_best->points[i].y;
          cloud_iter[2] = not_best->points[i].z;
          ++cloud_iter;
        }
        debug_points_.publish(cloud);
      }
    }
    candidates.pop();
  }

  // Did we find dock?
  if (!best)
  {
    ROS_DEBUG_NAMED("dock_perception", "DID NOT FIND THE DOCK");
    return;
  }

  ROS_DEBUG_NAMED("dock_perception", "Found the dock.");

  // Update
  if (debug_)
  {
    // Create point cloud
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = scan->header.stamp;
    cloud.header.frame_id = tracking_frame_;
    cloud.width = cloud.height = 0;

    // Allocate space for points
    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    cloud_mod.resize(best->points.size());

    // Fill in points
    sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");
    for (size_t i = 0; i < best->points.size(); i++)
    {
      cloud_iter[0] = best->points[i].x;
      cloud_iter[1] = best->points[i].y;
      cloud_iter[2] = best->points[i].z;
      ++cloud_iter;
    }
    debug_points_.publish(cloud);
  }

  // Everything after this modifies the dock_
  boost::mutex::scoped_lock lock(dock_mutex_);

  // Update stamp
  dock_.header.stamp = scan->header.stamp;
  dock_.header.frame_id = tracking_frame_;

  // If this is the first time we've found dock, take whole pose
  if (!found_dock_)
  {
    dock_.pose = best_pose;
    // Reset the dock pose filter.
    dock_pose_filter_->reset();
    // Set the filter state to the current pose estimate.
    dock_pose_filter_->setFilterState(dock_.pose, dock_.pose);
  }
  else
  {
    // Check that pose is not too far from current pose
    double d = getPoseDistance(dock_.pose, best_pose);
    if (d > 0.05)
    {
      ROS_DEBUG_STREAM_NAMED("dock_perception", "Dock pose jumped: " << d);
      return;
    }
  }

  // Filter the pose esitmate.
  dock_.pose = dock_pose_filter_->filter(best_pose);
  dock_stamp_ = scan->header.stamp;
  found_dock_ = true;
}

DockCandidatePtr DockPerception::extract(laser_processor::SampleSet* cluster)
{
  DockCandidatePtr candidate(new DockCandidate());

  tf::Point tf_point;
  tf::StampedTransform t_frame;
  try
  {
    listener_.waitForTransform(tracking_frame_,
                               cluster->header.frame_id,
                               cluster->header.stamp,
                               ros::Duration(0.1));
    listener_.lookupTransform(tracking_frame_,
                              cluster->header.frame_id,
                              ros::Time(0),
                              t_frame);
  }
  catch (tf::TransformException const &ex)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Couldn't transform laser point");
    return candidate;
  }
  // Transform each point into tracking frame
  size_t i = 0;
  for (laser_processor::SampleSet::iterator p = cluster->begin();
       p != cluster->end();
       p++, i++)
  {
    geometry_msgs::PointStamped pt;
    pt.header = cluster->header;
    pt.point.x = (*p)->x;
    pt.point.y = (*p)->y;
    pt.point.z = 0;
    tf::pointMsgToTF(pt.point, tf_point);
    tf_point = t_frame*tf_point;
    tf::pointTFToMsg(tf_point, pt.point);
    candidate->points.push_back(pt.point);
  }

  // Get distance from cloud center to previous pose
  geometry_msgs::Point centroid = icp_2d::getCentroid(candidate->points);
  double dx = centroid.x - dock_.pose.position.x;
  double dy = centroid.y - dock_.pose.position.y;
  candidate->dist = (dx*dx + dy*dy);

  return candidate;
}

double DockPerception::fit(const DockCandidatePtr& candidate, geometry_msgs::Pose& pose)
{
  // Setup initial pose
  geometry_msgs::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.rotation = pose.orientation;

  // Initial yaw. Presumably the initial goal orientation estimate.
  tf::Quaternion init_pose, cand_pose;
  quaternionMsgToTF(pose.orientation, init_pose);
  if (!isValid(init_pose))
  {
    ROS_ERROR_STREAM_NAMED("perception", 
                           "Initial dock orientation estimate is invalid.");
    ROS_DEBUG_STREAM_NAMED("perception",
                           "Quaternion magnitude is "
                           << init_pose.length()
                           << " Quaternion is ["
                           << init_pose.x() << ", " << init_pose.y() << ", "
                           << init_pose.z() << ", " << init_pose.w() << "]"
                           );
    return -1.0;
  }

  // ICP the dock
  double fitness = icp_2d::alignICP(ideal_cloud_, candidate->points, transform);
  quaternionMsgToTF(transform.rotation, cand_pose);
  if (!isValid(cand_pose))
  {
    ROS_WARN_STREAM_NAMED("perception", 
                          "Dock candidate orientation estimate is invalid."); 
    ROS_DEBUG_STREAM_NAMED("perception", 
                           "Quaternion magnitude is "
                           << cand_pose.length()
                           << " Quaternion is ["
                           << cand_pose.x() << ", " << cand_pose.y() << ", "
                           << cand_pose.z() << ", " << cand_pose.w() << "]"
                           );
  }

  // If the dock orientation seems flipped, flip it.
  // Check it by finding the relative roation between the two quaternions.
  if (fabs(angles::normalize_angle(tf::getYaw(tf::inverse(cand_pose)*init_pose))) > 3.1415*(2.0/3.0) )
  {
    transform.rotation = tf::createQuaternionMsgFromYaw(3.1415 + tf::getYaw(transform.rotation));
  }

  if (fitness >= 0.0)
  {
    // Initialize the number of times we retry if the fitness is bad.
    double retry = 5;
    // If the fitness is hosed or the angle is borked, try again.
    quaternionMsgToTF(transform.rotation, cand_pose);
    while (retry-- &&
            (
              fitness                                                       > max_alignment_error_ ||
              fabs(angles::normalize_angle(tf::getYaw(tf::inverse(cand_pose)*init_pose))) > 3.1415/4.0
            )
          )
    {
      // Try one more time.

      // Perturb the pose to try to get it out of the local minima.
      transform.translation.x += retry*(0.75/100.0)*static_cast<double>((rand() % 200) - 100);
      transform.translation.y += retry*(0.75/100.0)*static_cast<double>((rand() % 200) - 100);
      transform.rotation = tf::createQuaternionMsgFromYaw(retry*(0.28/100.0)*double((rand() % 200) - 100) + tf::getYaw(transform.rotation));

      // Rerun ICP.
      fitness = icp_2d::alignICP(ideal_cloud_, candidate->points, transform);

      // If the dock orientation seems flipped, flip it.
      quaternionMsgToTF(transform.rotation, cand_pose);
      if (fabs(angles::normalize_angle(tf::getYaw(tf::inverse(cand_pose)*init_pose))) > 3.1415*(2.0/3.0) )
      {
        transform.rotation = tf::createQuaternionMsgFromYaw(3.1415 + tf::getYaw(transform.rotation));
      }
    }
     
    // If the dock orientation is still really borked, fail.
    quaternionMsgToTF(transform.rotation, cand_pose);
    if (!isValid(cand_pose))
    {
      ROS_ERROR_STREAM_NAMED("perception", 
                             "Dock candidate orientation estimate is invalid.");
      ROS_DEBUG_STREAM_NAMED("perception","Quaternion magnitude is "
                             << cand_pose.length()
                             << " Orientation is ["
                             << cand_pose.x() << ", " << cand_pose.y() << ", "
                             << cand_pose.z() << ", " << cand_pose.w() << "]"
                             );
      return -1.0;
    }
    if (fabs(angles::normalize_angle(tf::getYaw(tf::inverse(cand_pose)*init_pose))) > 3.1415/2.0 )
    {
      fitness = -1.0;
    }

    // Check that fitness is good enough
    if (!found_dock_ && fabs(fitness) > max_alignment_error_)
    {
      // If not, signal no fit
      fitness = -1.0;
    }

    // If width of candidate is smaller than the width of dock
    // then the whole dock is not visible...
    if (candidate->width() < 0.375)
    {
      // ... and heading is unreliable when close to dock
      ROS_DEBUG_STREAM_NAMED("perception", "Dock candidate width is unreliable.");
      transform.rotation = pose.orientation;
      fitness = 0.001234;
      // Probably can use a different algorithm here, if necessary, which it might not be.
    }

    // Transform ideal cloud, and store for visualization
    candidate->points = icp_2d::transform(ideal_cloud_,
                                          transform.translation.x,
                                          transform.translation.y,
                                          icp_2d::thetaFromQuaternion(transform.rotation));

    // Get pose
    pose.position.x = transform.translation.x;
    pose.position.y = transform.translation.y;
    pose.position.z = transform.translation.z;
    pose.orientation = transform.rotation;
    return fitness;
  }

  // Signal no fit
  ROS_DEBUG_NAMED("dock_perception", "Did not converge");
  ROS_INFO_STREAM("Did not converge.");
  return -1.0;
}

bool DockPerception::isValid(const tf::Quaternion& q)
{
  return 1e-3 >= fabs(1.0 - q.length());
}
