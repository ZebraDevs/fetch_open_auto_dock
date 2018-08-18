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

#ifndef FETCH_AUTO_DOCK_DOCK_CANDIDATE_H
#define FETCH_AUTO_DOCK_DOCK_CANDIDATE_H

#include <geometry_msgs/Point.h>

#include <boost/shared_ptr.hpp>
#include <vector>

/**
 * @brief A cluster which is a candidate for a dock
 */
struct DockCandidate
{
  std::vector<geometry_msgs::Point> points;
  double dist;  // distance from initial/previous pose

  /** @brief Get the width of this segment */
  double width()
  {
    // If there are no points then there is no width.
    if (points.empty())
    {
      return 0;
    }

    geometry_msgs::Point& pt1 = points.front();
    geometry_msgs::Point& pt2 = points.back();
    return (sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2)));
  }

  /**
   * @brief Determine if this candidate meets our basic criteria
   * @param dock_found Has the dock been found in a previous frame?
   */
  bool valid(bool dock_found)
  {
    // If there are no points this cannot be valid.
    if (points.empty())
    {
      return false;
    }

    // Check overall size
    if (width() > 0.5 || width() < 0.25)
      return false;

    // If dock is found, we want to avoid large jumps
    if (dock_found)
    {
      // dist is squared
      return dist < (0.25*0.25);
    }

    // Not too far off from initial pose
    return dist < 1.0;
  }
};
typedef boost::shared_ptr<DockCandidate> DockCandidatePtr;

struct CompareCandidates
{
  bool operator()(DockCandidatePtr a, DockCandidatePtr b)
  {
    return (a->dist > b->dist);
  }
};

#endif  // FETCH_AUTO_DOCK_DOCK_CANDIDATE_H
