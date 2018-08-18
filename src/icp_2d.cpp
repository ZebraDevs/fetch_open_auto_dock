/*
 * Copyright 2015 Fetch Robotics Inc.
 * Author: Michael Ferguson
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

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Eigenvalues>

namespace icp_2d
{

double thetaFromQuaternion(const geometry_msgs::Quaternion& q)
{
  // If all zeros, is invalid, assume no rotation
  if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0)
  {
    return 0.0;
  }

  // Compute theta (this only works if q is in 2d)
  return 2.0*atan2(q.z, q.w);
}

std::vector<geometry_msgs::Point>
transform(const std::vector<geometry_msgs::Point>& points,
          double x,
          double y,
          double theta)
{
  std::vector<geometry_msgs::Point> points_t;

  double cos_th = cos(theta);
  double sin_th = sin(theta);

  for (size_t i = 0; i < points.size(); i++)
  {
    geometry_msgs::Point pt = points[i];
    geometry_msgs::Point pt_t;

    // Rotate then Translate
    pt_t.x = cos_th * pt.x - sin_th * pt.y + x;
    pt_t.y = sin_th * pt.x + cos_th * pt.y + y;

    points_t.push_back(pt_t);
  }

  return points_t;
}

geometry_msgs::Point getCentroid(const std::vector<geometry_msgs::Point> points)
{
  geometry_msgs::Point pt;
  for (size_t i = 0; i < points.size(); i++)
  {
    pt.x += points[i].x;
    pt.y += points[i].y;
  }
  pt.x /= points.size();
  pt.y /= points.size();
  return pt;
}

// Computes correspondences for alignment
// correspondences will be equal in size to source, and contain the
// closest point in target that corresponds to that in source.
bool computeCorrespondences(const std::vector<geometry_msgs::Point>& source,
                            const std::vector<geometry_msgs::Point>& target,
                            std::vector<geometry_msgs::Point>& correspondences)
{
  correspondences.clear();
  std::vector<size_t> c_num;

  for (size_t i = 0; i < source.size(); i++)
  {
    double d = 1.0;
    size_t best = 0;

    for (size_t j = 0; j < target.size(); j++)
    {
      double dx = source[i].x - target[j].x;
      double dy = source[i].y - target[j].y;
      double dist = dx * dx + dy * dy;
      if (dist < d)
      {
        best = j;
        d = dist;
      }
    }

    if (d >= 1.0)
    {
      // No correspondence found!
      return false;
    }
    correspondences.push_back(target[best]);
    c_num.push_back(best);
  }

  // All points have correspondence
  return true;
}

bool alignPCA(const std::vector<geometry_msgs::Point> source,
              const std::vector<geometry_msgs::Point> target,
              geometry_msgs::Transform& t)
{
  // Get initial rotation angle
  double theta = thetaFromQuaternion(t.rotation);

  // Transform source based on initial transfrom
  std::vector<geometry_msgs::Point> source_t = transform(source,
                                                         t.translation.x,
                                                         t.translation.y,
                                                         theta);

  // Get centroid of source and target
  geometry_msgs::Point cs = getCentroid(source_t);
  geometry_msgs::Point ct = getCentroid(target);

  // Update translation
  t.translation.x += ct.x - cs.x;
  t.translation.y += ct.y - cs.y;

  // Compute P for source
  Eigen::MatrixXf Ps(2, source_t.size());
  for (size_t i = 0; i < source_t.size(); i++)
  {
    Ps(0, i) = source_t[i].x - cs.x;
    Ps(1, i) = source_t[i].y - cs.y;
  }

  // Compute M for source (covariance matrix)
  Eigen::MatrixXf Ms = Ps * Ps.transpose();

  // Get EigenVectors of Covariance Matrix
  Eigen::EigenSolver<Eigen::MatrixXf> solver_s(Ms);
  Eigen::MatrixXf A = solver_s.eigenvectors().real();
  Eigen::MatrixXf eig_of_Ms = solver_s.eigenvalues().real();
  // Sort the eigen vectors for the ideal dock.
  // There are only two eigenvalues. If they come out not sorted,
  // swap the vectors, and make sure the axes are still right handed.
  if (eig_of_Ms(0, 0) < eig_of_Ms(1, 0))
  {
    A.col(0).swap(A.col(1));
    A.col(1) = -A.col(1);
  }
  // Compute P for target
  Eigen::MatrixXf Pt(2, target.size());
  for (size_t i = 0; i < target.size(); i++)
  {
    Pt(0, i) = target[i].x - ct.x;
    Pt(1, i) = target[i].y - ct.y;
  }

  // Compute M for target (covariance matrix)
  Eigen::MatrixXf Mt = Pt * Pt.transpose();

  // Get EigenVectors of Covariance Matrix
  Eigen::EigenSolver<Eigen::MatrixXf> solver_t(Mt);
  Eigen::MatrixXf B = solver_t.eigenvectors().real();
  Eigen::MatrixXf eig_of_Mt = solver_t.eigenvalues().real();
  // Sort the eigen vectors for the candidate dock.
  // There are only two eigenvalues. If they come out not sorted,
  // swap the vectors.
  if (eig_of_Mt(0, 0) < eig_of_Mt(1, 0))
  {
    B.col(0).swap(B.col(1));
    B.col(1) = -B.col(1);
  }

  // Get rotation
  Eigen::MatrixXf R = B * A.transpose();

  // Update rotation
  theta += atan2(R(1, 0), R(0, 0));
  t.rotation.z = sin(theta / 2.0);
  t.rotation.w = cos(theta / 2.0);
  return true;
}

bool alignSVD(const std::vector<geometry_msgs::Point> source,
              const std::vector<geometry_msgs::Point> target,
              geometry_msgs::Transform& t)
{
  double theta = thetaFromQuaternion(t.rotation);

  // Transform source based on initial transform
  std::vector<geometry_msgs::Point> source_t = transform(source,
                                                         t.translation.x,
                                                         t.translation.y,
                                                         theta);

  // Get Correspondences
  //  Technically this should be source_t -> target, however
  //  to keep the frame transforms simpler, we actually call this
  //  function with the ideal cloud as source, and the sensor
  //  data as target. The correspondences then need to be setup
  //  so that we map each point in the target onto a point in the
  //  source, since the laser may not see the whole dock. 
  std::vector<geometry_msgs::Point> corr;
  if (!computeCorrespondences(target, source_t, corr))
  {
    return false;
  }

  // Get centroid of source_t and corr
  geometry_msgs::Point cs = getCentroid(corr);
  geometry_msgs::Point ct = getCentroid(target);

  // Compute P
  Eigen::MatrixXf P(2, corr.size());
  for (size_t i = 0; i < corr.size(); i++)
  {
    P(0, i) = corr[i].x - cs.x;
    P(1, i) = corr[i].y - cs.y;
  }

  // Compute Q
  Eigen::MatrixXf Q(2, target.size());
  for (size_t i = 0; i < target.size(); i++)
  {
    Q(0, i) = target[i].x - ct.x;
    Q(1, i) = target[i].y - ct.y;
  }

  // Compute M
  Eigen::MatrixXf M = P * Q.transpose();

  // Find SVD of M
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Get R (rotation matrix from source to target)
  Eigen::MatrixXf R = svd.matrixV() * svd.matrixU().transpose();

  // Compute final transformation
  t.translation.x += (ct.x - cs.x);
  t.translation.y += (ct.y - cs.y);

  // Update rotation of transform
  theta += atan2(R(1, 0), R(0, 0));
  t.rotation.z = sin(theta / 2.0);
  t.rotation.w = cos(theta / 2.0);
  return true;
}

double getRMSD(const std::vector<geometry_msgs::Point> source,
               const std::vector<geometry_msgs::Point> target)
{
  // Compute correspondences
  std::vector<geometry_msgs::Point> corr;
  if (!computeCorrespondences(source, target, corr))
  {
    // No correspondences, return massive number
    // (we considered using std::numeric_limits<double>::max() here, but
    //  that could lead to under/overflow later on when comparing RMSD)
    return 10e6;
  }

  // Get Root Mean Squared Distance (RMSD)
  double rmsd = 0.0;
  for (size_t i = 0; i < source.size(); i++)
  {
    double dx = source[i].x - corr[i].x;
    double dy = source[i].y - corr[i].y;
    rmsd += dx * dx + dy * dy;
  }
  rmsd /= corr.size();
  rmsd = sqrt(rmsd);

  return rmsd;
}

double alignICP(const std::vector<geometry_msgs::Point> source,
                const std::vector<geometry_msgs::Point> target,
                geometry_msgs::Transform & t,
                size_t max_iterations,
                double min_delta_rmsd)
{
  // Initial alignment with PCA
  alignPCA(source, target, t);

  // Previous RMSD
  double prev_rmsd = -1.0;

  // Iteratively refine with SVD
  for (size_t iteration = 0; iteration < max_iterations; iteration++)
  {
    // Perform SVD
    if (!alignSVD(source, target, t))
    {
      // SVD failed
      return -1.0;
    }

    // Transform source to target
    std::vector<geometry_msgs::Point>
    source_t = transform(source,
                         t.translation.x,
                         t.translation.y,
                         thetaFromQuaternion(t.rotation));

    // As above, this should technically be the other way around,
    // However, at this phase we want to check the
    double rmsd = getRMSD(target, source_t);

    // Check termination condition
    if (prev_rmsd > 0.0)
    {
      if (fabs(prev_rmsd - rmsd) < min_delta_rmsd)
      {
        // Terminate on RMSD, return based on ideal -> observed
        return getRMSD(source_t, target);
      }
    }
    prev_rmsd = rmsd;
  }

  // Transform source to target one last time
  std::vector<geometry_msgs::Point>
  source_t = transform(source,
                       t.translation.x,
                       t.translation.y,
                       thetaFromQuaternion(t.rotation));

  // Return RMSD based on ideal -> observed
  return getRMSD(source_t, target);
}

}  // namespace icp_2d
