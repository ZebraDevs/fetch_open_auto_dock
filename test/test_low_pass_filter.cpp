/* Copyright 2015 Fetch Robotics Inc.
 * Author: Griswald Brooks
 */

// STL Includes.
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

// Third Party Includes.
#include <gtest/gtest.h>

// ROS Includes.
#include <tf/transform_datatypes.h>
#include <ros/package.h>
#include <ros/console.h>

// Fetch Includes.
#include <fetch_auto_dock/linear_pose_filter_2d.h>

const int NUMBER_OF_SAMPLES = 10;

/**
 * @brief Function to read a time history from a log file.
 * @param x_t     Vector of states.
 * @param fn      Name of the log file.
 */
void readHistoryFromFile(std::vector<float> & x_t, const char* fn )
{
  // Save history to file.
  std::fstream log;
  std::string path = ros::package::getPath("fetch_auto_dock");
  std::string abs_path = path + "/test/" + fn;
  log.open(abs_path.c_str(), std::fstream::in);

  // If we were able to open the file
  if (log.is_open())
  {
    std::string line;
    // Grab a line from the file.
    while (std::getline(log, line))
    {
      std::stringstream record(line);
      std::string token;
      // Grab the value from the record.
      std::getline(record, token);
      // Convert it to a number and add to history.
      x_t.push_back(atof(token.c_str()));
    }

    log.close();
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("test_low_pass_filter", abs_path << " could not be found." << std::endl);
  }
}

void printVector(const std::string& name, const std::vector<geometry_msgs::Pose> poses, size_t n = NUMBER_OF_SAMPLES)
{
  std::cout << name;
  std::cout << ".x = {";
  for (size_t i = 0; i < n; i++)
  {
    std::cout << poses[i].position.x << ", ";
  }
  std::cout << "\b\b}" << std::endl;
  std::cout << name;
  std::cout << ".y = {";
  for (size_t i = 0; i < n; i++)
  {
    std::cout << poses[i].position.y << ", ";
  }
  std::cout << "\b\b}" << std::endl;
  std::cout << name;
  std::cout << ".w = {";
  for (size_t i = 0; i < n; i++)
  {
    std::cout << tf::getYaw(poses[i].orientation) << ", ";
  }
  std::cout << "\b\b}" << std::endl;
}

TEST(TestLPF, low_pass_filter_uninitialized)
{
  // Random test poses.
  geometry_msgs::Pose p;
  std::vector<geometry_msgs::Pose> pn;
  float x_i[] = {0.447265, 0.677158, 0.490548, 0.896610, 0.948445, 0.748019, 0.050977, 0.457688, 0.448624, 0.810386};
  float y_i[] = {0.775992, 0.873013, 0.335330, 0.174261, 0.405399, 0.119830, 0.052253, 0.465377, 0.655518, 0.310589};
  float w_i[] = {0.991499, 0.539988, 0.447987, 0.544278, 0.601881, 0.976729, 0.402125, 0.419183, 0.074944, 0.571029};

  // Instantiate the poses.
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    p.position.x = x_i[i];
    p.position.y = y_i[i];
    p.position.z = 0;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = sin(w_i[i]/2.0);
    p.orientation.w = cos(w_i[i]/2.0);
    pn.push_back(p);
  }

  // Test coefficients.
  // First order, 0.2 rad/sec (normalized cutoff frequency).
  float b_fo_arr[] = {0.24524,  0.24524};
  float a_fo_arr[] = {1.00000, -0.50953};
  std::vector<float> b_fo(b_fo_arr, b_fo_arr + sizeof(b_fo_arr)/sizeof(float));
  std::vector<float> a_fo(a_fo_arr, a_fo_arr + sizeof(a_fo_arr)/sizeof(float));
  LinearPoseFilter2D lpf_fo(b_fo, a_fo);

  // Second order, 0.5 rad/sec (normalized cutoff frequency).
  float b_so_arr[] = {0.29289, 0.58579, 0.29289};
  float a_so_arr[] = {1.0000e+00, -1.3878e-16, 1.7157e-01};
  std::vector<float> b_so(b_so_arr, b_so_arr + sizeof(b_so_arr)/sizeof(float));
  std::vector<float> a_so(a_so_arr, a_so_arr + sizeof(a_so_arr)/sizeof(float));
  LinearPoseFilter2D lpf_so(b_so, a_so);

  // Third order, 0.7 rad/sec (normalized cutoff frequency).
  float b_to_arr[] = {0.37445, 1.12336, 1.12336, 0.37445};
  float a_to_arr[] = {1.00000, 1.16192, 0.69594, 0.13776};
  std::vector<float> b_to(b_to_arr, b_to_arr + sizeof(b_to_arr)/sizeof(float));
  std::vector<float> a_to(a_to_arr, a_to_arr + sizeof(a_to_arr)/sizeof(float));
  LinearPoseFilter2D lpf_to(b_to, a_to);

  // Test outputs.
  float x_fo_o[] = {0.10969, 0.33164, 0.45534, 0.57219, 0.74402, 0.79513, 0.60108, 0.43101, 0.44187, 0.53390};
  float y_fo_o[] = {0.19030, 0.50136, 0.55179, 0.40612, 0.34908, 0.30667, 0.19846, 0.22806, 0.39109, 0.43620};
  float w_fo_o[] = {0.24316, 0.49948, 0.49679, 0.49647, 0.53405, 0.65925, 0.67406, 0.54487, 0.39881, 0.36162};

  float x_so_o[] = {0.13100, 0.46034, 0.64887, 0.66932, 0.83536, 0.92245, 0.58758, 0.22474, 0.31363, 0.59565};
  float y_so_o[] = {0.22728, 0.71027, 0.79790, 0.38131, 0.18214, 0.25819, 0.17299, 0.15771, 0.45023, 0.58421};
  float w_so_o[] = {0.29040, 0.73897, 0.68811, 0.45321, 0.50827, 0.72031, 0.77902, 0.52083, 0.25163, 0.24457};


  float x_to_o[] = {0.16748, 0.56140, 0.67795, 0.61346, 0.90503, 0.96453, 0.42549, 0.13377, 0.43508, 0.68341};
  float y_to_o[] = {0.290572, 0.860998, 0.775358, 0.173090, 0.191825, 0.371442, 0.085910, 0.134627, 0.604432, 0.587207};
  float w_to_o[] = {0.37127, 0.88463, 0.60192, 0.31875, 0.63112, 0.78298, 0.73484, 0.44561, 0.17939, 0.29969};

  std::vector<geometry_msgs::Pose> p_fo_lpf, p_so_lpf, p_to_lpf;

  // Filter.
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    p_fo_lpf.push_back(lpf_fo.filter(pn[i]));
    p_so_lpf.push_back(lpf_so.filter(pn[i]));
    p_to_lpf.push_back(lpf_to.filter(pn[i]));
  }

  // Check.
  const float TOLERANCE = 1e-4;
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    EXPECT_NEAR(x_fo_o[i], p_fo_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_fo_o[i], p_fo_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_fo_o[i], tf::getYaw(p_fo_lpf[i].orientation), TOLERANCE);

    EXPECT_NEAR(x_so_o[i], p_so_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_so_o[i], p_so_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_so_o[i], tf::getYaw(p_so_lpf[i].orientation), TOLERANCE);

    EXPECT_NEAR(x_to_o[i], p_to_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_to_o[i], p_to_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_to_o[i], tf::getYaw(p_to_lpf[i].orientation), TOLERANCE);
  }
}

TEST(TestLPF, low_pass_filter_input_initalized)
{
  // Random test poses.
  geometry_msgs::Pose p;
  geometry_msgs::Pose origin;
  std::vector<geometry_msgs::Pose> pn;
  float x_i[] = {0.447265, 0.677158, 0.490548, 0.896610, 0.948445, 0.748019, 0.050977, 0.457688, 0.448624, 0.810386};
  float y_i[] = {0.775992, 0.873013, 0.335330, 0.174261, 0.405399, 0.119830, 0.052253, 0.465377, 0.655518, 0.310589};
  float w_i[] = {0.991499, 0.539988, 0.447987, 0.544278, 0.601881, 0.976729, 0.402125, 0.419183, 0.074944, 0.571029};

  // Initialize the origin.
  origin.position.x = 0;
  origin.position.y = 0;
  origin.position.z = 0;
  origin.orientation.x = 0;
  origin.orientation.y = 0;
  origin.orientation.z = 0;
  origin.orientation.w = 1;

  // Instantiate the poses.
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    p.position.x = x_i[i];
    p.position.y = y_i[i];
    p.position.z = 0;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = sin(w_i[i]/2.0);
    p.orientation.w = cos(w_i[i]/2.0);
    pn.push_back(p);
  }

  // Test coefficients.
  // First order, 0.2 rad/sec (normalized cutoff frequency).
  float b_fo_arr[] = {0.24524,  0.24524};
  float a_fo_arr[] = {1.00000, -0.50953};
  std::vector<float> b_fo(b_fo_arr, b_fo_arr + sizeof(b_fo_arr)/sizeof(float));
  std::vector<float> a_fo(a_fo_arr, a_fo_arr + sizeof(a_fo_arr)/sizeof(float));
  LinearPoseFilter2D lpf_fo(b_fo, a_fo);
  lpf_fo.setFilterState(*pn.begin(), origin);

  // // Second order, 0.5 rad/sec (normalized cutoff frequency).
  float b_so_arr[] = {0.29289, 0.58579, 0.29289};
  float a_so_arr[] = {1.0000e+00, -1.3878e-16, 1.7157e-01};
  std::vector<float> b_so(b_so_arr, b_so_arr + sizeof(b_so_arr)/sizeof(float));
  std::vector<float> a_so(a_so_arr, a_so_arr + sizeof(a_so_arr)/sizeof(float));
  LinearPoseFilter2D lpf_so(b_so, a_so);
  lpf_so.setFilterState(*pn.begin(), origin);

  // Third order, 0.7 rad/sec (normalized cutoff frequency).
  float b_to_arr[] = {0.37445, 1.12336, 1.12336, 0.37445};
  float a_to_arr[] = {1.00000, 1.16192, 0.69594, 0.13776};
  std::vector<float> b_to(b_to_arr, b_to_arr + sizeof(b_to_arr)/sizeof(float));
  std::vector<float> a_to(a_to_arr, a_to_arr + sizeof(a_to_arr)/sizeof(float));
  LinearPoseFilter2D lpf_to(b_to, a_to);
  lpf_to.setFilterState(*pn.begin(), origin);

  // Test outputs.
  float x_fo_o[] = {0.21937, 0.38753, 0.48383, 0.58671, 0.75143, 0.79892, 0.60302, 0.432, 0.44238, 0.53417};
  float y_fo_o[] = {0.38061, 0.59833, 0.6012, 0.4313, 0.36192, 0.31322, 0.20179, 0.22976, 0.39196, 0.43664};
  float w_fo_o[] = {0.48631, 0.62337, 0.55992, 0.52864, 0.55044, 0.6676, 0.67831, 0.54704, 0.39991, 0.36219};

  float x_so_o[] = {0.524, 0.59134, 0.58145, 0.64684, 0.84693, 0.92631, 0.58559, 0.22408, 0.31397, 0.59576};
  float y_so_o[] = {0.90913, 0.93755, 0.68092, 0.34231, 0.20221, 0.26488, 0.16954, 0.15656, 0.45082, 0.58441};
  float w_so_o[] = {1.1616, 1.0294, 0.53863, 0.40339, 0.53392, 0.72885, 0.77462, 0.51936, 0.25238, 0.24482};

  float x_to_o[] = {1.3398, -0.13086, 0.8339, 0.75254, 0.73028, 1.0493, 0.42944, 0.094246, 0.46657, 0.67378};
  float y_to_o[] = {2.3246, -0.34007, 1.0459, 0.41438, -0.11137, 0.51854, 0.092763, 0.066061, 0.65906, 0.5705};
  float w_to_o[] = {2.9702, -0.65, 0.94762, 0.62705, 0.24371, 0.97092, 0.7436, 0.358, 0.2492, 0.27835};

  std::vector<geometry_msgs::Pose> p_fo_lpf, p_so_lpf, p_to_lpf;

  // Filter.
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    p_fo_lpf.push_back(lpf_fo.filter(pn[i]));
    p_so_lpf.push_back(lpf_so.filter(pn[i]));
    p_to_lpf.push_back(lpf_to.filter(pn[i]));
  }

  // Check.
  const float TOLERANCE = 1e-4;
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    EXPECT_NEAR(x_fo_o[i], p_fo_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_fo_o[i], p_fo_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_fo_o[i], tf::getYaw(p_fo_lpf[i].orientation), TOLERANCE);

    EXPECT_NEAR(x_so_o[i], p_so_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_so_o[i], p_so_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_so_o[i], tf::getYaw(p_so_lpf[i].orientation), TOLERANCE);

    EXPECT_NEAR(x_to_o[i], p_to_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_to_o[i], p_to_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_to_o[i], tf::getYaw(p_to_lpf[i].orientation), TOLERANCE);
  }

  // Print.
  // printVector("p_fo_lpf", p_fo_lpf);
  // printVector("p_so_lpf", p_so_lpf);
  // printVector("p_to_lpf", p_to_lpf);
}

TEST(TestLPF, low_pass_filter_io_initalized)
{
  // Random test poses.
  geometry_msgs::Pose p;
  geometry_msgs::Pose origin;
  std::vector<geometry_msgs::Pose> pn;
  float x_i[] = {0.447265, 0.677158, 0.490548, 0.896610, 0.948445, 0.748019, 0.050977, 0.457688, 0.448624, 0.810386};
  float y_i[] = {0.775992, 0.873013, 0.335330, 0.174261, 0.405399, 0.119830, 0.052253, 0.465377, 0.655518, 0.310589};
  float w_i[] = {0.991499, 0.539988, 0.447987, 0.544278, 0.601881, 0.976729, 0.402125, 0.419183, 0.074944, 0.571029};

  // Initialize the origin.
  origin.position.x = 0;
  origin.position.y = 0;
  origin.position.z = 0;
  origin.orientation.x = 0;
  origin.orientation.y = 0;
  origin.orientation.z = 0;
  origin.orientation.w = 1;

  // Instantiate the poses.
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    p.position.x = x_i[i];
    p.position.y = y_i[i];
    p.position.z = 0;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = sin(w_i[i]/2.0);
    p.orientation.w = cos(w_i[i]/2.0);
    pn.push_back(p);
  }

  // Test coefficients.
  // First order, 0.2 rad/sec (normalized cutoff frequency).
  float b_fo_arr[] = {0.24524,  0.24524};
  float a_fo_arr[] = {1.00000, -0.50953};
  std::vector<float> b_fo(b_fo_arr, b_fo_arr + sizeof(b_fo_arr)/sizeof(float));
  std::vector<float> a_fo(a_fo_arr, a_fo_arr + sizeof(a_fo_arr)/sizeof(float));
  LinearPoseFilter2D lpf_fo(b_fo, a_fo);
  lpf_fo.setFilterState(*pn.begin(), *pn.begin());

  // Second order, 0.5 rad/sec (normalized cutoff frequency).
  float b_so_arr[] = {0.29289, 0.58579, 0.29289};
  float a_so_arr[] = {1.0000e+00, -1.3878e-16, 1.7157e-01};
  std::vector<float> b_so(b_so_arr, b_so_arr + sizeof(b_so_arr)/sizeof(float));
  std::vector<float> a_so(a_so_arr, a_so_arr + sizeof(a_so_arr)/sizeof(float));
  LinearPoseFilter2D lpf_so(b_so, a_so);
  lpf_so.setFilterState(*pn.begin(), *pn.begin());

  // Third order, 0.7 rad/sec (normalized cutoff frequency).
  float b_to_arr[] = {0.37445, 1.12336, 1.12336, 0.37445};
  float a_to_arr[] = {1.00000, 1.16192, 0.69594, 0.13776};
  std::vector<float> b_to(b_to_arr, b_to_arr + sizeof(b_to_arr)/sizeof(float));
  std::vector<float> a_to(a_to_arr, a_to_arr + sizeof(a_to_arr)/sizeof(float));
  LinearPoseFilter2D lpf_to(b_to, a_to);
  lpf_to.setFilterState(*pn.begin(), *pn.begin());

  // Test outputs.
  float x_fo_o[] = {0.44727, 0.50365, 0.54299, 0.61686, 0.76679, 0.80674, 0.60701, 0.43403, 0.44342, 0.53469};
  float y_fo_o[] = {0.776, 0.7998, 0.70385, 0.48361, 0.38857, 0.32679, 0.20871, 0.23329, 0.39376, 0.43756};
  float w_fo_o[] = {0.99151, 0.88079, 0.69108, 0.59547, 0.58449, 0.68495, 0.68716, 0.55154, 0.40221, 0.36336};

  float x_so_o[] = {0.44727, 0.5146, 0.59461, 0.66001, 0.84467, 0.92405, 0.58598, 0.22446, 0.3139, 0.59569};
  float y_so_o[] = {0.77599, 0.80441, 0.70376, 0.36516, 0.19829, 0.26097, 0.17022, 0.15724, 0.45071, 0.58429};
  float w_so_o[] = {0.9915, 0.85926, 0.56782, 0.43257, 0.52891, 0.72385, 0.77548, 0.52022, 0.25223, 0.24467};

  float x_to_o[] = {0.44726, 0.53335, 0.6217, 0.65981, 0.8942, 0.95262, 0.44049, 0.12612, 0.43517, 0.68657};
  float y_to_o[] = {0.77599, 0.81232, 0.67776, 0.25349, 0.17303, 0.35077, 0.11193, 0.12136, 0.60458, 0.59268};
  float w_to_o[] = {0.9915, 0.82243, 0.47722, 0.42149, 0.6071, 0.75656, 0.76809, 0.42867, 0.17958, 0.30669};


  std::vector<geometry_msgs::Pose> p_fo_lpf, p_so_lpf, p_to_lpf;

  // Filter.
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    p_fo_lpf.push_back(lpf_fo.filter(pn[i]));
    p_so_lpf.push_back(lpf_so.filter(pn[i]));
    p_to_lpf.push_back(lpf_to.filter(pn[i]));
  }

  // Check.
  const float TOLERANCE = 1e-4;
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    EXPECT_NEAR(x_fo_o[i], p_fo_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_fo_o[i], p_fo_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_fo_o[i], tf::getYaw(p_fo_lpf[i].orientation), TOLERANCE);

    EXPECT_NEAR(x_so_o[i], p_so_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_so_o[i], p_so_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_so_o[i], tf::getYaw(p_so_lpf[i].orientation), TOLERANCE);

    EXPECT_NEAR(x_to_o[i], p_to_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_to_o[i], p_to_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_to_o[i], tf::getYaw(p_to_lpf[i].orientation), TOLERANCE);
  }
}

TEST(TestLPF, low_pass_filter_uninit_pi_discontinuity)
{
  // Random test poses.
  geometry_msgs::Pose p;
  std::vector<geometry_msgs::Pose> pn;
  float x_i[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float y_i[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float w_i[] = {-3.1088, 3.1755, -3.1262, -3.1243, 3.1567, -3.1140, 3.1522, 3.1654, -3.1095, 3.1543};

  // Instantiate the poses.
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    p.position.x = x_i[i];
    p.position.y = y_i[i];
    p.position.z = 0;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = sin(w_i[i]/2.0);
    p.orientation.w = cos(w_i[i]/2.0);
    pn.push_back(p);
  }

  // Test coefficients.
  // First order, 0.2 rad/sec (normalized cutoff frequency).
  float b_fo_arr[] = {0.24524,  0.24524};
  float a_fo_arr[] = {1.00000, -0.50953};
  std::vector<float> b_fo(b_fo_arr, b_fo_arr + sizeof(b_fo_arr)/sizeof(float));
  std::vector<float> a_fo(a_fo_arr, a_fo_arr + sizeof(a_fo_arr)/sizeof(float));
  LinearPoseFilter2D lpf_fo(b_fo, a_fo);

  // Second order, 0.5 rad/sec (normalized cutoff frequency).
  float b_so_arr[] = {0.29289, 0.58579, 0.29289};
  float a_so_arr[] = {1.0000e+00, -1.3878e-16, 1.7157e-01};
  std::vector<float> b_so(b_so_arr, b_so_arr + sizeof(b_so_arr)/sizeof(float));
  std::vector<float> a_so(a_so_arr, a_so_arr + sizeof(a_so_arr)/sizeof(float));
  LinearPoseFilter2D lpf_so(b_so, a_so);

  // Third order, 0.7 rad/sec (normalized cutoff frequency).
  float b_to_arr[] = {0.37445, 1.12336, 1.12336, 0.37445};
  float a_to_arr[] = {1.00000, 1.16192, 0.69594, 0.13776};
  std::vector<float> b_to(b_to_arr, b_to_arr + sizeof(b_to_arr)/sizeof(float));
  std::vector<float> a_to(a_to_arr, a_to_arr + sizeof(a_to_arr)/sizeof(float));
  LinearPoseFilter2D lpf_to(b_to, a_to);

  // Test outputs.
  float x_fo_o[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float y_fo_o[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float w_fo_o[] = {-0.762402, -1.913, -2.50353, -2.8085, -2.96395, -3.04064, -3.08082, -3.10222, -3.10785, -3.11345};

  float x_so_o[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float y_so_o[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float w_so_o[] = {-0.910536, -2.73131, 2.79279, 3.09521, -3.06268, -3.11164, -3.13144, -3.12546, -3.11688, -3.11487};

  float x_to_o[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float y_to_o[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  float w_to_o[] = {-1.16409, 2.97979, 2.77765, -2.8044, 3.08044, 3.08061, -3.01621, 3.10426, -3.11606, -3.09102};

  std::vector<geometry_msgs::Pose> p_fo_lpf, p_so_lpf, p_to_lpf;

  // Filter.
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    p_fo_lpf.push_back(lpf_fo.filter(pn[i]));
    p_so_lpf.push_back(lpf_so.filter(pn[i]));
    p_to_lpf.push_back(lpf_to.filter(pn[i]));
  }

  // Check.
  const float TOLERANCE = 1e-4;
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
  {
    EXPECT_NEAR(x_fo_o[i], p_fo_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_fo_o[i], p_fo_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_fo_o[i], tf::getYaw(p_fo_lpf[i].orientation), TOLERANCE);

    EXPECT_NEAR(x_so_o[i], p_so_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_so_o[i], p_so_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_so_o[i], tf::getYaw(p_so_lpf[i].orientation), TOLERANCE);

    EXPECT_NEAR(x_to_o[i], p_to_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_to_o[i], p_to_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_to_o[i], tf::getYaw(p_to_lpf[i].orientation), TOLERANCE);
  }
}

TEST(TestLPF, low_pass_filter_io_init_pi_discontinuity)
{
  // Random test poses.
  geometry_msgs::Pose p;
  std::vector<geometry_msgs::Pose> pn;
  std::vector<float> x_i(1000,0);
  std::vector<float> y_i(1000,0);
  std::vector<float> w_i;
  readHistoryFromFile(w_i, "test_angles1.txt");

  // Instantiate the poses.
  for (int i = 0; i < 1000; i++)
  {
    p.position.x = x_i[i];
    p.position.y = y_i[i];
    p.position.z = 0;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = sin(w_i[i]/2.0);
    p.orientation.w = cos(w_i[i]/2.0);
    pn.push_back(p);
  }

  // Test coefficients.
  // First order, 0.2 rad/sec (normalized cutoff frequency).
  float b_fo_arr[] = {0.24524,  0.24524};
  float a_fo_arr[] = {1.00000, -0.50953};
  std::vector<float> b_fo(b_fo_arr, b_fo_arr + sizeof(b_fo_arr)/sizeof(float));
  std::vector<float> a_fo(a_fo_arr, a_fo_arr + sizeof(a_fo_arr)/sizeof(float));
  LinearPoseFilter2D lpf_fo(b_fo, a_fo);
  lpf_fo.setFilterState(*pn.begin(), *pn.begin());

  // Second order, 0.5 rad/sec (normalized cutoff frequency).
  float b_so_arr[] = {0.29289, 0.58579, 0.29289};
  float a_so_arr[] = {1.0000e+00, -1.3878e-16, 1.7157e-01};
  std::vector<float> b_so(b_so_arr, b_so_arr + sizeof(b_so_arr)/sizeof(float));
  std::vector<float> a_so(a_so_arr, a_so_arr + sizeof(a_so_arr)/sizeof(float));
  LinearPoseFilter2D lpf_so(b_so, a_so);
  lpf_so.setFilterState(*pn.begin(), *pn.begin());

  // Third order, 0.7 rad/sec (normalized cutoff frequency).
  float b_to_arr[] = {0.37445, 1.12336, 1.12336, 0.37445};
  float a_to_arr[] = {1.00000, 1.16192, 0.69594, 0.13776};
  std::vector<float> b_to(b_to_arr, b_to_arr + sizeof(b_to_arr)/sizeof(float));
  std::vector<float> a_to(a_to_arr, a_to_arr + sizeof(a_to_arr)/sizeof(float));
  LinearPoseFilter2D lpf_to(b_to, a_to);
  lpf_to.setFilterState(*pn.begin(), *pn.begin());

  // Test outputs.
  std::vector<float> x_fo_o(1000, 0);
  std::vector<float> y_fo_o(1000, 0);
  std::vector<float> w_fo_o;
  readHistoryFromFile(w_fo_o, "test_out_pidisc_fo1.txt");

  std::vector<float> x_so_o(1000, 0);
  std::vector<float> y_so_o(1000, 0);
  std::vector<float> w_so_o;
  readHistoryFromFile(w_so_o, "test_out_pidisc_so1.txt");

  std::vector<float> x_to_o(1000, 0);
  std::vector<float> y_to_o(1000, 0);
  std::vector<float> w_to_o;
  readHistoryFromFile(w_to_o, "test_out_pidisc_to1.txt");

  std::vector<geometry_msgs::Pose> p_fo_lpf, p_so_lpf, p_to_lpf;

  // Filter.
  for (int i = 0; i < 1000; i++)
  {
    p_fo_lpf.push_back(lpf_fo.filter(pn[i]));
    p_so_lpf.push_back(lpf_so.filter(pn[i]));
    p_to_lpf.push_back(lpf_to.filter(pn[i]));
  }

  // Check.
  const float TOLERANCE = 1e-4;
  for (int i = 0; i < 1000; i++)
  {
    EXPECT_NEAR(x_fo_o[i], p_fo_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_fo_o[i], p_fo_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_fo_o[i], tf::getYaw(p_fo_lpf[i].orientation), TOLERANCE);

    EXPECT_NEAR(x_so_o[i], p_so_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_so_o[i], p_so_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_so_o[i], tf::getYaw(p_so_lpf[i].orientation), TOLERANCE);

    EXPECT_NEAR(x_to_o[i], p_to_lpf[i].position.x, TOLERANCE);
    EXPECT_NEAR(y_to_o[i], p_to_lpf[i].position.y, TOLERANCE);
    EXPECT_NEAR(w_to_o[i], tf::getYaw(p_to_lpf[i].orientation), TOLERANCE);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}