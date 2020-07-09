//
// Created by rakesh on 17/08/18.
//

#include <icp_slam/utils.h>

namespace icp_slam
{
namespace utils
{

cv::Mat laserScanToPointMat(const sensor_msgs::LaserScanConstPtr &scan)
{
  // TODO
  int n_rows = scan->ranges.size();
  float angle_begin = scan->angle_min;
  float angle_increment = scan->angle_increment;

  cv::Mat scan_mat(n_rows, 2, CV_32F, cv::Scalar::all(0));
  for (int i = 0; i < n_rows; i++)
  {
      polarToCartesian(scan->ranges[i],
                       angle_begin + i*angle_increment,
                       scan_mat.at<float>(i, 0),
                       scan_mat.at<float>(i, 1));
  }
  return scan_mat;

}

cv::Mat transformPointMat(tf::Transform transform, cv::Mat &point_mat)
{
  assert(point_mat.data);
  assert(!point_mat.empty());

  cv::Mat point_mat_homogeneous(3, point_mat.rows, CV_32F, cv::Scalar(1.0f));

  cv::Mat(point_mat.t()).copyTo(point_mat_homogeneous.rowRange(0, 2));

  auto T = transformToMatrix(transform);
  cv::Mat transformed_point_mat =  T * point_mat_homogeneous;
  return cv::Mat(transformed_point_mat.t()).colRange(0, 2);
}

} // namespace utils
} // namespace icp_slam
