//
// Created by rakesh on 13/08/18.
//
#include <cmath>
#include <map>
#include <numeric>
#include <chrono>

#include <boost/atomic/atomic.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/utils.h>
#include <icp_slam/config.h>

#include <cstdlib>

#define TIME_DIFF(tic, toc) ((std::chrono::duration<double, std::milli>((toc) - (tic))).count())

namespace icp_slam
{

ICPSlam::ICPSlam(tfScalar max_keyframes_distance, tfScalar max_keyframes_angle, double max_keyframes_time)
  : max_keyframes_distance_(max_keyframes_distance),
    max_keyframes_angle_(max_keyframes_angle),
    max_keyframes_time_(max_keyframes_time),
    last_kf_laser_scan_(new sensor_msgs::LaserScan()),
    is_tracker_running_(false)
{
  last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
}

bool ICPSlam::track(const sensor_msgs::LaserScanConstPtr &laser_scan,
                    const tf::StampedTransform &current_frame_tf_odom_laser,
                    tf::StampedTransform &tf_map_laser)
{
  // boost::mutex laser_scan_mutex;
  // boost::mutex::scoped_lock lock(laser_scan_mutex);
  if (is_tracker_running_)
  {
    ROS_WARN_THROTTLE(1.0, "Couldn't track frame, tracker already running");
    return false;
  }

  // TODO: find the pose of laser in map frame

  // initialize the very first keyframe
  if (last_kf_tf_odom_laser_.stamp_ == ros::Time(0))
  {
    *last_kf_laser_scan_ = *laser_scan;
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    last_kf_tf_map_laser_ = current_frame_tf_odom_laser;
    // return true;
  }

  // check if we need a new key frame & update tf_map_laser
  bool is_new_keyframe = ICPSlam::isCreateKeyframe(current_frame_tf_odom_laser, last_kf_tf_odom_laser_);
  if (is_new_keyframe)
  {
    // if a new keyframe is created, run ICP
    is_tracker_running_ = true;

    // Find initial transform from keyframe1 to keyframe2 wrt odom
    // tf_1_2 = tf_odom_1.inverse()*tf_odom_2;
    tf::Transform tf_1_2 = last_kf_tf_odom_laser_.inverse()*current_frame_tf_odom_laser;

    // Run ICP Registeration
    tf::Transform tf_1_2_refined = ICPSlam::icpRegister(last_kf_laser_scan_, laser_scan, tf_1_2);
    // tf::Transform tf_1_2_refined = ICPSlam::icpRegistration(last_kf_laser_scan_, laser_scan, tf_1_2);
    // ICPRegisteration function has unknown linking error....

    // update tf_map_laser <= tf_map_2 = tf_map_1 * tf_1_2_refined
    tf::Transform tf_map_2 = last_kf_tf_map_laser_ * tf_1_2_refined;
    tf_map_laser.setOrigin(tf_map_2.getOrigin());
    tf_map_laser.setRotation(tf_map_2.getRotation());

    // update the last keyframe
    *last_kf_laser_scan_ = *laser_scan;
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    last_kf_tf_map_laser_ = tf_map_laser;

    is_tracker_running_ = false;
    return true;
  }
  else
  {
    // if not a keyframe, obtain the laser pose in map frame based on odometry update
    tf::Transform tf_1_2 = last_kf_tf_odom_laser_.inverse()*current_frame_tf_odom_laser;
    tf::Transform tf_map_2 = last_kf_tf_map_laser_ * tf_1_2;
    tf_map_laser.setOrigin(tf_map_2.getOrigin());
    tf_map_laser.setRotation(tf_map_2.getRotation());

    // return false;
  }
}


bool ICPSlam::isCreateKeyframe(const tf::StampedTransform &current_frame_tf, const tf::StampedTransform &last_kf_tf) const
{
  assert(current_frame_tf.frame_id_ == last_kf_tf.frame_id_);
  assert(current_frame_tf.child_frame_id_ == last_kf_tf.child_frame_id_);

  // TODO: check whether you want to create keyframe (based on max_keyframes_distance_, max_keyframes_angle_, max_keyframes_time_)
  // cv::Mat current_frame, last_frame;
  // current_frame = utils::transformToMatrix(current_frame_tf);
  // last_frame = utils::transformToMatrix(last_kf_tf);
  // ROS_INFO_STREAM(tf:getYaw());

  float diff_dist = utils::euclideanDistance(current_frame_tf.getOrigin().getX(),
                                             current_frame_tf.getOrigin().getY(),
                                             last_kf_tf.getOrigin().getX(),
                                             last_kf_tf.getOrigin().getY());
  float diff_angle = std::abs(utils::radianToDegree(tf::getYaw(current_frame_tf.getRotation()) - tf::getYaw(last_kf_tf.getRotation())));
  float diff_time = current_frame_tf.stamp_.toSec() - last_kf_tf_odom_laser_.stamp_.toSec();

  bool isCreateKeyframe = false;
  // ROS_WARN("%f,   %f", diff_dist, max_keyframes_distance_);
  isCreateKeyframe |= (diff_dist >= max_keyframes_distance_); // Distance Difference
  isCreateKeyframe |= (diff_angle >= max_keyframes_angle_); // Angle Difference
  isCreateKeyframe |= (diff_time >= max_keyframes_time_); // Time Difference

  return isCreateKeyframe;
}


tf::Transform ICPSlam::icpRegister(const sensor_msgs::LaserScanConstPtr &laser_scan1,
                                   const sensor_msgs::LaserScanConstPtr &laser_scan2,
                                   const tf::Transform &T_1_2)
{
  // init variables: transform 
  tf::Transform tf_1_2_est = T_1_2;
  tf::Transform tf_1_2_rfn = tf_1_2_est; // initialize

  // init variables: laser_scans (key_last, key_curr) --> point cloud
  auto point_mat1 = utils::laserScanToPointMat(laser_scan1);
  auto point_mat2 = utils::laserScanToPointMat(laser_scan2);

  // for viz ========
  std::string image_name = std::string("/tmp/icp_laser_") + "no_Corr" + ".png";
  ICPSlam::vizClosestPoints(point_mat1, point_mat2, tf_1_2_est, image_name);
  // end for viz======

  //---------------------------------------------------------
  // ICP Iteration Start
  bool is_converged = false;
  static bool is_visualized_ = false;
  for (int icp_itr=0; icp_itr < MAX_ICP_ITR; icp_itr++)
  {
    if (DEBUG) {ROS_INFO("[%d]translation: (%f, %f) , rotation: %f",
                icp_itr, tf_1_2_est.getOrigin().getX(),
                tf_1_2_est.getOrigin().getY(),
                tf::getYaw(tf_1_2_est.getRotation()));}

    // 1. Apply transform for the points in keyframe2: keyframe2 -> keyframe1
    auto point_mat2_inKey1 = utils::transformPointMat(tf_1_2_est, point_mat2);

    // 2. Find closet pointset
    std::vector<int> c_indices;
    std::vector<float> c_distances;
    ICPSlam::closestPoints(point_mat1, point_mat2_inKey1, c_indices, c_distances);

    // 3. Reject outliers
    float quartile_1(0), quartile_2(0), quartile_3(0);
    utils::findQuartiles(c_distances, quartile_1, quartile_2, quartile_3);

    std::vector<int> c_mat1_indices(point_mat1.size().height);
    std::iota(c_mat1_indices.begin(), c_mat1_indices.end(), 0);

    for (int i = c_distances.size()-1; i >= 0; i--)
    {
      // ignore over Q3 distance
      if(c_distances[i] > quartile_3)
      {
        c_mat1_indices.erase(c_mat1_indices.begin() + i);
        c_distances.erase(c_distances.begin() + i);
        c_indices.erase(c_indices.begin() + i);
      }
    }

    // 4. Reorder corresponding datasets
    cv::Mat point_mat1_corr(c_mat1_indices.size(), 2, CV_32F, cv::Scalar::all(0));
    cv::Mat point_mat2_corr(c_mat1_indices.size(), 2, CV_32F, cv::Scalar::all(0));
    for (int i = 0; i < c_mat1_indices.size(); i++)
    {
      point_mat1_corr.at<float>(i, 0) = point_mat1.at<float>(c_mat1_indices[i], 0);
      point_mat1_corr.at<float>(i, 1) = point_mat1.at<float>(c_mat1_indices[i], 1);
      point_mat2_corr.at<float>(i, 0) = point_mat2.at<float>(c_indices[i], 0);
      point_mat2_corr.at<float>(i, 1) = point_mat2.at<float>(c_indices[i], 1);
    }
    // check if correspondence is lower than Min 
    if (point_mat1_corr.rows < MIN_ICP_CORRESPONDENCES)
    {
      break;
    }
    // ROS_INFO("Preprocessed corr size: %d, %d",point_mat1_corr.rows, point_mat2_corr.rows);

    // 5. ICP Iteration 
    tf_1_2_rfn = ICPSlam::icpIteration(point_mat1_corr, point_mat2_corr);

    // 6. Check convergence
    float translation_err = utils::euclideanDistance(float(tf_1_2_est.getOrigin().getX()),
                                                      float(tf_1_2_est.getOrigin().getY()),
                                                      float(tf_1_2_rfn.getOrigin().getX()),
                                                      float(tf_1_2_rfn.getOrigin().getY()));
    //atan2(y, x) = atan(sin, cos)
    float rotation_err = atan2(float(tf_1_2_est.getBasis()[1][0]),float(tf_1_2_est.getBasis()[0][0]));
    rotation_err -= atan2(float(tf_1_2_rfn.getBasis()[1][0]),float(tf_1_2_rfn.getBasis()[0][0]));
    rotation_err = std::abs(rotation_err);
    // ROS_INFO("[%d]translation: %f, rotation: %f",icp_itr, translation_err, rotation_err);

    is_converged |= (translation_err < MIN_ICP_TRANS_ERR);
    is_converged |= (rotation_err < MIN_ICP_ROTAT_ERR);

    // for viz ========
    if (!is_visualized_)
    {
      std::string image_name = std::string("/tmp/icp_laser_") + std::to_string(icp_itr) + ".png";
      ICPSlam::vizClosestPoints(point_mat1_corr,point_mat2_corr,tf_1_2_est,image_name);
      if (icp_itr > 20 && (icp_itr >= MAX_ICP_ITR-1 || is_converged == true))
      {
        ROS_WARN("visualized!!!!: %d", icp_itr);
        is_visualized_ = true;
      }
    }
    // end for viz ========

    // 7. update transform
    tf_1_2_est = tf_1_2_rfn;

    if (is_converged)
    {
      if (DEBUG) {ROS_WARN("ICP is over: %d times", icp_itr);}
      break;
    }
  }
  return tf_1_2_rfn;
}

tf::Transform ICPSlam::icpIteration(cv::Mat &point_mat1, cv::Mat &point_mat2)
{
  // center of mass
  cv::Mat mat1_mean, mat2_mean;
  reduce(point_mat1, mat1_mean, 0, CV_REDUCE_AVG);
  reduce(point_mat2, mat2_mean, 0, CV_REDUCE_AVG);

  cv::Mat point_mat1_w(point_mat1.clone()), point_mat2_w(point_mat2.clone());
  point_mat1_w.col(0) = point_mat1.col(0) - mat1_mean.at<float>(0, 0);
  point_mat1_w.col(1) = point_mat1.col(1) - mat1_mean.at<float>(0, 1);
  point_mat2_w.col(0) = point_mat2.col(0) - mat2_mean.at<float>(0, 0);
  point_mat2_w.col(1) = point_mat2.col(1) - mat2_mean.at<float>(0, 1);

  // calculate W
  cv::Mat icp_w(2, 2, CV_32F, cv::Scalar::all(0));
  for (int i = 0; i < point_mat1_w.rows; i++)
  {
    icp_w += point_mat1_w.row(i).t() * point_mat2_w.row(i);
  }

  // Singular Value Decomposition(SVD)
  cv::SVD svd(icp_w);
  svd.u; // U matrix of SVD
  svd.vt; // VT^ matrix of SVD
  svd.w; // nx1 vector of singular values
  cv::Mat r_est = svd.u * svd.vt; // 2 X 2 Matrix
  cv::Mat t_est = mat1_mean.t() - r_est * mat2_mean.t(); // 2 X 1 vector
  float theta = atan2(r_est.at<float>(1,0), r_est.at<float>(0,0)); //atan2(y, x) = atan(sin, cos)

  // Estimate Transform
  tf::Transform tf_est;
  tf_est.setOrigin(tf::Vector3(t_est.at<float>(0,0), t_est.at<float>(1,0), 0));
  tf_est.setRotation(tf::createQuaternionFromYaw(theta)); //radian

  return tf_est;
}


void ICPSlam::closestPoints(cv::Mat &point_mat1,
                            cv::Mat &point_mat2,
                            std::vector<int> &closest_indices,
                            std::vector<float> &closest_distances_2)
{
  // uses FLANN for K-NN e.g. http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
  closest_indices = std::vector<int>(point_mat1.rows, -1);
  closest_distances_2 = std::vector<float>(point_mat1.rows, -1);

  cv::Mat multi_channeled_mat1; //create Matrix header
  cv::Mat multi_channeled_mat2;

  point_mat1.convertTo(multi_channeled_mat1, CV_32FC2); //CV_32FC2: 32bit float 2 channel
  point_mat2.convertTo(multi_channeled_mat2, CV_32FC2);

  cv::flann::Index flann_index(multi_channeled_mat2, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees

  cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);
  cv::Mat mat_dists(point_mat1.rows, 1, CV_32F);
  flann_index.knnSearch(multi_channeled_mat1, mat_indices, mat_dists, 1, cv::flann::SearchParams(64) );

  int* indices_ptr = mat_indices.ptr<int>(0);
  //float* dists_ptr = mat_dists.ptr<float>(0);
  for (int i=0;i<mat_indices.rows;++i) {
    closest_indices[i] = indices_ptr[i];
  }

  mat_dists.copyTo(cv::Mat(closest_distances_2));

  // ---------------------------- naive version ---------------------------- //
  // max allowed distance between corresponding points
//  const float max_distance = 0.5;
//
//  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
//  {
//    int closest_point_idx = -1;
//    float closest_distance_2 = std::pow(max_distance, 2.0f);
//
//    for (size_t j = 0, len_j = (size_t)point_mat2.rows; j < len_j; j++)
//    {
//      auto distance2 =
//        std::pow(point_mat2.at<float>(j, 0) - point_mat1.at<float>(i, 0), 2.0f)
//        + std::pow(point_mat2.at<float>(j, 1) - point_mat1.at<float>(i, 1), 2.0f);
//
//      if (distance2 < closest_distance_2)
//      {
//        closest_distance_2 = distance2;
//        closest_point_idx = (int)j;
//      }
//    }
//
//    if (closest_point_idx >= 0)
//    {
//      closest_indices[i] = closest_point_idx;
//      closest_distances_2[i] = closest_distance_2;
//    }
//  }
}

void ICPSlam::vizClosestPoints(cv::Mat &point_mat1,
                               cv::Mat &point_mat2,
                               const tf::Transform &T_2_1,
                               const std::string filename)
{
  assert(point_mat1.size == point_mat2.size);

  const float resolution = 0.005;

  float *float_array = (float*)(point_mat1.data);
  float size_m = std::accumulate(
    float_array, float_array + point_mat1.total(), std::numeric_limits<float>::min(),
    [](float max, float current)
    {
      return current > max ? current : max;
    }
  );
  // add some slack
  size_m += 0.5;

  int size_pix = (int)(size_m / resolution);

  cv::Mat img(
    size_pix,
    size_pix,
    CV_8UC3,
    cv::Scalar(0, 0, 0)
  );

  auto meters_to_pix = [&size_pix, resolution](float meters) {
    int pix = (int)(meters / resolution + size_pix / 2.0f);
    pix = std::max(0, pix);
    pix = std::min(size_pix - 1, pix);
    return pix;
  };

  cv::Mat transformed_point_mat2 = utils::transformPointMat(T_2_1.inverse(), point_mat2);

  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
  {
    float x1 = point_mat1.at<float>(i, 0);
    float y1 = point_mat1.at<float>(i, 1);
    float x2 = transformed_point_mat2.at<float>(i, 0);
    float y2 = transformed_point_mat2.at<float>(i, 1);

    auto pix_x1 = meters_to_pix(x1);
    auto pix_y1 = meters_to_pix(y1);
    auto pix_x2 = meters_to_pix(x2);
    auto pix_y2 = meters_to_pix(y2);

    cv::Point point1(pix_x1, pix_y1);
    cv::Point point2(pix_x2, pix_y2);

    cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(img, point2, 5, cv::Scalar(255, 0, 0), -1);

    cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 2);
  }

  cv::Mat tmp;
  cv::flip(img, tmp, 0);
  cv::imwrite(filename, img);
}

} // namespace icp_slam
