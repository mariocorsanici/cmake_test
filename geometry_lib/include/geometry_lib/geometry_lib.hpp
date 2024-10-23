#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP

#include <iostream>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include <kdl/kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <rclcpp/time.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <utility>         

std::pair<Eigen::Vector3d, Eigen::Quaterniond> pose_stamped_to_eigen_pose(const geometry_msgs::msg::PoseStamped& pose_stamped);

geometry_msgs::msg::PoseStamped eigen_pose_to_pose_stamped(const Eigen::Vector3d& position, 
                                                      const Eigen::Quaterniond& quaternion, 
                                                      const rclcpp::Time timestamp,
                                                      const std::string& frame_id = "");

// KDL::JntArray ros_multiarray_to_kdl_jnt_array(const geometry_msgs::msg::PoseStamped& ros_multiarray);                      

// std_msgs::msg::Float64MultiArray kdl_jnt_array_to_ros_multiarray(const KDL::JntArray& jnt_array);

inline void pseudo_inverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_,bool damped = true);

inline void skew_symmetric(KDL::Vector &v_, Eigen::Matrix<double,3,3> &skew_mat_);

void print_function();

#endif // GEOMETRY_UTILS_HPP