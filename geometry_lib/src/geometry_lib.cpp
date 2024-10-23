#include "geometry_lib.hpp" // Ensure this header is properly included

std::pair<Eigen::Vector3d, Eigen::Quaterniond> pose_stamped_to_eigen_pose(const geometry_msgs::msg::PoseStamped& pose_stamped)
{
    // Crea un vettore per la posizione
    Eigen::Vector3d position;
    position.x() = pose_stamped.pose.position.x;
    position.y() = pose_stamped.pose.position.y;
    position.z() = pose_stamped.pose.position.z;

    // Crea un quaternione per l'orientamento
    Eigen::Quaterniond quaternion;
    quaternion.x() = pose_stamped.pose.orientation.x;
    quaternion.y() = pose_stamped.pose.orientation.y;
    quaternion.z() = pose_stamped.pose.orientation.z;
    quaternion.w() = pose_stamped.pose.orientation.w;

    // Restituisci un std::pair con posizione e quaternione
    return std::make_pair(position, quaternion);
}

geometry_msgs::msg::PoseStamped eigen_pose_to_pose_stamped(const Eigen::Vector3d& position, 
                                                      const Eigen::Quaterniond& quaternion, 
                                                      const rclcpp::Time timestamp,
                                                      const std::string& frame_id) 
{
    geometry_msgs::msg::PoseStamped pose_stamped;

    // Set the header (frame ID and timestamp)
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = timestamp;

    // Set the position
    pose_stamped.pose.position.x = position.x();
    pose_stamped.pose.position.y = position.y();
    pose_stamped.pose.position.z = position.z();

    // Set the orientation
    pose_stamped.pose.orientation.x = quaternion.x();
    pose_stamped.pose.orientation.y = quaternion.y();
    pose_stamped.pose.orientation.z = quaternion.z();
    pose_stamped.pose.orientation.w = quaternion.w();

    return pose_stamped;
}

// KDL::JntArray ros_multiarray_to_kdl_jnt_array(const std_msgs::msg::Float64MultiArray& ros_multiarray)
// {

// }                      

// std_msgs::msg::Float64MultiArray kdl_jnt_array_to_ros_multiarray(const KDL::JntArray& jnt_array)
// {
//     std_msgs::msg::Float64MultiArray joint_array_msg;
// 	for(int i = 0; i < kdl_chain_.getNrOfJoints(); i++) 
// 	{
// 		joint_array_msg.data.push_back(jnt_array(i));
// 	}
//     return joint_array_msg;
// }                                        

inline void pseudo_inverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_, bool damped)
{	
	double lambda_ = damped ? 0.2 : 0.0;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
	Eigen::MatrixXd S_ = M_;	// copying the dimensions of M_, its content is not needed.
	S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV()*S_.transpose()*svd.matrixU().transpose());
}


inline void skew_symmetric(KDL::Vector &v_, Eigen::Matrix<double,3,3> &skew_mat_)
{
	skew_mat_ = Eigen::Matrix<double,3,3>::Zero();
	
	skew_mat_(0,1) = -v_(2);
	skew_mat_(0,2) =  v_(1);
	skew_mat_(1,0) =  v_(2);
	skew_mat_(1,2) = -v_(0);
	skew_mat_(2,0) = -v_(1);
	skew_mat_(2,1) =  v_(0);
}

void print_function() {
    std::cout << "PRINT_FUNCTION: The function is included" << std::endl;
}