#ifndef __POSE_COV_ROS_HPP_OPT
#define __POSE_COV_ROS_HPP_OPT


#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <algorithm>
#include <urdf/model.h>
#include <ros/console.h>
#include <Eigen/Core>
#include <XmlRpc.h>
// #include <tf/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pose_lie/pose_lie.hpp>
#include <boost/range/combine.hpp>
#include <pose_covariance_ros/srv_opt.h>
#include <pose_covariance_ros/opt_return.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <trac_ik/trac_ik.hpp>
#include <Eigen/StdVector>
#include <std_msgs/Int32.h>
#include <boost/dll/shared_library.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>



// Get matrix from parameter server
inline bool getParamMatrix(const ros::NodeHandle& nh, const std::string& key, Eigen::Matrix<double, 6, 6, Eigen::RowMajor>& Mat)
{
  XmlRpc::XmlRpcValue Mat_config;

  if (nh.hasParam(key))
  {
    try
    {
      nh.getParam(key, Mat_config);

      ROS_ASSERT(Mat_config.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int matSize = 6;

      for (int i = 0; i < matSize; i++)
      {
        for (int j = 0; j < matSize; j++)
        {
          try
          {
            std::ostringstream ostr;
            ostr << Mat_config[matSize * i + j];
            std::istringstream istr(ostr.str());
            istr >> Mat(i, j);
          }
          catch(const XmlRpc::XmlRpcException &e)
          {
            ROS_ERROR_STREAM("ERROR reading matrix in yaml: " << e.getMessage());
            throw ;
          }

        }
      }
    }
    catch (const XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("ERROR reading matrix in yaml: " << e.getMessage());
    }

  }
}

// Implement a single node in the tree, associated with a joint in the URDF, with a pointer to the previous joints. 
class NodeTree
{
  private:
    NodeTree*               prev_;
    std::list<NodeTree*>    next_;
    // pose e covariance respect to previous joint
    PoseCov3Ns::PoseCov3   pose_;
    // pose e covariance respect to tree root
    PoseCov3Ns::PoseCov3   pose_base_;
    const int               type_;
    const Eigen::Vector3d   axis_;
  public:
    const  std::string       name_;
    double upper_limit       ;
    double lower_limit       ;
    
    // Class contructor for single tree node
    NodeTree(std::string name, NodeTree* ptr_prev, int type, Eigen::Vector3d axis);

    // Initialize pose realtive to previous node in the tree
    void initPose(double x, double y, double z, double qx, double qy, double qz, double qw);
  

    // Initialize covariance matrix relative to previous node in the tree
    void initCovariance(Eigen::Matrix<double, 6, 6, Eigen::RowMajor> cov_in );

    // Dumps node info
    void plotInfo();

    // Sets value of position and covariance related to tree root 
    void setPoseBase(PoseCov3Ns::PoseCov3 pose_in);

    // Update value of node based on the input variable(e.g joint angle)
    void updateNode(double val);

    // Set covariance of control variable to zero
    void fix_joint_cov();

    std::string            getName();
    NodeTree*              getPrevious();
    std::list<NodeTree*>   getNext();
    PoseCov3Ns::PoseCov3  getPose();
    PoseCov3Ns::PoseCov3  getPoseBase();

};

// Configuration structure for
struct tree_config{
  urdf::Model                                               model_;
  std::vector<std::string>                                  jnt_ignore_;
  std::vector<std::string>                                  jnt_add_;
  std::string                                               joint_pub_name_   = "/joint_states";
  std::string                                               planning_group_name = "manipulator";
  bool                                                      ignore_fixed_     = false;
  bool                                                      cov_only_valid_   = true;
  bool                                                      ignore_joint_cov_ = false;
  std::vector<std::string>                                  override_list_;
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor>              def_cov_;
  std::list<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>   covariance_override_;

  tree_config() = default;
  tree_config(const tree_config& cfg_in) = default;

};

// Implement a tree, associated to the kinematic described in the URDF
class TreeStructure
{
  private:
    std::list<NodeTree>                                       poses_;
    std::string                                               root_link_;
    ros::NodeHandle                                           nh_;
    std::list<NodeTree*>                                      it_names_;
    int                                                       act_joints_;
    tf2_ros::TransformBroadcaster                             br_;
    tree_config                                               cfg_;
    // ros::ServiceServer                                        srv_opt_;
  public:
  
    //TODO ignore fixed joints
    //TODO change rate

    TreeStructure(tree_config cfg);

    ~TreeStructure();

    // Read a joint state message and initialize the list of actuated joints
    void initJoints();

    // Looks for joint with given name in NodeTree list
    NodeTree* nameLookup(std::string name_in );

    //Add node to tree structure, by inputting a urdf::LinkSharedPointer
    void addNode(urdf::LinkSharedPtr ln_ptr);

    //plots tree structure
    void plotTree();

    // Computes kinematic chain 
    void computeChain();

    // Fills PoseWithCovarianceStamped message
    void fillCovMsg(Eigen::Matrix<double, 6, 6, Eigen::RowMajor>cov , geometry_msgs::PoseWithCovarianceStamped& pose_msg);

    void randChain();

    void updateall(std::vector<double> jnt);

    void getPosesMU(std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>& vec_poses)  ; 

    void getPosesBaseMU(std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>& vec_poses)  ;

    void getPosesC(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& vec_poses)  ;

    void getPosesBaseC(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& vec_poses)  ;

    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> getTipCov();

    bool optimize_joints(pose_covariance_ros::srv_opt::Request  &req, pose_covariance_ros::srv_opt::Response &res);

};

#endif
