#ifndef __POSE_COV_ROS_HPP
#define __POSE_COV_ROS_HPP


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
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch(...)
          {
            throw;
          }
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e)
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
    PoseCov3_ns::PoseCov3   pose_;
    // pose e covariance respect to tree root
    PoseCov3_ns::PoseCov3   pose_base_;
    const int               type_;
    const Eigen::Vector3d   axis_;
  public:
    const std::string       name_;
    
    // Class contructor for single tree node
    NodeTree(std::string name, NodeTree* ptr_prev, int type, Eigen::Vector3d axis);

    // Initialize pose realtive to previous node in the tree
    void initPose(double x, double y, double z, double qx, double qy, double qz, double qw);
  

    // Initialize covariance matrix relative to previous node in the tree
    void initCovariance(Eigen::Matrix<double, 6, 6, Eigen::RowMajor> cov_in );

    // Dumps node info
    void plotInfo();

    // Sets value of position and covariance related to tree root 
    void setPoseBase(PoseCov3_ns::PoseCov3 pose_in);

    // Update value of node based on the input variable(e.g joint angle)
    void updateNode(double val);

    // Set covariance of control variable to zero
    void fix_joint_cov();

    std::string            getName();
    NodeTree*              getPrevious();
    std::list<NodeTree*>   getNext();
    PoseCov3_ns::PoseCov3  getPose();
    PoseCov3_ns::PoseCov3  getPoseBase();

};

// Configuration structure for
struct tree_config{
  urdf::Model                                               model_;
  std::vector<std::string>                                  jnt_ignore_;
  std::vector<std::string>                                  jnt_add_;
  std::string                                               joint_pub_name_   = "/joint_states";
  ros::Subscriber                                           jnt_sub_;
  bool                                                      ignore_fixed_     = false;
  bool                                                      cov_only_valid_   = true;
  bool                                                      ignore_joint_cov_ = false;
  std::vector<std::string>                                  override_list_;
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor>              def_cov_;
  std::list<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>   covariance_override_;

  tree_config(){}
  tree_config(const tree_config& cfg_in) : model_(cfg_in.model_),jnt_ignore_(cfg_in.jnt_ignore_),jnt_add_(cfg_in.jnt_add_),joint_pub_name_(cfg_in.joint_pub_name_),
                                           jnt_sub_(cfg_in.jnt_sub_),ignore_fixed_(cfg_in.ignore_fixed_),cov_only_valid_(cfg_in.cov_only_valid_),ignore_joint_cov_(cfg_in.ignore_joint_cov_),
                                           override_list_(cfg_in.override_list_),def_cov_(cfg_in.def_cov_),covariance_override_(cfg_in.covariance_override_){}

};

// Implement a tree, associated to the kinematic described in the URDF
class TreeStructure
{
  private:
    std::list<NodeTree>                                       poses_;
    std::string                                               root_link_;
    ros::NodeHandle                                           nh_;
    std::vector<ros::Publisher>                               pose_pub_;
    std::list<NodeTree*>                                      it_names_;
    tf2_ros::TransformBroadcaster                             br_;
    tree_config                                               cfg_;
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

    void poseCallback(const sensor_msgs::JointStateConstPtr &msg);

    void getPosesMU(std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>& vec_poses); 

    void getPosesBaseMU(std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>& vec_poses);

    void getPosesC(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& vec_poses);

    void getPosesBaseC(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& vec_poses);



};

#endif
