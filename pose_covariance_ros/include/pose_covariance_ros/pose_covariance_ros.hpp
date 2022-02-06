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
    const std::string       name_;
    
    // Class contructor for single tree node
    NodeTree(NodeTree* ptr_prev, int type, Eigen::Vector3d axis,std::string const &name);
    NodeTree(const NodeTree&) = default;

    // Initialize pose realtive to previous node in the tree
    void initPose(double x, double y, double z, double qx, double qy, double qz, double qw);
  

    // Initialize covariance matrix relative to previous node in the tree
    void initCovariance(Eigen::Matrix<double, 6, 6, Eigen::RowMajor> const &cov_in );

    // Dumps node info
    void plotInfo() const;

    // Sets value of position and covariance related to tree root 
    void setPoseBase(PoseCov3Ns::PoseCov3 const &pose_in);

    // Update value of node based on the input variable(e.g joint angle)
    void updateNode(double val);

    // Set covariance of control variable to zero
    void fix_joint_cov();

    std::string            getName() const;
    NodeTree*              getPrevious() const;
    std::list<NodeTree*>   getNext() const;
    PoseCov3Ns::PoseCov3   getPose() const;
    PoseCov3Ns::PoseCov3  getPoseBase() const;

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
    std::vector<ros::Publisher>                               pose_pub_;
    std::list<NodeTree*>                                      it_names_;
    tf2_ros::TransformBroadcaster                             br_;
    tree_config                                               cfg_;
  public:
  
    //TODO ignore fixed joints
    //TODO change rate
    TreeStructure(const TreeStructure&) = default;

    explicit TreeStructure(tree_config const &cfg);

    ~TreeStructure() = default;

    // Read a joint state message and initialize the list of actuated joints
    void initJoints();

    // Looks for joint with given name in NodeTree list
    NodeTree* nameLookup(std::string const &name_in );

    //Add node to tree structure, by inputting a urdf::LinkSharedPointer
    void addNode(urdf::LinkSharedPtr ln_ptr);

    //plots tree structure
    void plotTree() const;

    // Computes kinematic chain 
    void computeChain();

    // Fills PoseWithCovarianceStamped message
    void fillCovMsg(Eigen::Matrix<double, 6, 6, Eigen::RowMajor>cov , geometry_msgs::PoseWithCovarianceStamped& pose_msg);

    void randChain() const;

    void poseCallback(const sensor_msgs::JointStateConstPtr &msg);

    void getPosesMU(std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>& vec_poses) const; 

    void getPosesBaseMU(std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>& vec_poses) const;

    void getPosesC(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& vec_poses) const;

    void getPosesBaseC(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& vec_poses) const;



};

#endif
