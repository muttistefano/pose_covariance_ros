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
#include <boost/range/combine.hpp>
#include <pose_covariance_ros/pose_covariance_ros.hpp>




// Implement a single node in the tree, associated with a joint in the URDF, with a pointer to the previous joints. 

// Class contructor for single tree node
NodeTree::NodeTree(NodeTree* ptr_prev, int type, Eigen::Vector3d axis,std::string const &name)
  : prev_(ptr_prev),type_(type),axis_(axis),name_(name)
  {
     if (prev_!=nullptr)
     {
      ROS_INFO_STREAM("created name :" << name_ );
      ROS_INFO_STREAM("  parent " << prev_->name_);
      ROS_INFO_STREAM("  type " << type_);
      ROS_INFO_STREAM("  axis " << axis_[0] << axis_[1] << axis_[2] << "\n");
     }
     else
     {
       ROS_INFO_STREAM("created nome name :" << name_ );
       ROS_INFO_STREAM("  parent  NULL");
       ROS_INFO_STREAM("  type " << type_);
       ROS_INFO_STREAM("  axis " << axis_[0] << axis_[1] << axis_[2] << std::endl);
     }
  }

// Initialize pose realtive to previous node in the tree
void NodeTree::initPose(double x, double y, double z, double qx, double qy, double qz, double qw)
{
  auto q_in = Eigen::Quaterniond(qw,qx,qy,qz);
  pose_ = PoseCov3Ns::PoseCov3(x,y,z,q_in,type_,axis_);
}

// Initialize covariance matrix relative to previous node in the tree
void NodeTree::initCovariance(Eigen::Matrix<double, 6, 6, Eigen::RowMajor> const &cov_in )
{
  pose_.setC(cov_in);
}

// Dumps node info
void NodeTree::plotInfo() const
{
  ROS_INFO_STREAM("Logname name:" << name_ << " ");
  if(prev_!=nullptr)
  {
    ROS_INFO_STREAM("prev_name :" << prev_->name_ << "\n");
  }
  else{
    ROS_INFO_STREAM("prev_name :" << "Null" << "\n");
  }

  ROS_INFO_STREAM("MU: \n"       << pose_.getMU()      << "\n");
  ROS_INFO_STREAM("cov: \n"      << pose_.getC()       << "\n\n\n");

}

// Sets value of position and covariance related to tree root
void NodeTree::setPoseBase(PoseCov3Ns::PoseCov3 const  &pose_in)
{
  this->pose_base_ = PoseCov3Ns::PoseCov3(pose_in);
}

// Update value of node based on the input variable(e.g joint angle)
void NodeTree::updateNode(double val)
{
  pose_.update(val);
}

// Set covariance of control variable to zero
void NodeTree::fix_joint_cov()
{
  pose_.fix_joint_cov();
}

std::string            NodeTree::getName()     const {return this->name_;}
NodeTree*              NodeTree::getPrevious() const {return this->prev_;}
std::list<NodeTree*>   NodeTree::getNext()     const {return this->next_;}
PoseCov3Ns::PoseCov3     NodeTree::getPose()     const {return this->pose_;}
PoseCov3Ns::PoseCov3     NodeTree::getPoseBase() const {return this->pose_base_;}




// Implement a tree, associated to the kinematic described in the URDF

TreeStructure::TreeStructure(tree_config const &cfg)
  :nh_(ros::NodeHandle("~")),cfg_(cfg)
  {

    bool tree_exp = false;
    std::vector<urdf::LinkSharedPtr> ref_link{cfg_.model_.root_link_};
    root_link_ = cfg_.model_.root_link_->name;
    std::vector<urdf::LinkSharedPtr> tmp_link;
    while (!tree_exp)
    {
        tmp_link.clear();
        for (const auto& ix :ref_link)
        {
            if(!ix->child_links.empty())
            {
                for (const auto& ln :ix->child_links)
                {
                    tmp_link.push_back(ln);
                }

            }
            this->addNode(ix);
        }
        ref_link = tmp_link;
        if (ref_link.empty()) tree_exp = true;

    }

    initJoints();


    cfg_.jnt_sub_  = nh_.subscribe(cfg_.joint_pub_name_, 1, &TreeStructure::poseCallback, this);
    for (auto& jnt:it_names_)
    {
      pose_pub_.push_back(nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_cov_" + jnt->name_, 1));
    }
//    plotTree();
  }


// Read a joint state message and initialize the list of actuated joints
void TreeStructure::initJoints()
{
   boost::shared_ptr<sensor_msgs::JointState const> jnt_0 = ros::topic::waitForMessage<sensor_msgs::JointState>(cfg_.joint_pub_name_, nh_);

  for (const auto& namemsg:(*jnt_0).name)
  {
    for (auto& it:poses_)
    {
      if(it.name_ == namemsg) it_names_.push_back(&it);
    }
  }

  for (const auto& namemsg:cfg_.jnt_add_)
  {
    for (auto& it:poses_)
    {
      if(it.name_ == namemsg) it_names_.push_back(&it);
    }
  }



  ROS_INFO_STREAM("published joints :" << "\n");
  for (const auto& pd:it_names_)
  {
    ROS_INFO_STREAM(pd->name_ << "\n");
  }


}

// Looks for joint with given name in NodeTree list
NodeTree* TreeStructure::nameLookup(std::string const &name_in )
{
  for (auto& jn :poses_)
  {
      if (name_in == jn.name_) return &jn;
  }
  return nullptr;
}

//Add node to tree structure, by inputting a urdf::LinkSharedPointer
void TreeStructure::addNode(urdf::LinkSharedPtr ln_ptr)
{
  for (const auto& jn :ln_ptr->child_joints)
  {
    NodeTree* parent_ptr = nullptr;
    if(ln_ptr->parent_joint != NULL)
    {
      parent_ptr = nameLookup(ln_ptr->parent_joint->name);

    }

    auto jnt_axis = Eigen::Vector3d(jn->axis.x, jn->axis.y, jn->axis.z);
    
    poses_.emplace_back(parent_ptr,jn->type,jnt_axis,jn->name);
    poses_.back().initPose(jn->parent_to_joint_origin_transform.position.x,
                            jn->parent_to_joint_origin_transform.position.y,
                            jn->parent_to_joint_origin_transform.position.z,
                            jn->parent_to_joint_origin_transform.rotation.x,
                            jn->parent_to_joint_origin_transform.rotation.y,
                            jn->parent_to_joint_origin_transform.rotation.z,
                            jn->parent_to_joint_origin_transform.rotation.w);



    //TODO cerca cov in paramtri per singolo giunto per nome
    //TODO covarianza per variabile controllo

    auto it_ov = std::find(cfg_.override_list_.begin(), cfg_.override_list_.end(), jn->name);
    int index  = it_ov - cfg_.override_list_.begin();

    if( it_ov != cfg_.override_list_.end())
    {
      auto it_mat = cfg_.covariance_override_.begin();
      std::advance(it_mat, index);
      poses_.back().initCovariance(*it_mat);
    }
    else if (poses_.size()==1 || ( cfg_.ignore_fixed_ && jn->type==6))
    {
      poses_.back().initCovariance(Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero());
    }
    else
    {
      if(cfg_.cov_only_valid_)
      {
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> tmp_cov_ = cfg_.def_cov_;
        tmp_cov_(0,0) = tmp_cov_(0,0) * int(abs(jn->parent_to_joint_origin_transform.position.x) > 1e-5);
        tmp_cov_(1,1) = tmp_cov_(1,1) * int(abs(jn->parent_to_joint_origin_transform.position.y) > 1e-5);
        tmp_cov_(2,2) = tmp_cov_(2,2) * int(abs(jn->parent_to_joint_origin_transform.position.z) > 1e-5);
        auto q = Eigen::Quaterniond(jn->parent_to_joint_origin_transform.rotation.w,jn->parent_to_joint_origin_transform.rotation.x,jn->parent_to_joint_origin_transform.rotation.y,jn->parent_to_joint_origin_transform.rotation.z);
        Eigen::Vector3d rpy  = q.normalized().toRotationMatrix().eulerAngles(0, 1, 2);
        tmp_cov_(3,3) = tmp_cov_(3,3) * int(abs(rpy[0]) > 1e-5);
        tmp_cov_(4,4) = tmp_cov_(4,4) * int(abs(rpy[1]) > 1e-5);
        tmp_cov_(5,5) = tmp_cov_(5,5) * int(abs(rpy[2]) > 1e-5);
        poses_.back().initCovariance(tmp_cov_);
      }
      else
      {
        poses_.back().initCovariance(cfg_.def_cov_);
      }
    }
  if(cfg_.ignore_joint_cov_) poses_.back().fix_joint_cov();
  }
}

//plots tree structure
void TreeStructure::plotTree() const
{
  for (const auto& nd :poses_)
  {
    nd.plotInfo();
  }

ROS_INFO_STREAM("tree length " << poses_.size() << "\n" );
}

// Computes kinematic chain
void TreeStructure::computeChain()
{
  for (auto& nd :poses_)
  {
    if (nd.getPrevious()==nullptr)
    {
      nd.setPoseBase(nd.getPose());
    }
    else
    {
      nd.setPoseBase(nd.getPrevious()->getPoseBase().compose4(nd.getPose()));
    }
  }
}

// Fills PoseWithCovarianceStamped message
void TreeStructure::fillCovMsg(Eigen::Matrix<double, 6, 6, Eigen::RowMajor>cov , geometry_msgs::PoseWithCovarianceStamped& pose_msg)
{

  for (int i=0;i<36;i++)
  {
    auto rid = int(floor(i/6));
    auto cid = int(i%6);
    pose_msg.pose.covariance[i] = cov(rid,cid);
  }

}

void TreeStructure::randChain() const
{
  for (auto& it:it_names_) {
    double val = (3.14 + 3.14) * ( (double)rand() / (double)RAND_MAX ) + -3.14;
    it->updateNode(val);
  }
}

void TreeStructure::poseCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  for (int j=0;j<msg->name.size();j++) {
    // std :: cout << msg->name[j] << " " << msg->position[j] << std::endl;
    std::list<NodeTree*>::iterator it = it_names_.begin();
    std::advance(it, j);
    (*it)->updateNode(msg->position[j]);
  }

  this->computeChain();

  for (int j=0;j<pose_pub_.size();j++) {

    auto it = it_names_.begin();
    std::advance(it, j);

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> tmpM4 = (*it)->getPoseBase().getMU();

    // tf2::Vector3 origin;
    // origin.setValue(tmpM4(0,3),tmpM4(1,3),tmpM4(2,3));

    tf2::Matrix3x3 tf3d;
    tf3d.setValue(tmpM4(0,0), tmpM4(0,1), tmpM4(0,2),
                  tmpM4(1,0), tmpM4(1,1), tmpM4(1,2),
                  tmpM4(2,0), tmpM4(2,1), tmpM4(2,2));

    tf2::Quaternion tfqt;
    tf3d.getRotation(tfqt);


//TODO cerca cov in paramtri per singolo giunto per nome
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp    = ros::Time::now();
    pose_msg.header.frame_id = root_link_;
    pose_msg.pose.pose.position.x    = tmpM4(0,3);
    pose_msg.pose.pose.position.y    = tmpM4(1,3);
    pose_msg.pose.pose.position.z    = tmpM4(2,3);
    pose_msg.pose.pose.orientation.x = tfqt.x();
    pose_msg.pose.pose.orientation.y = tfqt.y();
    pose_msg.pose.pose.orientation.z = tfqt.z();
    pose_msg.pose.pose.orientation.w = tfqt.w();
    fillCovMsg((*it)->getPoseBase().getC(),pose_msg);

    pose_pub_[j].publish(pose_msg);


  }
  // plotTree();
}

void TreeStructure::getPosesMU(std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>& vec_poses) const
{
  for(const auto& ps:poses_)
  {
    vec_poses.push_back(ps.getPose().getMU());
  }
}

void TreeStructure::getPosesBaseMU(std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>& vec_poses) const
{
  for(const auto& ps:poses_)
  {
    vec_poses.push_back(ps.getPoseBase().getMU());
  }
}

void TreeStructure::getPosesC(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& vec_poses) const
{
  for(const auto& ps:poses_)
  {
    vec_poses.push_back(ps.getPose().getC());
  }
}

void TreeStructure::getPosesBaseC(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& vec_poses) const
{
  for(const auto& ps:poses_)
  {
    vec_poses.push_back(ps.getPoseBase().getC());
  }
}


