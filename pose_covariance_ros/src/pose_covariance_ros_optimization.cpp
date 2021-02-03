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
#include <pose_covariance_ros/pose_covariance_ros_optimization.hpp>




// Implement a single node in the tree, associated with a joint in the URDF, with a pointer to the previous joints. 

// Class contructor for single tree node
NodeTree::NodeTree(std::string name, NodeTree* ptr_prev, int type, Eigen::Vector3d axis)
  : name_(name),prev_(ptr_prev),type_(type),axis_(axis)
  {
     if (prev_!=nullptr)
     {
      std::cout << "created name :" << name_ ;
      std::cout << "  parent " << prev_->name_;
      std::cout << "  type " << type_;
      std::cout << "  axis " << axis_[0] << axis_[1] << axis_[2] << std::endl;
     }
     else
     {
       std::cout << "created nome name :" << name_ ;
       std::cout << "  parent  NULL";
       std::cout << "  type " << type_;
      std::cout << "  axis " << axis_[0] << axis_[1] << axis_[2] << std::endl;
     }
  }

// Initialize pose realtive to previous node in the tree
void NodeTree::initPose(double x, double y, double z, double qx, double qy, double qz, double qw)
{
  double roll,pitch,yaw;
  Eigen::Quaterniond q_in = Eigen::Quaterniond(qw,qx,qy,qz);
  pose_ = PoseCov3_ns::PoseCov3(x,y,z,q_in,type_,axis_);
}

// Initialize covariance matrix relative to previous node in the tree
void NodeTree::initCovariance(Eigen::Matrix<double, 6, 6, Eigen::RowMajor> cov_in )
{
  pose_.setC(cov_in);
}

// Dumps node info
void NodeTree::plotInfo()
{
  std::cout << "Logname name:" << name_ << " ";
  if(prev_!=nullptr)
  {
    std::cout << "prev_name :" << prev_->name_ << "\n";
  }
  else{
    std::cout << "prev_name :" << "Null" << "\n";
  }

  // std::cout << "next_size :"  << next_.size()       << std::endl;
  std::cout << "MU: \n"       << pose_.getMU()      << "\n";
  std::cout << "cov: \n"      << pose_.getC()       << "\n\n\n";
  // std::cout << "base MU: \n"  << pose_base_.getMU() << "\n";
  // std::cout << "base cov: \n" << pose_base_.getC()  << "\n\n\n\n\n\n\n\n\n\n";

}

// Sets value of position and covariance related to tree root
void NodeTree::setPoseBase(PoseCov3_ns::PoseCov3 pose_in)
{
  this->pose_base_ = PoseCov3_ns::PoseCov3(pose_in);
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

std::string            NodeTree::getName()       {return this->name_;}
NodeTree*              NodeTree::getPrevious()   {return this->prev_;}
std::list<NodeTree*>   NodeTree::getNext()       {return this->next_;}
PoseCov3_ns::PoseCov3  NodeTree::getPose()       {return this->pose_;}
PoseCov3_ns::PoseCov3  NodeTree::getPoseBase()   {return this->pose_base_;}




// Implement a tree, associated to the kinematic described in the URDF

TreeStructure::TreeStructure(tree_config cfg)
  :nh_(ros::NodeHandle("~")),cfg_(cfg)
  {

    bool tree_exp = false;
    std::vector<urdf::LinkSharedPtr> ref_link{cfg_.model_.root_link_};
    root_link_ = cfg_.model_.root_link_->name;
    std::vector<urdf::LinkSharedPtr> tmp_link;
    while (!tree_exp)
    {
        tmp_link.clear();
        for (auto& ix :ref_link)
        {
            if(ix->child_links.size() > 0)
            {
                for (auto& ln :ix->child_links)
                {
                    tmp_link.push_back(ln);
                }

            }
            this->addNode(ix);
        }
        ref_link = tmp_link;
        if (ref_link.size() == 0) tree_exp = true;

    }

    initJoints();
    // srv_opt_ = nh_.advertiseService("srv_opt", (bool)&TreeStructure::optimize_joints,this);

//    plotTree();
  }

TreeStructure::~TreeStructure(){}

// Read a joint state message and initialize the list of actuated joints
void TreeStructure::initJoints()
{
   boost::shared_ptr<sensor_msgs::JointState const> jnt_0 = ros::topic::waitForMessage<sensor_msgs::JointState>(cfg_.joint_pub_name_, nh_);

 ;

  for (auto& namemsg:(*jnt_0).name)
  {
    for (auto& it:poses_)
    {
      if(it.name_ == namemsg) it_names_.push_back(&it);
    }
  }

  for (auto& namemsg:cfg_.jnt_add_)
  {
    for (auto& it:poses_)
    {
      if(it.name_ == namemsg) it_names_.push_back(&it);
    }
  }



  std::cout << "published joints :" <<std::endl;
  for (auto& pd:it_names_)
  {
    std::cout << pd->name_ <<std::endl;
  }


}

// Looks for joint with given name in NodeTree list
NodeTree* TreeStructure::nameLookup(std::string name_in )
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
  for (auto& jn :ln_ptr->child_joints)
  {
    NodeTree* parent_ptr = nullptr;
    if(ln_ptr->parent_joint != NULL)
    {
      parent_ptr = nameLookup(ln_ptr->parent_joint->name);

    }
    Eigen::Vector3d jnt_axis = Eigen::Vector3d(jn->axis.x, jn->axis.y, jn->axis.z);
    poses_.push_back(NodeTree(jn->name,parent_ptr,jn->type,jnt_axis));
    poses_.back().initPose(jn->parent_to_joint_origin_transform.position.x,
                            jn->parent_to_joint_origin_transform.position.y,
                            jn->parent_to_joint_origin_transform.position.z,
                            jn->parent_to_joint_origin_transform.rotation.x,
                            jn->parent_to_joint_origin_transform.rotation.y,
                            jn->parent_to_joint_origin_transform.rotation.z,
                            jn->parent_to_joint_origin_transform.rotation.w);


    if(jn->limits != NULL)
    {
      std::cout << jn->limits->lower << " " << jn->limits->upper << std::endl;
      poses_.back().upper_limit = jn->limits->upper;
      poses_.back().lower_limit = jn->limits->lower;
    }
    
    
    //TODO cerca cov in paramtri per singolo giunto per nome
    //TODO covarianza per variabile controllo

    auto it_ov = std::find(cfg_.override_list_.begin(), cfg_.override_list_.end(), jn->name);
    int index  = it_ov - cfg_.override_list_.begin();

    if( it_ov != cfg_.override_list_.end())
    {
      std::list<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>::iterator it_mat = cfg_.covariance_override_.begin();
      std::advance(it_mat, index);
      poses_.back().initCovariance(*it_mat);
    }
    else if (poses_.size()==1 || ( cfg_.ignore_fixed_ and jn->type==6))
    {
      poses_.back().initCovariance(Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero());
      // std::cout << "fixed joint" << std::endl;
    }
    else
    {
      if(cfg_.cov_only_valid_)
      {
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> tmp_cov_ = cfg_.def_cov_;
        tmp_cov_(0,0) = tmp_cov_(0,0) * int(abs(jn->parent_to_joint_origin_transform.position.x) > 1e-5);
        tmp_cov_(1,1) = tmp_cov_(1,1) * int(abs(jn->parent_to_joint_origin_transform.position.y) > 1e-5);
        tmp_cov_(2,2) = tmp_cov_(2,2) * int(abs(jn->parent_to_joint_origin_transform.position.z) > 1e-5);
        Eigen::Quaterniond q = Eigen::Quaterniond(jn->parent_to_joint_origin_transform.rotation.w,jn->parent_to_joint_origin_transform.rotation.x,jn->parent_to_joint_origin_transform.rotation.y,jn->parent_to_joint_origin_transform.rotation.z);
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
void TreeStructure::plotTree()
{
  for (auto& nd :poses_)
  {
    nd.plotInfo();
  }

std::cout << "tree length " << poses_.size() << std::endl;
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
    int rid = int(floor(i/6));
    int cid = int(i%6);
    pose_msg.pose.covariance[i] = cov(rid,cid);
  }

}

void TreeStructure::randChain()
{
  for (auto& it:it_names_) {
    double val = (3.14 + 3.14) * ( (double)rand() / (double)RAND_MAX ) + -3.14;
    it->updateNode(val);
  }
}

void TreeStructure::updateall(std::array<double, 6> jnt) //#TODO
{

  std::list<NodeTree*>::iterator it = it_names_.begin();
  for (int j=0;j<6;j++) {
    // std::cout << (*it)->getName() << "" << jnt[j] <<std::endl;
    (*it)->updateNode(jnt[j]);
    // std::cout << (*it)->getName() << std::endl; //TODO testa con 2 catene , con il camera joint in cfg file
    std::advance(it, 1);
  }

  this->computeChain();

  // std::cout << (*it_names_.back()).getName() << std::endl;
  // std::cout << (*it_names_.back()).getPoseBase().getC() << std::endl;

}

void TreeStructure::getPosesMU(std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>& vec_poses)
{
  for(auto& ps:poses_)
  {
    vec_poses.push_back(ps.getPose().getMU());
  }
}

void TreeStructure::getPosesBaseMU(std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>& vec_poses)
{
  for(auto& ps:poses_)
  {
    vec_poses.push_back(ps.getPoseBase().getMU());
  }
}

void TreeStructure::getPosesC(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& vec_poses)
{
  for(auto& ps:poses_)
  {
    vec_poses.push_back(ps.getPose().getC());
  }
}

void TreeStructure::getPosesBaseC(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& vec_poses)
{
  for(auto& ps:poses_)
  {
    vec_poses.push_back(ps.getPoseBase().getC());
  }
}

Eigen::Matrix<double, 6, 6, Eigen::RowMajor> TreeStructure::getTipCov()
{
  auto it = it_names_.back();
  return (*it).getPoseBase().getC();
};

bool TreeStructure::optimize_joints(pose_covariance_ros::srv_opt::Request  &req, pose_covariance_ros::srv_opt::Response &res)
{
  std::vector<int>                                              max_idx = {0,0,0,0,0,0} ;
  std::vector<double>                                           max_val = {10,10,10,10,10,10} ;
  std::vector<std::array<double,6>>                             jnts; //TODO
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor>                  cov_tmp;
  // std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>,6>   cov_vec;
  std::vector<Eigen::Matrix<double,6, 6, Eigen::RowMajor>, Eigen::aligned_allocator<Eigen::Matrix<double,6, 6, Eigen::RowMajor> > > cov_vec;

  for(int ini=0;ini<6;ini++)
  {
    cov_vec.push_back(Eigen::Matrix<double,6, 6, Eigen::RowMajor>());
    jnts.push_back(std::array<double, 6>());
  }

  res.out.jnt_num.data = 6;

  for (auto& ps:req.in.poses)
  {

    geometry_msgs::Point      pnt = ps.position;
    geometry_msgs::Quaternion qua = ps.orientation;

    tf2::Quaternion quat_tf;
    tf2::convert(ps.orientation,quat_tf);
    tf2::Matrix3x3 m(quat_tf);
    Eigen::Matrix4d tf_base;

    tf_base(0,0)=m[0][0];tf_base(0,1)=m[0][1];tf_base(0,2)=m[0][2];tf_base(0,3)=pnt.x;
    tf_base(1,0)=m[1][0];tf_base(1,1)=m[1][1];tf_base(1,2)=m[1][2];tf_base(1,3)=pnt.y;
    tf_base(2,0)=m[2][0];tf_base(2,1)=m[2][1];tf_base(2,2)=m[2][2];tf_base(2,3)=pnt.z;
    tf_base(3,0)=0;      tf_base(3,1)=0;      tf_base(3,2)=0;      tf_base(3,3)=1;  

    double* T = new double[16];

    T[0]=m[0][0];T[1]=m[0][1];T[2]=m[0][2]; T[3]=pnt.x;
    T[4]=m[1][0];T[5]=m[1][1];T[6]=m[1][2]; T[7]=pnt.y;
    T[8]=m[2][0];T[9]=m[2][1];T[10]=m[2][2];T[11]=pnt.z;
    T[12]=0;     T[13]=0;     T[14]=0;      T[15]=1;

    double q_sols[8*6];
    int num_sols;
    num_sols = ur_kinematics::inverse(T, q_sols);

    bool skip = false;
    for (int j = 0; j<8*6 ; j++){if (isnan(q_sols[j])) skip = true;}
    if (skip == true) continue;
    std::cout << "sols : " << num_sols << std::endl;
    for(int i=0;i<num_sols;i++)
    {
      printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
      std::array<double, 6> arr_jnt = {q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]};
      
      this->updateall(arr_jnt);
      std::cout << this->getTipCov() << std::endl;
      cov_tmp = this->getTipCov();
      for(int id=0;id<6;id++)
      {
        if(cov_tmp(id,id) < max_val[id])
        {
          max_val[id] = cov_tmp(id,id);
          max_idx[id] = i;
          cov_vec[id] = cov_tmp;
          jnts[id]    = arr_jnt;
        }
      }
      printf("\n");

    }
  }

  printf("\n");
  for (std::vector<double>::const_iterator i = max_val.begin(); i != max_val.end(); ++i)
    std::cout << *i << ' ';
  for (std::vector<int>::const_iterator i = max_idx.begin(); i != max_idx.end(); ++i)
    std::cout << *i << ' ';
  std::cout << std::endl;

  

  res.out.joints.resize(36);
  res.out.cov.resize(36*6);

  for(int pd=0;pd<6;pd++)
  {
    res.out.joints[pd*6    ].data = jnts[pd][0];
    res.out.joints[pd*6 + 1].data = jnts[pd][1];
    res.out.joints[pd*6 + 2].data = jnts[pd][2];
    res.out.joints[pd*6 + 3].data = jnts[pd][3];
    res.out.joints[pd*6 + 4].data = jnts[pd][4];
    res.out.joints[pd*6 + 5].data = jnts[pd][5];

    for(int cv = 0;cv<36;cv++)
    {
      res.out.cov[pd*36 + cv].data = cov_vec[pd](floor(cv/6.0),cv%6);
    }

  }

  return true;
}
