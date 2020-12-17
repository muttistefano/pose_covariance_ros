#include "pose_covariance_ros/pose_covariance_ros.hpp"

#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite1, Test1)
{
  ros::NodeHandle nh("~");
  tree_config cfg;

  std::string     urdf_param_name = "/robot_description";

  // retrieves joint state publisher name
  if (nh.hasParam("joint_pub_topic"))
  {
      nh.getParam("joint_pub_topic", cfg.joint_pub_name_);
      ROS_INFO("joint_pub_topic :  %s", cfg.joint_pub_name_.c_str());
  }

  // retrieves parameter to ingore covariance in fixed joints
  if (nh.hasParam("ignore_fixed_cov"))
  {
      nh.getParam("ignore_fixed_cov", cfg.ignore_fixed_);
      ROS_INFO("ignore_fixed_cov :  %d", cfg.ignore_fixed_);
  }


  // retrieves parameter to ingore covariance of joints value
  if (nh.hasParam("ignore_joint_cov"))
  {
      nh.getParam("ignore_joint_cov", cfg.ignore_joint_cov_);
      ROS_INFO("ignore_joint_cov :  %d", cfg.ignore_joint_cov_);
  }

  // retrieves parameter to set covariance to parameters(x,y,z,a,b,c) that are different from zero
  if (nh.hasParam("cov_only_valid"))
  {
      nh.getParam("cov_only_valid", cfg.cov_only_valid_);
      ROS_INFO("cov_only_valid :  %d", cfg.cov_only_valid_);
  }

  // retrieves a list of joints to add to the covariance publisher
  if (nh.hasParam("additional_joints"))
  {
      nh.getParam("additional_joints", cfg.jnt_add_);
      std::cout << "additional_joints : "  << std::endl;
      for(auto& jt:cfg.jnt_add_)
      {
          std::cout << jt << std::endl;
      }
      std::cout << std::endl;
  }

  // retrieves a list of joints to override covariance and their matrix
  if (nh.hasParam("override_list"))
  {
      std::vector<std::string> override_list_tmp;
      nh.getParam("override_list", override_list_tmp);

      for (std::vector<std::string>::iterator i=override_list_tmp.begin(); i!=override_list_tmp.end(); i++)
      {
          if (nh.hasParam(*i))
          {
              Eigen::Matrix<double, 6, 6, Eigen::RowMajor> tmp;
              getParamMatrix(nh,*i,tmp);
              cfg.override_list_.push_back(*i);
              cfg.covariance_override_.push_back(tmp);
              ROS_INFO("%s covariance matrix found",(*i).c_str());
          }
          else
          {
              ROS_INFO("%s covariance matrix not found",(*i).c_str());

          }
      }

      std::cout << "override_list : "  << std::endl;
      for(auto& jt:cfg.override_list_)
      {
          std::cout << jt << std::endl;
      }
      std::cout << std::endl;
  }

  // retrieves the default covariance matrix
  if (nh.hasParam("cov_def"))
  {
      getParamMatrix(nh,"cov_def",cfg.def_cov_);
      ROS_INFO("cov_def parameter for default covariance found");
  }
  else
  {
      ROS_INFO("cov_def not defined");
      return ;
  }


  if (nh.hasParam("robot_description_param"))
  {
      nh.getParam("robot_description_param", urdf_param_name);
      ROS_INFO("robot description param robot_description_param :  %s", urdf_param_name.c_str());
  }

  urdf::Model model;
  if (model.initParamWithNodeHandle(urdf_param_name,nh)){
      cfg.model_ = model;
  }
  else
  {
      ROS_ERROR("Failed to parse urdf param");
      return ;
  }

  EXPECT_NO_FATAL_FAILURE(TreeStructure tree = TreeStructure(cfg));
  ros::Duration(5).sleep();

}

TEST(TestSuite2, Test2)
{
  ros::NodeHandle nh("~");
  tree_config cfg;

  std::string     urdf_param_name = "/robot_description";

  // retrieves joint state publisher name
  if (nh.hasParam("joint_pub_topic"))
  {
      nh.getParam("joint_pub_topic", cfg.joint_pub_name_);
      ROS_INFO("joint_pub_topic :  %s", cfg.joint_pub_name_.c_str());
  }

  // retrieves parameter to ingore covariance in fixed joints
  if (nh.hasParam("ignore_fixed_cov"))
  {
      nh.getParam("ignore_fixed_cov", cfg.ignore_fixed_);
      ROS_INFO("ignore_fixed_cov :  %d", cfg.ignore_fixed_);
  }


  // retrieves parameter to ingore covariance of joints value
  if (nh.hasParam("ignore_joint_cov"))
  {
      nh.getParam("ignore_joint_cov", cfg.ignore_joint_cov_);
      ROS_INFO("ignore_joint_cov :  %d", cfg.ignore_joint_cov_);
  }

  // retrieves parameter to set covariance to parameters(x,y,z,a,b,c) that are different from zero
  if (nh.hasParam("cov_only_valid"))
  {
      nh.getParam("cov_only_valid", cfg.cov_only_valid_);
      ROS_INFO("cov_only_valid :  %d", cfg.cov_only_valid_);
  }

  // retrieves a list of joints to add to the covariance publisher
  if (nh.hasParam("additional_joints"))
  {
      nh.getParam("additional_joints", cfg.jnt_add_);
      std::cout << "additional_joints : "  << std::endl;
      for(auto& jt:cfg.jnt_add_)
      {
          std::cout << jt << std::endl;
      }
      std::cout << std::endl;
  }

  // retrieves a list of joints to override covariance and their matrix
  if (nh.hasParam("override_list"))
  {
      std::vector<std::string> override_list_tmp;
      nh.getParam("override_list", override_list_tmp);

      for (std::vector<std::string>::iterator i=override_list_tmp.begin(); i!=override_list_tmp.end(); i++)
      {
          if (nh.hasParam(*i))
          {
              Eigen::Matrix<double, 6, 6, Eigen::RowMajor> tmp;
              getParamMatrix(nh,*i,tmp);
              cfg.override_list_.push_back(*i);
              cfg.covariance_override_.push_back(tmp);
              ROS_INFO("%s covariance matrix found",(*i).c_str());
          }
          else
          {
              ROS_INFO("%s covariance matrix not found",(*i).c_str());

          }
      }

      std::cout << "override_list : "  << std::endl;
      for(auto& jt:cfg.override_list_)
      {
          std::cout << jt << std::endl;
      }
      std::cout << std::endl;
  }

  // retrieves the default covariance matrix
  if (nh.hasParam("cov_def"))
  {
      getParamMatrix(nh,"cov_def",cfg.def_cov_);
      ROS_INFO("cov_def parameter for default covariance found");
  }
  else
  {
      ROS_INFO("cov_def not defined");
      return ;
  }


  if (nh.hasParam("robot_description_param"))
  {
      nh.getParam("robot_description_param", urdf_param_name);
      ROS_INFO("robot description param robot_description_param :  %s", urdf_param_name.c_str());
  }

  urdf::Model model;
  if (model.initParamWithNodeHandle(urdf_param_name,nh)){
      cfg.model_ = model;
  }
  else
  {
      ROS_ERROR("Failed to parse urdf param");
      return ;
  }


  TreeStructure tree = TreeStructure(cfg);
  ros::Duration(2).sleep();

    std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> vec_poses;
    std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> vec_posesbase;
    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> vec_c;
    std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> vec_cbase;


  for(int tx=0;tx<20;tx++)
  {

    tree.getPosesMU(vec_poses); 

    tree.getPosesBaseMU(vec_posesbase);

    tree.getPosesC(vec_c);

    tree.getPosesBaseC(vec_cbase);

    tree.randChain();
    vec_poses.clear();
    vec_posesbase.clear();
    vec_c.clear();
    vec_cbase.clear();
  }

  for(auto& el:vec_poses)
  {
    for (int j = 0; j < el.cols(); ++j) 
    {
        for (int i = 0; i < el.rows(); ++i) {
            ASSERT_FALSE(isnan(el(i,j)));
        }
    }
  }

  for(auto& el:vec_posesbase)
  {
    for (int j = 0; j < el.cols(); ++j) 
    {
        for (int i = 0; i < el.rows(); ++i) {
            ASSERT_FALSE(isnan(el(i,j)));
        }
    }
  }

  for(auto& el:vec_c)
  {
    for (int j = 0; j < el.cols(); ++j) 
    {
        for (int i = 0; i < el.rows(); ++i) {
            ASSERT_FALSE(isnan(el(i,j)));
        }
    }
  }

  for(auto& el:vec_cbase)
  {
    for (int j = 0; j < el.cols(); ++j) 
    {
        for (int i = 0; i < el.rows(); ++i) {
            ASSERT_FALSE(isnan(el(i,j)));
        }
    }
  }

  ros::Duration(2).sleep();

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PosCov_ROS_test");
  ros::NodeHandle nh("~");

  ros::Duration(2).sleep();

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  ::testing::GTEST_FLAG(filter) = "TestSuite1*";
  RUN_ALL_TESTS();
  

  ::testing::GTEST_FLAG(filter) = "TestSuite2*";
//   RUN_ALL_TESTS();
//   ros::Duration(5).sleep();
//   ros::shutdown();

  // tree.plotTree();
//  ros::waitForShutdown();
  return RUN_ALL_TESTS();
}
