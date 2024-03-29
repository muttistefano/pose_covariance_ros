#include <pose_covariance_ros/pose_covariance_ros.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PosCov_ROS_test");
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
        ROS_INFO_STREAM("additional_joints : \n");
        for(auto& jt:cfg.jnt_add_)
        {
            ROS_INFO_STREAM(jt << "\n");
        }
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

        ROS_INFO_STREAM("override_list: \n");
        for(auto& jt:cfg.override_list_)
        {
            ROS_INFO_STREAM(jt << "\n");
        }
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
        return -1;
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
        return -1;
    }
    


    TreeStructure tree = TreeStructure(cfg);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    ros::Duration(5).sleep();  

    // tree.plotTree();
    ros::waitForShutdown();
    // tree.plot_tree();
    // tree.compute_chain();
    // tree.plot_tree();
    // ros::spin();     

    // ros::AsyncSpinner spinner(4); // Use 4 threads
    // spinner.start(); 
    // ros::Duration(5).sleep();  
 
    // tf::TransformListener listener;
    // listener.getFrames()

}
