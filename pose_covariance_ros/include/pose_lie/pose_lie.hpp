#ifndef __POSE_COV_HPP
#define __POSE_COV_HPP


#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <chrono>

namespace utils_PoseCov
{
    Eigen::Vector3d R2rpy(const  Eigen::Matrix<double, 3, 3, Eigen::RowMajor>& R)
    {
      Eigen::Vector3d rpy(3);
      rpy(2) = atan2(R(1,0), R(0,0));

      double s = sin(rpy(2));
      double c = cos(rpy(2));
        
      rpy(1) = atan2(-R(2,0),R(0,0)*c+R(1,0)*s);
      rpy(0) = atan2(R(0,2)*s-R(1,2)*c,-R(0,1)*s+ R(1,1)*c);

      return rpy;
    }

    bool  rpy_to_mat(double r, double p, double y, Eigen::Matrix<double, 3, 3, Eigen::RowMajor> &R_mat)
    {
        double sr = sin(r);
        double cr = cos(r);
        double sp = sin(p);
        double cp = cos(p);
        double sy = sin(y);
        double cy = cos(y);    
        
        R_mat <<
            cy*cp, -sy*cr + cy*sp*sr,  sy*sr + cy*sp*cr,
            sy*cp,  cy*cr + sy*sp*sr, -cy*sr + sy*sp*cr,
            -sp,    cp*sr,             cp*cr;
        
        return true;
    }

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> make_skew(const Eigen::Vector3d& vec)
    {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor>  v_out;
        v_out <<
            0      , -vec(2),  vec(1),
            vec(2) , 0      , -vec(0),
            -vec(1), vec(0) ,     0;

        return v_out;
    }

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> op_single(Eigen::Matrix<double, 3, 3, Eigen::RowMajor> A)
    {
      return -1 * A.trace() * Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity() + A;
    }

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> op_multi(Eigen::Matrix<double, 3, 3, Eigen::RowMajor> A, Eigen::Matrix<double, 3, 3, Eigen::RowMajor> B)
    {
      return op_single(A) * op_single(B) + op_single(A*B);
    }

}

namespace PoseCov3_ns
{

typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> M4R;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> M3Rrot;
typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> M6;
typedef Eigen::Matrix<double, 6, 1>                  V6;
typedef Eigen::Vector3d                              V3trans;
typedef Eigen::Vector3d                              V3rot;
typedef Eigen::Vector3d                              M3ax;

class PoseCov3 
{
 private:
  M4R        MU_;
  M4R        MU_base_;
  M3Rrot     R_;
  V3trans    t_;
  M6         C_;
  V3rot      v_;
  M3ax       axis_;
  int        type_;
  std::default_random_engine generator_;
  

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseCov3();
  
  PoseCov3(const PoseCov3& pose_in);

  PoseCov3(const M4R& M4_in);

  PoseCov3(const M4R& M4_in, const M6& C_in);

  PoseCov3(const M3Rrot& R, const V3trans& t);
  
  PoseCov3(double x,double y,double z,double r,double p,double w,int type, M3ax axis);
  
  PoseCov3(double x,double y,double z,Eigen::Quaterniond q,int type, M3ax axis);

  ~PoseCov3() {}

  void setC(M6 C_in);

  void update(double inp);

  void fix_joint_cov();

  M4R getMU() const;

  M3Rrot getR()  const;

  M6 getC()  const;

  Eigen::Vector3d gett() const;
  
  Eigen::Vector3d getv() const;

  PoseCov3 inverseMU();

  PoseCov3 operator*(const PoseCov3& PoseCov3_in) const;

  //TODO static?
  PoseCov3 LieExp(const V6& xi);

  V6 LieLog();

  M6 LieAd();

  PoseCov3 compose1(const PoseCov3& pos_in);
  
  PoseCov3 compose4(const PoseCov3& pos_in);

  PoseCov3 composeJointed(const PoseCov3& pos_in, M6& joint_cov);
  
  PoseCov3 Lieinverse();
  
  void draw_sample(M4R& mat1, M4R& mat2);

};

}; 

#endif
