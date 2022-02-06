#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <chrono>
#include <pose_lie/pose_lie.hpp>

//namespace PoseCov3
//{


PoseCov3Ns::PoseCov3::PoseCov3()
    : MU_(),R_(),t_(),C_(),v_(),generator_()
{
  MU_ = M4R::Identity();
  MU_base_ = M4R::Identity();
  R_  = M3Rrot::Identity();
  t_  = V3trans(0,0,0);
  C_  = M6::Zero();
  v_  = V3rot::Zero();
  generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

PoseCov3Ns::PoseCov3::PoseCov3(const PoseCov3& pose_in)
{
  MU_   = pose_in.getMU();
  MU_base_   = pose_in.getMU();
  R_    = pose_in.R_;
  t_    = pose_in.t_;
  C_    = pose_in.C_;
  v_    = pose_in.v_;
  axis_ = pose_in.axis_;
  type_ = pose_in.type_;
  generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
  // std::cout << "copy called" << pose_in.getC();
}

PoseCov3Ns::PoseCov3::PoseCov3(const M4R& M4_in)
    : MU_(M4_in),MU_base_(M4_in),R_(),t_(),C_(),v_(),generator_()
{
    R_ = MU_.block(0, 0, 3, 3);
    t_ = MU_.block(0, 3, 3, 1);
    v_ = utilsPoseCov::R2rpy(R_);
    C_ = M6::Zero();
    generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

PoseCov3Ns::PoseCov3::PoseCov3(const M4R& M4_in, const M6& C_in)
    : MU_(M4_in),MU_base_(M4_in),R_(),t_(),C_(C_in),generator_()
{
    R_ = MU_.block(0, 0, 3, 3);
    t_ = MU_.block(0, 3, 3, 1);
    v_ = utilsPoseCov::R2rpy(R_);
    generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

PoseCov3Ns::PoseCov3::PoseCov3(const M3Rrot& R, const V3trans& t)
    : MU_(),R_(R),t_(t),C_(),v_(),generator_()
{
  MU_.block(0, 0, 3, 3) = R;
  MU_.block(0, 3, 3, 1) = t;
  v_ = utilsPoseCov::R2rpy(R);

  MU_(3, 0) = 0;
  MU_(3, 1) = 0;
  MU_(3, 2) = 0;
  MU_(3, 3) = 1;
  C_ = M6::Zero();
  MU_base_= MU_;
  generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

PoseCov3Ns::PoseCov3::PoseCov3(double x,double y,double z,double r,double p,double w,int type, M3ax axis)
    : MU_(),MU_base_(),R_(),t_(),C_(),v_(),generator_(),type_(type),axis_(axis)
{

  utilsPoseCov::rpy_to_mat(r, p, w, R_);
  MU_.block(0, 0, 3, 3) = R_;
  v_ = V3rot(r,p,w);
  t_ << x,y,z;
  MU_(0, 3) = x;
  MU_(1, 3) = y;
  MU_(2, 3) = z;

  MU_(3, 0) = 0;
  MU_(3, 1) = 0;
  MU_(3, 2) = 0;
  MU_(3, 3) = 1;
  C_ = M6::Zero();
  MU_base_= MU_;
  generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

PoseCov3Ns::PoseCov3::PoseCov3(double x,double y,double z,Eigen::Quaterniond q,int type, M3ax axis)
    : MU_(),MU_base_(),R_(),t_(),C_(),v_(),generator_(),type_(type),axis_(axis)
{
  R_ = q.normalized().toRotationMatrix();
  v_ = q.normalized().toRotationMatrix().eulerAngles(0, 1, 2);
  MU_.block(0, 0, 3, 3) = R_;
  t_ << x,y,z;
  MU_(0, 3) = x;
  MU_(1, 3) = y;
  MU_(2, 3) = z;

  MU_(3, 0) = 0;
  MU_(3, 1) = 0;
  MU_(3, 2) = 0;
  MU_(3, 3) = 1;
  C_ = M6::Zero();
  MU_base_= MU_;
  generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

void PoseCov3Ns::PoseCov3::setC(M6 C_in)
{
  C_ = C_in;
}

void PoseCov3Ns::PoseCov3::update(double inp)
{
  M4R     tmp_mat = M4R::Identity();
  M3Rrot  tmp_rot = M3Rrot::Identity();


  if (type_ == 1 || type_ == 2)
  {
     utilsPoseCov::rpy_to_mat(inp * axis_(0), inp * axis_(1), inp * axis_(2), tmp_rot);
     tmp_mat.block(0, 0, 3, 3) = tmp_rot;
  }

  if (type_ == 3 )
  {
    tmp_mat(0,3) = inp * axis_(0);
    tmp_mat(1,3) = inp * axis_(1);
    tmp_mat(2,3) = inp * axis_(2);
  }
  //TODO fai giunto lineare

  MU_ = MU_base_ * tmp_mat;
  R_ = MU_.block(0, 0, 3, 3);
  t_ = MU_.block(0, 3, 3, 1);
  v_ = utilsPoseCov::R2rpy(R_);

}

void PoseCov3Ns::PoseCov3::fix_joint_cov()
{

  if (type_ == 1 || type_ == 2)
  {
    if(axis_(0)==1) C_(3,3) = 0;
    if(axis_(1)==1) C_(4,4) = 0;
    if(axis_(2)==1) C_(5,5) = 0;
  }
  else if (type_ == 3 )
  {
    if(axis_(0)==1) C_(0,0) = 0;
    if(axis_(1)==1) C_(1,1) = 0;
    if(axis_(2)==1) C_(2,2) = 0;
  }
  else if (type_ == 6 )
  {
    return;
  }
  else
  {
    std::cout << "FIX_JOINT_COV CALLED FOR WRONG TYPE OF JOINT!" << std::endl;
  }
}

PoseCov3Ns::M4R PoseCov3Ns::PoseCov3::getMU() const
{
    return MU_;
}

PoseCov3Ns::M3Rrot PoseCov3Ns::PoseCov3::getR() const
{
    return R_;
}

PoseCov3Ns::M6 PoseCov3Ns::PoseCov3::getC() const
{
  return C_;
}

Eigen::Vector3d PoseCov3Ns::PoseCov3::gett() const
{
    return t_;
}

Eigen::Vector3d PoseCov3Ns::PoseCov3::getv() const
{
    return v_;
}

PoseCov3Ns::PoseCov3 PoseCov3Ns::PoseCov3::inverseMU()
{
  PoseCov3 inv_pose(MU_.inverse());
  return inv_pose;
}

PoseCov3Ns::PoseCov3 PoseCov3Ns::PoseCov3::operator*(const PoseCov3& PoseCov3_in) const
{
  PoseCov3 out(MU_ * PoseCov3_in.getMU());
  return out;
}

PoseCov3Ns::PoseCov3 PoseCov3Ns::PoseCov3::LieExp(const V6& xi)
{
  Eigen::Vector3d pos = xi.head<3>();
  Eigen::Vector3d rot = xi.tail<3>();

  M4R out_mat = M4R::Identity();

  double theta = sqrt( rot(0) * rot(0) + rot(1) * rot(1) + rot(2) * rot(2));

  if(pow(theta, 2.0) <= std::numeric_limits<double>::epsilon())
  {
    out_mat.block(0,0,3,3) = M3Rrot::Identity();
    out_mat.block(0,3,3,1) = pos;
    return PoseCov3(out_mat);
  }

  auto a = rot/theta;
  M3Rrot exp_mat = utilsPoseCov::make_skew(a);
  out_mat.block(0,0,3,3) = cos(theta)*M3Rrot::Identity() + (1-cos(theta))*a*a.transpose() + sin(theta) * exp_mat;
  out_mat.block(0,3,3,1) = ((sin(theta)/theta)*M3Rrot::Identity() + (1-sin(theta)/theta)*a*a.transpose() + ((1-cos(theta))/theta) * exp_mat) * pos;
  return PoseCov3(out_mat);
}

PoseCov3Ns::V6 PoseCov3Ns::PoseCov3::LieLog()
{

  V6    xi;
  V3rot wv;
  double theta = acos((R_.trace() - 1.)/2.);

  if(pow(theta, 2.0) <= std::numeric_limits<double>::epsilon())
  {
    wv << 0,0,0;
  }
  else
  {
    M3Rrot log_mat = (theta/(2.*sin(theta)))*(R_ - R_.transpose());
    wv = Eigen::Vector3d(log_mat(2, 1), -log_mat(2, 0), log_mat(1, 0));
  }

  xi.tail<3>() = wv;

  M3Rrot Jinv;
  double theta_J = sqrt(  wv(0) * wv(0) + wv(1) * wv(1) + wv(2) * wv(2));

  if(pow(theta_J, 2.0) <= std::numeric_limits<double>::epsilon())
  {
     Jinv = M3Rrot::Identity();
  }
  else
  {
    auto a =  wv/theta_J;
    M3Rrot ji_mat = utilsPoseCov::make_skew(a);
    double cot_theta_half = cos(theta_J/2.0)/sin(theta_J/2.0);
    M3Rrot Jinv = (theta_J/2.0)*cot_theta_half*M3Rrot::Identity() + (1.0 - (theta_J/2.0)*cot_theta_half)*a*a.transpose() - (theta_J/2.0)*ji_mat;
  }

  xi.head<3>() = Jinv*t_;

  return xi;
}

PoseCov3Ns::M6 PoseCov3Ns::PoseCov3::LieAd()
{

  V6 xi = this->LieLog();
  Eigen::Vector3d phi = xi.tail<3>();

  double theta = sqrt( phi(0) * phi(0) + phi(1) * phi(1) + phi(2) * phi(2));

  M3Rrot J;

  if(pow(theta, 2.0) <= std::numeric_limits<double>::epsilon())
  {
    J = M3Rrot::Identity();
  }
  else
  {
    auto a = phi/theta;
    M3Rrot j_mat = utilsPoseCov::make_skew(a);
    J = (sin(theta)/theta)*M3Rrot::Identity() + (1-sin(theta)/theta)*a*a.transpose() + ((1-cos(theta))/theta) * j_mat;
  }
  M6 ad_T;
  ad_T.block(0, 0, 3, 3) = R_;
  ad_T.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
  ad_T.block(3, 3, 3, 3) = R_;
  ad_T.block(0, 3, 3, 3) = utilsPoseCov::make_skew(J * xi.head<3>()) * R_;

  return ad_T;
}

PoseCov3Ns::PoseCov3 PoseCov3Ns::PoseCov3::compose1(const PoseCov3& pos_in)
{

  M4R mu_out         = MU_ * pos_in.getMU()  ;
  M6  Ad_mat         = this->LieAd();
  M6  cov1           = C_ + Ad_mat * pos_in.getC() * Ad_mat.transpose();
  return PoseCov3(mu_out,cov1);
}

PoseCov3Ns::PoseCov3 PoseCov3Ns::PoseCov3::compose4(const PoseCov3& pos_in)
{

  M4R mu_out         = MU_ * pos_in.getMU()  ;
  M6  Ad_mat         = this->LieAd();

  M6 A1 = M6::Zero();
  M6 A2 = M6::Zero();
  M6 B1 = M6::Zero();

  A1.block(0, 0, 3, 3) = utilsPoseCov::op_single(C_.block(3,3,3,3));
  A1.block(3, 3, 3, 3) = utilsPoseCov::op_single(C_.block(3,3,3,3));
  A1.block(0, 3, 3, 3) = utilsPoseCov::op_single(C_.block(0,3,3,3) + C_.block(0,3,3,3).transpose());

  A2.block(0, 0, 3, 3) = utilsPoseCov::op_single(pos_in.getC().block(3,3,3,3));
  A2.block(3, 3, 3, 3) = utilsPoseCov::op_single(pos_in.getC().block(3,3,3,3));
  A2.block(0, 3, 3, 3) = utilsPoseCov::op_single(pos_in.getC().block(0,3,3,3) + pos_in.getC().block(0,3,3,3).transpose());

  B1.block(0, 0, 3, 3) = utilsPoseCov::op_multi(C_.block(3,3,3,3),pos_in.getC().block(0,0,3,3)) +
                         utilsPoseCov::op_multi(C_.block(0,3,3,3).transpose(),pos_in.getC().block(0,3,3,3)) +
                         utilsPoseCov::op_multi(C_.block(0,3,3,3),pos_in.getC().block(0,3,3,3).transpose()) +
                         utilsPoseCov::op_multi(C_.block(0,0,3,3),pos_in.getC().block(3,3,3,3)) ;

  B1.block(0, 3, 3, 3) = utilsPoseCov::op_multi(C_.block(3,3,3,3),pos_in.getC().block(0,3,3,3).transpose()) +
                         utilsPoseCov::op_multi(C_.block(0,3,3,3).transpose(),pos_in.getC().block(3,3,3,3));

  B1.block(3, 0, 3, 3) = B1.block(0, 3, 3, 3).transpose();

  B1.block(3, 3, 3, 3) = utilsPoseCov::op_multi(C_.block(3,3,3,3),pos_in.getC().block(3,3,3,3));


  M6  sig2             =  Ad_mat * pos_in.getC() * Ad_mat.transpose();
  M6  cov1             = C_ + sig2 + (1.0/12.0)*(A1 * sig2 + sig2 * A1.transpose() + A2 * C_ + C_ * A2.transpose()) + 0.25 * B1 ;

  PoseCov3 result = PoseCov3(mu_out,cov1);

  return result;
}

PoseCov3Ns::PoseCov3 PoseCov3Ns::PoseCov3::composeJointed(const PoseCov3& pos_in, M6& joint_cov)
{

  M4R mu_out         = MU_ * pos_in.getMU()  ;
  M6  Ad_mat         = this->LieAd();
  M6  cov1           = C_ + Ad_mat * pos_in.getC() * Ad_mat.transpose() + joint_cov * Ad_mat.transpose() + Ad_mat * joint_cov.transpose();

  PoseCov3 result = PoseCov3(mu_out,cov1);

  return result;
}

PoseCov3Ns::PoseCov3 PoseCov3Ns::PoseCov3::Lieinverse()
{
  PoseCov3 invMU = this->inverseMU();

  M6 LieInv = invMU.LieAd();
  M6 cov_inv = LieInv * C_ * LieInv.transpose();
  invMU.setC(cov_inv);
  return invMU;
}

void PoseCov3Ns::PoseCov3::draw_sample(M4R& mat1, M4R& mat2)
{
  PoseCov3 out;

  std::normal_distribution<double> distribution1(t_(0),C_(0,0));
  std::normal_distribution<double> distribution2(t_(1),C_(1,1));
  std::normal_distribution<double> distribution3(t_(2),C_(2,2));
  std::normal_distribution<double> distribution4(v_(0),C_(3,3));
  std::normal_distribution<double> distribution5(v_(1),C_(4,4));
  std::normal_distribution<double> distribution6(v_(2),C_(5,5));

  double d1 = double(distribution1(generator_));
  double d2 = double(distribution2(generator_));
  double d3 = double(distribution3(generator_));
  double d4 = double(distribution4(generator_));
  double d5 = double(distribution5(generator_));
  double d6 = double(distribution6(generator_));

  V6 vin;
  vin <<      d1 - t_(0),
              d2 - t_(1),
              d3 - t_(2),
              d4 - v_(0),
              d5 - v_(1),
              d6 - v_(2);

  out = LieExp(vin) * PoseCov3(MU_);

  PoseCov3 out2(d1,d2,d3,d4,d5,d6,type_,axis_);

  mat1 = out.getMU();
  mat2 = out2.getMU();

}




// std::ostream& operator<<(std::ostream& os, const PoseCov3& rhs)  
// {
//   os << rhs.getMU();
//   return os;
// }

//};

