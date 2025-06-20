#include "factors.h"

namespace uavfactor
{
   //------------------------------------------------------------------------------
   std::pair<Vector3, Vector3> correctMeasurementsBySensorPose(
      const Vector3& unbiasedAcc, const Vector3& unbiasedOmega, const gtsam::Pose3& body_P_sensor,
      OptionalJacobian<3, 3> correctedAcc_H_unbiasedAcc,
      OptionalJacobian<3, 3> correctedAcc_H_unbiasedOmega,
      OptionalJacobian<3, 3> correctedOmega_H_unbiasedOmega) 
   {

      // Compensate for sensor-body displacement if needed: we express the quantities
      // (originally in the IMU frame) into the body frame
      // Equations below assume the "body" frame is the CG

      // Get sensor to body rotation matrix
      const Matrix3 bRs = body_P_sensor.rotation().matrix();

      // Convert angular velocity and acceleration from sensor to body frame
      Vector3 correctedAcc = bRs * unbiasedAcc;
      const Vector3 correctedOmega = bRs * unbiasedOmega;

      // Jacobians
      if (correctedAcc_H_unbiasedAcc) *correctedAcc_H_unbiasedAcc = bRs;
      if (correctedAcc_H_unbiasedOmega) *correctedAcc_H_unbiasedOmega = Z_3x3;
      if (correctedOmega_H_unbiasedOmega) *correctedOmega_H_unbiasedOmega = bRs;

      // Centrifugal acceleration
      const Vector3 b_arm = body_P_sensor.translation();
      if (!b_arm.isZero()) {
         // Subtract out the the centripetal acceleration from the unbiased one
         // to get linear acceleration vector in the body frame:
         const Matrix3 body_Omega_body = skewSymmetric(correctedOmega);
         const Vector3 b_velocity_bs = body_Omega_body * b_arm; // magnitude: omega * arm
         correctedAcc -= body_Omega_body * b_velocity_bs;

         // Update derivative: centrifugal causes the correlation between acc and omega!!!
         if (correctedAcc_H_unbiasedOmega) {
         double wdp = correctedOmega.dot(b_arm);
         const Matrix3 diag_wdp = Vector3::Constant(wdp).asDiagonal();
         *correctedAcc_H_unbiasedOmega = -( diag_wdp
               + correctedOmega * b_arm.transpose()) * bRs.matrix()
               + 2 * b_arm * unbiasedOmega.transpose();
         }
      }

      return std::make_pair(correctedAcc, correctedOmega);
   }

   std::pair<Pose3, Vector3> propagateIMU(const gtsam::Pose3& posei, const gtsam::Vector3& veli, const gtsam::Vector3 &unbiasAcc, const gtsam::Vector3 &unbiasGyro, const float dt)
   {
      float          dtt  = dt * dt;
      gtsam::Vector3 gI   = gtsam::Vector3(0, 0, 9.81);
      gtsam::Rot3    wRbi = posei.rotation();
      gtsam::Vector3 pj   = posei.translation() + veli * dt + 0.5f * ( -gI + wRbi.rotate(unbiasAcc) ) * dtt;
      gtsam::Rot3    rj   = wRbi* gtsam::Rot3::Expmap(unbiasGyro * dt);
      gtsam::Vector3 velj = veli - gI * dt + wRbi.rotate(unbiasAcc) * dt;
      gtsam::Pose3   posej(rj, pj);
      return std::make_pair(posej, velj);
   }

   DynamicsFactorTGyro::DynamicsFactorTGyro(Key p_i, Key vel_i, Key input_i, Key p_j, Key vel_j, float dt, double mass, gtsam::Vector3 drag_k, const SharedNoiseModel &model)
       : Base(model, p_i, vel_i, input_i, p_j, vel_j), dt_(dt), mass_(mass), drag_k_(drag_k) {};

   Vector DynamicsFactorTGyro::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector4 &input_i,
                                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j,
                                             boost::optional<Matrix &> H1, boost::optional<Matrix &> H2, boost::optional<Matrix &> H3, boost::optional<Matrix &> H4, boost::optional<Matrix &> H5) const
   {
      gtsam::Vector9 err;

      Matrix36 jac_t_posei, jac_t_posej;
      Matrix36 jac_r_posei, jac_r_posej;

      const Point3 p_w_mi = pos_i.translation(jac_t_posei);
      const Rot3 r_w_mi = pos_i.rotation(jac_r_posei);
      const Point3 p_w_mj = pos_j.translation(jac_t_posej);
      const Rot3 r_w_mj = pos_j.rotation(jac_r_posej);

      double dtt = dt_ * dt_;
      gtsam::Matrix33 _un_rbi = r_w_mi.inverse().matrix();

      gtsam::Matrix33 J_rwg, J_pe_roti, J_ve_rot1, J_dv_rit, J_dv_v;
      gtsam::Matrix33 J_ri, J_rj, J_dr;

      gtsam::Vector3 pos_err = mass_ * r_w_mi.unrotate(p_w_mj - vel_i * dt_ + 0.5f * gI_ * dtt - p_w_mi, J_pe_roti) - 0.5f * gtsam::Vector3(0, 0, 1) * input_i(0) * dtt;
      gtsam::Vector3 rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - input_i.tail(3) * dt_;

      gtsam::Matrix3 drag_matrix;
      drag_matrix.setZero();
      // drag_matrix.diagonal() << drag_k_;

      // Suction force F in the world frame = 
      // Fx = 0, Fz = 0, Fy = k* (y - D + D1) k = 8 N/m, D ? D1 = 0.10m
      float F_w_suction = drag_k_[0] * (p_w_mi.y() - drag_k_[1] + drag_k_[2]);
      if(F_w_suction < 0)
      {
         F_w_suction = 0;
      }
      if(F_w_suction >= 1.5)
      {
         F_w_suction = 1.5;
      }
      gtsam::Vector3 F_w_suction3 = gtsam::Vector3::Zero();
      F_w_suction3 << 0, F_w_suction, 0;

      printf(" F_w_suction: [ %f ] \n", F_w_suction);

      gtsam::Vector3 vel_err = mass_ * r_w_mi.unrotate(vel_j - vel_i + gI_ * dt_ - F_w_suction3 * dt_, J_ve_rot1) - gtsam::Vector3(0, 0, 1) * input_i(0) * dt_;
      // - drag_matrix * r_w_mi.unrotate(vel_i, J_dv_rit, J_dv_v) * dt_; // - dT * dt_;

      Matrix96 J_e_pi, J_e_posej;

      if (H1)
      {
         Matrix33 Jac_perr_p = -mass_ * _un_rbi;
         Matrix33 Jac_perr_r = mass_ * J_pe_roti;
         Matrix33 Jac_rerr_r = J_dr * J_ri;
         Matrix33 Jac_verr_r = mass_ * J_ve_rot1 - drag_matrix * J_dv_rit * dt_; // - A_mat * J_da_ri * dt_;

         Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei + Jac_perr_r * jac_r_posei;
         Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
         Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;

         J_e_pi.setZero();
         J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
         J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
         J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

         *H1 = J_e_pi;
      }

      if (H2)
      {
         Matrix93 J_e_v;
         J_e_v.setZero();
         Matrix33 Jac_perr_veli = -mass_ * _un_rbi * dt_;
         Matrix33 Jac_verr_v = -mass_ * _un_rbi;
         J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
         J_e_v.block(6, 0, 3, 3) = Jac_verr_v - drag_matrix * dt_ * J_dv_v;

         *H2 = J_e_v;
      }

      if (H4)
      {
         J_e_posej.setZero();
         J_e_posej.block(0, 0, 3, 6) = mass_ * _un_rbi * jac_t_posej;
         J_e_posej.block(3, 0, 3, 6) = J_dr * J_rj * jac_r_posej;
         *H4 = J_e_posej;
      }

      if (H5)
      {
         Matrix93 J_e_vj;
         J_e_vj.setZero();
         J_e_vj.block(6, 0, 3, 3) = mass_ * _un_rbi;
         *H5 = J_e_vj;
      }

      if (H3)
      {
         Matrix94 J_T_gyro;
         J_T_gyro.setZero();
         J_T_gyro.block(3, 1, 3, 3) = -gtsam::I_3x3 * dt_;
         J_T_gyro.block(6, 0, 3, 1) = -gtsam::Vector3(0, 0, 1) * dt_;

         *H3 = J_T_gyro;
      }

      err.head(3) = pos_err;
      err.block(3, 0, 3, 1) = rot_err;
      err.block(6, 0, 3, 1) = vel_err;

      return err;
   }

   IMUFactor::IMUFactor(Key p_i, Key vel_i, Key bias_i, Key p_j, Key vel_j,
                        float dt, gtsam::Vector3 acc, gtsam::Vector3 gyro, const SharedNoiseModel &model)
       : Base(model, p_i, vel_i, bias_i, p_j, vel_j), dt_(dt), acc_(acc), gyro_(gyro) {}

   Vector IMUFactor::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i,
                                   const gtsam_imuBi &bias_i,
                                   const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j,
                                   boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                                   boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
                                   boost::optional<Matrix &> H5) const
   {
      gtsam::Vector9 err;

      Matrix36 jac_t_posei, jac_t_posej;
      Matrix36 jac_r_posei, jac_r_posej;

      const Point3 p_w_mi = pos_i.translation(jac_t_posei);
      const Rot3 r_w_mi   = pos_i.rotation(jac_r_posei);
      const Point3 p_w_mj = pos_j.translation(jac_t_posej);
      const Rot3 r_w_mj   = pos_j.rotation(jac_r_posej);

      double dtt = dt_ * dt_;
      gtsam::Matrix33 _un_rbi = r_w_mi.inverse().matrix();

      gtsam::Matrix33 J_rwg, J_pe_roti, J_ve_rot1, J_dv_rit, J_dv_v;
      gtsam::Matrix33 J_ri, J_rj, J_dr;
      gtsam::Matrix36 J_acc_bias, J_gyro_bias;
      gtsam::Vector3  cor_acc  = bias_i.correctAccelerometer(acc_, J_acc_bias);
      gtsam::Vector3  cor_gyro = bias_i.correctGyroscope(gyro_, J_gyro_bias);
      
      gtsam::Vector3 pos_err = r_w_mi.unrotate(p_w_mj - vel_i * dt_ + 0.5f * gI_ * dtt - p_w_mi, J_pe_roti) - 0.5f * cor_acc * dtt;
      gtsam::Vector3 rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - cor_gyro * dt_;
      gtsam::Vector3 vel_err = r_w_mi.unrotate(vel_j - vel_i + gI_ * dt_, J_ve_rot1)  - cor_acc * dt_;

      Matrix96 J_e_pi, J_e_posej;

      if (H1)
      {
         Matrix33 Jac_perr_p = - _un_rbi;
         Matrix33 Jac_perr_r = J_pe_roti;
         Matrix33 Jac_rerr_r = J_dr * J_ri;
         Matrix33 Jac_verr_r = J_ve_rot1; // - A_mat * J_da_ri * dt_;

         Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei + Jac_perr_r * jac_r_posei;
         Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
         Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;

         J_e_pi.setZero();
         J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
         J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
         J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

         *H1 = J_e_pi;
      }

      if (H2)
      {
         Matrix93 J_e_v;
         J_e_v.setZero();
         Matrix33 Jac_perr_veli = -_un_rbi * dt_;
         Matrix33 Jac_verr_v = -_un_rbi;
         J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
         J_e_v.block(6, 0, 3, 3) = Jac_verr_v;

         *H2 = J_e_v;
      }

      if (H4)
      {
         J_e_posej.setZero();
         J_e_posej.block(0, 0, 3, 6) = _un_rbi * jac_t_posej;
         J_e_posej.block(3, 0, 3, 6) = J_dr * J_rj * jac_r_posej;
         *H4 = J_e_posej;
      }

      if (H5)
      {
         Matrix93 J_e_vj;
         J_e_vj.setZero();
         J_e_vj.block(6, 0, 3, 3) = _un_rbi;
         *H5 = J_e_vj;
      }

      if (H3)
      {
         Matrix96 J_bias;
         J_bias.setZero();
         J_bias.block(0, 0, 3, 6) = - 0.5f * J_acc_bias * dtt; // p_err w.r.t bias
         J_bias.block(3, 0, 3, 6) = - J_gyro_bias * dt_;        // r_err w.r.t bias
         J_bias.block(6, 0, 3, 6) = - J_acc_bias * dt_;        // v_err w.r.t bias

         *H3 = J_bias;
      }

      err.head(3) = pos_err;
      err.block(3, 0, 3, 1) = rot_err;
      err.block(6, 0, 3, 1) = vel_err;

      return err;
   }

   IMUFactorRg::IMUFactorRg(Key p_i, Key vel_i, Key bias_i, Key p_j, Key vel_j, Key Rg,
                        float dt, gtsam::Vector3 acc, gtsam::Vector3 gyro, const SharedNoiseModel &model)
       : Base(model, p_i, vel_i, bias_i, p_j, vel_j, Rg), dt_(dt), acc_(acc), gyro_(gyro) {}

   Vector IMUFactorRg::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i,
                                     const gtsam_imuBi &bias_i,
                                     const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j,
                                     const gtsam::Rot3& Rg, 
                                     boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                                     boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
                                     boost::optional<Matrix &> H5, boost::optional<Matrix &> H6) const
   {
      gtsam::Vector9 err;

      Matrix36 jac_t_posei, jac_t_posej;
      Matrix36 jac_r_posei, jac_r_posej;
      Matrix93 jac_Rg;
      double dtt = dt_ * dt_;

      const Point3 p_w_mi = pos_i.translation(jac_t_posei);
      const Rot3   r_w_mi = pos_i.rotation(jac_r_posei);
      const Point3 p_w_mj = pos_j.translation(jac_t_posej);
      const Rot3   r_w_mj = pos_j.rotation(jac_r_posej);

      gtsam::Matrix33 J_rwg, J_pe_roti, J_ve_rot1, J_dv_rit, J_dv_v, J_rg;
      gtsam::Matrix33 J_ri, J_rj, J_dr;
      gtsam::Matrix36 J_acc_bias, J_gyro_bias;
      gtsam::Vector3  cor_acc  = bias_i.correctAccelerometer(acc_, J_acc_bias);
      gtsam::Vector3  cor_gyro = bias_i.correctGyroscope(gyro_, J_gyro_bias);

      gtsam::Matrix33 _un_rbi = r_w_mi.inverse().matrix();
      gtsam::Vector3  pos_err = r_w_mi.unrotate(p_w_mj - vel_i * dt_ - 0.5f * Rg.rotate(gI_) * dtt - p_w_mi, J_pe_roti) - 0.5f * cor_acc * dtt;
      gtsam::Vector3  rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr)                  - cor_gyro * dt_;
      gtsam::Vector3  vel_err = r_w_mi.unrotate(vel_j - vel_i + Rg.rotate(gI_, J_rg) * dt_, J_ve_rot1)  - cor_acc * dt_;

      Matrix96 J_e_pi, J_e_posej;

      if (H6)
      {
         jac_Rg.setZero();
         jac_Rg.block(0, 0, 3, 3) = _un_rbi * -0.5f * J_rg * dtt;
         jac_Rg.block(6, 0, 3, 3) = _un_rbi * J_rg * dt_;
         *H6 = jac_Rg;
      }

      if (H1)
      {
         Matrix33 Jac_perr_p = - _un_rbi;
         Matrix33 Jac_perr_r = J_pe_roti;
         Matrix33 Jac_rerr_r = J_dr * J_ri;
         Matrix33 Jac_verr_r = J_ve_rot1; // - A_mat * J_da_ri * dt_;

         Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei + Jac_perr_r * jac_r_posei;
         Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
         Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;

         J_e_pi.setZero();
         J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
         J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
         J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

         *H1 = J_e_pi;
      }

      if (H2)
      {
         Matrix93 J_e_v;
         J_e_v.setZero();
         Matrix33 Jac_perr_veli  = -_un_rbi * dt_;
         Matrix33 Jac_verr_v     = -_un_rbi;
         J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
         J_e_v.block(6, 0, 3, 3) = Jac_verr_v;

         *H2 = J_e_v;
      }

      if (H4)
      {
         J_e_posej.setZero();
         J_e_posej.block(0, 0, 3, 6) = _un_rbi * jac_t_posej;
         J_e_posej.block(3, 0, 3, 6) = J_dr * J_rj * jac_r_posej;
         *H4 = J_e_posej;
      }

      if (H5)
      {
         Matrix93 J_e_vj;
         J_e_vj.setZero();
         J_e_vj.block(6, 0, 3, 3) = _un_rbi;
         *H5 = J_e_vj;
      }

      if (H3)
      {
         Matrix96 J_bias;
         J_bias.setZero();
         J_bias.block(0, 0, 3, 6) = - 0.5f * J_acc_bias * dtt; // p_err w.r.t bias
         J_bias.block(3, 0, 3, 6) = - J_gyro_bias * dt_;        // r_err w.r.t bias
         J_bias.block(6, 0, 3, 6) = - J_acc_bias * dt_;        // v_err w.r.t bias

         *H3 = J_bias;
      }

      err.head(3) = pos_err;
      err.block(3, 0, 3, 1) = rot_err;
      err.block(6, 0, 3, 1) = vel_err;

      return err;
   }

   Vector ControlLimitTGyroFactor::evaluateError(const gtsam::Vector4 &input, boost::optional<Matrix &> H1) const
   {
      gtsam::Vector4 error;
      gtsam::Matrix4 jac;
      jac.setZero();

      uint i = 0;
      if (input[i] >= T_low_ + T_thr_ && input[i] <= T_high_ - T_thr_)
      {
         error(i) = 0;
         jac(i, i) = 0;
      }
      else if (input[i] < T_low_ + T_thr_)
      {
         error(i) = alpha_ * (T_low_ + T_thr_ - input[i]);
         jac(i, i) = -alpha_;
      }
      else
      {
         error(i) = alpha_ * (input[i] - T_high_ + T_thr_);
         jac(i, i) = alpha_;
      }

      for (i = 1; i < 4; i++)
      {
         if (input[i] >= Gyro_low_ + Gyro_thr_ && input[i] <= Gyro_high_ - Gyro_thr_)
         {
            error(i) = 0;
            jac(i, i) = 0;
         }
         else if (input[i] < Gyro_low_ + Gyro_thr_)
         {
            error(i) = alpha_ * (Gyro_low_ + Gyro_thr_ - input[i]);
            jac(i, i) = -alpha_;
         }
         else
         {
            error(i) = alpha_ * (input[i] - Gyro_high_ + Gyro_thr_);
            jac(i, i) = alpha_;
         }
      }
      if (H1)
      {
         *H1 = jac;
      }

      return error;
   }

   // Force and moments between factor.
   BetForceMoments::BetForceMoments(Key input_i, Key input_j, const SharedNoiseModel &model)
       : Base(model, input_i, input_j) {};

   Vector BetForceMoments::evaluateError(const gtsam::Vector4 &input_i, const gtsam::Vector4 &input_j,
                                         boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const
   {
      gtsam::Vector4 err;
      err = input_j - input_i;
      if (H1)
      {
         *H1 = -gtsam::Matrix4::Identity();
      }
      if (H2)
      {
         *H2 = gtsam::Matrix4::Identity();
      }

      return err;
   }

}
