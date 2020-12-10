#include "drake/examples/cassie/cassie.h"

#define PI 3.1416

namespace drake {
namespace examples {
namespace cassie{

  Cassie::Cassie()
  {  
    DeclareDiscreteState(131); 
    DeclareVectorOutputPort("floating_base_state", drake::systems::BasicVector<double>(131),
                            &Cassie::CopyDiscreteStateOut);
    DeclarePeriodicDiscreteUpdate(0.001);
  }

  void Cassie::InitSystem(
      std::vector<Eigen::Matrix<double, 9, 1>> COM_l,
      std::vector<Eigen::Matrix<double, 9, 1>> l_foot_l,
      std::vector<Eigen::Matrix<double, 9, 1>> r_foot_l,
      std::vector<Eigen::Matrix<double, 6, 1>> l_wrist_l,
      std::vector<Eigen::Matrix<double, 6, 1>> r_wrist_l,
      std::vector<Eigen::Matrix<double, 1, 1>> heading_l,
      std::vector<Eigen::Matrix<double, 3, 1>> box_l,
      std::vector<Eigen::Matrix<double, 3, 1>> quadrotor_l,
      std::vector<Eigen::Matrix<double, 1, 1>> heading_quadrotor_l,
      std::vector<Eigen::Matrix<double, 3, 1>> box_l1,
      std::vector<Eigen::Matrix<double, 1, 1>> heading_box1_l,
      std::vector<Eigen::Matrix<double, 3, 1>> box_l2,
      std::vector<Eigen::Matrix<double, 1, 1>> heading_box2_l,
      std::vector<Eigen::Matrix<double, 3, 1>> box_l3,
      std::vector<Eigen::Matrix<double, 1, 1>> heading_box3_l,
      std::vector<Eigen::Matrix<double, 3, 1>> box_l4,
      std::vector<Eigen::Matrix<double, 1, 1>> heading_box4_l,
      std::vector<Eigen::Matrix<double, 3, 1>> obstacle_l,
      std::vector<Eigen::Matrix<double, 3, 1>> obstacle_l2)
  {
      COM_list = COM_l;
      l_foot_list = l_foot_l;
      r_foot_list = r_foot_l;
      l_wrist_list = l_wrist_l;
      r_wrist_list = r_wrist_l;
      heading_list = heading_l;
      box_list = box_l;
      quadrotor_list = quadrotor_l;
      heading_quadrotor_list = heading_quadrotor_l;
      box_list1 = box_l1;
      heading_box1_list = heading_box1_l;
      box_list2 = box_l2;
      heading_box2_list = heading_box2_l;
      box_list3 = box_l3;
      heading_box3_list = heading_box3_l;
      box_list4 = box_l4;
      heading_box4_list = heading_box4_l;
      obstacle_list = obstacle_l;
      obstacle_list2 = obstacle_l2;
  }

  Matrix<double, 6, 1> Cassie::LeftArmFK(const Matrix<double, 7, 1>& q_gue_l) const
  {
    double theta1, theta2, theta3, theta4, theta5, theta6, theta7, roll, pitch, yaw;
    theta1 = q_gue_l[0];
    theta2 = q_gue_l[1];
    theta3 = q_gue_l[2];
    theta4 = q_gue_l[3];
    theta5 = q_gue_l[4];
    theta6 = q_gue_l[5];
    theta7 = q_gue_l[6];
    double shoulderLength = 0.156165;
    double bitriLength = 0.257165;
    double elbowLength = 0.25947;
    double wristLength = 0.084;
    double shldr_ang = PI / 4;
    Matrix<double, 6, 1> q;
    Matrix4d R0_1, R1_2, R2_3, R3_4, R4_5, R5_6, R6_7, R7_8, R8_9, R9_10, R10_11, R11_12, R12_13, HTM;

    // Rotating Inertial Frame towards the Shoulder Joint
    R0_1 << 1, 0, 0, 0,
         0, cos(shldr_ang), sin(shldr_ang), 0,
         0, -sin(shldr_ang), cos(shldr_ang), 0,
         0, 0, 0, 1;
    // Translating the Intertial Frame towards the Shoulder Joint
    R1_2 << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -shoulderLength,
         0, 0, 0, 1;
    // Shoulder Theta
    R2_3 << cos(theta1), sin(theta1), 0, 0,
         -sin(theta1), cos(theta1), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    // Upper Arm Theta
    R4_5 << cos(theta2), 0, sin(theta2), 0,
           0, 1, 0, 0,
           -sin(theta2), 0, cos(theta2), 0,
           0, 0, 0, 1;
    // Upper Arm Phi
    R5_6 << 1, 0, 0, 0,
         0, cos(theta3), -sin(theta3), 0,
         0, sin(theta3), cos(theta3), 0,
         0, 0, 0, 1;
    // Translation To The Elbow Joint
    R6_7 << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -bitriLength,
         0, 0, 0, 1;
    // Elbow Theta
    R7_8 << cos(theta4), 0, sin(theta4), 0,
           0, 1, 0, 0,
           -sin(theta4), 0, cos(theta4), 0,
           0, 0, 0, 1;
    // Forearm Theta
    R8_9 << cos(theta5), sin(theta5), 0, 0,
         -sin(theta5), cos(theta5), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    // Translation To The Wrist Joint
    R9_10 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, -elbowLength,
          0, 0, 0, 1;
    // Wrist Theta
    R10_11 << 1, 0, 0, 0,
         0, cos(theta6), -sin(theta6), 0,
         0, sin(theta6), cos(theta6), 0,
         0, 0, 0, 1;
    // Wrist Phi
    R11_12 << cos(theta7), 0, sin(theta7), 0,
           0, 1, 0, 0,
           -sin(theta7), 0, cos(theta7), 0,
           0, 0, 0, 1;
    // Translation To The Wrist Length
    R12_13 << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, -wristLength,
           0, 0, 0, 1;

    HTM = R0_1 * R1_2 * R2_3 * R4_5 * R5_6 * R6_7 * R7_8 * R8_9 * R9_10 * R10_11 * R11_12 * R12_13;
    roll = atan2(HTM(2,1),HTM(2,2));
    yaw = atan2(HTM(1,0),HTM(0,0));
    pitch = atan2(-HTM(2,0), sqrt(HTM(2,2) * HTM(2,2) + HTM(2,1) * HTM(2,1)));
    q << HTM(0, 3), HTM(1, 3), HTM(2, 3), roll, pitch, yaw;
    return q;
  }

  Matrix<double, 6, 1> Cassie::RightArmFK(const Matrix<double, 7, 1>& q_gue_r) const
  {
    double theta1, theta2, theta3, theta4, theta5, theta6, theta7, roll, pitch, yaw;
    theta1 = q_gue_r[0];
    theta2 = q_gue_r[1];
    theta3 = q_gue_r[2];
    theta4 = q_gue_r[3];
    theta5 = q_gue_r[4];
    theta6 = q_gue_r[5];
    theta7 = q_gue_r[6];
    double shoulderLength = 0.156165;
    double bitriLength = 0.257165;
    double elbowLength = 0.25947;
    double wristLength = 0.084;
    double shldr_ang = PI / 4;
    Matrix<double, 6, 1> q;
    Matrix4d R0_1, R1_2, R2_3, R3_4, R4_5, R5_6, R6_7, R7_8, R8_9, R9_10, R10_11, R11_12, R12_13, HTM;

    // Rotating Inertial Frame towards the Shoulder Joint
    R0_1 << 1, 0, 0, 0,
         0, cos(shldr_ang), -sin(shldr_ang), 0,
         0, sin(shldr_ang), cos(shldr_ang), 0,
         0, 0, 0, 1;
    // Translating the Intertial Frame towards the Shoulder Joint
    R1_2 << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -shoulderLength,
         0, 0, 0, 1;
    // Shoulder Theta
    R2_3 << cos(-theta1), -sin(-theta1), 0, 0,
         sin(-theta1), cos(-theta1), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    // Upper Arm Theta
    R4_5 << cos(-theta2), 0, sin(-theta2), 0,
         0, 1, 0, 0,
         -sin(-theta2), 0, cos(-theta2), 0,
         0, 0, 0, 1;
    // Upper Arm Phi
    R5_6 << 1, 0, 0, 0,
         0, cos(theta3), -sin(theta3), 0,
         0, sin(theta3), cos(theta3), 0,
         0, 0, 0, 1;
   // Translation To The Elbow Joint
    R6_7 << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -bitriLength,
         0, 0, 0, 1;
    // Elbow Theta
    R7_8 << cos(-theta4), 0, sin(-theta4), 0,
         0, 1, 0, 0,
    	 -sin(-theta4), 0, cos(-theta4), 0,
        0, 0, 0, 1;
    // Forearm Theta
    R8_9 << cos(-theta5), -sin(-theta5), 0, 0,
         sin(-theta5), cos(-theta5), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    // Translation To The Wrist Joint
    R9_10 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, -elbowLength,
          0, 0, 0, 1;
    // Wrist Theta
    R10_11 << 1, 0, 0, 0,
         0, cos(theta6), -sin(theta6), 0,
         0, sin(theta6), cos(theta6), 0,
         0, 0, 0, 1;
    // Wrist Phi
    R11_12 << cos(theta7), 0, sin(theta7), 0,
         0, 1, 0, 0,
         -sin(theta7), 0, cos(theta7), 0,
         0, 0, 0, 1;
    // Translation To The Wrist Length
    R12_13 << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, -wristLength,
           0, 0, 0, 1;

    HTM = R0_1 * R1_2 * R2_3 * R4_5 * R5_6 * R6_7 * R7_8 * R8_9 * R9_10 * R10_11 * R11_12 * R12_13;
    roll = atan2(HTM(2,1),HTM(2,2));
    yaw = atan2(HTM(1,0),HTM(0,0));
    pitch = atan2(-HTM(2,0), sqrt(HTM(2,2) * HTM(2,2) + HTM(2,1) * HTM(2,1)));
    q << HTM(0, 3), HTM(1, 3), HTM(2, 3), roll, pitch, yaw;
    return q;
  }

  Matrix<double, 6, 7> Cassie::LeftArmJacobianFunction(const Matrix<double, 7, 1>& q_in_l) const
  {
    int i, j;
    double hh = 0.0001;
    Matrix<double, 6, 1> FK0, FK1;
    Matrix<double, 7, 1> q;
    q = q_in_l;
    FK0 = LeftArmFK(q);
    Matrix<double, 6, 7> Jq;
    for (j=0; j < 7; j++){
        q[j] += hh;
        FK1 = LeftArmFK(q);
        q[j] -= hh;
        for (i=0; i < 6; i++){
            Jq(i, j) = (FK1[i] - FK0[i]) / hh;
        }
    }
    return Jq;
  }

    Matrix<double, 6, 7> Cassie::RightArmJacobianFunction(const Matrix<double, 7, 1>& q_in_r) const
  {
    int i, j;
    double hh = 0.0001;
    Matrix<double, 6, 1> FK0, FK1;
    Matrix<double, 7, 1> q;
    q = q_in_r;
    FK0 = RightArmFK(q);
    Matrix<double, 6, 7> Jq;
    for (j=0; j < 7; j++){
        q[j] += hh;
        FK1 = RightArmFK(q);
        q[j] -= hh;
        for (i=0; i < 6; i++){
            Jq(i, j) = (FK1[i] - FK0[i]) / hh;
        }
    }
    return Jq;
  }

  Matrix<double, 7, 1> Cassie::DoIKLeftArm(const Matrix<double, 7, 1>& q_guess_l,
                             const Matrix<double, 6, 1>& x_d_l) const
  {
    int i;
    double thresh = 0.005;
    Matrix<double, 6, 1> X_guess, error, mask;
    Matrix<double, 7, 1> q, q_0 = q_guess_l, q_1 = q_guess_l;
    Matrix<double, 6, 7> J;

    X_guess = LeftArmFK(q_guess_l);
    error = x_d_l - X_guess;
    for (i=0; i<6; i++){
        mask[i] = fabs(error[i]) > thresh;
    }

    while (mask.sum() > 0) {
        J = LeftArmJacobianFunction(q_0);
        q_1 = q_0 + J.transpose()*(J*J.transpose()).inverse()*(x_d_l - X_guess);
        q_0 = q_1;

        X_guess = LeftArmFK(q_0);
        error = x_d_l - X_guess;
        for (i=0; i<6; i++){
            mask[i] = fabs(error[i]) > thresh;
        }
    }
    for (i=0; i<7; i++){
        q[i] = q_1[i] -  2 * PI * floor((q_1[i] + PI ) / 2 / PI);
    }

    return q;
  }

  Matrix<double, 7, 1> Cassie::DoIKRightArm(const Matrix<double, 7, 1>& q_guess_r,
                             const Matrix<double, 6, 1>& x_d_r) const
  {
    int i;
    double thresh = 0.005;
    Matrix<double, 6, 1> X_guess, error, mask;
    Matrix<double, 7, 1> q, q_0 = q_guess_r, q_1 = q_guess_r;
    Matrix<double, 6, 7> J;

    X_guess = RightArmFK(q_guess_r);
    error = x_d_r - X_guess;
    for (i=0; i<6; i++){
        mask[i] = fabs(error[i]) > thresh;
    }

    while (mask.sum() > 0) {
        J = RightArmJacobianFunction(q_0);
        q_1 = q_0 + J.transpose()*(J*J.transpose()).inverse()*(x_d_r - X_guess);
        q_0 = q_1;

        X_guess = RightArmFK(q_0);
        error = x_d_r - X_guess;
        for (i=0; i<6; i++){
            mask[i] = fabs(error[i]) > thresh;
        }
    }
    for (i=0; i<7; i++){
        q[i] = q_1[i] -  2 * PI * floor((q_1[i] + PI ) / 2 / PI);
    }

    return q;
  }

  Matrix<double, 27, 1> Cassie::DoIKCassie(const Matrix<double, 9, 1>& COM,
                                                      const Matrix<double, 9, 1>& l_foot,
                                                      const Matrix<double, 9, 1>& r_foot,
                                                      const Matrix<double, 1, 1>& heading) const
  {
    Matrix<double, 27, 1> res;

    // Floating base quaternions
    res(0, 0) = std::cos(heading(0, 0) / 2);
    res(1, 0) = 0;
    res(2, 0) = 0;
    res(3, 0) = std::sin(heading(0, 0) / 2);

    // Floating base XYZ
    res(4, 0) = COM(0, 0);
    res(5, 0) = COM(1, 0);
    res(6, 0) = COM(2, 0);

    // this IK uses the analytical method by adding enough reasonable constraints
    // 1) hip yaw = 0;
    // 2) toe parallel to ground
    // 3) thigh and tarsus remain 13 degrees from parallel

    // comments and suggestions are welcomed - Jialin
    // all parameters are from: https://github.com/agilityrobotics/agility-cassie-doc/wiki/Kinematic-Model
    // this link is provided by Hongchang 

    // left leg
    // hip-roll
    double x_roll_left = COM(0, 0) + 0.021*std::cos(heading(0, 0)) - 0.135*std::sin(heading(0, 0));
    double y_roll_left = COM(1, 0) + 0.021*std::sin(heading(0, 0)) + 0.135*std::cos(heading(0, 0));
    double z_roll_left = COM(2, 0); // absolute coordinates of hip-roll joint

    double x_yaw_left = COM(0, 0) + (0.021 - 0.07)*std::cos(heading(0, 0)) - 0.135*std::sin(heading(0, 0));
    double y_yaw_left = COM(1, 0) + (0.021 - 0.07)*std::sin(heading(0, 0)) + 0.135*std::cos(heading(0, 0));
    double z_yaw_left = COM(2, 0); // absolute coordinates of hip-yaw joint

    // normal vector of the plane (plane A) of the 3 points: hip-roll joint, hip yaw joint, left foot
    double x_cross_left = (y_yaw_left - l_foot(1, 0))*(z_roll_left - l_foot(2, 0)) -
                          (z_yaw_left - l_foot(2, 0))*(y_roll_left - l_foot(1, 0));
    double y_cross_left = (z_yaw_left - l_foot(2, 0))*(x_roll_left - l_foot(0, 0)) -
                          (x_yaw_left - l_foot(0, 0))*(z_roll_left - l_foot(2, 0));
    double z_cross_left = (x_yaw_left - l_foot(0, 0))*(y_roll_left - l_foot(1, 0)) -
                          (y_yaw_left - l_foot(1, 0))*(x_roll_left - l_foot(0, 0));

    // note that from the hip-pitch joint to the toe joint, links do 2-D rotation.
    // those links' motion remains in a family of parallel planes, denoted by plane B
    // only hip-roll joint can change plane B's angle of slope

    double alpha_left = std::atan2(z_cross_left, std::sqrt(x_cross_left*x_cross_left + y_cross_left*y_cross_left)); // PI / 2 - plane A's angle of slope
    double beta_left = std::asin(0.0045 / (COM(2, 0) - l_foot(2, 0)) * std::cos(alpha_left)); // angle between plane A and B 
    res(7, 0) = alpha_left + beta_left; // hip-roll joint angle, namely PI / 2 - plane B's angle of slope


    // hip-yaw set 0
    res(9, 0) = 0;


    // hip-pitch, knee motor, knee, ankle, ankle spring

    // normalized vector pointing from hip-yaw joint to hip-roll joint
    double tx = (x_roll_left - x_yaw_left) / 0.07, ty = (y_roll_left - y_yaw_left) / 0.07;
    // left foot 2-D cordinates when projected to plane B, w.r.t hip-pitch joint
    double p_left = (l_foot(0, 0) - x_yaw_left)*tx + (l_foot(1, 0) - y_yaw_left)*ty;
    double q_left = 0.0045 / std::tan(beta_left)-0.090;
    // toe joint 2-D cordinates when projected to plane B, w.r.t hip-pitch joint
    double p_toe_left = p_left/* - 0.02865*/, q_toe_left = q_left/* - 0.04704*/; // not sure about the offsets, so they are not included yet

    // after calculating p, q_toe_left, remaining work is to solve the triangles
    double a = 0.500, b = 0.120, c = 0.410; // a: distance between knee joint and ankle joint, b: length of thigh, c: length of tarsus
    double d = std::sqrt(b*b + c*c + 2*b*c*std::cos(13 * PI / 180)); // tarsus and thigh are always 13 degrees from parallel when assuming no deformation of leaf springs
    double l_left = std::sqrt(p_toe_left*p_toe_left + q_toe_left*q_toe_left);
    double gamma_left = std::acos((l_left*l_left + d*d - a*a) / 2 / l_left / d);
    double delta_left = std::acos((d*d + c*c - b*b) / 2 / d / c);
    res(11, 0) = PI / 2 - 13 * PI / 180 - (std::atan2(q_toe_left, p_toe_left) - gamma_left - delta_left); // hip-pitch joint angle
    double phi_left = PI / 2 - std::atan2(q_toe_left, p_toe_left) - std::acos((l_left*l_left + a*a - d*d) / 2 / l_left / a);
    res(15, 0) = phi_left - res(11, 0) - 5.11 * PI / 180; // knee motor joint angle
    res(12, 0) = 0.089 + res(11, 0) + res(15, 0) + 0.05; // left achilles rod
    res(17, 0) = 0; // knee joint angle, 0 cuz no deformation
    res(19, 0) = 13 * PI / 180 - res(15, 0); // ankle joint angle
    res(21, 0) = 0; // left_toe_crank
    res(22, 0) = 0.4 - 13 * PI / 180 - 0.1; // ankle spring joint angle

    // toe, always parallel to ground
    res(23,0) = - 0.88 - res(19, 0) - res(15, 0) - res(11, 0); // toe joint angle, 0.88 got by trial (when other joint angles are 0, angle the toe needs to rotate to be parallel to ground)



    // right leg (very similar to left leg)
    double x_roll_right = COM(0, 0) + 0.021*std::cos(heading(0, 0)) + 0.135*std::sin(heading(0, 0));
    double y_roll_right = COM(1, 0) + 0.021*std::sin(heading(0, 0)) - 0.135*std::cos(heading(0, 0));
    double z_roll_right = COM(2, 0);

    double x_yaw_right = COM(0, 0) + (0.021 - 0.07)*std::cos(heading(0, 0)) + 0.135*std::sin(heading(0, 0));
    double y_yaw_right = COM(1, 0) + (0.021 - 0.07)*std::sin(heading(0, 0)) - 0.135*std::cos(heading(0, 0));
    double z_yaw_right = COM(2, 0);

    double x_cross_right = (y_yaw_right - r_foot(1, 0))*(z_roll_right - r_foot(2, 0)) -
                           (z_yaw_right - r_foot(2, 0))*(y_roll_right - r_foot(1, 0));
    double y_cross_right = (z_yaw_right - r_foot(2, 0))*(x_roll_right - r_foot(0, 0)) -
                           (x_yaw_right - r_foot(0, 0))*(z_roll_right - r_foot(2, 0));
    double z_cross_right = (x_yaw_right - r_foot(0, 0))*(y_roll_right - r_foot(1, 0)) -
                           (y_yaw_right - r_foot(1, 0))*(x_roll_right - r_foot(0, 0));

    double alpha_right = std::atan2(z_cross_right, std::sqrt(x_cross_right*x_cross_right + y_cross_right*y_cross_right));
    double beta_right = std::asin(0.0045 / (COM(2, 0) - r_foot(2, 0)) * std::cos(alpha_right));
    res(8, 0) = alpha_right - beta_right;


    res(10, 0) = 0;


    double p_right = (r_foot(0, 0) - x_yaw_right)*tx + (r_foot(1, 0) - y_yaw_right)*ty;
    double q_right = 0.0045 / std::tan(beta_right) - 0.090;
    double p_toe_right = p_right /*- 0.02865*/, q_toe_right = q_right/* - 0.04704*/;

    double l_right = std::sqrt(p_toe_right*p_toe_right + q_toe_right*q_toe_right);
    double gamma_right = std::acos((l_right*l_right + d*d - a*a) / 2 / l_right / d);
    double delta_right = std::acos((d*d + c*c - b*b) / 2 / d / c);
    res(13, 0) = PI / 2-13 * PI / 180 - (std::atan2(q_toe_right, p_toe_right) - gamma_right - delta_right);
    double phi_right = PI / 2 - std::atan2(q_toe_right, p_toe_right) - std::acos((l_right*l_right + a*a - d*d) / 2 / l_right / a);
    res(16, 0) = phi_right - res(13, 0) - 5.11 * PI / 180;
    res(14, 0) = 0.089 + res(13, 0) + res(16, 0) + 0.05;
    res(18, 0) = 0;
    res(20, 0) = 13 * PI / 180 - res(16, 0);
    res(24, 0) = 0;
    res(25, 0) = 0.4 - 13 * PI / 180 - 0.1;


    res(26,0) = - 0.88 - res(20, 0) - res(16, 0) - res(13, 0);

    return res;
  }

  void Cassie::DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
      drake::systems::DiscreteValues<double>* updates) const
  {
    t = context.get_time();

    step = int(t / h);
     
    if (step == 0) {
    //ang_r << 0, -0.769, 0, -0.793, -1.561, -0.253, 0;
    //ang_l << 0, 0.769, 0, 0.793, 1.561, 0.253, 0;
    ang_r << -0.4657, -0.725099, 0.378249, -0.737147, -1.23264, -0.184066, -0.190421;
    ang_l << 0.140704, 0.76357, -0.123621,  0.818773, 1.45755, 0.272982, -0.0609805;
    }

    Eigen::VectorXd box(7);
    Eigen::VectorXd quadrotor(7);
    Eigen::VectorXd turtle(7);
    Eigen::VectorXd turtle2(7);
    Eigen::VectorXd box1(7);
    Eigen::VectorXd box2(7);
    Eigen::VectorXd box3(7);
    Eigen::VectorXd box4(7);
    Eigen::VectorXd p, r, s;
    Eigen::VectorXd state(131);
    quadrotor << std::cos(heading_quadrotor_list[step](0, 0) / 2), 0, 0, std::sin(heading_quadrotor_list[step](0, 0) / 2), quadrotor_list[step];
    box4 << std::cos(heading_box4_list[step](0, 0) / 2), 0, 0, std::sin(heading_box4_list[step](0, 0) / 2), box_list4[step];
    if (step > 51125) {
    box1 << std::cos(heading_box1_list[51125](0, 0) / 2), 0, 0, std::sin(heading_box1_list[51125](0, 0) / 2), box_list1[51125]; 
    box2 << std::cos(heading_box2_list[51125](0, 0) / 2), 0, 0, std::sin(heading_box2_list[51125](0, 0) / 2), box_list2[51125];
    box3 << std::cos(heading_box3_list[51125](0, 0) / 2), 0, 0, std::sin(heading_box3_list[51125](0, 0) / 2), box_list3[51125];
    turtle << 1, 0, 0, 0, obstacle_list[51125];
    turtle2 << 1, 0, 0, 0, obstacle_list2[51125];
    box << std::cos(heading_list[51125](0, 0) / 2), 0, 0, std::sin(heading_list[51125](0, 0) / 2), box_list[51125];
    
    p = DoIKCassie(COM_list[51125], l_foot_list[51125], r_foot_list[51125], heading_list[51125]);
    
    r = DoIKLeftArm(ang_l, l_wrist_list[51125]);
    s = DoIKRightArm(ang_r, r_wrist_list[51125]);
    }
    else {
    box1 << std::cos(heading_box1_list[step](0, 0) / 2), 0, 0, std::sin(heading_box1_list[step](0, 0) / 2), box_list1[step];
    box2 << std::cos(heading_box2_list[step](0, 0) / 2), 0, 0, std::sin(heading_box2_list[step](0, 0) / 2), box_list2[step];
    box3 << std::cos(heading_box3_list[step](0, 0) / 2), 0, 0, std::sin(heading_box3_list[step](0, 0) / 2), box_list3[step];
    turtle << 1, 0, 0, 0, obstacle_list[step];
    turtle2 << 1, 0, 0, 0, obstacle_list2[step];
    box << std::cos(heading_list[step](0, 0) / 2), 0, 0, std::sin(heading_list[step](0, 0) / 2), box_list[step];

    p = DoIKCassie(COM_list[step], l_foot_list[step], r_foot_list[step], heading_list[step]);

    r = DoIKLeftArm(ang_l, l_wrist_list[step]);
    s = DoIKRightArm(ang_r, r_wrist_list[step]);
    }
    //std::cout << "left" << step << std::endl << r.transpose() << std::endl;
    //std::cout << "right" << step << std::endl << s.transpose() << std::endl;
    //Eigen::VectorXd s = - r;
    //s[6] = - s[6];
    //ang_l = r;
    //ang_r = s;
    //Eigen::VectorXd r = l_result_list[step];
    //Eigen::VectorXd s = r_result_list[step];

    Eigen::VectorXd hand = Eigen::VectorXd::Constant(22, 0.0); //hands
    hand[5] = 1.3;
    hand[15] = - 0.35;
   // po << 1, 0, 0, 0, location_list[step](0, 0), location_list[step](1, 0), location_list[step](2, 0);

   //original state << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], 0, 0, 0, p[11], p[12], p[13], p[14], 0, p[15], p[16], 0, p[17], p[18], 0, 0, 0, p[19], p[20], 0,  p[21], p[22], p[23], p[24], p[25], p[26], 0,  r[0], 0, 0, 0, s[0], 0, 0, 0, r[1], s[1], r[2], s[2], r[3], s[3], r[4], s[4], r[5], s[5], r[6], s[6], hand;

   state << p[0], p[1], p[2], p[3], p[4], p[5], p[6] + 0.10, box, quadrotor, box1, box2, box3, box4, turtle, turtle2,  p[7], p[8], p[9], p[10], 0,  p[11], p[12], p[13], p[14], 0, 0, 0, p[15], p[16], 0, p[17], p[18], 0, p[19], p[20], p[21], p[22], p[23], p[24], p[25], p[26], r[0], 0, 0, 0, s[0], 0, 0, 0, r[1], s[1], r[2], s[2], r[3], s[3], r[4], s[4], r[5], s[5], r[6], s[6], hand;  
  
   //new state << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], 0, 0, 0, p[11], p[12], p[13], p[14], 0, p[15], p[16], 0, p[17], p[18], p[19], p[20], r[0], 0, 0, 0, s[0], 0, 0, 0, p[21], p[22], p[23], p[24], p[25], p[26],r[1], s[1], r[2], s[2], r[3], s[3], r[4], s[4], r[5], s[5], r[6], s[6], hand;

   //state << p[0], p[1], p[2], p[3], 1, 0, 0, 0, p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15], p[16], p[17], p[18], p[19], p[20], p[21], p[22], location_list[step](0, 0), location_list[step](1, 0), location_list[step](2, 0);

     //state <<   p[0], p[1], p[2], p[3], p[4], p[5] + 0.5, p[6], 1, 0, 0, 0, location_list[step](0, 0) + 9, location_list[step](1, 0), location_list[step](2, 0) + 0.652, p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15], p[16], p[17], p[18], p[19], p[20], p[21], p[22], p[23], p[24], p[25], p[26];
   
    updates->get_mutable_vector().SetFromVector(state);
  }

  void Cassie::CopyDiscreteStateOut(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const 
  {
    auto d_state = context.get_discrete_state().get_vector().CopyToVector();
    output->SetFromVector(d_state);
  }

}  // namespace cassie
}  // namespace examples
}  // namespace drake
