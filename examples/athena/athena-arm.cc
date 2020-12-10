#include "drake/examples/athena/athena.h"

#define PI 3.1416

namespace drake {
namespace examples {
namespace athena {

double const Athena::limits[7][2] = {
    {},
    {},
    {},
    {},
    {},
    {},
    {}
}

void Athena::initAutoGuess(const int &segments) const {
    autoguess_segments = segments;
    int elements = std::pow(segments, 7);

    autoguess_start = new Eigen::Matrix<double, 6, 1>[elements];

    for(int a = 0; a < segments; ++a){
    for(int b = 0; b < segments; ++b){
    for(int c = 0; c < segments; ++c){
    for(int d = 0; d < segments; ++d){
    for(int e = 0; e < segments; ++e){
    for(int f = 0; f < segments; ++f){
    for(int g = 0; g < segments; ++g){
        
    }
    }
    }
    }
    }
    }
    }
}

Eigen::Matrix<double, 7, 1> Athena::LeftArmIKAutoGuess(const Eigen::Matrix<double, 6, 1> &x_des) const {

}

Eigen::Matrix<double, 7, 1> Athena::LeftArmIK(const Eigen::Matrix<double, 7, 1> &q_guess,
                             const Eigen::Matrix<double, 6, 1> &x_des) const {
    int i;
    double step_size = 0.1;
    double thresh = 0.05;

    Eigen::Matrix<double, 6, 1> x_0, error, mask;
    Eigen::Matrix<double, 7, 1> q_out, q_0 = q_guess, q_1 = q_guess;
    Eigen::Matrix<double, 6, 7> J;
    Eigen::Matrix<double, 7, 2> limits;

    x_0 = LeftArmFK(q_guess);
    error = x_des - x_0;

    // int counter = 0;
    do {
        J = LeftArmJacobian(q_0);
        q_1 = q_0 + step_size*J.transpose()*(J*J.transpose()).inverse()*(error);
        q_0 = q_1;

        x_0 = LeftArmFK(q_0);
        error = x_des - x_0;
        for (i = 0; i < 6; ++i){
            mask[i] = fabs(error[i]) > thresh;
        }
        // ++counter;
    } while (mask.sum() > 0);

    // std::cout << "Took " << counter << " cycles to complete." << std::endl;

    for (i = 0; i < 7; ++i){
        q_out[i] = q_1[i] -  2 * PI * floor((q_1[i] + PI ) / 2 / PI);
    }
    return q_out;
}


Eigen::Matrix<double, 6, 7> Athena::LeftArmJacobian(const Eigen::Matrix<double, 7, 1> &q_in) const {
    int i, j;
    double hh = 0.0001;
    Eigen::Matrix<double, 6, 1> FK0, FK1;
    Eigen::Matrix<double, 7, 1> q = q_in;
    FK0 = LeftArmFK(q);
    Eigen::Matrix<double, 6, 7> Jq;
    for (j = 0; j < 7; j++) {
        q[j] += hh;
        FK1 = LeftArmFK(q);
        q[j] -= hh;
        for (i = 0; i < 6; i++) {
            Jq(i, j) = (FK1[i] - FK0[i]) / hh;
        }
    }
    return Jq;
}

Eigen::Matrix<double, 6, 1> Athena::LeftArmFK(const Eigen::Matrix<double, 7, 1> q) const {
    double shoulderLength = 148.171;
    double bitriLength = 257.287;
    double elbowLength = 185.528;
    double wristLength = 84;
    double shldr_ang = PI/4;

    Eigen::Matrix<double, 4, 4> R0_1, R1_2, R2_3, R3_4, R4_5, R5_6, R6_7, R7_8, R8_9, R9_10, R10_11, R11_12, HTM;

    // Rotating Inertial Frame towards the Shoulder Joint
    R0_1 << 1,                  0,                  0,                  0,
            0,                  cos(-shldr_ang),    -sin(-shldr_ang),   0,
            0,                  sin(-shldr_ang),    cos(-shldr_ang),    0,
            0,                  0,                  0,                  1;
    // Translating the Intertial Frame towards the Shoulder Joint
    R1_2 << 1,                  0,                  0,                  0,
            0,                  1,                  0,                  0,
            0,                  0,                  1,                  -shoulderLength,
            0,                  0,                  0,                  1;
    // Shoulder Theta
    R2_3 << cos(-q(0)),         -sin(-q(0)),        0,                  0,
            sin(-q(0)),         cos(-q(0)),         0,                  0,
            0,                  0,                  1,                  0,
            0,                  0,                  0,                  1;
    // Upper Arm Phi
    R3_4 << cos(-q(1)),         0,                  sin(-q(1)),         0,
            0,                  1,                  0,                  0,
            -sin(-q(1)),        0,                  cos(-q(1)),         0,
            0,                  0,                  0,                  1;
    // Upper Arm Theta
    R4_5 << 1,                  0,                  0,                  0,
            0,                  cos(-q(2)),         -sin(-q(2)),        0, 
            0,                  sin(-q(2)),         cos(-q(2)),         0,
            0,                  0,                  0,                  1;
    // Translation To The Elbow Joint
    R5_6 << 1,                  0,                  0,                  0, 
            0,                  1,                  0,                  0,
            0,                  0,                  1,                  -bitriLength,
            0,                  0,                  0,                  1;
    // Elbow Theta
    R6_7 << cos(-q(3)),         0,                  sin(-q(3)),         0, 
            0,                  1,                  0,                  0,
            -sin(-q(3)),        0,                  cos(-q(3)),         0,
            0,                  0,                  0,                  1;
    // Forearm Theta
    R7_8 << cos(-q(4)),         -sin(-q(4)),        0,                  0,
            sin(-q(4)),         cos(-q(4)),         0,                  0,
            0,                  0,                  1,                  0,
            0,                  0,                  0,                  1;
    // Translation To The Wrist Joint
    R8_9 << 1,                  0,                  0,                  0,
            0,                  1,                  0,                  0,
            0,                  0,                  1,                  -elbowLength,
            0,                  0,                  0,                  1;
    // Wrist Theta
    R9_10 <<1,                  0,                  0,                  0,
            0,                  cos(-q(5)),         -sin(-q(5)),        0,
            0,                  sin(-q(5)),         cos(-q(5)),         0,
            0,                  0,                  0,                  1;
    // Wrist Phi
    R10_11<<cos(-q(6)),         0,                  sin(-q(6)),         0,
            0,                  1,                  0,                  0,
            -sin(-q(6)),        0,                  cos(-q(6)),         0,
            0,                  0,                  0,                  1;
    // Translation To The Wrist Length
    R11_12<<1,                  0,                  0,                  0,
            0,                  1,                  0,                  0,
            0,                  0,                  1,                  -wristLength,
            0,                  0,                  0,                  1;

    HTM = R0_1 * R1_2 * R2_3 * R3_4 * R4_5 * R5_6 * R6_7 * R7_8 * R8_9 * R9_10 * R10_11 * R11_12;

    double roll  = atan2(HTM(2,1),HTM(2,2));
    double yaw   = atan2(HTM(1,0),HTM(0,0));
    double pitch = atan2(-HTM(2,0), sqrt(HTM(2,2) * HTM(2,2) + HTM(2,1) * HTM(2,1)));

    Eigen::Matrix<double, 6, 1> x;
    x << HTM(0, 3), HTM(1, 3), HTM(2, 3), roll, pitch, yaw;
    return x;
}

Eigen::Matrix<double, 6, 1> Athena::RightArmFK(const Eigen::Matrix<double, 7, 1> q) const {
    double shoulderLength = 156.165;
    double bitriLength = 257.165;
    double elbowLength = 259.47;
    double wristLength = 84;
    double shldr_ang = PI/4;

    Eigen::Matrix<double, 4, 4> R0_1, R1_2, R2_3, R3_4, R4_5, R5_6, R6_7, R7_8, R8_9, R9_10, R10_11, R11_12, HTM;

    // Rotating Inertial Frame towards the Shoulder Joint
    R0_1 << 1,                  0,                  0,                  0,
            0,                  cos(shldr_ang),     -sin(shldr_ang),    0,
            0,                  sin(shldr_ang),     cos(shldr_ang),     0,
            0,                  0,                  0,                  1;
    // Translating the Intertial Frame towards the Shoulder Joint
    R1_2 << 1,                  0,                  0,                  0,
            0,                  1,                  0,                  0,
            0,                  0,                  1,                  shoulderLength,
            0,                  0,                  0,                  1;
    // Shoulder Theta
    R2_3 << cos(q(0)),          -sin(q(0)),         0,                  0,
            sin(q(0)),          cos(q(0)),          0,                  0,
            0,                  0,                  1,                  0,
            0,                  0,                  0,                  1;
    // Upper Arm Phi
    R3_4 << cos(q(1)),          0,                  sin(q(1)),          0,
            0,                  1,                  0,                  0,
            -sin(q(1)),         0,                  cos(q(1)),          0,
            0,                  0,                  0,                  1;
    // Upper Arm Theta
    R4_5 << 1,                  0,                  0,                  0,
            0,                  cos(q(2)),          -sin(q(2)),         0, 
            0,                  sin(q(2)),          cos(q(2)),          0,
            0,                  0,                  0,                  1;
    // Translation To The Elbow Joint
    R5_6 << 1,                  0,                  0,                  0, 
            0,                  1,                  0,                  0,
            0,                  0,                  1,                  bitriLength,
            0,                  0,                  0,                  1;
    // Elbow Theta
    R6_7 << cos(q(3)),          0,                  sin(q(3)),          0, 
            0,                  1,                  0,                  0,
            -sin(q(3)),         0,                  cos(q(3)),          0,
            0,                  0,                  0,                  1;
    // Forearm Theta
    R7_8 << cos(q(4)),          -sin(q(4)),         0,                  0,
            sin(q(4)),          cos(q(4)),          0,                  0,
            0,                  0,                  1,                  0,
            0,                  0,                  0,                  1;
    // Translation To The Wrist Joint
    R8_9 << 1,                  0,                  0,                  0,
            0,                  1,                  0,                  0,
            0,                  0,                  1,                  elbowLength,
            0,                  0,                  0,                  1;
    // Wrist Theta
    R9_10 <<1,                  0,                  0,                  0,
            0,                  cos(q(5)),          -sin(q(5)),         0,
            0,                  sin(q(5)),          cos(q(5)),          0,
            0,                  0,                  0,                  1;
    // Wrist Phi
    R10_11<<cos(q(6)),          0,                  sin(q(6)),          0,
            0,                  1,                  0,                  0,
            -sin(q(6)),         0,                  cos(q(6)),          0,
            0,                  0,                  0,                  1;
    // Translation To The Wrist Length
    R11_12<<1,                  0,                  0,                  0,
            0,                  1,                  0,                  0,
            0,                  0,                  1,                  wristLength,
            0,                  0,                  0,                  1;

    HTM = R0_1 * R1_2 * R2_3 * R3_4 * R4_5 * R5_6 * R6_7 * R7_8 * R8_9 * R9_10 * R10_11 * R11_12;

    double roll  = atan2(HTM(2,1),HTM(2,2));
    double yaw   = atan2(HTM(1,0),HTM(0,0));
    double pitch = atan2(-HTM(2,0), sqrt(HTM(2,2) * HTM(2,2) + HTM(2,1) * HTM(2,1)));

    Eigen::Matrix<double, 6, 1> x;
    x << HTM(0, 3), HTM(1, 3), HTM(2, 3), roll, pitch, yaw;
    return x;
}

}
}
}
