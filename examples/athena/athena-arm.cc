#include "drake/examples/athena/athena.h"

#define PI 3.1416

namespace drake {
namespace examples {
namespace athena {

double const Athena::limits[7][2] = {
    {-PI/2, PI/2},
    {-PI/2, PI/2},
    {-PI/4, PI/4},
    {-PI/4, 2*PI/3},
    {-PI/4, PI/4},
    {-PI/4, PI/4},
    {-PI/4, PI/2}
};

void Athena::initAutoGuess(const int &segments) const {
    autoguess_segments = segments;
    autoguess_elements = std::pow(segments, 7);

    autoguess_x = new Eigen::Matrix<double, 6, 1>[autoguess_elements];
    autoguess_q = new Eigen::Matrix<double, 7, 1>[autoguess_elements];

    int index = 0;

    for(int a = 0; a < segments; ++a){ double a_edge = limits[0][0] + (a + 0.5) * (limits[0][1] - limits[0][0]) / segments;
    for(int b = 0; b < segments; ++b){ double b_edge = limits[1][0] + (b + 0.5) * (limits[1][1] - limits[1][0]) / segments;
    for(int c = 0; c < segments; ++c){ double c_edge = limits[2][0] + (c + 0.5) * (limits[2][1] - limits[2][0]) / segments;
    for(int d = 0; d < segments; ++d){ double d_edge = limits[3][0] + (d + 0.5) * (limits[3][1] - limits[3][0]) / segments;
    for(int e = 0; e < segments; ++e){ double e_edge = limits[4][0] + (e + 0.5) * (limits[4][1] - limits[4][0]) / segments;
    for(int f = 0; f < segments; ++f){ double f_edge = limits[5][0] + (f + 0.5) * (limits[5][1] - limits[5][0]) / segments;
    for(int g = 0; g < segments; ++g){ double g_edge = limits[6][0] + (g + 0.5) * (limits[6][1] - limits[6][0]) / segments;
        autoguess_q[index] << a_edge, b_edge, c_edge, d_edge, e_edge, f_edge, g_edge;
        autoguess_x[index] = LeftArmFK(autoguess_q[index]);
        ++index;
    }
    }
    }
    }
    }
    }
    }
}

Eigen::Matrix<double, 7, 1> Athena::LeftArmIKAutoGuess(const Eigen::Matrix<double, 6, 1> &x_des) const {
    int min_index = 0;
    double min_distance = (x_des - autoguess_x[0]).norm();
    int index = 1;
    do {
        double distance = (x_des - autoguess_x[index]).norm();
        if (distance < min_distance) {
            min_distance = distance;
            min_index = index;
        }
    } while (++index < autoguess_elements);
    std::cout << min_distance << std::endl;
    return LeftArmIK(autoguess_q[min_index], x_des);
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
