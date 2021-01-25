#include "drake/examples/athena/athena.h"

#define PI 3.1416

namespace drake {
namespace examples {
namespace athena {

Athena::Athena() {
    DeclareDiscreteState(59);
    DeclareVectorOutputPort("floating_base_state",
                            drake::systems::BasicVector<double>(59),
                            &Athena::CopyDiscreteStateOut);
    DeclarePeriodicDiscreteUpdate(0.001);
}

void Athena::init(std::vector<Eigen::Matrix<double, 6, 1>> _l_traj,
                  std::vector<Eigen::Matrix<double, 6, 1>> _r_traj)
{
    l_traj = _l_traj;
    r_traj = _r_traj;
    q_l << 0, 0, PI/4, PI/2, 0, 0, 0;
    q_r << 0, 0, PI/4, PI/2, 0, 0, 0;
    _x_0 = LeftArmFK(q_l);
    x_c = _x_0;
}

void Athena::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<double>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&,
    drake::systems::DiscreteValues<double>* updates) const
{
//     static int dir = 1;
    t = context.get_time();

    // step = int(t / h);
    // Eigen::Matrix<double, 59, 1> state = // updates->get_mutable_vector().get_mutable_value(); //Eigen::Matrix<double,
    // 59, 1>::Random(); //updates->get_mutable_vector().get_mutable_value();
    Eigen::Matrix<double, 59, 1> state2 = Eigen::Matrix<double, 59, 1>::Constant(0.0);  // updates->get_mutable_vector().get_mutable_value();

//     double eps = 0.2;
//     if (x_c(2) > (_x_0(2) + 100)) dir = -1;
//     if (x_c(2) < (_x_0(2) - 100)) dir = 1;
//     x_c(2) = x_c(2) + dir * eps;

    /**
     * 0,1,2,3 --> Rotations, need to keep one non-zero to avoid seg fault.
     * 4,5,6   --> Cartesian shifting
     * 7       --> Bottom spine joint
     * 8,9     --> Back two things
     * 10,11,12--> Another spine
     * 13,14   --> Front two things
     * 15,16   --> More spine
     * 17      --> Left arm 0
     * 18,19,20--> Left three back things
     * 21      --> Right arm 0
     * 22,23,24--> Right three back things
     * 25-36   --> Left arm 1-6, Right arm 1-6
     * 37-40   --> Right fingers 0
     * 41      --> Right thumb 0
     * 42      --> Left thumb 0
     * 43-46   --> Left fingers 0
     * 47-50   --> Right fingers 1
     * 51      --> Right thumb 1
     * 52      --> Left thumb 1
     * 53-56   --> Left fingers 1
     * 57      --> Right thumb 2
     * 58      --> Left thumb 2
     */
    // Shoulder angles (2,3) need to be swapped
    state2(3) = 1;
    // int n = static_cast<int>(t);
    // if (n < 10) state2(25) = ang;
    // else if (n < 20) state2(27) = ang;
    // else if (n < 30) state2(29) = ang;
    // else if (n < 40) state2(31) = ang;
    // else if (n < 50) state2(33) = ang;
    // else if (n < 60) state2(35) = ang;

    // state2(7) = 0;//10 * PI / 180;

    Eigen::Matrix<double, 7, 1> ik;
    // static int i = 0;
    // if(!i) {
    //     i = 1;
        // Eigen::Matrix<double, 12, 1> v;
        // v << LeftArmFK(q_l) , RightArmFK(q_r);
        // std::cout << v(0) << ", " << v(1) << ", " << v(2) << ", " << v(3) << ", " << v(4)  << ", " << v(5)  << std::endl;
        // std::cout << v(6) << ", " << v(7) << ", " << v(8) << ", " << v(9) << ", " << v(10) << ", " << v(11) << std::endl;
        
        // Eigen::Matrix<double, 7, 1> guess = Eigen::Matrix<double, 7, 1>::Constant(0.0);
        // ik = LeftArmIK(guess, LeftArmFK(q_l));

        // std::cout << "q_l\t" << q_l.transpose() << std::endl;
        // std::cout << "ik\t" << ik.transpose() << std::endl;
        // std::cout << "FK(q_l)\t" << LeftArmFK(q_l).transpose() << std::endl;
        // std::cout << "FK(ik)\t" << LeftArmFK(ik).transpose() << std::endl;
    // }

    // static double theta = 0.0;
    // static double rho = 3*PI/2;

    // double eps = 4 * PI * 0.001;
    // theta = theta + eps;
    // if (theta > 2*PI) {
    //     theta = 0;
    //     rho = rho + eps*10;
    // }
    // if (rho > 2*PI){
    //     rho = 0;
    // }

    // x_c(2) = 100*cos(rho)*cos(theta) + _x_0(2);
    // x_c(1) = 100*cos(rho)*sin(theta) + _x_0(1);
    // x_c(0) = 100*sin(rho) + _x_0(0);


    // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    // ik = LeftArmIK(q_l, x_c);
    // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    // ik = LeftArmIKAutoGuess(x_c);
    // std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << "IK (n AG): " << dur.count() << std::endl;
    // dur = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);
    // std::cout << "IK (y AG): " << dur.count() << std::endl;

    double ppp = t * 78125 / 60;
    int pppf = floor(ppp);
    // double angle = Athena::limits[pppf][0] + (ppp - pppf)*(Athena::limits[pppf][1] - Athena::limits[pppf][0]);
    // ik(pppf) = angle;
    ik = autoguess_q[pppf];



    q_l = Eigen::Matrix<double, 7, 1>::Constant(0.0);
    q_l(2) = PI/4;
    q_l(3) = PI/2;


    state2(17) = ik(0);
    state2(25) = ik(1);
    state2(27) = ik(2);
    state2(29) = ik(3);
    state2(31) = ik(4);
    state2(33) = ik(5);
    state2(35) = ik(6);

    state2(21) = -(q_r(0) + PI/16);
    state2(26) = -q_r(1);
    state2(28) = -q_r(2);
    state2(30) = -q_r(3);
    state2(32) = -q_r(4);
    state2(34) = -q_r(5);
    state2(36) = -q_r(6);

    // state2(11) = 1;

    // Eigen::Matrix<double, 59, 1> state2 = Eigen::Matrix<double, 59,
    // 1>::Constant(ang); //updates->get_mutable_vector().get_mutable_value();
    // for(int i = 0; i < 59; i++){
    //     if (i != 0){
    //         state2(i) = 0;
    //     }
    // }
    // std::cout << "n: " << n << std::endl;
    // std::cout << "step " << step << std::endl << std::endl;
    // std::cout << updates->get_mutable_vector();
    updates->get_mutable_vector().SetFromVector(state2);
}

void Athena::CopyDiscreteStateOut(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* output) const
{
    auto d_state = context.get_discrete_state().get_vector().CopyToVector();
    output->SetFromVector(d_state);
}

}  // namespace athena
}  // namespace examples
}  // namespace drake
