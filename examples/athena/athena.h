#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "drake/common/autodiff.h"
#include "drake/common/find_resource.h"
#include "drake/math/jacobian.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace examples {
namespace athena {

class Athena : public drake::systems::LeafSystem<double> {

DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Athena);

private:
    // Trajectory
    std::vector<Eigen::Matrix<double, 6, 1>> l_traj;
    std::vector<Eigen::Matrix<double, 6, 1>> r_traj;

    double h = 0.1;
    static double const limits[7][2];
    mutable Eigen::Matrix<double, 7, 1> q_l = Eigen::Matrix<double, 7, 1>::Constant(0.0);
    mutable Eigen::Matrix<double, 7, 1> q_r = Eigen::Matrix<double, 7, 1>::Constant(0.0);
    mutable Eigen::Matrix<double, 6, 1> _x_0 = Eigen::Matrix<double, 6, 1>::Constant(0.0);
    mutable Eigen::Matrix<double, 6, 1> x_c = Eigen::Matrix<double, 6, 1>::Constant(0.0);

public:
    explicit Athena();
    void init(std::vector<Eigen::Matrix<double, 6, 1>> l_traj, 
              std::vector<Eigen::Matrix<double, 6, 1>> r_traj);

    mutable int autoguess_segments = 0;
    mutable int autoguess_elements = 0;
    mutable Eigen::Matrix<double, 6, 1> *autoguess_x = NULL;
    mutable Eigen::Matrix<double, 7, 1> *autoguess_q = NULL;

    mutable double t = 0;
    void initAutoGuess(const int &segments) const;

    Eigen::Matrix<double, 6, 1> LeftArmFK (const Eigen::Matrix<double, 7, 1> q) const;
    Eigen::Matrix<double, 6, 1> RightArmFK(const Eigen::Matrix<double, 7, 1> q) const;
    Eigen::Matrix<double, 7, 1> LeftArmIK(const Eigen::Matrix<double, 7, 1>& q_guess, const Eigen::Matrix<double, 6, 1>& x_des) const;
    Eigen::Matrix<double, 7, 1> LeftArmIKAutoGuess(const Eigen::Matrix<double, 6, 1> &x_des) const;
    Eigen::Matrix<double, 6, 7> LeftArmJacobian(const Eigen::Matrix<double, 7, 1>& q) const;
    
    void DoCalcDiscreteVariableUpdates(
        const drake::systems::Context<double>& context,
        const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
        drake::systems::DiscreteValues<double>* updates) const override;
    void CopyDiscreteStateOut(
        const drake::systems::Context<double>& context,
        drake::systems::BasicVector<double>* output) const;
    

};


}
}
}