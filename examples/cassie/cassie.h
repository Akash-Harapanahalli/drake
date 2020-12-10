#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "drake/common/find_resource.h"
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

using namespace std;
using namespace Eigen;

namespace drake {
namespace examples {
namespace cassie{

class Cassie : public drake::systems::LeafSystem<double>
{

DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Cassie);

private:
  // Traj
  vector<Matrix<double, 9, 1>> COM_list;
  vector<Matrix<double, 9, 1>> l_foot_list;
  vector<Matrix<double, 9, 1>> r_foot_list;  
  vector<Matrix<double, 6, 1>> l_wrist_list;
  vector<Matrix<double, 6, 1>> r_wrist_list;
  //vector<Matrix<double, 7, 1>> l_result_list;
  //vector<Matrix<double, 7, 1>> r_result_list;
  vector<Matrix<double, 1, 1>> heading_list;
  vector<Matrix<double, 3, 1>> box_list;
  vector<Matrix<double, 3, 1>> quadrotor_list;
  vector<Matrix<double, 1, 1>> heading_quadrotor_list;
  vector<Matrix<double, 3, 1>> box_list1;
  vector<Matrix<double, 1, 1>> heading_box1_list;
  vector<Matrix<double, 3, 1>> box_list2;
  vector<Matrix<double, 1, 1>> heading_box2_list;
  vector<Matrix<double, 3, 1>> box_list3;
  vector<Matrix<double, 1, 1>> heading_box3_list;
  vector<Matrix<double, 3, 1>> box_list4;
  vector<Matrix<double, 1, 1>> heading_box4_list;
  vector<Matrix<double, 3, 1>> obstacle_list;
  vector<Matrix<double, 3, 1>> obstacle_list2;

  mutable int step = 0;

  // Simulation
  double h = 0.001;
  mutable double t = 0;
  mutable VectorXd ang_l = VectorXd::Constant(7, 0.0);
  mutable VectorXd ang_r = VectorXd::Constant(7, 0.0);

public:
  explicit Cassie();
  void InitSystem(vector<Matrix<double, 9, 1>> COM_l,
                  vector<Matrix<double, 9, 1>> l_foot_l,
                  vect 0;or<Matrix<double, 9, 1>> r_foot_l,
                  vector<Matrix<double, 6, 1>> l_wrist_l,
                  vector<Matrix<double, 6, 1>> r_wrist_l,
                  //vector<Matrix<double, 7, 1>> l_result_l,
                  //vector<Matrix<double, 7, 1>> r_result_l,
                  vector<Matrix<double, 1, 1>> heading_l,
                  vector<Matrix<double, 3, 1>> box_l,
                  vector<Matrix<double, 3, 1>> quadrotor_l,
                  vector<Matrix<double, 1, 1>> heading_quadrotor_l,
                  vector<Matrix<double, 3, 1>> box_l1,
                  vector<Matrix<double, 1, 1>> heading_box1_l,
                  vector<Matrix<double, 3, 1>> box_l2,
                  vector<Matrix<double, 1, 1>> heading_box2_l,
                  vector<Matrix<double, 3, 1>> box_l3,
                  vector<Matrix<double, 1, 1>> heading_box3_l,
                  vector<Matrix<double, 3, 1>> box_l4,
                  vector<Matrix<double, 1, 1>> heading_box4_l,
                  vector<Matrix<double, 3, 1>> obstacle_l,
                  vector<Matrix<double, 3, 1>> obstacle_l2);

private:
  Matrix<double, 6, 1> LeftArmFK(const Matrix<double, 7, 1>& q_gue_l) const;
  Matrix<double, 6, 1> RightArmFK(const Matrix<double, 7, 1>& q_gue_r) const;
  Matrix<double, 6, 7> LeftArmJacobianFunction(const Matrix<double, 7, 1>& q_in_l) const;
  Matrix<double, 6, 7> RightArmJacobianFunction(const Matrix<double, 7, 1>& q_in_r) const;
  Matrix<double, 7, 1> DoIKLeftArm(const Matrix<double, 7, 1>& q_guess_l,
                             const Matrix<double, 6, 1>& x_d_l) const;
  Matrix<double, 7, 1> DoIKRightArm(const Matrix<double, 7, 1>& q_guess_r,
                             const Matrix<double, 6, 1>& x_d_r) const;
  Eigen::Matrix<double, 27, 1> DoIKCassie(const Matrix<double, 9, 1>& COM,
                                    const Matrix<double, 9, 1>& l_foot,
                                    const Matrix<double, 9, 1>& r_foot,
                                    const Matrix<double, 1, 1>& heading) const;
  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
      drake::systems::DiscreteValues<double>* updates) const override;
  void CopyDiscreteStateOut(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

};

}  // namespace cassie
}  // namespace examples
}  // namespace drake
