#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include "drake/examples/cassie/cassie.h"
DEFINE_double(simulation_time, 67.072	, "Desired duration of the simulation in seconds");
DEFINE_double(max_time_step, 1.0e-4, "Simulation time step used for integrator.");

namespace drake {
namespace examples {
namespace cassie {

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Simulator;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using Eigen::VectorBlock;
using Eigen::VectorXd;
using Eigen::Vector3d;
using drake::math::RigidTransformd;

template <int Derived>
int read_data(std::vector<Eigen::Matrix<double, Derived, 1>>& data, std::string& file_name)
{
  std::ifstream inFile;
  inFile.open(file_name, std::ios::in);
  std::string line;
  while (getline(inFile, line))
  {
    std::istringstream linestream(line);
    std::vector<std::string> vv;
    std::string v;
    while (getline(linestream, v, ' '))
    {
        vv.push_back(v);
    }

    Eigen::Matrix<double, Derived, 1> dd;
    for (int i = 0; i < Derived; i++)
    {
      dd(i, 0) = std::atof(vv[i].c_str());
    }
    data.push_back(dd);
  }
  inFile.close();

  return 1;
}

void DoMain() {
  if (FLAGS_max_time_step < 0) {
    throw std::runtime_error(
        "mbp_discrete_update_period must be a non-negative number.");
  }

  std::string file_name;
  
  file_name = "/app/examples/cassie/lists/log_COM.txt";
  std::vector<Eigen::Matrix<double, 9, 1>> COM_list;
  read_data<9>(COM_list, file_name);

  file_name = "/app/examples/cassie/lists/log_l_foot.txt";
  std::vector<Eigen::Matrix<double, 9, 1>> l_foot_list;
  read_data<9>(l_foot_list, file_name);

  file_name = "/app/examples/cassie/lists/log_r_foot.txt";
  std::vector<Eigen::Matrix<double, 9, 1>> r_foot_list;
  read_data<9>(r_foot_list, file_name);

  file_name = "/app/examples/cassie/lists/log_l_wrist.txt";
  std::vector<Eigen::Matrix<double, 6, 1>> l_wrist_list;
  read_data<6>(l_wrist_list, file_name);

  file_name = "/app/examples/cassie/lists/log_r_wrist.txt";
  std::vector<Eigen::Matrix<double, 6, 1>> r_wrist_list;
  read_data<6>(r_wrist_list, file_name);

  file_name = "/app/examples/cassie/lists/log_heading.txt";
  std::vector<Eigen::Matrix<double, 1, 1>> heading_list;
  read_data<1>(heading_list, file_name);

  file_name = "/app/examples/cassie/lists/log_box.txt";
  std::vector<Eigen::Matrix<double, 3, 1>> box_list;
  read_data<3>(box_list, file_name);

  file_name = "/app/examples/cassie/lists/log_box1.txt";
  std::vector<Eigen::Matrix<double, 3, 1>> box_list1;
  read_data<3>(box_list1, file_name);

  file_name = "/app/examples/cassie/lists/log_heading_box1.txt";
  std::vector<Eigen::Matrix<double, 1, 1>> heading_box1_list;
  read_data<1>(heading_box1_list, file_name);

  file_name = "/app/examples/cassie/lists/log_box2.txt";
  std::vector<Eigen::Matrix<double, 3, 1>> box_list2;
  read_data<3>(box_list2, file_name);

  file_name = "/app/examples/cassie/lists/log_heading_box2.txt";
  std::vector<Eigen::Matrix<double, 1, 1>> heading_box2_list;
  read_data<1>(heading_box2_list, file_name);

  file_name = "/app/examples/cassie/lists/log_box3.txt";
  std::vector<Eigen::Matrix<double, 3, 1>> box_list3;
  read_data<3>(box_list3, file_name);

  file_name = "/app/examples/cassie/lists/log_heading_box3.txt";
  std::vector<Eigen::Matrix<double, 1, 1>> heading_box3_list;
  read_data<1>(heading_box3_list, file_name);

  file_name = "/app/examples/cassie/lists/log_box4.txt";
  std::vector<Eigen::Matrix<double, 3, 1>> box_list4;
  read_data<3>(box_list4, file_name);

  file_name = "/app/examples/cassie/lists/log_heading_box4.txt";
  std::vector<Eigen::Matrix<double, 1, 1>> heading_box4_list;
  read_data<1>(heading_box4_list, file_name);

  file_name = "/app/examples/cassie/lists/log_quadrotor.txt";
  std::vector<Eigen::Matrix<double, 3, 1>> quadrotor_list;
  read_data<3>(quadrotor_list, file_name);

  file_name = "/app/examples/cassie/lists/log_heading_quadrotor.txt";
  std::vector<Eigen::Matrix<double, 1, 1>> heading_quadrotor_list;
  read_data<1>(heading_quadrotor_list, file_name);

  file_name = "/app/examples/cassie/lists/log_obstacle.txt";
  std::vector<Eigen::Matrix<double, 3, 1>> obstacle_list;
  read_data<3>(obstacle_list, file_name);

  file_name = "/app/examples/cassie/lists/log_obstacle2.txt";
  std::vector<Eigen::Matrix<double, 3, 1>> obstacle_list2;
  read_data<3>(obstacle_list2, file_name);

  // Build a multibody plant.
  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double> plant(0.2);
  plant.set_name("plant");

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  // Load Cassie    
  std::string full_name;
  full_name = FindResourceOrThrow("drake/examples/cassie/models/cassie_with_athena_intermediate.urdf");

  Parser parser(&plant, &scene_graph);
  parser.AddModelFromFile(full_name);

  full_name = FindResourceOrThrow("drake/examples/cassie/models/box.urdf");
  parser.AddModelFromFile(full_name);

  full_name = FindResourceOrThrow("drake/examples/cassie/models/quadrotor_team.urdf");
  parser.AddModelFromFile(full_name);

  full_name = FindResourceOrThrow("drake/examples/cassie/models/box1.urdf");
  parser.AddModelFromFile(full_name);

  full_name = FindResourceOrThrow("drake/examples/cassie/models/box2.urdf");
  parser.AddModelFromFile(full_name);

  full_name = FindResourceOrThrow("drake/examples/cassie/models/box3.urdf");
  parser.AddModelFromFile(full_name);

  full_name = FindResourceOrThrow("drake/examples/cassie/models/box4.urdf");
  parser.AddModelFromFile(full_name);

  full_name = FindResourceOrThrow("drake/examples/cassie/models/turtlebot.urdf");
  parser.AddModelFromFile(full_name);

  full_name = FindResourceOrThrow("drake/examples/cassie/models/turtlebot2.urdf");
  parser.AddModelFromFile(full_name);

  multibody::ModelInstanceIndex  floor1_instance = parser.AddModelFromFile(FindResourceOrThrow("drake/examples/cassie/models/environment_floor_1.sdf"));

  const RigidTransformd X_floor1(Vector3d(0, 0, 0));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("floor", floor1_instance), X_floor1);

  multibody::ModelInstanceIndex  floor2_instance = parser.AddModelFromFile(FindResourceOrThrow("drake/examples/cassie/models/environment_floor_2.sdf"));

  const RigidTransformd X_floor2(Vector3d(-0.05, 0, 0));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("center", floor2_instance), X_floor2);

  multibody::ModelInstanceIndex  station1_instance = parser.AddModelFromFile(FindResourceOrThrow("drake/examples/cassie/models/charging_station.sdf"));

  const RigidTransformd X_station1(Vector3d(3, -2, 0.05));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", station1_instance), X_station1);

  multibody::ModelInstanceIndex  station2_instance = parser.AddModelFromFile(FindResourceOrThrow("drake/examples/cassie/models/charging_station_2.sdf"));

  const RigidTransformd X_station2(Vector3d(3, -1, 0.05));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", station2_instance), X_station2);

  multibody::ModelInstanceIndex  static_box_instance = parser.AddModelFromFile(FindResourceOrThrow("drake/examples/cassie/models/multi_static_boxes.urdf"));

  const RigidTransformd X_staticbox(Vector3d(0, 0, 0));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", static_box_instance), X_staticbox);

  multibody::ModelInstanceIndex  stairs_instance = parser.AddModelFromFile(FindResourceOrThrow("drake/examples/cassie/models/stairs.sdf"));
  const RigidTransformd X_stairs(
      math::RollPitchYaw<double>(0, 0, M_PI_2), Vector3d(-1.2, -3, -0.0375));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(
      "step_0", stairs_instance), X_stairs);
  // Now the model is complete.
  plant.Finalize();

  auto cas = builder.AddSystem<drake::examples::cassie::Cassie>();
  cas->InitSystem(COM_list, l_foot_list, r_foot_list, l_wrist_list, r_wrist_list, heading_list, box_list, quadrotor_list, heading_quadrotor_list, box_list1, heading_box1_list, box_list2, heading_box2_list, box_list3, heading_box3_list, box_list4, heading_box4_list, obstacle_list, obstacle_list2);
  
  auto cassie_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  
  builder.Connect(*cas, *cassie_to_pose);
  builder.Connect(cassie_to_pose->get_output_port(), scene_graph.get_source_pose_port(
    plant.get_source_id().value()));
  drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  
  // Set up simulator.
  auto simulator = std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();
  simulator->AdvanceTo(FLAGS_simulation_time);

}

}  // namespace cassie
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Cassie robot following CoM and feet trajectories"
      " with joint angles calculated by IK.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::cassie::DoMain();
  return 0;
}

