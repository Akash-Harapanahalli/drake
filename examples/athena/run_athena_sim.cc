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

#include "drake/examples/athena/athena.h"

DEFINE_double(simulation_time, 60, "Desired duration of the simulation in seconds");
DEFINE_double(max_time_step, 1.0e-4, "Simulation time step used for integrator.");

namespace drake {
namespace examples {
namespace athena {

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

void main() {
    if (FLAGS_max_time_step < 0) {
        throw std::runtime_error(
            "mbp_discrete_update_period must be a non-negative number.");
    }
    std::string file_name;

    file_name = "/app/examples/athena/lists/log_l_wrist.txt";
    std::vector<Eigen::Matrix<double, 6, 1>> l_list;
    read_data<6>(l_list, file_name);

    file_name = "/app/examples/athena/lists/log_r_wrist.txt";
    std::vector<Eigen::Matrix<double, 6, 1>> r_list;
    read_data<6>(r_list, file_name);

    // Build a multibody plant.
    systems::DiagramBuilder<double> builder;
    MultibodyPlant<double> plant(0.2);
    plant.set_name("plant");

    SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
    scene_graph.set_name("scene_graph");

    plant.RegisterAsSourceForSceneGraph(&scene_graph);

    std::string full_name;
    full_name = FindResourceOrThrow("drake/examples/athena/models/upperbody_noinertial.urdf");
    Parser parser(&plant, &scene_graph);
    parser.AddModelFromFile(full_name);

    plant.Finalize();

    auto ath = builder.AddSystem<drake::examples::athena::Athena>();
    ath->init(l_list, r_list);

    auto athena_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);

    builder.Connect(*ath, *athena_to_pose);
    builder.Connect(athena_to_pose->get_output_port(), scene_graph.get_source_pose_port(
        plant.get_source_id().value()));
    drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
    auto diagram = builder.Build();

    auto context = diagram->CreateDefaultContext();

    auto simulator = std::make_unique<Simulator<double>>(*diagram, std::move(context));
    simulator->set_target_realtime_rate(0.5);
    simulator->Initialize();
    simulator->AdvanceTo(FLAGS_simulation_time);
}

}
}
}

int main(int argc, char* argv[]){
    gflags::SetUsageMessage(
        "Athena robot following trajectories using IK"
    );
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::examples::athena::main();

    return 0;
}