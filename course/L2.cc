#include <cmath>
#include <iostream>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace drake {
    namespace course {
        namespace L2 {
            void main(){
                systems::DiagramBuilder<double> builder;
                auto [plant, scene_graph] = multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

                std::string model_name = FindResourceOrThrow("drake/course/models/iiwa_description/urdf/iiwa14_no_collision.urdf"); 
                multibody::Parser parser(&plant, &scene_graph);
                parser.AddModelFromFile(model_name);
                plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
                plant.Finalize();

                geometry::ConnectDrakeVisualizer(&builder, scene_graph);

                auto diagram = builder.Build();
                auto context = diagram->CreateDefaultContext();
                auto& plant_context = plant.GetMyMutableContextFromRoot(context.get());
                
                Eigen::Matrix<double, 7, 1> v;
                v << -1.57, 0.1, 0, -1.2, 0, 1.6, 0;
                Eigen::Matrix<double, 7, 1> z;
                z << 0, 0, 0, 0, 0, 0, 0;
                plant.SetPositions(&plant_context, v);
                plant.get_actuation_input_port().FixValue(&plant_context, z);

                auto context2 = diagram->CreateDefaultContext();

                auto simulator = std::make_unique<systems::Simulator<double>>(*diagram, std::move(context));
                simulator->set_target_realtime_rate(1.0);
                simulator->Initialize();
                simulator->AdvanceTo(5.0);
            }
        }
    }
}

int main(){//int argc, char* argv[]){
    drake::course::L2::main();
}