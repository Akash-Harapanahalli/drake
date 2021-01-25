#include <cmath>
#include <iostream>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/systems/primitives/integrator.h"

namespace drake {
    namespace course {
        namespace L3 {
            class PseudoInverseController : public systems::LeafSystem<double> {
            public:
                const multibody::MultibodyPlant<double>& _plant;
                systems::Context<double>& _plant_context;
                multibody::ModelInstanceIndex _iiwa;
                const multibody::BodyFrame<double>& _G;
                const multibody::BodyFrame<double>& _W;

                PseudoInverseController(const multibody::MultibodyPlant<double>& plant) : 
                    systems::LeafSystem<double>(),
                    _plant(plant),
                    _plant_context(*plant.CreateDefaultContext()),
                    _iiwa(plant.GetModelInstanceByName("iiwa")),
                    _G(plant.GetBodyByName("body").body_frame()),
                    _W(plant.world_frame()){
                    DeclareVectorInputPort ("iiwa_position", systems::BasicVector<double>(7));
                    DeclareVectorOutputPort("iiwa_velocity", systems::BasicVector<double>(7), &PseudoInverseController::CalcOutputVector, {this->nothing_ticket()});
                }

                void CalcOutputVector(const systems::Context<double>& context, systems::BasicVector<double>* output) const {
                    const Eigen::Matrix<double, 7, 1>& b = this->get_input_port().Eval(context);
                    _plant.SetPositions(&_plant_context, _iiwa, b);

                    Eigen::MatrixXd iJ_G (6, _plant.num_velocities());
                    _plant.CalcJacobianSpatialVelocity(
                        _plant_context, multibody::JacobianWrtVariable::kQDot,
                        _G, Eigen::Matrix<double, 3, 1>::Zero(), _W, _W, &iJ_G
                    );

                    Eigen::Matrix<double, 6, 7> J_G = iJ_G.block(0,0,5,6);
                    // Eigen::Matrix<double, 7, 6> J_G_inv = J_G.completeOrthogonalDecomposition().pseudoInverse();
                    Eigen::Matrix<double, 7, 6> J_G_inv = J_G.transpose() * (J_G*J_G.transpose()).inverse();
                    Eigen::Matrix<double, 6, 1> V_G_desired;
                    V_G_desired << 0, 0, -0.5, 0, -0.05, 0;

                    Eigen::Matrix<double, 7, 1> out = J_G_inv * V_G_desired;
                    output->SetFromVector(out);
                }

            };

            void jacobianControllerExample(){
                systems::DiagramBuilder<double> builder;
                auto& station = *(builder.AddSystem(std::make_unique<examples::manipulation_station::ManipulationStation<double>>()));
                builder.AddSystem(station.get_mutable_scene_graph())
                station.SetupClutterClearingStation();
                station.Finalize();
                auto& controller = *(builder.AddSystem(std::make_unique<PseudoInverseController>(station.get_multibody_plant())));
                auto& integrator = *(builder.AddSystem(std::make_unique<systems::Integrator<double>>(7)));

                builder.Connect(controller.get_output_port(), integrator.get_input_port());
                builder.Connect(integrator.get_output_port(), station.GetInputPort("iiwa_position"));
                builder.Connect(station.GetOutputPort("iiwa_position_measured"), controller.get_input_port());

                geometry::ConnectDrakeVisualizer(&builder, station.get_scene_graph());

                auto& diagram = *(builder.Build());
                auto context = diagram.CreateDefaultContext();
                auto simulator = std::make_unique<systems::Simulator<double>>(diagram, std::move(context));
                auto& station_context = station.GetMyMutableContextFromRoot(&simulator->get_mutable_context());
                station.GetInputPort("iiwa_forward_torque").FixValue(&station_context, Eigen::Matrix<double, 7, 1>::Zero());
                station.GetInputPort("wsg_position").FixValue(&station_context, Eigen::Matrix<double, 1, 1>::Constant(0.1));

                integrator.GetMyMutableContextFromRoot(&simulator->get_mutable_context())
                                .get_mutable_continuous_state_vector()
                                .SetFromVector(station.GetIiwaPosition(station_context));
                
                simulator->set_target_realtime_rate(1.0);
                simulator->Initialize();
                simulator->AdvanceTo(5.0);
            }
        }
    }
}

int main(){//int argc, char* argv[]){
    drake::course::L3::jacobianControllerExample();
}