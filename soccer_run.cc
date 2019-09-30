#include <iostream>

#include "drake/examples/robocup/robot_simulation.h"
#include "drake/examples/robocup/robot_control.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/geometry/geometry_visualization.h"


namespace drake {
namespace examples {
namespace robocup {
namespace {

class soccer_run : public systems::Diagram<double>
{
private:
    
public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(soccer_run);
    soccer_run(/* args */);
    ~soccer_run();
    std::unique_ptr<systems::Context<double>> createContext() const;
    systems::SignalLogger<double>* logger;
};

soccer_run::soccer_run(/* args */)
{
    systems::DiagramBuilder<double> builder;
    // VectorX<double> voltages(2);
    // voltages << 2,2;
    // auto constantValue = builder.AddSystem(std::make_unique<systems::ConstantVectorSource<double>>(voltages));
    auto plant = builder.AddSystem(std::make_unique<multibody::MultibodyPlant<double>>());
    auto scene_graph = builder.AddSystem(std::make_unique<geometry::SceneGraph<double>>());

    multibody::Parser parser(plant, scene_graph);
    const std::string path = FindResourceOrThrow("drake/examples/robocup/models/differential_drive.urdf");
    geometry::SourceId source_id = plant->RegisterAsSourceForSceneGraph(scene_graph);
    multibody::ModelInstanceIndex modelInstanceName = parser.AddModelFromFile(path);
    // std::cout << scene_graph->model_inspector().FramesForSource(source_id).size() << std::endl;
    std::vector<geometry::FrameId> frame_ids;
    for(int i = 0; i < 7; i++) {
        frame_ids.push_back({plant->GetBodyFrameIdOrThrow(plant->GetBodyIndices(modelInstanceName)[i])});
    }

    plant->RegisterVisualGeometry(plant->world_body(), math::RigidTransformd(), geometry::HalfSpace(), "Floor");
    plant->Finalize();
    
    auto vector_source_system = builder.AddSystem(std::make_unique<robot_control<double>>());
    auto robot_simulation_system = builder.AddSystem(std::make_unique<robot_simulation<double>>(frame_ids)); 
    
    geometry::ConnectDrakeVisualizer(&builder, *scene_graph);

    builder.Connect(vector_source_system->get_output_port(0), 
                    robot_simulation_system->get_input_port(0));

    builder.Connect(robot_simulation_system->get_output_port(1),
                    plant->get_actuation_input_port());

    builder.Connect(robot_simulation_system->get_output_port(2),
                    scene_graph->get_source_pose_port(source_id));
                    
    builder.Connect(scene_graph->get_query_output_port(),
                    plant->get_geometry_query_input_port());

    logger = LogOutput(robot_simulation_system->get_output_port(0), &builder);
    builder.BuildInto(this);
}

soccer_run::~soccer_run()
{
}

std::unique_ptr<systems::Context<double>> soccer_run::createContext() const {
    auto context = this->AllocateContext();
    systems::VectorBase<double>& cstate = context->get_mutable_continuous_state_vector();
    cstate.SetAtIndex(0, 1.0);
    cstate.SetAtIndex(1, 2.0);
    cstate.SetAtIndex(4, 0);
    return context;
}

int doMain() {
    auto system = std::make_unique<soccer_run>();
    systems::Simulator<double> simulator(*system, system->createContext());
    simulator.Initialize();
    simulator.set_target_realtime_rate(1);
    simulator.AdvanceTo(3);

    for(int i = 0; i < system->logger->sample_times().size(); i++) {
        const double logOut = system->logger->sample_times()[i];
        std::cout << system->logger->data()(0,i) << "  " << system->logger->data()(1,i) << "  " << system->logger->data()(2,i) << "  " << system->logger->data()(3,i) << "  " << system->logger->data()(4,i) << "  ( " << logOut << " )  " << std::endl;
    }
    return 0;
}

}
}
}
}
int main(int argc, char** argv) {
    argc = 0;
    argv = nullptr;
    drake::examples::robocup::doMain();
    return 0;
}