#pragma once

#include <vector>

#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace examples {
namespace robocup {

//using Eigen::Vector2d;

template<typename T>
class robot_simulation : public systems::LeafSystem<T>
{
private:
    //const Eigen::Vector2d radius = {0.075, 0.060};
    const double radius = 0.096;
    // inline Eigen::Vector2d calcYdRobot(T ydWheel) const {
    //     return (radius * ((0.060*ydWheel)/0.009225));
    // }
    int input_port_index = -1;
    int output_port_index = -1;
    int pose_output_port = -1;
    std::vector<geometry::FrameId> frame_ids;
public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(robot_simulation);
    robot_simulation(std::vector<geometry::FrameId> _frame_ids);
    ~robot_simulation();
    void CopyStateOut(const systems::Context<T>& context, systems::BasicVector<T>* state_output_vector) const;
    void DoCalcTimeDerivatives(const systems::Context<T>& context, systems::ContinuousState<T>* derivatives) const;
    void CalcFramePoseOutput(const systems::Context<T>& context, geometry::FramePoseVector<T>* poses) const;
    void CalcPoseOutput(const systems::Context<T>& context, systems::BasicVector<T>* output) const;
};

}
}
}