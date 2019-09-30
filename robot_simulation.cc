#include "drake/examples/robocup/robot_simulation.h"

namespace drake {
namespace examples {
namespace robocup {

/*
Input: motor velocities, motor voltage

Output: robot x dot, y dot, theta dot
*/
template<typename T>
robot_simulation<T>::robot_simulation(std::vector<geometry::FrameId> _frame_ids)
:frame_ids(_frame_ids)
{
    input_port_index = this->DeclareVectorInputPort("input", systems::BasicVector<T>(4)).get_index(); //Voltage
    this->DeclareContinuousState(2, 2, 1); //x,y v theta
    output_port_index = this->DeclareVectorOutputPort("velocities_output", systems::BasicVector<T>(5), &robot_simulation::CopyStateOut).get_index(); //Context?
    pose_output_port = this->DeclareVectorOutputPort("pose_output", systems::BasicVector<T>(2), &robot_simulation::CalcPoseOutput).get_index();
    this->DeclareAbstractOutputPort("scene_graph_output", geometry::FramePoseVector<T>(), &robot_simulation::CalcFramePoseOutput);
}

template<typename T>
robot_simulation<T>::~robot_simulation() {
}

template<typename T>
void robot_simulation<T>::CopyStateOut(const systems::Context<T>& context, systems::BasicVector<T>* state_output_vector) const{
    state_output_vector->set_value(context.get_continuous_state_vector().CopyToVector());
}

template<typename T>
void robot_simulation<T>::CalcPoseOutput(const systems::Context<T>& context, systems::BasicVector<T>* output) const {
    const auto state_vector = context.get_continuous_state_vector().CopyToVector();
    output->SetAtIndex(0, state_vector[0]);
    output->SetAtIndex(1, state_vector[1]);
}

template<typename T>
void robot_simulation<T>::CalcFramePoseOutput(const systems::Context<T>& context, geometry::FramePoseVector<T>* poses) const {
    Eigen::Vector3d position_vector;
    Eigen::Vector3d rotation_vector;
    rotation_vector.setZero();
    position_vector.setZero();
    Eigen::VectorXd state_vector = context.get_continuous_state_vector().CopyToVector();

    position_vector.head<2>() = state_vector.head<2>();
    rotation_vector.tail<1>() = state_vector.tail<1>();
    math::RollPitchYawd rotation(rotation_vector);
    math::RigidTransformd pose(rotation, position_vector);
    math::RigidTransformd pose1{};
    *poses = {{frame_ids.at(0), pose}, {frame_ids.at(1), pose}, {frame_ids.at(2), pose}, {frame_ids.at(3), pose}, {frame_ids.at(4), pose}, {frame_ids.at(5), pose}, {frame_ids.at(6), pose}};
}

template<typename T>
void robot_simulation<T>::DoCalcTimeDerivatives(const systems::Context<T>& context, systems::ContinuousState<T>* derivatives) const {
    const systems::VectorBase<T>& state_vector = context.get_continuous_state_vector();
    const auto input_vector = systems::System<T>::get_input_port(input_port_index).Eval(context); //Whyyyyy systems::System<T>
    const T yd_wheel_r = input_vector[0];
    const T yd_wheel_l = input_vector[1];
    const T rotation_v = (yd_wheel_r-yd_wheel_l);
    const T tranlation_v = (yd_wheel_r+yd_wheel_l)/2;
    const T theta = state_vector[4];
    const T theta_dot = rotation_v / radius;
    systems::VectorBase<T>& positionDeriv = derivatives->get_mutable_generalized_position(); //Robot velocity components
    systems::VectorBase<T>& velocityDeriv = derivatives->get_mutable_generalized_velocity(); //Robot acceleration components
    systems::VectorBase<T>& miscDeriv = derivatives->get_mutable_misc_continuous_state(); //Theta components
    positionDeriv.SetAtIndex(0, std::sin(theta) * (tranlation_v + rotation_v));
    positionDeriv.SetAtIndex(1, std::cos(theta) * (tranlation_v + rotation_v));
    velocityDeriv.SetAtIndex(0, 0);
    velocityDeriv.SetAtIndex(1, 0);
    miscDeriv.SetAtIndex(0, theta_dot);
}

template class robot_simulation<double>;

}
}
}