#include "drake/examples/robocup/robot_control.h"

namespace drake {
namespace examples {
namespace robocup {

template<typename T>
robot_control<T>::robot_control(/* args */)
{
    this->DeclareVectorOutputPort(systems::BasicVector<T>(4), &robot_control::CopyStateOut);
}

template<typename T>
robot_control<T>::~robot_control() {}

template<typename T>
void robot_control<T>::CopyStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const {
    //systems::BasicVector<T> out(4);
    VectorX<double> vec(4);
    vec << 2,3,24,24;
    //out.set_value(vec);
    output->set_value(vec);
}

template class robot_control<double>;

}
}
}