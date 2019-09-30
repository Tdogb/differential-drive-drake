#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace robocup {

template<typename T>
class robot_control : public systems::LeafSystem<T>
{
private:
    /* data */
public:
    robot_control(/* args */);
    ~robot_control();
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(robot_control); 
    void CopyStateOut(const systems::Context<T>& context, systems::BasicVector<T>* output) const;
};

}
}
}