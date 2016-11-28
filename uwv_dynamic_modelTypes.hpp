#ifndef uwv_dynamic_model_TYPES_HPP
#define uwv_dynamic_model_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/RigidBodyAcceleration.hpp>
#include <base/commands/LinearAngular6DCommand.hpp>

namespace uwv_dynamic_model
{
    struct SecondaryStates
    {
        base::samples::RigidBodyAcceleration    linear_acceleration;
        base::samples::RigidBodyAcceleration    angular_acceleration;
        base::LinearAngular6DCommand            efforts;
        base::Time time;
    };

    struct DynamicStates
    {
        base::samples::RigidBodyState pose;
        uwv_dynamic_model::SecondaryStates secondary_states;
        base::Time time;
    };

}

#endif
