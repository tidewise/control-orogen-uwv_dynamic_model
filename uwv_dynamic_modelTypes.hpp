#ifndef uwv_dynamic_model_TYPES_HPP
#define uwv_dynamic_model_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */
#include <base/samples/RigidBodyAcceleration.hpp>
#include <orocos/auv_control/6dControl.hpp>

namespace uwv_dynamic_model
{
    struct SecondaryStates
    {
        base::samples::RigidBodyAcceleration    linear_acceleration;
        base::samples::RigidBodyAcceleration    angular_acceleration;
        base::LinearAngular6DCommand            efforts;
    };
}

#endif
