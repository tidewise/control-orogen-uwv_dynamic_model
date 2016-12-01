/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef uwv_dynamic_model_TASK_TASK_HPP
#define uwv_dynamic_model_TASK_TASK_HPP

#include "uwv_dynamic_model/TaskBase.hpp"
#include "uwv_dynamic_model/ModelSimulation.hpp"
#include "uwv_dynamic_model/DataTypes.hpp"

namespace uwv_dynamic_model {

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','uwv_dynamic_model::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

	ModelSimulation* model_simulation;
	ModelSimulator simulator;
	base::Time last_control_input;

    /**
	 *  Check controlInput for NaN and repeated timestamp.
	 */
	bool checkInput(const base::LinearAngular6DCommand &control_input);

    /**
	 *  Convert PoseVelocityState representation to RigidBodyState
	 */
	base::samples::RigidBodyState toRBS(const PoseVelocityState &states);

    /**
     *  Convert RigidBodyState representation to PoseVelocityState
     */
	PoseVelocityState fromRBS(const base::samples::RigidBodyState &states);

    /**
     *  Convert LinearAngular6DCommand representation to Vector6d
     */
	base::Vector6d toVector6d(const base::LinearAngular6DCommand &control_input);

    /**
     *  Get secondary states
     */
	SecondaryStates getSecondaryStates(const base::LinearAngular6DCommand &control_input, const AccelerationState &acceleration);

	/**
	 * Sets the uncertainty value for pose and velocity
	 */
	void setUncertainty(base::samples::RigidBodyState &states);

    /**
    * Sets the simulator used.
    */
	void setSimulator(ModelSimulator simulator);

    /**
     * Do something with data in derived class
     * @param pose state
     * @param effort control_input
     */
	virtual void handleStates(const base::samples::RigidBodyState &state, const base::LinearAngular6DCommand &control_input);

	/**
	 * Set Runtime component state independently
	 * Default: SIMULATING
	 */
	virtual void setRuntimeState();

	/**
	 * Resets the states of the model (position and velocity)
	 */
	virtual void resetStates(void);

    /**
     * Set the model's states to pose_state
     * @param pose_state, the position, orientation and velocities states used to set the model
     */
    virtual void setStates(::base::samples::RigidBodyState const & pose_state);

    public:
        Task(std::string const& name = "uwv_dynamic_model::Task", ModelSimulator simulator = DYNAMIC_KINEMATIC);
        Task(std::string const& name, RTT::ExecutionEngine* engine, ModelSimulator simulator = DYNAMIC_KINEMATIC);
	~Task();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif
