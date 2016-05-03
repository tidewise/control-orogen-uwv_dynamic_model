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

	boost::shared_ptr<ModelSimulation> model_simulation;
	DynamicSimulator* simulator;
	UWVParameters    model_parameters;
	base::Time last_control_input;
	base::samples::RigidBodyState states;
	SecondaryStates secondary_states;

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
	 * Transforms a set of coordinates from euler to axis-angle representation
	 */
	base::Vector3d eulerToAxisAngle(const base::Vector3d &states);
	/**
	 * Sets the uncertainty value for pose and velocity
	 */
	void setUncertainty(base::samples::RigidBodyState &states);

    /**
    * Sets the simulator used. Default is a DynamicKinematicSimulator
    */
    virtual DynamicSimulator* setSimulator(void);

    /**
	 * Do something with data in derived class
	 * @param controlInput
	 */
	virtual void handleControlInput(const base::LinearAngular6DCommand &control_input);

    /**
     * Do something with data in derived class
     * @param state
     */
	virtual void handlePoseState(const base::samples::RigidBodyState &state);

	/**
	 * Resets the states of the model (position and velocity)
	 */
	virtual void resetStates(void);

    public:
        Task(std::string const& name = "uwv_dynamic_model::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);
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
