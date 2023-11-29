#ifndef _PEXOD_PLUGIN_HH_
#define _PEXOD_PLUGIN_HH_


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>

namespace gazebo
{
  /// \brief A plugin to control an hexapod.
  class PexodPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: PexodPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        gzmsg << "Starting Hexopod Plugin...";
        // Safety check
        if (_model->GetJointCount() == 0)
        {
            std::cerr << "Invalid joint count, Pexapod plugin not loaded\n";
        return;
        }
        
        // Store the model pointer for convenience.
        this->model = _model;
        // Leg 1
        this->joint_body_leg_0 = _model->GetJoint("body_leg_0");
        this->joint_leg_0_1_2 = _model->GetJoint("leg_0_1_2");
        this->joint_leg_0_2_3 = _model->GetJoint("leg_0_2_3");
        // Leg 2
        this->joint_body_leg_1 = _model->GetJoint("body_leg_1");
        this->joint_leg_1_1_2 = _model->GetJoint("leg_1_1_2");
        this->joint_leg_1_2_3 = _model->GetJoint("leg_1_2_3");
        // Leg 3
        this->joint_body_leg_2 = _model->GetJoint("body_leg_2");
        this->joint_leg_2_1_2 = _model->GetJoint("leg_2_1_2");
        this->joint_leg_2_2_3 = _model->GetJoint("leg_2_2_3");
        // Leg 4
        this->joint_body_leg_3 = _model->GetJoint("body_leg_3");
        this->joint_leg_3_1_2 = _model->GetJoint("leg_3_1_2");
        this->joint_leg_3_2_3 = _model->GetJoint("leg_3_2_3");
        // Leg 5
        this->joint_body_leg_4 = _model->GetJoint("body_leg_4");
        this->joint_leg_4_1_2 = _model->GetJoint("leg_4_1_2");
        this->joint_leg_4_2_3 = _model->GetJoint("leg_4_2_3");
        // Leg 6
        this->joint_body_leg_5 = _model->GetJoint("body_leg_5");
        this->joint_leg_5_1_2 = _model->GetJoint("leg_5_1_2");
        this->joint_leg_5_2_3 = _model->GetJoint("leg_5_2_3");
        
        // Setup a P-controller, with a gain of 0.1.
        // Leg 1
        this->pid_body_leg_0 = common::PID(1.0, 0, 0);
        this->pid_leg_0_1_2 = common::PID(1.0, 0, 0);
        this->pid_leg_0_2_3 = common::PID(1.0, 0, 0);
        // Leg 2
        this->pid_body_leg_1 = common::PID(1.0, 0, 0);
        this->pid_leg_1_1_2 = common::PID(1.0, 0, 0);
        this->pid_leg_1_2_3 = common::PID(1.0, 0, 0);
        // Leg 3
        this->pid_body_leg_2 = common::PID(1.0, 0, 0);
        this->pid_leg_2_1_2 = common::PID(1.0, 0, 0);
        this->pid_leg_2_2_3 = common::PID(1.0, 0, 0);
        // Leg 4
        this->pid_body_leg_3 = common::PID(1.0, 0, 0);
        this->pid_leg_3_1_2 = common::PID(1.0, 0, 0);
        this->pid_leg_3_2_3 = common::PID(1.0, 0, 0);
        // Leg 5
        this->pid_body_leg_4 = common::PID(1.0, 0, 0);
        this->pid_leg_4_1_2 = common::PID(1.0, 0, 0);
        this->pid_leg_4_2_3 = common::PID(1.0, 0, 0);
        // Leg 6
        this->pid_body_leg_5 = common::PID(1.0, 0, 0);
        this->pid_leg_5_1_2 = common::PID(1.0, 0, 0);
        this->pid_leg_5_2_3 = common::PID(1.0, 0, 0);
        
        // Leg 1
        this->model->GetJointController()->SetPositionPID(this->joint_body_leg_0->GetScopedName(), this->pid_body_leg_0);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_1_2->GetScopedName(), this->pid_leg_0_1_2);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_2_3->GetScopedName(), this->pid_leg_0_2_3);
        // Leg 2
        this->model->GetJointController()->SetPositionPID(this->joint_body_leg_0->GetScopedName(), this->pid_body_leg_1);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_1_2->GetScopedName(), this->pid_leg_1_1_2);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_2_3->GetScopedName(), this->pid_leg_1_2_3);
        // Leg 3
        this->model->GetJointController()->SetPositionPID(this->joint_body_leg_0->GetScopedName(), this->pid_body_leg_2);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_1_2->GetScopedName(), this->pid_leg_2_1_2);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_2_3->GetScopedName(), this->pid_leg_2_2_3);
        // Leg 4
        this->model->GetJointController()->SetPositionPID(this->joint_body_leg_0->GetScopedName(), this->pid_body_leg_3);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_1_2->GetScopedName(), this->pid_leg_3_1_2);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_2_3->GetScopedName(), this->pid_leg_3_2_3);
        // Leg 5
        this->model->GetJointController()->SetPositionPID(this->joint_body_leg_0->GetScopedName(), this->pid_body_leg_4);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_1_2->GetScopedName(), this->pid_leg_4_1_2);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_2_3->GetScopedName(), this->pid_leg_4_2_3);
        // Leg 6
        this->model->GetJointController()->SetPositionPID(this->joint_body_leg_0->GetScopedName(), this->pid_body_leg_5);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_1_2->GetScopedName(), this->pid_leg_5_1_2);
        this->model->GetJointController()->SetPositionPID(this->joint_leg_0_2_3->GetScopedName(), this->pid_leg_5_2_3);

        
        this->SetJointPositions(0.0);

        
        
    }
    
    /// \brief Set positions of the Pexod Hexapod Legs
    /// \param[in] _position New target position for all the legs
    public: void SetJointPositions(const double &_position)
    {
        // Set the joint's target velocity.
        gzmsg << "Setting Pose All Joints...";
        // Leg 1
        this->model->GetJointController()->SetPositionTarget(this->joint_body_leg_0->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_0_1_2->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_0_2_3->GetScopedName(), _position);
        // Leg 2
        this->model->GetJointController()->SetPositionTarget(this->joint_body_leg_1->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_1_1_2->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_1_2_3->GetScopedName(), _position);
        // Leg 3
        this->model->GetJointController()->SetPositionTarget(this->joint_body_leg_2->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_2_1_2->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_2_2_3->GetScopedName(), _position);
        // Leg 4
        this->model->GetJointController()->SetPositionTarget(this->joint_body_leg_3->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_3_1_2->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_3_2_3->GetScopedName(), _position);
        // Leg 5
        this->model->GetJointController()->SetPositionTarget(this->joint_body_leg_4->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_4_1_2->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_4_2_3->GetScopedName(), _position);
        // Leg 6
        this->model->GetJointController()->SetPositionTarget(this->joint_body_leg_5->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_5_1_2->GetScopedName(), _position);
        this->model->GetJointController()->SetPositionTarget(this->joint_leg_5_2_3->GetScopedName(), _position);
        
    }
    
    
    
    
    
    
    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
        this->SetJointPositions(_msg->x());
    }
    
    /// \brief A node used for transport
    private: transport::NodePtr node;
    
    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;
    
    /// \brief Pointer to the model.
    private: physics::ModelPtr model;
    
    /// \brief Pointer to the joint.
    // Leg 1
    private: physics::JointPtr joint_body_leg_0;
    private: physics::JointPtr joint_leg_0_1_2;
    private: physics::JointPtr joint_leg_0_2_3;
    // Leg 2
    private: physics::JointPtr joint_body_leg_1;
    private: physics::JointPtr joint_leg_1_1_2;
    private: physics::JointPtr joint_leg_1_2_3;
    // Leg 3
    private: physics::JointPtr joint_body_leg_2;
    private: physics::JointPtr joint_leg_2_1_2;
    private: physics::JointPtr joint_leg_2_2_3;
    // leg 4 
    private: physics::JointPtr joint_body_leg_3;
    private: physics::JointPtr joint_leg_3_1_2;
    private: physics::JointPtr joint_leg_3_2_3;
    // Leg 5
    private: physics::JointPtr joint_body_leg_4;
    private: physics::JointPtr joint_leg_4_1_2;
    private: physics::JointPtr joint_leg_4_2_3;
    // Leg 6
    private: physics::JointPtr joint_body_leg_5;
    private: physics::JointPtr joint_leg_5_1_2;
    private: physics::JointPtr joint_leg_5_2_3;
    
    /// \brief A PID controller for the joint.
    // Leg 1
    private: common::PID pid_body_leg_0;
    private: common::PID pid_leg_0_1_2;
    private: common::PID pid_leg_0_2_3;
    // Leg 2
    private: common::PID pid_body_leg_1;
    private: common::PID pid_leg_1_1_2;
    private: common::PID pid_leg_1_2_3;
    // Leg 3
    private: common::PID pid_body_leg_2;
    private: common::PID pid_leg_2_1_2;
    private: common::PID pid_leg_2_2_3;
    // Leg 4
    private: common::PID pid_body_leg_3;
    private: common::PID pid_leg_3_1_2;
    private: common::PID pid_leg_3_2_3;
    // Leg 5
    private: common::PID pid_body_leg_4;
    private: common::PID pid_leg_4_1_2;
    private: common::PID pid_leg_4_2_3;
    // Leg 6
    private: common::PID pid_body_leg_5;
    private: common::PID pid_leg_5_1_2;
    private: common::PID pid_leg_5_2_3;
    
    
    
    
    
    
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(PexodPlugin)
}
#endif