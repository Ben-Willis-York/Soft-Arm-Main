#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <stdio.h>
#include <vector>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <thread>


namespace gazebo
{
  class JointController : public ModelPlugin
  {
    public:
    private:
      bool paramsFound = false;

      physics::ModelPtr model; // Pointer to the model
      event::ConnectionPtr updateConnection; // Pointer to the update event connection
      sdf::ElementPtr sdf; //Pointer to SDF element

      std::vector<physics::JointPtr> joints; //Array of spring joints

      std::vector<common::PID> pids;

      std::vector<std::string> jointNames;

      std::unique_ptr<ros::NodeHandle> rosNode;
      ros::Subscriber rosSub;
      ros::CallbackQueue rosQueue;
      std::thread rosQueueThread;

      std::vector<float> targets;
      std::vector<float> setpoints;
      float speed = 0.0001;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->sdf = _sdf;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&JointController::OnUpdate, this));

      //Setup ROS node
      std::string node_name = std::string("my_controller_node");
      if (!ros::isInitialized()) 
      {
        int argc = 0;
        char **argv = nullptr;

        ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle(node_name));
      
      ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
      "/" + this->model->GetName() + "/commands",
      1,
      boost::bind(&JointController::OnRosMsg, this, _1),
      ros::VoidPtr(), &this->rosQueue);

      this->rosSub = this->rosNode->subscribe(so);

      this->rosQueueThread = std::thread(std::bind(&JointController::QueueThread, this));
      
      this->GetJoints();
    }

    
    // Called by the world update start event
    public: void OnUpdate()
    {
      //Get joint names
      if(this->paramsFound == false) { this-> GetJoints(); }
      
      if(this->paramsFound)
      {
        for(int i = 0; i < targets.size(); i++)
        {
          float pos = this->joints[i]->Position();
          if(this->setpoints[i] < this->targets[i]-0.002) { this->setpoints[i] += this->speed; }
          if(this->setpoints[i] > this->targets[i]+0.002) { this->setpoints[i] -= this->speed; }
          else{this->setpoints[i] = this->targets[i]; }
          this->model->GetJointController()->SetPositionTarget(this->joints[i]->GetScopedName(), this->setpoints[i]);
          //std::cerr << this->setpoints[i] << ",";
        }
        //std::cerr << std::endl;

      }
    }

    public: void GetJoints()
    {
      const std::string paramName = this->sdf->Get<std::string>("jointNamesParam");

      std::cerr << paramName << std::endl;

      if(!rosNode->getParam("/arm_joints/joints", this->jointNames))
      { std::cerr << "Not Found" << std::endl; }
      else
      { 
        this->paramsFound = true;
        std::cerr << "FOUND JOINTS" << std::endl;
        for(int i = 0; i < this->jointNames.size(); i++)
        {
          std::cerr << this->jointNames[i] << std::endl;
          this->joints.push_back(this->model->GetJoint(this->jointNames[i]));
          targets.push_back(-0.7);
          setpoints.push_back(0);
          pids.push_back(common::PID(2000,0,0));
          this->model->GetJointController()->SetPositionPID(this->joints[i]->GetScopedName(), this->pids[i]);
          this->model->GetJointController()->SetPositionTarget(this->joints[i]->GetScopedName(), 1.0);
        }
      }
    }
  
  
    public: void OnRosMsg(const std_msgs::Float32MultiArray::ConstPtr& _msg)
    {
      std::vector<float> arr = _msg->data;
      for(int i =0; i < arr.size(); i++)
      {
        this->targets[i] = arr[i];
        std::cerr << arr[i] << std::endl;
      }
      //this->kConstant = _msg->data;
      //std::cerr << this->kConstant << std::endl;
      //this->rosNode->setParam("/Design/kConstant", this->kConstant);
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(JointController)
}
