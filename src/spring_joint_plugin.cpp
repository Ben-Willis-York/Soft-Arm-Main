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
  class SpringJoint : public ModelPlugin
  {
    public:
    private:
      bool paramsFound = false;
      double kConstant = 0.0;

      physics::ModelPtr model; // Pointer to the model

      event::ConnectionPtr updateConnection; // Pointer to the update event connection

      sdf::ElementPtr sdf; //Pointer to SDF element

      std::vector<physics::JointPtr> joints; //Array of spring joints

      std::vector<float> springStates;

      std::unique_ptr<ros::NodeHandle> rosNode;
      ros::Subscriber rosSub;
      ros::Publisher rosPub;
      ros::CallbackQueue rosQueue;
      std::thread rosQueueThread;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->sdf = _sdf;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SpringJoint::OnUpdate, this));

      //Setup ROS node
      std::string node_name = std::string("my_node");
      if (!ros::isInitialized()) 
      {
        int argc = 0;
        char **argv = nullptr;

        ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
      "/" + this->model->GetName() + "/kConstant",
      1,
      boost::bind(&SpringJoint::OnRosMsg, this, _1),
      ros::VoidPtr(), &this->rosQueue);

      this->rosSub = this->rosNode->subscribe(so);
      this->rosPub = this->rosNode->advertise<std_msgs::Float32MultiArray>("/"+this->model->GetName()+"/spring_states", 1);
      this->rosQueueThread = std::thread(std::bind(&SpringJoint::QueueThread, this));

      this->kConstant = _sdf->Get<double>("kConstant");
      this->rosNode->setParam("/Design/kConstant", this->kConstant);

      std::cerr << "K = " << this->kConstant << std::endl;
      

    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      //Get joint names
      if(this->paramsFound == false) { this-> GetJoints(); }
      
      if(this->paramsFound)
      {
        for(int i = 0; i < this->joints.size(); i++)
        {
          physics::JointPtr j = this->joints[i];
          double position = j->Position();
          //std::cerr << j->GetName() << ", " << position << std::endl;
          double force = position * -this->kConstant;
          //printf("%+7.5f\n\r", position);

          //std::cerr << j->GetName() << "," << position << std::endl;
          j->SetForce(0,force);
          springStates[i]=force;
        }
        PublishStates();
      }
    }

    private: void PublishStates()
    {
      std_msgs::Float32MultiArray msg = std_msgs::Float32MultiArray();
      msg.data = this->springStates;
      rosPub.publish(msg);
    }

    public: void GetJoints()
    {
      std::vector<std::string> param;
      if(!rosNode->getParam("/hand_controller/joints", param))
      { std::cerr << "Not Found" << std::endl; }
      else
      { 
        std::cerr << param[0] << std::endl; 
        this->paramsFound = true;
      }

      for(int i = 0; i < param.size(); i++)
      {
        this->joints.push_back(this->model->GetJoint(param[i]));
        this->springStates.push_back(0.0f);
      }
    }
  

    public: void OnRosMsg(const std_msgs::Float32ConstPtr& _msg)
    {
      this->kConstant = _msg->data;
      std::cerr << this->kConstant << std::endl;
      this->rosNode->setParam("/Design/kConstant", this->kConstant);
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
  GZ_REGISTER_MODEL_PLUGIN(SpringJoint)
}
