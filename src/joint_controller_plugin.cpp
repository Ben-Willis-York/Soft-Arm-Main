#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
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
      bool commandRecieved = false;

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

      std::vector<double> targets;
      std::vector<double> setpoints;
      std::vector<double> speeds;
      double maxSpeed = 0.0003;
      
      common::Time prevTime = 0;



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
      std::string node_name = std::string("");
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

      //common::Time time = physics::World::SimTime();

      //Get joint names
      if(this->paramsFound == false) { this-> GetJoints(); }
      
      if(this->paramsFound)
      {
        for(int i = 0; i < targets.size(); i++)
        {
          double thresh = this->speeds[i];
          double difference = this->setpoints[i] - this->targets[i];

          if((difference <= thresh) && (difference >= -thresh))
          {
            this->setpoints[i] = this->targets[i];
          }
          else if(difference > thresh)
          {
            this->setpoints[i] -= this->speeds[i];
          }
          else if(difference < -thresh)
          {
            this->setpoints[i] += this->speeds[i];
          }
          else { std::cerr << "elsed" << std::endl;}

          this->model->GetJointController()->SetPositionTarget(this->joints[i]->GetScopedName(), this->setpoints[i]);
          this->model->GetJointController()->Update();
          //std::cerr << this->setpoints[i] << ",";
        }
        //std::cerr << std::endl;

      }
    }

    public: void GetJoints()
    {
      std::string paramName;
      //paramName = "/arm_joints/joints";
      //std::cerr << paramName << std::endl;
      std::string paramBase = this->sdf->Get<std::string>("jointNamesParam");
      std::cerr << paramBase << std::endl;

      if(!rosNode->getParam(paramBase+"joints", this->jointNames))
      { std::cerr << "Not Found" << std::endl; }
      else
      { 
        this->paramsFound = true;

        //std::cerr << "FOUND JOINTS" << std::endl;
        float s;
        std::string name = jointNames[0];
        std::string n = paramBase + name + "/pid/p";
        //std::cerr << n << std::endl;
        //std::cerr << "true or false? " << this->rosNode->getParam(n, s) << std::endl;
        //std::cerr << "Rev1 p: " << s << std::endl;



        for(int j = 0; j < this->jointNames.size(); j++)
        {
          
          float p;
          float i;
          float d;
          //std::string jointName = jointNames[0];
          //std::string param = paramBase + jointName + "/pid/p";
          //std::cerr << param << std::endl;

   
          std::string name = jointNames[j];
          std::string n = paramBase + name + "/pid/p";
          if(!rosNode->getParam(n, p))
          { p=0; }

          n = paramBase + name + "/pid/i";
          if(!rosNode->getParam(n, i))
          { i=0; }

          
          n = paramBase + name + "/pid/d";
          if(!rosNode->getParam(n, d))
          { d=0; }

          /*
          std::cerr << jointNames[j] << ": ";
          if(!rosNode->getParam(n, p))
          { std::cerr << ", p=" << p; }
          if(!rosNode->getParam("arm_controller/Rev1/pid/i", i))
          { std::cerr << ", i= " << i; }
          if(!rosNode->getParam(param + "/d", d))
          { std::cerr << ", d=" << d; }
          std::cerr << std::endl;
          */
         
          std::cerr << jointNames[j] << " P: " << p << " I: " << i << " D: " << d << std::endl;

          this->joints.push_back(this->model->GetJoint(this->jointNames[j]));
          float startPos = this->joints[j]->Position();
          targets.push_back(startPos);
          setpoints.push_back(startPos);
          speeds.push_back(0.00001);
          pids.push_back(common::PID(p, i, d, 0, 0));
          this->model->GetJointController()->SetPositionPID(this->joints[j]->GetScopedName(), this->pids[j]);
          this->model->GetJointController()->SetPositionTarget(this->joints[j]->GetScopedName(), startPos);
        }
      }
    }
  
  
    public: void OnRosMsg(const std_msgs::Float32MultiArray::ConstPtr& _msg)
    {


      std::cerr << "Setting Angles: ";
      std::vector<float> arr = _msg->data;

      std::vector<double> differences = std::vector<double>(jointNames.size());
      double maxDiff = 0;

      for(int i =0; i < arr.size(); i++)
      {
        this->targets[i] = arr[i];
        differences[i] = this->targets[i] - this->setpoints[i];
        if( std::abs(differences[i]) > maxDiff) {maxDiff = std::abs(differences[i]);}

        std::cerr << arr[i] << ", ";
      }
      std::cerr << std::endl;

      std::cerr << "Speeds: ";
      for(int i = 0; i < differences.size(); i++)
      {
        if(maxDiff > 0)
        {
          this->speeds[i] = std::abs(this->maxSpeed * differences[i]/maxDiff);
          std::cerr << this->speeds[i] << ", ";
        }
      }
      std::cerr << std::endl;

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
