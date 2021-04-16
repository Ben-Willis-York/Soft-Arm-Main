#include <functional>
#include <gazebo/gazebo.hh>
//#include <plugin.hh>
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
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"


namespace gazebo
{
  class JointForceVisualiser: public ModelPlugin
  {
    public:
    private:
      bool jointsFound = false;

      physics::ModelPtr model; // Pointer to the model
      event::ConnectionPtr updateConnection; // Pointer to the update event connection
      sdf::ElementPtr sdf; //Pointer to SDF element

      std::vector<std::string> jointNames; //Array of joint names to visualise
      std::vector<physics::JointPtr> joints; //Array of joint pointers
       
      std::vector<gazebo::msgs::Material*> materialMsgs; //Material for each joint
      std::vector<gazebo::msgs::Color*> colorMsgs; //Colour for each joint
      std::vector<gazebo::msgs::Color*>  diffuseMsgs; //Diffuse for each joint
      std::vector<gazebo::msgs::Visual> visualMsgs; //Visual message for each joint


      transport::PublisherPtr visualPub;


      std::unique_ptr<ros::NodeHandle> rosNode;
      ros::Subscriber rosSub;
      ros::CallbackQueue rosQueue;
      std::thread rosQueueThread;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

      std::cerr << "LOADING FORCE VISUALISER PLUGIN" << std::endl;
      // Store the pointer to the model
      this->model = _parent;
      this->sdf = _sdf;

      transport::NodePtr node(new transport::Node());
      node->Init(_parent->GetName());

      visualPub = node->Advertise<msgs::Visual>("/gazebo/default/visual");

      physics::JointPtr joint = this->model->GetJoint("Rev3");
      physics::LinkPtr linkPtr = joint->GetChild();

      /*
      ignition::math::Vector3d scale(3,1,1);

      
      this->visualMsg = linkPtr->GetVisualMessage("Rev3_visual");
      visualMsg.set_parent_name(_parent->GetScopedName());
      visualMsg.set_name(linkPtr->GetScopedName());

      gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;   
      visualMsg.set_allocated_material(materialMsg);

      gazebo::common::Color newColor(1, 0.5, 0, 0.5);
      gazebo::msgs::Color *colorMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(newColor));
      gazebo::msgs::Color *diffuseMsg = new gazebo::msgs::Color(*colorMsg);

        if (materialMsg->has_ambient())
        {
            materialMsg->clear_ambient();
        }
        materialMsg->set_allocated_ambient(colorMsg);
        if (materialMsg->has_diffuse())
        {
            materialMsg->clear_diffuse();
        }

      materialMsg->set_allocated_diffuse(diffuseMsg);

      visualPub->Publish(visualMsg);
       */
      std::cerr << joint->GetName() << std::endl;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&JointForceVisualiser::OnUpdate, this));

      //Setup ROS node
      std::string node_name = std::string("FingerForces");
      if (!ros::isInitialized()) 
      {
        int argc = 0;
        char **argv = nullptr;

        ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle(node_name));
      
      ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
      "/" + this->model->GetName() + "/forces",
      1,
      boost::bind(&JointForceVisualiser::OnRosMsg, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      

      this->rosSub = this->rosNode->subscribe(so);

      this->rosQueueThread = std::thread(std::bind(&JointForceVisualiser::QueueThread, this));
      
      std::cerr << "FINISHED LOADED VISUALISER PLUGIN" << std::endl;

    }

    
    // Called by the world update start event
    public: void OnUpdate()
    {
        //this->visualPub->Publish(this->visualMsg);
        if(!jointsFound)
        {
            GetJoints();
            for(int i = 0; i < 10; i++)
            {
                SetLinkColour(i, 0, 0 ,1);
                
            }
        }
    }

    public: void SetLinkColour(int index, float r, float g, float b)
    {
        //gazebo::msgs::Visual visualMsg = this->joints[jointIndex]->GetChild();
        physics::LinkPtr linkPtr = this->joints[index]->GetChild();
        gazebo::msgs::Visual visualMsg = linkPtr->GetVisualMessage("Rev3_visual");
        visualMsg.set_parent_name(this->model->GetScopedName());
        visualMsg.set_name(linkPtr->GetScopedName());

        gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;   
        visualMsg.set_allocated_material(materialMsg);

        gazebo::common::Color newColor(r, g, b, 1);
        gazebo::msgs::Color *colorMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(newColor));
        gazebo::msgs::Color *diffuseMsg = new gazebo::msgs::Color(*colorMsg);

            if (materialMsg->has_ambient())
            {
                materialMsg->clear_ambient();
            }
            materialMsg->set_allocated_ambient(colorMsg);
            if (materialMsg->has_diffuse())
            {
                materialMsg->clear_diffuse();
            }

        materialMsg->set_allocated_diffuse(diffuseMsg);
        visualPub->Publish(visualMsg);
       
        

    }


    public: void GetJoints()
    {
        std::string paramName = "/hand_controller/joints";
        if(!rosNode->getParam(paramName, this->jointNames))
        { std::cerr << "Force Visualiser: Joints not found!" << std::endl; }
        else
        {
            jointsFound = true;
            for(int i = 0; i < jointNames.size(); i++)
            {
                physics::JointPtr joint = this->model->GetJoint(this->jointNames[i]);
                this->joints.push_back(joint);
                /*
                physics::LinkPtr linkPtr = joint->GetChild();

                this->visualMsgs.push_back(linkPtr->GetVisualMessage(linkPtr->GetName()+"_visual"));
                
                visualMsgs[i].set_parent_name(this->model->GetScopedName());
                visualMsgs[i].set_name(linkPtr->GetScopedName());

                materialMsgs.push_back(new gazebo::msgs::Material);
                //gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;   
                visualMsgs[i].set_allocated_material(materialMsgs[i]);

                
                gazebo::common::Color newColor(1, 0.5, 0, 1);
                colorMsgs.push_back(new gazebo::msgs::Color(gazebo::msgs::Convert(newColor)));
                diffuseMsgs.push_back(new gazebo::msgs::Color(*colorMsgs[i]));
                //gazebo::msgs::Color *diffuseMsg = new gazebo::msgs::Color(*colorMsg);

                if (materialMsgs[i]->has_ambient())
                {
                    materialMsgs[i]->clear_ambient();
                }
                materialMsgs[i]->set_allocated_ambient(colorMsgs[i]);
                if (materialMsgs[i]->has_diffuse())
                {
                    materialMsgs[i]->clear_diffuse();
                }

                materialMsgs[i]->set_allocated_diffuse(diffuseMsgs[i]);

                //visualPub->Publish(visualMsg);
                */


            }
            std::cerr << "JOINTS FOUND" << std::endl;
        }



    }
  

    
  
    public: void OnRosMsg(const std_msgs::Float32MultiArray::ConstPtr& _msg)
    {
        //std::cerr << "Recieved Forces Input" << std::endl;
        std::vector<float> arr = _msg->data;
        if(arr.size() > this->joints.size()*3)
        {   
            //std::cerr << "Colour Array to big - Cropping to size" << std::endl;
            for(int i = 0; i < joints.size(); i++)
            {
            float r = arr[i*3];
            float g = arr[i*3+1];
            float b = arr[i*3+2];
            SetLinkColour(i, r, g, b);
            }
        }
        else
        {
            //std::cerr << "Colour Array to small - Appling to subset" << std::endl;
            for(int i = 0; i < arr.size()/3; i++)
            {
                float r = arr[i*3];
                float g = arr[i*3+1];
                float b = arr[i*3+2];
                SetLinkColour(i, r, g, b);
            }
        }
        

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
  GZ_REGISTER_MODEL_PLUGIN(JointForceVisualiser)
}
