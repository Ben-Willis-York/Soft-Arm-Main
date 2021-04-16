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
    class AppliedForceFetcher: public ModelPlugin
    {
        physics::ModelPtr modelPtr; // Pointer to the model
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
        sdf::ElementPtr sdfPtr; //Pointer to SDF element

        physics::LinkPtr link; //Pointer to Link of interest

        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Publisher rosPub;
        //ros::Subscriber rosSub;
        //ros::CallbackQueue rosQueue;
        //std::thread rosQueueThread;

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {

            modelPtr = _parent;
            sdfPtr = _sdf;

            std::string linkName = sdfPtr->Get<std::string>("LinkName");
            link = modelPtr->GetLink(linkName);

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&AppliedForceFetcher::OnUpdate, this));

            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }

            std::string topicName = linkName+"_forces";
            this->rosNode.reset(new ros::NodeHandle(linkName+"_force_fetcher"));
            rosPub = this->rosNode->advertise<std_msgs::Float32MultiArray>(link->GetName()+"_forces", 1);

        }

        public: void OnUpdate()
        {
            ignition::math::Vector3d worldForce = link->RelativeForce();
            std::vector<float> vec = {worldForce.X(), worldForce.Y(), worldForce.Z()};

            std_msgs::Float32MultiArray msg = std_msgs::Float32MultiArray();
            msg.data = vec;
            rosPub.publish(msg);
        }

    };
    GZ_REGISTER_MODEL_PLUGIN(AppliedForceFetcher)
}