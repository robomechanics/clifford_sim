#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>



namespace gazebo
{
  class cliffordPlugin : public ModelPlugin
  {
  public:
    cliffordPlugin():ModelPlugin()
    {
      if (!ros::isInitialized())
      {
       ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
       return;
      }
      /*this->rosNode.reset(new ros::NodeHandle("cliffordControl"));
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/cliffordDrive",
            1,
            boost::bind(&cliffordPlugin::rosDrive, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);*/

      this->rosNode.reset(new ros::NodeHandle("cliffordControl"));
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/cliffordDrive",
            1,
            boost::bind(&cliffordPlugin::rosFullCommand, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      this->rosQueueThread = std::thread(std::bind(&cliffordPlugin::QueueThread, this));
    }
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->model = _model;
        this->drive[0] = this->model->GetJoint("frwheel2tire");
        this->drive[1] = this->model->GetJoint("flwheel2tire");
        this->drive[2] = this->model->GetJoint("brwheel2tire");
        this->drive[3] = this->model->GetJoint("blwheel2tire");
        this->DriveVelocityPid = common::PID(1, 0, 0);
        this->steering[0] = this->model->GetJoint("axle2frwheel");
        this->steering[1] = this->model->GetJoint("axle2brwheel");
        this->steeringPositionPid = common::PID(10,0,0);

        //this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&cliffordPlugin::OnUpdate, this, _1));

    }

    void OnUpdate(const common::UpdateInfo &)
    {
      common::Time current_time = this->model->GetWorld()->SimTime();
      double timeNow = current_time.Double();
      double numOut = sin(timeNow)/2.0;
      //if (timeNow>3)
      for(int i=0;i<2;i++)
      {
        this->model->GetJointController()->SetPositionTarget(this->steering[i]->GetScopedName(), numOut);
      }
      //ROS_INFO("steering position: [%f] Joint Force: [%f]", this->steering->Position(), this->steering->GetForce(0));
    }

    public: void rosFullCommand(const geometry_msgs::Twist::ConstPtr& _msg)//const std_msgs::Float32MultiArray::ConstPtr& _msg)
    {
      double K = 1;
      double R = 1;
      float driveCommand = 30.0*_msg->linear.x;//_msg->data[0];
      float steerCommand = -0.5*_msg->angular.z;//_msg->data[1];
      if (true)
      {
        //Control Velocity
        for(int i=0;i<4;i++)
        {
          this->model->GetJointController()->SetVelocityPID(this->drive[i]->GetScopedName(), this->DriveVelocityPid);
          this->model->GetJointController()->SetVelocityTarget(this->drive[i]->GetScopedName(), driveCommand);
        }
      }
      else
      {
        // Control Voltage
        for(int i=0;i<4;i++)
        {
          //this->model->GetJointController()->SetVelocityPID(this->drive[i]->GetScopedName(), this->DriveVelocityPid);
          //this->model->GetJointController()->SetVelocityTarget(this->drive[i]->GetScopedName(), _msg->data);
          this->drive[i]->SetForce(0,K/R*(driveCommand-K*this->drive[i]->GetVelocity(0)));
        }
      }
      for(int i=0;i<2;i++)
      {
        this->model->GetJointController()->SetPositionPID(this->steering[i]->GetScopedName(), this->steeringPositionPid);
        this->model->GetJointController()->SetPositionTarget(this->steering[i]->GetScopedName(), steerCommand);
      }
    }

    /// brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  private:
    physics::ModelPtr model;
    physics::JointPtr drive[4];
    physics::JointPtr steering[2];
    common::PID DriveVelocityPid;
    common::PID steeringPositionPid;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber rosSub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(cliffordPlugin)
}
