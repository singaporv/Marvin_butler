#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <queue>



class Move
{
  public:
    Move()
    {
      // Min and Max angular velocities
      n_.param("/movement_node/min_angular", minAngular_, 0.1);
      n_.param("movement_node/max_angular", maxAngular_, 0.5);

      // Min and Max linear velocities
      n_.param("/movement_node/min_linear", minLinear_, 0.1);
      n_.param("/movement_node/max_linear", maxLinear_, 0.2);
      
      // Gains for linear PID controller
      n_.param("/movement_node/kp_linear", kpLinear_, 1.0);
      n_.param("/movement_node/kd_linear", kdLinear_, 0.8);

      // Gains for angular PID controller
      n_.param("/movement_node/kp_angular", kpAngular_, 1.0);
      n_.param("/movement_node/kd_angular", kdAngular_, 0.2);

      n_.param("/movement_node/linear_threshold", linearThreshold_, 0.5);
      n_.param("/movement_node/angular_threshold", angularThreshold_, 0.1);
      n_.param("/movement_node/angular_threshold_movement", angularThresholdMovement_, 0.1);

      // Flags
      n_.param("/movement_node/debug", debug_, true); // Enforce debug behavior (no external inputs needed)
      delivering_ = false; // Marvin is delivering a drink, don't move
      search_ = true; // Perform search pattern for goal tag
      stop_ = false; // Don't move


      // Initialize Error terms
      errLinear_ = 0.0;
      errDiffLinear_ = 0.0;
      prevErrLinear_ = 0;

      errAngular_ = 0.0;
      errDiffAngular_ = 0.0;
      prevErrAngular_ = 0;

      // Initialize loop terms
      rotationComplete_ = false;
      linearComplete_ = false;
      linearMove_ = false;

      numberOfGoals_ = 7;
      currGoalID_ = 1;


      // Set publishers and subscribers 
      pubCommand_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      pubAchieved_ = n_.advertise<std_msgs::Int32>("can_take_photo", 1);
      subTag_ = n_.subscribe("ar_pose_marker", 1, &Move::pidCallback, this);
      subDeliverNext_ = n_.subscribe("drink_delivered", 1, &Move::nextGoalCallback, this);
      emergencyStop_ = n_.advertiseService("/emergency_stop",&Move::emergencyStop,this);
      continueMovement_ = n_.advertiseService("/continue_service",&Move::continueMovement,this);

      // Stack all goal ids (except goal 0) int goalQueue_
      for (int i=1; i < numberOfGoals_; i++)
      {
        if (i != currGoalID_)
        {
          goalQueue_.push(i);
        }
      }
    }

    int signNum(double num)
    // Return the sign of a number
    {
      return (num < 0.0) ? -1 : 1;
    }

    double calculateDistance(geometry_msgs::Pose currPose, geometry_msgs::Pose goalPose)
    // This function calculates the linear distance between two points
    {
      double xErr = currPose.position.x - goalPose.position.x;
      double yErr = currPose.position.y - goalPose.position.y;
      return sqrt(pow(xErr,2) + pow(yErr,2));
    }

    double calculateYawFromQuaterion(geometry_msgs::Pose pose)
    // Calculate and return the yaw of a pose quaternion
    {
      double roll, pitch, yaw;
      double quatx = pose.orientation.x;
      double quaty = pose.orientation.y;
      double quatz = pose.orientation.z;
      double quatw = pose.orientation.w;
      tf::Quaternion quaternion(quatx, quaty, quatz, quatw);
      tf::Matrix3x3 rotMatrix(quaternion);
      rotMatrix.getRPY(roll, pitch, yaw);
      return yaw;
    }

    void setMarkerGoal(const ar_track_alvar_msgs::AlvarMarkers detectedMarkers)
    // This function sets the value of the variable 
    {
      if (!delivering_) // If marvin is not currently delivering a drink, you may move
      {
        if(!detectedMarkers.markers.empty()) // If you can see an AR tag, see if it is the goal AR tag
        {
          std::cout << "Number of Markers Detected " <<detectedMarkers.markers.size()<< std::endl;
          for (size_t i=0; i<detectedMarkers.markers.size(); i++)
          {
            std::cout << "Marker seen: " <<detectedMarkers.markers[i].id << std::endl;
            if (detectedMarkers.markers[i].id == currGoalID_) // If you see the goal AR tag, move towards it
            {
              stop_ = false;
              search_ = false;
              goalMarkerPose_ = detectedMarkers.markers[i];
              ROS_INFO("GO");
              break;
            }
          }
        }
      }      
      else // If marvin is delivering a drink, don't move
      {
        stop();
      }
    }

    void resetLoopTerms()
    {
      prevErrLinear_ = 0;
      prevErrAngular_ = 0;
      rotationComplete_ = false;
      linearComplete_ = false;
      linearMove_ = false;
    }

    void stop()
    {
      stop_ = true;
      resetLoopTerms();
    }

    void search()
    {
      stop_ = false;
      search_ = true;
      resetLoopTerms();
    }


    void pidCallback(const ar_track_alvar_msgs::AlvarMarkers detectedMarkers)
    // This function has the robot move to a goal position if a goal is active
    // It is broken down such that the robot will first rotate and then move forward to simplify things
    {
      setMarkerGoal(detectedMarkers);
      geometry_msgs::Twist command;
      if (!stop_)
      {
        if (search_)
        {
          command.angular.z = maxAngular_;
        }
        else
        {
          command = PID();
          // If there are more than 10 loops without a command, reset and move on
          std::cout << "linear complete: " << linearComplete_ << std::endl;
          if(linearComplete_)
          {
            // std_msgs::Int32 drink;
            // drink.data = currGoalID_;
            // pubAchieved_.publish(drink);
            if (debug_)
            {
              ROS_INFO("Next Goal");
              setNextGoal();
            }
            else
            {
              delivering_ = true;
            }
            
          }
        }
      }
      pubCommand_.publish(command);
    }

    void nextGoalCallback(const std_msgs::Bool msg)
    {
      delivering_ = false;
      setNextGoal();
    }

    void setNextGoal()
    {
      goalQueue_.push(currGoalID_);
      std::cout << "Old Goal ID " << currGoalID_ << std::endl;
      currGoalID_ = goalQueue_.front();
      goalQueue_.pop();
      std::cout << "New Goal ID " << currGoalID_ << std::endl;
      search();
    }

    geometry_msgs::Twist PID()
    // This function performs a PID for the angular rotation.
    {
     
      geometry_msgs::Twist command;
      errAngular_ = -atan2(goalMarkerPose_.pose.pose.position.x,goalMarkerPose_.pose.pose.position.z);
      linearMove_ = true;
      if (fabs(errAngular_) < angularThreshold_ && !rotationComplete_)
      {
        ROS_INFO("Rotation Complete");
        rotationComplete_ = true;
        std_msgs::Int32 drink;
        drink.data = currGoalID_;
        pubAchieved_.publish(drink);
      }
      if (fabs(errAngular_ < angularThresholdMovement_))
      {
        linearMove_ = true;
      }
      else
      {
        linearMove_ = false;
      }
      errDiffAngular_ = errAngular_ - prevErrAngular_;
      command.angular.z = kpAngular_*errAngular_ + kdAngular_*errDiffAngular_;
      if (command.angular.z < minAngular_)
      {
        command.angular.z = signNum(command.angular.z)*minAngular_;
      }
      else if (command.angular.z > maxAngular_)
      {
        command.angular.z = signNum(command.angular.z)*maxAngular_;
      }
      prevErrAngular_ = errAngular_;
      ROS_INFO("Angular command: %f", command.angular.z);

      // Ensure the robot has reached certain angular accuracy before moving forward      
      
      

      if (rotationComplete_ && linearMove_)
      {
        errLinear_ = goalMarkerPose_.pose.pose.position.z;

        // Conditions of goal completed
        if (fabs(errLinear_) < linearThreshold_)
        {
          command.linear.x = 0;
          linearComplete_ = true;
        }
        else
        {
          // Linear PID
          errDiffLinear_ = errLinear_ - prevErrLinear_;
          command.linear.x = kpLinear_*errLinear_ + kdLinear_*errDiffLinear_;
          if (command.linear.x < minLinear_)
          {
            command.linear.x = minLinear_;
          }
          else if (command.linear.x > maxLinear_)
          {
            command.linear.x = maxLinear_;
          }

        }
        prevErrLinear_ = errLinear_;
      }
      return command;
    }

    bool emergencyStop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    // This service stops the robot
    {
      stop();
      return true;
    }

    bool continueMovement(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    // This service will undo emergency stop
    {
      stop_ = false;
      return true;
    }

  private:
    double kpLinear_;
    double kdLinear_;

    double kpAngular_;
    double kdAngular_;

    double errLinear_;
    double prevErrAngular_;
    double errDiffLinear_;

    double errAngular_;
    double prevErrLinear_;
    double errDiffAngular_;

    double angularThreshold_;
    double linearThreshold_;
    double angularThresholdMovement_;

    ar_track_alvar_msgs::AlvarMarker goalMarkerPose_;

    bool stop_;
    bool search_;
    bool delivering_;
    bool debug_;
    bool linearMove_;

    bool rotationComplete_;
    bool linearComplete_;

    double minAngular_;
    double maxAngular_;
    double minLinear_;
    double maxLinear_;

    int numberOfGoals_;
    std::queue<int> goalQueue_;
    int currGoalID_;

    ros::Publisher pubCommand_;
    ros::Publisher pubAchieved_;
    ros::Subscriber subTag_;
    ros::Subscriber subDeliverNext_;
    ros::ServiceServer emergencyStop_;
    ros::ServiceServer continueMovement_;
    ros::NodeHandle n_;
};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "movement_node");
  
  Move move;
  
  

  ros::Rate loop_rate(200);
  ROS_INFO("In main\n");
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
