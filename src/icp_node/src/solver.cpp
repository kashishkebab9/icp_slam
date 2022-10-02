#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>

class Solver{
  private:
    std::vector<float> source_vec;
    std::vector<float> target_vec;

  public:

    void SetSourceVec(std::vector<float> source) {
      this->source_vec = source;
    }

    void SetTargetVec(std::vector<float> target) {
      this->target_vec = target;
    }

    std::vector<float> GetSourceVec() {
      return this->source_vec;
    }

    std::vector<float> GetTargetVec() {
      return this->target_vec;
    }

    void SolverCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
      
      auto angle_min = msg->angle_min;
      auto angle_inc = msg->angle_increment;
      //ROS_INFO("Angle Increment: %f", angle_inc);
      //ROS_INFO("Angle Min: %f", angle_min);

      auto angle = angle_min;
      std::vector<float> newest;

      for (int j = 0; j < msg->ranges.size(); j++) {
        angle = angle_min + (angle_inc * j);
        //ROS_INFO("RangeVal(Angle, Range): %f, %f ", angle, msg->ranges[j]);
        newest.push_back(msg->ranges[j]);
        
      }

      if (!this->GetTargetVec().size() == 0) {
        this->SetSourceVec(this->GetTargetVec());
      }

      this->SetTargetVec(newest);
      StartSolver();
      
    }

    void StartSolver() {
      if (!GetTargetVec().size() == 0 && !GetSourceVec().size() == 0) {
        //std::cout << "Size of Target_Vec: " << GetTargetVec().size() << std::endl;
        //std::cout << "Size of Source_Vec: " << GetSourceVec().size() << std::endl;

      }


    }

};


int main(int argc, char **argv)
{
  //Need to make two empty vectors 
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  Solver solver;
  ros::Subscriber sub = n.subscribe("scan", 1000, &Solver::SolverCallback, &solver);
  ros::spin();

  return 0;
}


