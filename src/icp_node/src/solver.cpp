#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>

class Solver{
  private:
    //Both target and source vecs will contain inf values if that is what is recorded
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

      auto angle = angle_min;
      std::vector<float> newest;

      for (int j = 0; j < msg->ranges.size(); j++) {
        angle = angle_min + (angle_inc * j);
        newest.push_back(msg->ranges[j]);
        
      }

      if (!this->GetTargetVec().size() == 0) {
        this->SetSourceVec(this->GetTargetVec());
      }

      this->SetTargetVec(newest);
      this->PrintTargetVec();
      this->PrintSourceVec();
      StartSolver();
      
    }

    void StartSolver() {
      if (!GetTargetVec().size() == 0 && !GetSourceVec().size() == 0) {
        //std::cout << "Size of Target_Vec: " << GetTargetVec().size() << std::endl;
        //std::cout << "Size of Source_Vec: " << GetSourceVec().size() << std::endl;
        

      }


    }

    void PrintTargetVec() {
      if(this->target_vec.size() > 0) {
        for (int i=0; i < this->target_vec.size(); i++) {
          if(!isinf(target_vec[i])) {
            ROS_INFO("Target: %f", this->target_vec[i]);

          }
        }
      }
    }
    void PrintSourceVec() {
      if(this->source_vec.size() > 0) {
        for (int i=0; i < this->source_vec.size(); i++) {
          if(!isinf(source_vec[i])) {
            ROS_INFO("Source: %f", this->source_vec[i]);

          }
        }
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


