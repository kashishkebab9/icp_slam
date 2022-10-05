#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>
#include <utility>
#include <visualization_msgs/Marker.h>

class Solver{
  private:
    //Both target and source vecs will contain inf values if that is what is recorded
    std::vector<float> source_vec;
    std::vector<float> target_vec;
    std::vector<std::pair<float, float>> com_displacements;
    std::vector<std::pair<float, float>> com_vec;

  public:
    float angle_min;
    float angle_inc;
    std::pair<float, float> source_com;
    std::pair<float, float> target_com;
    std::pair<float, float> com_displacement;

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
      std::cout <<"Enter SolverCallback" << std::endl;
      
      this->angle_min = msg->angle_min;
      this->angle_inc = msg->angle_increment;

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

      //this->PrintTargetVec();
      //this->PrintSourceVec();

      auto target_cart = this->Polar2Cartesian(this->target_vec);
      auto source_cart = this->Polar2Cartesian(this->source_vec);

//      for (int j = 0; j < target_cart.size(); j++) {
//        ROS_INFO("Target Cart||Coordinate %i [x, y]: %f, %f", j, target_cart[j].first, target_cart[j].second);
//      }

      this->source_com = CalcCom(source_cart);
      this->target_com = CalcCom(target_cart);
      this->com_vec.push_back(target_com);

      ROS_INFO("Source COM[x,y]: %f, %f", this->source_com.first, this->source_com.second);
      ROS_INFO("Target COM[x,y]: %f, %f", this->target_com.first, this->target_com.second);
      auto displacement_com_x =this->target_com.first - this->source_com.first;
      auto displacement_com_y =this->target_com.second - this->source_com.second;
      this->com_displacement = std::make_pair(displacement_com_x, displacement_com_y);
      ROS_INFO("Translation[x,y]: %f, %f", displacement_com_x, displacement_com_y);
      this->com_displacements.push_back(com_displacement);


      
    }

    std::vector<std::pair<float, float>> Polar2Cartesian(std::vector<float> input) {
      std::vector<std::pair<float, float>> output_vec; 

      for(int i= 0; i < input.size(); i++) {
        float x, y;
        float ang = this->angle_min + this->angle_inc * i; 
        x = input[i] * cos(ang);
        y = input[i] * sin(ang);
        std::pair<float, float> coordinate(x,y);
        output_vec.push_back(std::make_pair(x, y));
      }
      return output_vec;

    }
    
    std::pair<float, float> CalcCom(std::vector<std::pair<float, float>> input) {
      float x_out{0};
      float y_out{0};
      int counter{0};

      for (auto i: input) {
        if (!isinf(i.first) && !isinf(i.second)){
          x_out += i.first;
          y_out += i.second;
          counter++;

        }
        

      }
      x_out = x_out/counter;
      y_out = y_out/counter;
      return std::make_pair(x_out, y_out);
      

    }


    std::vector<std::pair<float, float>> GetComVector() {
      return this->com_vec;
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
  ros::init(argc, argv, "icp");
  ros::NodeHandle n;
  Solver solver;
  ros::Subscriber sub = n.subscribe("scan", 1000, &Solver::SolverCallback, &solver);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Rate loop_rate(10);

  while(ros::ok()) {
    visualization_msgs::Marker points;
    if(solver.GetComVector().size() > 0) {
      std::cout << "Step 2" << std::endl;
      points.header.frame_id = "laser";
      points.header.stamp = ros::Time::now();
      points.ns = "icp_points";
      points.action = visualization_msgs::Marker::ADD;
      points.pose.orientation.w = 1.0;
      points.id = 0;
      points.type = visualization_msgs::Marker::POINTS;
      points.scale.x = .2;
      points.scale.y = .2;
      points.color.g = 1.0f;
      points.color.a = 1.0;

      for(auto i : solver.GetComVector()) {
        std::cout << "Adding Point [x,y]: " << i.first << ", " << i.second << std::endl;
        geometry_msgs::Point p;
        p.x = i.first;
        p.y = i.second;
        p.z = 0;
        points.points.push_back(p);
      }

    }
    marker_pub.publish(points);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();

  return 0;
}


