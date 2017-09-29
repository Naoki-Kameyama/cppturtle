#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>

ros::Subscriber map_sub;
int i;

void area(const nav_msgs::OccupancyGrid::ConstPtr& msg){

	nav_msgs::MapMetaData info = msg->info;//地図の設定を取得
	std::vector<int8_t> data = msg->data;//地図の値を取得
  int x = info.width;//地図の横サイズ
  int y = info.height;//地図の縦サイズ
  int m=0;
	std::cout << "area" << std::endl;
  std::cout << x*y << std::endl;

	for(i=0; i<x*y; i++){
      //std::cout <<  +data[i] << std::endl;
      if(data[i]!=-1){
        m+=1;
        std::cout << m*0.0025 << std::endl;
      }
  	}

}


int main(int argc, char** argv){
  ros::init(argc, argv, "area");
  ros::NodeHandle nh ;
  map_sub = nh.subscribe("/map",1,area);
	ros::spin();
	return 0;
}
