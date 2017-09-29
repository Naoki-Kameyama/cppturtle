#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>


//グローバル変数////////////////////////////////////////////////////////////////////////////////////////////
ros::Publisher vel_pub;
ros::Subscriber map_sub;
ros::Publisher swi_pub;
//std::vector<float> log_x;
//std::vector<float> log_y;
//int log_num = 0;
bool stop = false;
float search_radius=100000000;//スタート地点からの探査範囲制限(半径m)
move_base_msgs::MoveBaseGoal goal;
ros::Subscriber switch_sub;
bool switcher=false;

//目標座標にナビゲーションする関数/////////////////////////////////////////////////////////////////////////////
bool navigation(float far_x,float far_y){

	//std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標を受信＊＊＊＊＊＊＊＊＊＊" << std::endl;

	//define a client for to send goal requests to the move_base server through a SimpleActionClient

	//actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("robot1/move_base", true);
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	/*actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", false);
	ros::spinOnce();
	*/
	//wait for the action server to come up//5.0秒まで待つ
	while(!ac.waitForServer(ros::Duration(5.0))){
		std::cout << "＊＊＊＊＊＊＊＊＊＊待機中＊＊＊＊＊＊＊＊＊＊" << std::endl;;
	}

	//move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	//goal.target_pose.header.frame_id = "robot1_tf/map";
	/*goal.target_pose.header.frame_id = "map";

	goal.target_pose.header.stamp = ros::Time::now();*/

	/* moving towards the goal*/

	goal.target_pose.pose.position.x =  far_x;
	goal.target_pose.pose.position.y =  far_y;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標セット(" << far_x << "," << far_y << ")＊＊＊＊＊＊＊＊＊＊" << std::endl;

	std::cout << "＊＊＊＊＊＊＊＊＊＊経路を作成中＊＊＊＊＊＊＊＊＊＊" << std::endl;
	ac.sendGoal(goal);
	std::cout << "＊＊＊＊＊＊＊＊＊＊sendend＊＊＊＊＊＊＊＊＊＊" << std::endl;
	ac.waitForResult();
	std::cout << "＊＊＊＊＊＊＊＊＊＊waitend＊＊＊＊＊＊＊＊＊＊" << std::endl;
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標に到着＊＊＊＊＊＊＊＊＊＊" << std::endl;
		return  true;
	}
	else{
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標への移動不可＊＊＊＊＊＊＊＊＊＊" << std::endl;
		return  false;
	}
}


//未探査領域がなくなったらループクロージングをするために同じ領域を繰り返し探査する///////////////////////////////////////
/*void repeat_explore(){

	std::cout << "start:繰り返し探査" << std::endl;
	bool nav_success;
	int i=0;

	for (i=0;i<log_num;i++){

		std::cout << "!!!!＊＊＊＊＊＊＊＊＊＊繰り返し探査中＊＊＊＊＊＊＊＊＊＊!!!!" << std::endl;
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標を配信＊＊＊＊＊＊＊＊＊＊" << std::endl;

		nav_success = navigation(log_x[i],log_y[i]);

		//std::cout << "＊＊＊＊＊＊＊＊＊＊移動結果を受信＊＊＊＊＊＊＊＊＊＊" << std::endl;

		if(nav_success == true){
			std::cout << "＊＊＊＊＊＊＊＊＊＊移動完了＊＊＊＊＊＊＊＊＊＊" << std::endl;
		}
		else{
			std::cout << "＊＊＊＊＊＊＊＊＊＊何かエラーがおきた＊＊＊＊＊＊＊＊＊＊" << std::endl;
		}
	}
	std::cout << "end  :繰り返し探査" << std::endl;
}*/


//現在位置に対して制限範囲の中で最も遠い領域を探す関数////////////////////////////////////////////////////////////

void far_frontier(std::vector<float> fro_x, std::vector<float> fro_y, int fro_num){

//ロボットの現在座標を取得//////////////////////////////////////////////////////////////////////////////////////
	float ro_x_map;//ロボットの現在のx座標
	float ro_y_map;//ロボットの現在のy座標
	std::vector<float> fro_x_tmp = fro_x;
	std::vector<float> fro_y_tmp = fro_y;
	int fro_num_tmp = fro_num;
	float far_x;
	float far_y;
	float x_tmp;
	float y_tmp;
	float dis_tmp;
	int i;
	int num = 0;
	float dis_rad;
	bool nav_success;
	float dis_for_view;
	int retry_counter = 0;

	std::cout << "start:far_frontier" << std::endl;

	std::cout << "＊＊＊＊＊＊＊＊＊＊現在の探査半径 (" << search_radius << " m )＊＊＊＊＊＊＊＊＊＊" << std::endl;

research://再検索のときに帰ってくる場所
	bool limit_fro =true;
	float distance=0;

	std::cout << "start:ロボットの現在座標を取得" << std::endl;

	tf::TransformListener listener;
    	tf::StampedTransform transform;

     	ros::Time now = ros::Time::now();
      	//listener.waitForTransform("/robot1_tf/map", "/robot1_tf/base_footprint", now, ros::Duration(1.0));
      	//listener.lookupTransform("/robot1_tf/map", "/robot1_tf/base_footprint",ros::Time(0), transform);
      	listener.waitForTransform("/map", "/base_footprint", now, ros::Duration(1.0));
      	listener.lookupTransform("/map", "/base_footprint",ros::Time(0), transform);

	ro_x_map = transform.getOrigin().x();
	ro_y_map = transform.getOrigin().y();

	std::cout << "現在座標 (" << ro_x_map << "," << ro_y_map <<  ")" <<std::endl;

	std::cout << "end  :ロボットの現在座標を取得" << std::endl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//一番遠いフロンティアを検索/////////////////////////////////////////////////////////////////////////////////////



	std::cout << "start:探査制限の中で一番遠い未探査領域を検索" << std::endl;

	int count = 0;

	for(i=0;i<fro_num_tmp;i++){
		dis_rad = sqrt(pow(fro_x_tmp[i], 2) + pow(fro_y_tmp[i],2));
		if(dis_rad <= search_radius){
			std::cout << "距離の計算結果 (" << fro_x_tmp[i] << "," << fro_y_tmp[i] <<  ") " << dis_rad << "m  判定:true" <<std::endl;
			x_tmp = fro_x_tmp[i] - ro_x_map;
			y_tmp = fro_y_tmp[i] - ro_y_map;
			dis_tmp = sqrt(pow(x_tmp, 2) + pow(y_tmp,2));
			count++;
			if(distance < dis_tmp){
				dis_for_view = dis_rad;
				distance = dis_tmp;
				far_x = fro_x_tmp[i];
				far_y = fro_y_tmp[i];
				num =i;
				limit_fro = false;
				std::cout << "距離の上書き (" << far_x << "," << far_y <<  ") " << distance << "m (座標間距離)" <<std::endl;
			}

		}
		else{
			std::cout << "距離の計算結果 (" << fro_x_tmp[i] << "," << fro_y_tmp[i] <<  ") " << dis_rad << "m  判定:false" <<std::endl;
		}

	}
	std::cout << "関数に入れる前の目標座標 (" << far_x << "," << far_y <<  ")  原点からの距離" << dis_for_view << "m,  座標間距離" << distance << "m"  <<std::endl;
	std::cout << "end  :探査制限の中で一番遠い未探査領域を検索" << std::endl;
//制限範囲内に未探査領域がなくなったら範囲を広げる////////////////////////////////////////////////////////
	if(limit_fro){
		//repeat_explore();
		//stop = true;
		//goto end_explore;
		if(search_radius == 30 ){
			stop = true;
			goto far_end;
		}
		search_radius += 2.0;
		std::cout << "＊＊＊＊＊＊＊＊＊＊探査半径を+2m ( 現在" << search_radius << "m ) ＊＊＊＊＊＊＊＊＊＊" << std::endl;
	}
///////////////////////////////////////////////////////////////////////////////////////////////////////
	std::cout << "＊＊＊＊＊＊＊＊＊＊現在の探査半径 (" << search_radius << " m )＊＊＊＊＊＊＊＊＊＊" << std::endl;
	nav_success = navigation(far_x,far_y);//ナビゲション用の関数を呼び出す


	if(nav_success == true){
		std::cout << "＊＊＊＊＊＊＊＊＊＊移動完了＊＊＊＊＊＊＊＊＊＊" << std::endl;
		/*std::cout << "logに座標追加 (" << far_x << "," << far_y <<  ")　　" << log_num+1 << "個目" <<std::endl;
		log_x.push_back(far_x);
		log_y.push_back(far_y);
		log_num++;
	*/}
	else if(nav_success == false){
		std::cout << "座標削除 (" << fro_x_tmp[num] << "," << fro_y_tmp[num] <<  ")　座標間距離" << distance << "m 　残り" << count-1 << "個" <<std::endl;
		fro_x_tmp.erase(fro_x_tmp.begin() + num);
		fro_y_tmp.erase(fro_y_tmp.begin() + num);
		fro_num_tmp--;

		retry_counter++;

		if(retry_counter == 6){
			std::cout << "＊＊＊＊＊＊＊＊＊＊再検索回数オーバー＊＊＊＊＊＊＊＊＊＊" << std::endl;
			/*if (search_radius == 30){
				stop = false;
			}*/
			goto far_end;
		}
		std::cout << "＊＊＊＊＊＊＊＊＊＊再検索スタート＊＊＊＊＊＊＊＊＊＊" << std::endl;
		goto research;
	}
	else{
		std::cout << "＊＊＊＊＊＊＊＊＊＊何かエラーがおきた＊＊＊＊＊＊＊＊＊＊" << std::endl;
	}

far_end:

	std::cout << "end  :far_frontier" << std::endl;
///////////////////////////////////////////////////////////////////////////////////////////////////////////

}



//フロンティアを検索する関数//////////////////////////////////////////////////////////////////////////////////////////
void frontier_search(const nav_msgs::OccupancyGrid::ConstPtr& msg){

//地図データを配列に格納////////////////////////////////////////////////////////////////////////////////////////
	int fro_num;
	std::vector<float> fro_x;//見つけたフロンティアのx座標
	std::vector<float> fro_y;//見つけたフロンティアのy座標
	nav_msgs::MapMetaData info = msg->info;//地図の設定を取得
	std::vector<int8_t> data = msg->data;//地図の値を取得
	int x = info.width;//地図の横サイズ
	int y = info.height;//地図の縦サイズ
	int8_t map_array[x][y];//地図を行列に格納
	int frontier_flag[x][y];//探査済みと未探査の境界を判定するフラグ
	int i,j;//for文
	int k = 0;//for文

	std::cout << "start:地図データを配列に格納" << std::endl;

	for(i=0;i<y;i++){
    		for(j=0;j<x;j++){
      			map_array[j][i] = data[k];
			if(map_array[j][i]!=0 && map_array[j][i]!=100 && map_array[j][i]!=-1){
					std::cout << "exception:" << map_array[j][i] << std::endl;
			}
			frontier_flag[j][i] = 0;
      			k++;
    		}
  	}
	std::cout << "end  :地図データを配列に格納" << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//横方向で未探査と探査済の境界を探す//////////////////////////////////////////////////////////////////////////////

	std::cout << "start:横方向で境界を検索" << std::endl;

	for(i=0;i<y;i++){
    		for(j=0;j<(x-1);j++){
      			if(map_array[j][i] == 0 && map_array[j+1][i] == -1){
	       			frontier_flag[j][i] = 1;
			}
        		else if(map_array[j][i] == -1 && map_array[j+1][i] == 0){
				frontier_flag[j+1][i] = 1;
			}
    		}
  	}

	std::cout << "end  :横方向で境界を検索" << std::endl;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//縦方向で未探査と探査済の境界を探す/////////////////////////////////////////////////////////////////////////////

	std::cout << "start:縦方向で境界を検索" << std::endl;

	for(j=0;j<x;j++){
    		for(i=0;i<(y-1);i++){
      			if(map_array[j][i] == 0 && map_array[j][i+1] == -1){
	       			frontier_flag[j][i] = 1;
			}
        		else if(map_array[j][i] == -1 && map_array[j][i+1] == 0){
				frontier_flag[j][i+1] = 1;
			}
    		}
  	}

	std::cout << "end  :縦方向で境界を検索" << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////////////

//横方向にフラグが8マス続いてる場所を探す////////////////////////////////////////////////////////////////////////
	float m_per_cell = info.resolution;//[m/cell]
	float robot_diameter = 0.4; //ロボットの直径[m]
	int robot_cellsize = robot_diameter / m_per_cell;//セル換算したロボットサイズ
	int frontier_sum;//フラグが続いているかの判定用
	float frontier_center;//フロンティア境界の中点
	float low_left_x = info.origin.position.x;//地図の左下のx座標
	float low_left_y = info.origin.position.y;//地図の左下のy座標
	fro_num = 0;//フロンティアの個数を初期化

	std::cout << "start:横方向で境界が連続している場所を検索" << std::endl;

	//for(i=1;i<(y-1);i++){//一行ごとに検索
	for(i=1;i<(y-1);i=i+3){//三行ごとに検索
		for(j=0;j<(x-robot_cellsize);j=j+3){//三列ごとに検索
    		//for(j=0;j<(x-robot_cellsize);j++){//一列ごとに検索
			frontier_sum = 0;
			for(k=j;k<(j+robot_cellsize);k++){
				frontier_sum=frontier_sum+frontier_flag[k][i];
				if(frontier_flag[k][i] == 0 && (frontier_flag[k][i-1] || frontier_flag[k][i+1])){
					frontier_sum++;
				}
			}
      			if(frontier_sum == robot_cellsize){
				frontier_center = (j+robot_cellsize-1)-(robot_cellsize/2);
				fro_x.push_back(frontier_center * m_per_cell + low_left_x);
				fro_y.push_back(low_left_y + (m_per_cell * i));
				fro_num++;
			}
    		}
  	}
	std::cout << "end  :横方向で境界が連続している場所を検索" << std::endl;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//縦方向にフラグが8マス続いてる場所を探す////////////////////////////////////////////////////////////////////////

	std::cout << "start:縦方向で境界が連続している場所を検索" << std::endl;

	//for(j=1;j<(x-1);j++){//一列ごとに検索
	for(j=1;j<(x-1);j=j+3){//三列ごとに検索
		for(i=0;i<(y-robot_cellsize);i=i+3){//三列ごとに検索
    		//for(i=0;i<(y-robot_cellsize);i++){//一列ごとに検索
			frontier_sum = 0;
			for(k=i;k<(i+robot_cellsize);k++){
				frontier_sum=frontier_sum+frontier_flag[j][k];
				if(frontier_flag[j][k] == 0 && (frontier_flag[j-1][k] || frontier_flag[j+1][k])){
					frontier_sum++;
				}
			}
			if(frontier_sum == robot_cellsize){
				frontier_center = (i+robot_cellsize -1)-(robot_cellsize/2);
				fro_x.push_back(low_left_x + (m_per_cell * j));
				fro_y.push_back(frontier_center * m_per_cell + low_left_y);
				fro_num++;
			}
    		}
  	}
	std::cout << "end  :縦方向で境界が連続している場所を検索" << std::endl;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	far_frontier(fro_x, fro_y, fro_num);

	if(stop){
		std::cout << "end  :探査プログラム" << std::endl;
		map_sub.shutdown();
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////


//360[deg]回転////////////////////////////////////////////////////////////////////////////////////////////////
void robot_rotate(){
 	geometry_msgs::Twist vel;
	vel.angular.z = 0.5;
	ros::Duration timeout(16.8);
	ros::Time start_time = ros::Time::now();

	while(ros::Time::now() - start_time < timeout){
		ros::Rate rate(10.0);
		vel_pub.publish(vel);
		rate.sleep();
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

//スイッチ用の関数///////////////////////
void switching(const std_msgs::Bool::ConstPtr& msg){
	bool judge = false;
	judge = msg->data;

	if(judge){
		std::cout << "＊＊＊＊＊＊＊＊＊＊探査プログラム切り替え中＊＊＊＊＊＊＊＊＊＊" << std::endl;
		switcher = true;
		switch_sub.shutdown();
	}
}
////////////////////////////////////////

//メイン関数////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv){

  	ros::init(argc, argv, "explore_program_mat2c");
  	ros::NodeHandle nh ;

	//actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//while(!ac.waitForServer(ros::Duration(5.0))){
	//	std::cout << "＊＊＊＊＊＊＊＊＊＊待機中＊＊＊＊＊＊＊＊＊＊" << std::endl;;
	//}


	//スイッチ部分//////////////////////////
	swi_pub = nh.advertise<std_msgs::Bool>("mat_end", 1000);
	ros::Rate r(1);
	while(!switcher){
		std::cout << "＊＊＊＊＊＊＊＊＊＊切り替え待機中＊＊＊＊＊＊＊＊＊＊" << std::endl;
		switch_sub = nh.subscribe("/switch",1,switching);
		ros::spinOnce();
		r.sleep();
	}

	////////////////////////////////////////
	std::cout << "＊＊＊＊＊＊＊＊＊＊探査プログラム切り替え完了＊＊＊＊＊＊＊＊＊＊" << std::endl;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	std::cout << "start:探査プログラム" << std::endl;

	vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

	std::cout << "start:360°回転" << std::endl;
	robot_rotate();
	std::cout << "end  :360°回転" << std::endl;

  	map_sub = nh.subscribe("/map",1,frontier_search);
	ros::spin();

	return 0;
}
