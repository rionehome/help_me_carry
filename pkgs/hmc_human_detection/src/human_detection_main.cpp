#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <std_msgs/Int32.h>
//#include "ros_posenet/Keypoint.h"
//#include "ros_posenet/Poses.h"
//#include "ros_posenet/Pose.h"
#include <float.h>


using namespace std;

class Human_Detection {
public:
	Human_Detection();

	~Human_Detection();

	//インデックスごとの構造体を作成
	typedef struct {
		int index;
		cv::Point point;
		double existence_rate;
	} SampleData;

	//posenet構造体
	typedef struct {
		int id;
		std::vector<cv::Point> key_point;
	} PoseData;

	ros::Publisher move_pub;
	ros::Subscriber signal;
	ros::Subscriber ydlider;


	std_msgs::Float64MultiArray info;
	std::vector<cv::Point> ydlider_points;

	int player_index;
	bool status = true;
	bool move_status = false;
	cv::Point player_point;

	ros::NodeHandle n;

	cv::Point calc_nearest_point() {
		int min = INT_MAX;
		cv::Point min_point;
		double result_distance = 0;
		for (int i = 0; i < (int)ydlider_points.size(); ++i) {
			result_distance = hypot(ydlider_points[i].x, ydlider_points[i].y);
			if (result_distance < min) {
				min = result_distance;
				min_point = ydlider_points[i];
			}
		}
		cout << min_point << '\n';
		return min_point;
	}

	void view_ydlider(std::vector<cv::Point> points);

	void signal_callback(const std_msgs::String::ConstPtr &msgs) {
		status = msgs->data == "start";
	}

	void move_callback(const std_msgs::Int32::ConstPtr &msgs) {
		if (msgs->data == 1 && !move_status) move_status = true;
		if (msgs->data == 0 && move_status) move_status = false;
	}

	void ydlider_callback(const sensor_msgs::LaserScan::ConstPtr &msgs);

	double calcAngle(const cv::Point &target_point);

	void turn(double angle) {
		info.data.clear();
		info.data.push_back(0);
		info.data.push_back(0);
		info.data.push_back(angle);
		info.data.push_back(0.5);
		move_pub.publish(info);
	}
};

Human_Detection::Human_Detection() {

	printf("Start class of 'Human_Detection'\n");


	this->ydlider = n.subscribe("/scan", 1, &Human_Detection::ydlider_callback, this);
	this->move_pub = n.advertise<std_msgs::Float64MultiArray>("/move/velocity", 1000);
	this->signal = n.subscribe("/Human_Detection_me_nlp/Human_Detection_me", 1, &Human_Detection::signal_callback,
	                           this);
	this->signal = n.subscribe("/move/signal", 1, &Human_Detection::move_callback, this);
}

Human_Detection::~Human_Detection() {
	printf("Shutdown class of 'Human_Detection'\n");
}

double Human_Detection::calcAngle(const cv::Point &target_point) {
	double result = -target_point.x * 0.02;
	printf("a %f\n", result);
	return result;
}

void Human_Detection::view_ydlider(std::vector<cv::Point> points) {

	cv::Mat img = cv::Mat::zeros(2000, 2000, CV_8UC3);
	cv::Scalar color(0, 255, 0);
	for (int i = 0; i < (int) points.size(); ++i) {
		int x = points[i].x + 1000;
		int y = points[i].y + 1000;
		cv::circle(img, cv::Point(x, y), 1, color, 1);
	}

	cv::circle(img, cv::Point(player_point.x + 1000, player_point.y + 1000), 5, cv::Scalar(0, 0, 255), 1);
	cv::namedWindow("window", CV_WINDOW_NORMAL);
	cv::imshow("window", img);

	cv::waitKey(1);
}

//ydliderからの情報を取得
void Human_Detection::ydlider_callback(const sensor_msgs::LaserScan::ConstPtr &msgs) {
	double rad = msgs->angle_min;
	ydlider_points.clear();

	for (int i = 0; i < (int) msgs->ranges.size(); ++i) {
		cv::Point position;
		if (msgs->range_min + 0.08 < msgs->ranges[i] && msgs->range_max > msgs->ranges[i]) {
			position = cv::Point(msgs->ranges[i] * sin(rad) * 100, -msgs->ranges[i] * cos(rad) * 100);
			ydlider_points.push_back(position);
		}
		rad += msgs->angle_increment;
	}

	view_ydlider(ydlider_points);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "human_detection");

	Human_Detection human_detection;

	double angle = 0;

	while (ros::ok()) {
		ros::spinOnce();
		angle = human_detection.calcAngle(human_detection.calc_nearest_point());

		human_detection.turn(angle);

		//動きが鈍くなってきたら脱出
		cout << angle << '\n';
		if (angle != 0 && abs(angle) < 0.1) {
			human_detection.turn(0);
			break;
		}
	}

	return 0;
}