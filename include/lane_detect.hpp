#pragma once

// C++
#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <sys/time.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudafilters.hpp>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/cmd_data.hpp"

#include "spline.h"

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

namespace LaneDetect {

class LaneDetector : public rclcpp::Node
{
public:
	LaneDetector();
	~LaneDetector();

	//Timer
	struct timeval start_, end_;

	float display_img(Mat _frame, int _delay, bool _view);
	void get_steer_coef(float vel);
	float K1_, K2_, K3_, K4_;
	int distance_ = 0;
	float est_dist_ = 0.0f;
	float est_pose_ = 0.0f;
	Mat frame_;
	float rotation_angle_ = 0.0f;
	float lateral_offset_ = 0.0f;
	Point left_, right_;
	float y_offset_ = 0.0f;

	/********** bbox *********/
	std::string name_;
	unsigned int x_ = 0, y_ = 0, w_ = 0, h_ = 0;
	Point center_, warp_center_;
	float SteerAngle2_ = 0.0f;
	float log_e1_ = 0.0f;
	float log_el_ = 0.0f;
	float vel_ = 0.0f;

private:
	void LoadParams(void);
	int arrMaxIdx(int hist[], int start, int end, int Max);
	Mat polyfit(vector<int> x_val, vector<int> y_val);
	Mat detect_lines_sliding_window(Mat _frame, bool _view);
	Point warpPoint(Point center, Mat trans);
	float lowPassFilter(double sampling_time, float est_value, float prev_res);
	Mat estimateDistance(Mat frame, Mat trans, double cycle_time, bool _view);
	Mat estimatePose(Mat frame, double cycle_time, bool _view);
	Mat draw_lane(Mat _sliding_frame, Mat _frame);
	Mat drawBox(Mat frame);
	void controlSteer();
	void clear_release();
	void cspline();

	//Publisher
        rclcpp::Publisher<ros2_msg::msg::CmdData>::SharedPtr XavPublisher_;

        //Subscriber
        rclcpp::Subscription<ros2_msg::msg::CmdData>::SharedPtr XavSubscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ImageSubscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rearImageSubscriber_;
    
    	//Callback Func
        void XavSubCallback(const ros2_msg::msg::CmdData::SharedPtr msg);
        void ImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rearImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg);

        bool viewImage_;
        int waitKeyDelay_;
	bool droi_ready_ = false;
        bool isNodeRunning_ = true;
        bool controlDone_ = false;
        ros2_msg::msg::CmdData lane_coef_;
	int center_select_;

	//image
    	bool imageStatus_ = false;
	std_msgs::msg::Header imageHeader_;
        cv::Mat camImageCopy_;
	float AngleDegree_;

        //rear
    	bool rearImageStatus_ = false;
	std_msgs::msg::Header rearImageHeader_;
        cv::Mat rearCamImageCopy_;

        std::thread lanedetect_Thread;
        void lanedetectInThread();

	/********** Camera calibration **********/
	Mat map1_, map2_, f_map1_, f_map2_;
	int canny_thresh1_, canny_thresh2_;

	/********** Lane_detect ***********/
	vector<Point2f> corners_, fROIcorners_, lcROIcorners_;
	vector<Point2f> warpCorners_, fROIwarpCorners_, lcROIwarpCorners_;
	float wide_extra_upside_[2], wide_extra_downside_[2];

	int last_Llane_base_;
	int last_Rlane_base_;
	int last_Elane_base_;

	vector<int> left_lane_inds_;
	vector<int> right_lane_inds_;
	vector<int> extra_lane_inds_;
	vector<int> left_x_;
	vector<int> left_y_;
	vector<int> right_x_;
	vector<int> right_y_;
	vector<int> extra_x_;
	vector<int> extra_y_;
	vector<int> center_x_;
	vector<int> center_y_;
	vector<int> center2_x_;
	vector<int> center2_y_;
	
	vector<Point> left_lane_, right_lane_, extra_lane_;

	Mat left_coef_;
	Mat right_coef_;
	Mat extra_coef_;
	Mat center_coef_;
	Mat center2_coef_;
	float left_curve_radius_;
	float right_curve_radius_;
	float center_position_;
	float SteerAngle_;
	float eL_height_, trust_height_, e1_height_, lp_;
	float eL_height2_;
	float K_;
	double a_[5], b_[5];
	vector<float> e_values_;
	
	int lc_flag_;
	float target_x_; 
	float target_y_; 

	/********** PID control ***********/
	int prev_lane_, prev_pid_;
	double Kp_term_, Ki_term_, Kd_term_, err_, prev_err_, I_err_, D_err_, result_;

	int width_, height_;
	bool option_; // dynamic ROI
	int threshold_;
	double diff_;

	int crop_x_, crop_y_, crop_width_, crop_height_;

	int Threshold_box_size_, Threshold_box_offset_;
};

}
