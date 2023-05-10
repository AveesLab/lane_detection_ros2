#include "lane_detect.hpp"

namespace LaneDetect {

LaneDetector::LaneDetector()
       : Node("LaneDetector", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
  /**************/
  /* ROS2 Topic */
  /**************/
  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string ImageSubTopicName;
  int ImageSubQueueSize;

  std::string XavPubTopicName;
  int XavPubQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("subscribers/xavier_to_lane/topic", XavSubTopicName, std::string("xav2lane_msg"));
  this->get_parameter_or("subscribers/xavier_to_lane/queue_size", XavSubQueueSize, 1);
  this->get_parameter_or("subscribers/image_to_lane/topic", ImageSubTopicName, std::string("image_raw"));
  this->get_parameter_or("subscribers/image_to_lane/queue_size", ImageSubQueueSize, 1);

  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  this->get_parameter_or("publishers/lane_to_xavier/topic", XavPubTopicName, std::string("lane2xav_msg"));
  this->get_parameter_or("publishers/lane_to_xavier/queue_size", XavPubQueueSize, 1);
  
  /************************/
  /* Ros Topic Subscriber */
  /************************/
  XavSubscriber_ = this->create_subscription<ros2_msg::msg::CmdData>(XavSubTopicName, XavSubQueueSize, std::bind(&LaneDetector::XavSubCallback, this, std::placeholders::_1));

  ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(ImageSubTopicName, ImageSubQueueSize, std::bind(&LaneDetector::ImageSubCallback, this, std::placeholders::_1));

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  XavPublisher_ = this->create_publisher<ros2_msg::msg::CmdData>(XavPubTopicName, XavPubQueueSize);

  /***************/
  /* View Option */
  /***************/
  this->get_parameter_or("image_view/enable_opencv", viewImage_, true);
  this->get_parameter_or("image_view/wait_key_delay", waitKeyDelay_, 3);


  /******* recording log *******/    
  gettimeofday(&start_, NULL);

      /******* Camera  calibration *******/
  double f_matrix[9], f_dist_coef[5];
  this->get_parameter_or("Calibration/f_matrix/a",f_matrix[0], 3.2918100682757097e+02);
  this->get_parameter_or("Calibration/f_matrix/b",f_matrix[1], 0.);
  this->get_parameter_or("Calibration/f_matrix/c",f_matrix[2], 320.);
  this->get_parameter_or("Calibration/f_matrix/d",f_matrix[3], 0.);
  this->get_parameter_or("Calibration/f_matrix/e",f_matrix[4], 3.2918100682757097e+02);
  this->get_parameter_or("Calibration/f_matrix/f",f_matrix[5], 240.);
  this->get_parameter_or("Calibration/f_matrix/g",f_matrix[6], 0.);
  this->get_parameter_or("Calibration/f_matrix/h",f_matrix[7], 0.);
  this->get_parameter_or("Calibration/f_matrix/i",f_matrix[8], 1.);

  this->get_parameter_or("Calibration/f_dist_coef/a",f_dist_coef[0], -3.2566540239089398e-01);
  this->get_parameter_or("Calibration/f_dist_coef/b",f_dist_coef[1], 1.1504807178349362e-01);
  this->get_parameter_or("Calibration/f_dist_coef/c",f_dist_coef[2], 0.);
  this->get_parameter_or("Calibration/f_dist_coef/d",f_dist_coef[3], 0.);
  this->get_parameter_or("Calibration/f_dist_coef/e",f_dist_coef[4], -2.1908791800876997e-02);

  Mat f_camera_matrix = Mat::eye(3, 3, CV_64FC1);
  Mat f_dist_coeffs = Mat::zeros(1, 5, CV_64FC1);
  f_camera_matrix = (Mat1d(3, 3) << f_matrix[0], f_matrix[1], f_matrix[2], f_matrix[3], f_matrix[4], f_matrix[5], f_matrix[6], f_matrix[7], f_matrix[8]);
  f_dist_coeffs = (Mat1d(1, 5) << f_dist_coef[0], f_dist_coef[1], f_dist_coef[2], f_dist_coef[3], f_dist_coef[4]);
  initUndistortRectifyMap(f_camera_matrix, f_dist_coeffs, Mat(), f_camera_matrix, Size(640, 480), CV_32FC1, f_map1_, f_map2_);

  map1_ = f_map1_.clone();
  map2_ = f_map2_.clone();

  /********** PID control ***********/
  prev_err_ = 0;

  last_Llane_base_ = 0;
  last_Rlane_base_ = 0;
  left_coef_ = Mat::zeros(3, 1, CV_32F);
  right_coef_ = Mat::zeros(3, 1, CV_32F);
  center_coef_ = Mat::zeros(3, 1, CV_32F);

  this->get_parameter_or("ROI/width", width_, 640);
  this->get_parameter_or("ROI/height", height_, 480);
  center_position_ = width_/2;

  e_values_.resize(3);

  float t_gap[2], b_gap[2], t_height[2], b_height[2], f_extra[2], b_extra[2];
  int top_gap[2], bot_gap[2], top_height[2], bot_height[2], extra_up[2], extra_down[2];

  this->get_parameter_or("ROI/dynamic_roi",option_, true);
  this->get_parameter_or("ROI/threshold",threshold_, 128);
  this->get_parameter_or("ROI/canny/thresh1",canny_thresh1_, 100);
  this->get_parameter_or("ROI/canny/thresh2",canny_thresh2_, 200);

  this->get_parameter_or("ROI/front_cam/top_gap",t_gap[0], 0.336f);
  this->get_parameter_or("ROI/front_cam/bot_gap",b_gap[0], 0.078f);
  this->get_parameter_or("ROI/front_cam/top_height",t_height[0], 0.903f);
  this->get_parameter_or("ROI/front_cam/bot_height",b_height[0], 0.528f);
  this->get_parameter_or("ROI/front_cam/extra_f",f_extra[0], 0.0f);
  this->get_parameter_or("ROI/front_cam/extra_b",b_extra[0], 0.0f);
  this->get_parameter_or("ROI/front_cam/extra_up",extra_up[0], 0);
  this->get_parameter_or("ROI/front_cam/extra_down",extra_down[0], 0);

  this->get_parameter_or("ROI/lc_mode/top_gap",t_gap[1], 0.886f);
  this->get_parameter_or("ROI/lc_mode/bot_gap",b_gap[1], 0.078f);
  this->get_parameter_or("ROI/lc_mode/top_height",t_height[1], 0.903f);
  this->get_parameter_or("ROI/lc_mode/bot_height",b_height[1], 0.528f);
  this->get_parameter_or("ROI/lc_mode/extra_f",f_extra[1], 0.0f);
  this->get_parameter_or("ROI/lc_mode/extra_b",b_extra[1], 0.0f);
  this->get_parameter_or("ROI/lc_mode/extra_up",extra_up[1], 0);
  this->get_parameter_or("ROI/lc_mode/extra_down",extra_down[1], 0);

  this->get_parameter_or("crop/x", crop_x_, 100);
  this->get_parameter_or("crop/y", crop_y_, 0);
  this->get_parameter_or("crop/width", crop_width_, 0);
  this->get_parameter_or("crop/height", crop_height_, 0);

  this->get_parameter_or("threshold/box_size", Threshold_box_size_, 51);
  this->get_parameter_or("threshold/box_offset", Threshold_box_offset_, 51);

  distance_ = 0;

  corners_.resize(4);
  warpCorners_.resize(4);

  /*** front cam ROI setting ***/
  fROIcorners_.resize(4);
  fROIwarpCorners_.resize(4);

  top_gap[0] = width_ * t_gap[0]; 
  bot_gap[0] = width_ * b_gap[0];
  top_height[0] = height_ * t_height[0];
  bot_height[0] = height_ * b_height[0];

  fROIcorners_[0] = Point2f(top_gap[0]+f_extra[0], bot_height[0]);
  fROIcorners_[1] = Point2f((width_ - top_gap[0])+f_extra[0], bot_height[0]);
  fROIcorners_[2] = Point2f(bot_gap[0]+b_extra[0], top_height[0]);
  fROIcorners_[3] = Point2f((width_ - bot_gap[0])+b_extra[0], top_height[0]);
  
  wide_extra_upside_[0] = extra_up[0];
  wide_extra_downside_[0] = extra_down[0];
  
  fROIwarpCorners_[0] = Point2f(wide_extra_upside_[0], 0.0);
  fROIwarpCorners_[1] = Point2f(width_ - wide_extra_upside_[0], 0.0);
  fROIwarpCorners_[2] = Point2f(wide_extra_downside_[0], height_);
  fROIwarpCorners_[3] = Point2f(width_ - wide_extra_downside_[0], height_);
  /*** front cam ROI setting ***/

  std::copy(fROIcorners_.begin(), fROIcorners_.end(), corners_.begin());
  std::copy(fROIwarpCorners_.begin(), fROIwarpCorners_.end(), warpCorners_.begin());

  /* Lateral Control coefficient */
  this->get_parameter_or("params/K", K_, 0.15f);
  this->get_parameter_or("params/a/a", a_[0], 0.);
  this->get_parameter_or("params/a/b", a_[1], -0.37169);
  this->get_parameter_or("params/a/c", a_[2], 1.2602);
  this->get_parameter_or("params/a/d", a_[3], -1.5161);
  this->get_parameter_or("params/a/e", a_[4], 0.70696);
  this->get_parameter_or("params/b/a", b_[0], 0.);
  this->get_parameter_or("params/b/b", b_[1], -1.7536);
  this->get_parameter_or("params/b/c", b_[2], 5.0931);
  this->get_parameter_or("params/b/d", b_[3], -4.9047);
  this->get_parameter_or("params/b/e", b_[4], 1.6722);

  LoadParams();
  
  lanedetect_Thread = std::thread(&LaneDetector::lanedetectInThread, this);
}

LaneDetector::~LaneDetector(void) 
{
  isNodeRunning_ = false;

  ros2_msg::msg::CmdData xav;
  xav.coef = lane_coef_.coef;
  xav.cur_angle = AngleDegree_;

  XavPublisher_->publish(xav);
  lanedetect_Thread.join();

  clear_release();
  RCLCPP_INFO(this->get_logger(), "Stop.");
}

void LaneDetector::lanedetectInThread()
{
  const auto wait_duration = std::chrono::milliseconds(2000);
  while(!imageStatus_) {
    printf("Waiting for image.\n");
    if(!isNodeRunning_) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }

  ros2_msg::msg::CmdData xav;

  while(!controlDone_ && rclcpp::ok()) 
  {
    if(imageStatus_ && droi_ready_) {  
      AngleDegree_ = display_img(camImageCopy_, waitKeyDelay_, viewImage_);
      droi_ready_ = false;
     
      xav.coef = lane_coef_.coef;
      xav.cur_angle = AngleDegree_;
      XavPublisher_->publish(xav);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if(!isNodeRunning_) {
      controlDone_ = true;
      rclcpp::shutdown();
    }
  }

}
void LaneDetector::LoadParams(void)
{
  this->get_parameter_or("LaneDetector/eL_height",eL_height_, 1.0f);  
  this->get_parameter_or("LaneDetector/e1_height",e1_height_, 1.0f);  
  this->get_parameter_or("LaneDetector/trust_height",trust_height_, 1.0f);  
  this->get_parameter_or("LaneDetector/lp",lp_, 756.0f);  
  this->get_parameter_or("LaneDetector/steer_angle",SteerAngle_, 0.0f);
}

void LaneDetector::XavSubCallback(const ros2_msg::msg::CmdData::SharedPtr msg)
{
  float cur_vel_ = msg->cur_vel;
  distance_ = msg->cur_dist;

  get_steer_coef(cur_vel_);
  droi_ready_ = true;
}

void LaneDetector::ImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cam_image;
  try{
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception : %s", e.what());
  }

  if(cam_image) {
    imageHeader_ = msg->header;
    camImageCopy_ = cam_image->image.clone();
    imageStatus_ = true;
  }
}

int LaneDetector::arrMaxIdx(int hist[], int start, int end, int Max) {
  int max_index = -1;
  int max_val = 0;

  if (end > Max)
    end = Max;

  for (int i = start; i < end; i++) {
    if (max_val < hist[i]) {
      max_val = hist[i];
      max_index = i;
    }
  }
  if (max_index == -1) {
    cout << "ERROR : hist range" << endl;
    return -1;
  }
  return max_index;
}

Mat LaneDetector::polyfit(vector<int> x_val, vector<int> y_val) {
  Mat coef(3, 1, CV_32F);
  int i, j, k, n, N;
  N = (int)x_val.size();
  n = 2;
  double* x, * y;
  x = new double[N];
  y = new double[N];
  for (int q = 0; q < N; q++) {
    x[q] = (double)(x_val[q]);
    y[q] = (double)(y_val[q]);
  }
  double* X;
  X = new double[2 * n + 1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
  for (i = 0; i < (2 * n + 1); i++)
  {
    X[i] = 0;
    for (j = 0; j < N; j++)
      X[i] = X[i] + pow(x[j], i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
  }
  double** B, * a;
  B = new double* [n + 1];
  for (int i = 0; i < (n + 1); i++)
    B[i] = new double[n + 2];
  a = new double[n + 1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
  for (i = 0; i <= n; i++)
    for (j = 0; j <= n; j++)
      B[i][j] = X[i + j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
  double* Y;
  Y = new double[n + 1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
  for (i = 0; i < (n + 1); i++)
  {
    Y[i] = 0;
    for (j = 0; j < N; j++)
      Y[i] = Y[i] + pow(x[j], i) * y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
  }
  for (i = 0; i <= n; i++)
    B[i][n + 1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
  n = n + 1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations

  for (i = 0; i < n; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
    for (k = i + 1; k < n; k++)
      if (B[i][i] < B[k][i])
        for (j = 0; j <= n; j++)
        {
          double temp = B[i][j];
          B[i][j] = B[k][j];
          B[k][j] = temp;
        }

  for (i = 0; i < (n - 1); i++)            //loop to perform the gauss elimination
    for (k = i + 1; k < n; k++)
    {
      double t = B[k][i] / B[i][i];
      for (j = 0; j <= n; j++)
        B[k][j] = B[k][j] - t * B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
    }
  for (i = n - 1; i >= 0; i--)                //back-substitution
  {                        //x is an array whose values correspond to the values of x,y,z..
    a[i] = B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
    for (j = 0; j < n; j++)
      if (j != i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
        a[i] = a[i] - B[i][j] * a[j];
    a[i] = a[i] / B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    coef.at<float>(i, 0) = (float)a[i];
  }

  delete[] x;
  delete[] y;
  delete[] X;
  delete[] Y;
  delete[] B;
  delete[] a;

  return coef;
}

Mat LaneDetector::detect_lines_sliding_window(Mat _frame, bool _view) {
  Mat frame, result;
  int width = _frame.cols;
  int height = _frame.rows;

  _frame.copyTo(frame);
  Mat nonZero;
  findNonZero(frame, nonZero);

  vector<int> good_left_inds;
  vector<int> good_right_inds;
  vector<int> good_extra_inds;

  int* hist = new int[width];

  for (int i = 0; i < width; i++) {
    hist[i] = 0;
  }

  for (int j = (height / 2); j < height; j++) { // hist 범위 절반부터 읽기
    for (int i = 0; i < width; i++) {
      if (frame.at <uchar>(j, i) == 255) {
        hist[i] += 1;
      }
    }
  }  
 
  cvtColor(frame, result, COLOR_GRAY2BGR);

  int mid_point = width / 2; // 320
  int n_windows = 9;
  int margin = 120 * width / 1280;
  int min_pix = 30 * width / 1280;

  int window_width = margin * 2;  // 120
  int window_height;
  int distance;
  if (option_) {
    window_height = (height >= distance_) ? ((height-distance_) / n_windows) : (height / n_windows);  // defalut = 53
    distance = distance_;
  } else {
    distance = 0;
    window_height = height / n_windows;
  }
  int Llane_base = arrMaxIdx(hist, 100, mid_point, width);
  int Rlane_base = arrMaxIdx(hist, mid_point, width - 100, width);
  int Elane_base = arrMaxIdx(hist, width - 100, width, width);
  if (Llane_base == -1 || Rlane_base == -1)
    return result;

  int Llane_current = Llane_base;
  int Rlane_current = Rlane_base;
  int Elane_current = Elane_base;
  printf("Llane | Rlane : %d | %d\n", Llane_current, Rlane_current);

//  if (last_Llane_base_!=0 || last_Rlane_base_!=0) {
//    int Llane_current = Llane_base;
//    int Rlane_current = Rlane_base;
//  }

  int L_prev =  Llane_current;
  int R_prev =  Rlane_current;
  int E_prev =  Elane_current;
  int L_gap = 0;
  int R_gap = 0;
  int E_gap = 0;

  unsigned int index;


  for (int window = 0; window < n_windows; window++) {
    int  Ly_pos = height - (window + 1) * window_height - 1; // win_y_low , win_y_high = win_y_low - window_height
    int  Ry_pos = height - (window + 1) * window_height - 1;
    int  Ey_pos = height - (window + 1) * window_height - 1;
    int  Ly_top = height - window * window_height;
    int  Ry_top = height - window * window_height;
    int  Ey_top = height - window * window_height;

    int  Lx_pos = Llane_current - margin; // win_xleft_low, win_xleft_high = win_xleft_low + margin*2
    int  Rx_pos = Rlane_current - margin; // win_xrignt_low, win_xright_high = win_xright_low + margin*2
    int  Ex_pos = Elane_current - margin; // win_xrignt_low, win_xright_high = win_xright_low + margin*2
    if (_view) {
      rectangle(result, \
        Rect(Lx_pos, Ly_pos, window_width, window_height), \
        Scalar(255, 50, 100), 1);
      rectangle(result, \
        Rect(Rx_pos, Ry_pos, window_width, window_height), \
        Scalar(100, 50, 255), 1);
      rectangle(result, \
        Rect(Ex_pos, Ey_pos, window_width, window_height), \
        Scalar(50, 255, 255), 1);
    }
    int nZ_y, nZ_x;
    good_left_inds.clear();
    good_right_inds.clear();
    good_extra_inds.clear();

    for (unsigned int index = (nonZero.total() - 1); index > 1; index--) {
      nZ_y = nonZero.at<Point>(index).y;
      nZ_x = nonZero.at<Point>(index).x;
      if ((nZ_y >= Ly_pos) && \
        (nZ_y > (distance)) && \
        (nZ_y < Ly_top) && \
        (nZ_x >= Lx_pos) && \
        (nZ_x < (Lx_pos + window_width))) {
        if (_view) {
          result.at<Vec3b>(nonZero.at<Point>(index))[0] = 255;
          result.at<Vec3b>(nonZero.at<Point>(index))[1] = 0;
          result.at<Vec3b>(nonZero.at<Point>(index))[2] = 0;
        }
        good_left_inds.push_back(index);
      }
      
      if ((nZ_y >= (Ry_pos)) && \
        (nZ_y > (distance)) && \
        (nZ_y < Ry_top) && \
        (nZ_x >= Rx_pos) && \
        (nZ_x < (Rx_pos + window_width))) {
        if (_view) {
          result.at<Vec3b>(nonZero.at<Point>(index))[0] = 0;
          result.at<Vec3b>(nonZero.at<Point>(index))[1] = 0;
          result.at<Vec3b>(nonZero.at<Point>(index))[2] = 255;
        }
        good_right_inds.push_back(index);
      }

      if ((nZ_y >= (Ey_pos)) && \
        (nZ_y > (distance)) && \
        (nZ_y < Ey_top) && \
        (nZ_x >= Ex_pos) && \
        (nZ_x < (Ex_pos + window_width))) {
        if (_view) {
          result.at<Vec3b>(nonZero.at<Point>(index))[0] = 0;
          result.at<Vec3b>(nonZero.at<Point>(index))[1] = 255;
          result.at<Vec3b>(nonZero.at<Point>(index))[2] = 255;
        }
        good_extra_inds.push_back(index);
      }
    }
    
    int Lsum, Rsum, Esum;
    Lsum = Rsum = Esum = 0;
    unsigned int _size;
    vector<int> Llane_x;
    vector<int> Llane_y;
    vector<int> Rlane_x;
    vector<int> Rlane_y;
    vector<int> Elane_x;
    vector<int> Elane_y;

    if (good_left_inds.size() > (size_t)min_pix) {
      _size = (unsigned int)(good_left_inds.size());
      for (int i = Ly_top-1; i >= Ly_pos ; i--)
      {
        int Ly_sum = 0;
        int count = 0;
        for (index = 0; index < _size; index++) {
          int j = nonZero.at<Point>(good_left_inds.at(index)).y;
          if(i == j)
          {
            Ly_sum += nonZero.at<Point>(good_left_inds.at(index)).x;
            count++;
            Lsum += nonZero.at<Point>(good_left_inds.at(index)).x;
            //left_x_.insert(left_x_.end(), nonZero.at<Point>(good_right_inds.at(index)).x);
            //left_y_.insert(left_y_.end(), nonZero.at<Point>(good_right_inds.at(index)).y);
          }
        }
        if(count != 0)
        {
          left_x_.insert(left_x_.end(), Ly_sum/count);
          left_y_.insert(left_y_.end(), i);
          Llane_x.insert(Llane_x.end(), Ly_sum/count);
          Llane_y.insert(Llane_y.end(), i);
        } else {
          Llane_x.insert(Llane_x.end(), -1);
          Llane_y.insert(Llane_y.end(), i);
        }
      }
      Llane_current = Lsum / _size;
      //left_x_.insert(left_x_.end(), Llane_current);
      //left_y_.insert(left_y_.end(), Ly_pos + (window_height / 2));
    } else{
      Llane_current += (L_gap);
    }
    if (good_right_inds.size() > (size_t)min_pix) {
      _size = (unsigned int)(good_right_inds.size());
      for (int i = Ry_top - 1 ; i >= Ry_pos ; i--)
      {
        int Ry_sum = 0;
        int count = 0;
        for (index = 0; index < _size; index++) {
          int j = nonZero.at<Point>(good_right_inds.at(index)).y;
          if(i == j)
          {
            Ry_sum += nonZero.at<Point>(good_right_inds.at(index)).x;
            count++;
            Rsum += nonZero.at<Point>(good_right_inds.at(index)).x;
            //right_x_.insert(right_x_.end(), nonZero.at<Point>(good_right_inds.at(index)).x);
            //right_y_.insert(right_y_.end(), nonZero.at<Point>(good_right_inds.at(index)).y);
          }
        }
        if(count != 0)
        {
          right_x_.insert(right_x_.end(), Ry_sum/count);
          right_y_.insert(right_y_.end(), i);
          Rlane_x.insert(Rlane_x.end(), Ry_sum/count);
          Rlane_y.insert(Rlane_y.end(), i);
        } else {
          Rlane_x.insert(Rlane_x.end(), -1);
          Rlane_y.insert(Rlane_y.end(), i);
        }
      }
      Rlane_current = Rsum / _size;
      //right_x_.insert(right_x_.end(), Rlane_current);
      //right_y_.insert(right_y_.end(), Ry_pos + (window_height / 2));
    } else{
      Rlane_current += (R_gap);
    }
    if (good_extra_inds.size() > (size_t)min_pix) {
      _size = (unsigned int)(good_extra_inds.size());
      for (int i = Ey_top - 1 ; i >= Ey_pos ; i--)
      {
        int Ey_sum = 0;
        int count = 0;
        for (index = 0; index < _size; index++) {
          int j = nonZero.at<Point>(good_extra_inds.at(index)).y;
          if(i == j)
          {
            Ey_sum += nonZero.at<Point>(good_extra_inds.at(index)).x;
            count++;
            Esum += nonZero.at<Point>(good_extra_inds.at(index)).x;
          }
        }
        if(count != 0)
        {
          extra_x_.insert(extra_x_.end(), Ey_sum/count);
          extra_y_.insert(extra_y_.end(), i);
          Elane_x.insert(Elane_x.end(), Ey_sum/count);
          Elane_y.insert(Elane_y.end(), i);
        } else {
          Elane_x.insert(Elane_x.end(), -1);
          Elane_y.insert(Elane_y.end(), i);
        }
      }
      Elane_current = Esum / _size;
    } else{
      Elane_current += (E_gap);
    }
    if (window != 0) {  
      if (Rlane_current != R_prev) {
        R_gap = (Rlane_current - R_prev);
      }
      if (Llane_current != L_prev) {
        L_gap = (Llane_current - L_prev);
      }
      if (Elane_current != E_prev) {
        E_gap = (Elane_current - E_prev);
      }
    }
    if ((Lsum != 0) && (Rsum != 0)) {
      for (int i = 0; i < Llane_x.size() ; i++)
      {
        if((Llane_x.at(i) != -1) && (Rlane_x.at(i) != -1)) {
          center_x_.insert(center_x_.end(), (Llane_x.at(i)+Rlane_x.at(i)) / 2 );
          center_y_.insert(center_y_.end(), Llane_y.at(i));
        }
      }
      //center_x_.insert(center_x_.end(), (Llane_current + Rlane_current) / 2);
      //center_y_.insert(center_y_.end(), Ly_pos + (window_height / 2));  
    }
    if ((Rsum != 0) && (Esum != 0)) {
      for (int i = 0; i < Elane_x.size() ; i++)
      {
        if((Rlane_x.at(i) != -1) && (Elane_x.at(i) != -1)) {
          center2_x_.insert(center2_x_.end(), (Rlane_x.at(i)+Elane_x.at(i)) / 2 );
          center2_y_.insert(center2_y_.end(), Elane_y.at(i));
        }
      }
      //center_x_.insert(center_x_.end(), (Llane_current + Rlane_current) / 2);
      //center_y_.insert(center_y_.end(), Ly_pos + (window_height / 2));  
    }
    L_prev = Llane_current;
    R_prev = Rlane_current;
    E_prev = Elane_current;
  }

  if (left_x_.size() != 0) {
    left_coef_ = polyfit(left_y_, left_x_);
  }
  if (right_x_.size() != 0) {
    right_coef_ = polyfit(right_y_, right_x_);
  }
  if (extra_x_.size() != 0) {
    extra_coef_ = polyfit(extra_y_, extra_x_);
  }
  
  if (center_x_.size() != 0){
    center_coef_ = polyfit(center_y_, center_x_);
  }
  if (center2_x_.size() != 0){
    center2_coef_ = polyfit(center2_y_, center2_x_);
  }

  delete[] hist;

  return result;
}


float LaneDetector::lowPassFilter(double sampling_time, float est_value, float prev_res){
  float res = 0;
  float tau = 0.10f;
  double st = 0.0;

  if (sampling_time > 1.0) st = 1.0;
  else st = sampling_time;
  res = ((tau * prev_res) + (st * est_value)) / (tau + st);

  return res;
}

Mat LaneDetector::draw_lane(Mat _sliding_frame, Mat _frame) {
  Mat new_frame, left_coef(left_coef_), right_coef(right_coef_), extra_coef(extra_coef_), center_coef(center_coef_), center2_coef(center2_coef_), trans;

  static struct timeval endTime, startTime;
  static bool flag;
  double diffTime;

  //trans = getPerspectiveTransform(fROIwarpCorners_, fROIcorners_);
  trans = getPerspectiveTransform(warpCorners_, corners_);
  _frame.copyTo(new_frame);

  vector<Point> left_point;
  vector<Point> right_point;
  vector<Point> extra_point;
  vector<Point> center_point;
  vector<Point> center2_point;

  vector<Point2f> left_point_f;
  vector<Point2f> right_point_f;
  vector<Point2f> extra_point_f;
  vector<Point2f> center_point_f;
  vector<Point2f> center2_point_f;

  vector<Point2f> warped_left_point;
  vector<Point2f> warped_right_point;
  vector<Point2f> warped_extra_point;
  vector<Point2f> warped_center_point;
  vector<Point2f> warped_center2_point;

  vector<Point> left_points;
  vector<Point> right_points;
  vector<Point> extra_points;
  vector<Point> center_points;
  vector<Point> center2_points;

  if ((!left_coef.empty()) && (!right_coef.empty()) && (!extra_coef.empty())) {
    for (int i = 0; i <= height_; i++) {
      Point temp_left_point;
      Point temp_right_point;
      Point temp_extra_point;
      Point temp_center_point;
      Point temp_center2_point;

      temp_left_point.x = (int)((left_coef.at<float>(2, 0) * pow(i, 2)) + (left_coef.at<float>(1, 0) * i) + left_coef.at<float>(0, 0));
      temp_left_point.y = (int)i;
      temp_right_point.x = (int)((right_coef.at<float>(2, 0) * pow(i, 2)) + (right_coef.at<float>(1, 0) * i) + right_coef.at<float>(0, 0));
      temp_right_point.y = (int)i;
      temp_extra_point.x = (int)((extra_coef.at<float>(2, 0) * pow(i, 2)) + (extra_coef.at<float>(1, 0) * i) + extra_coef.at<float>(0, 0));
      temp_extra_point.y = (int)i;
      temp_center_point.x = (int)((center_coef.at<float>(2, 0) * pow(i, 2)) + (center_coef.at<float>(1, 0) * i) + center_coef.at<float>(0, 0));
      temp_center_point.y = (int)i;
      temp_center2_point.x = (int)((center2_coef.at<float>(2, 0) * pow(i, 2)) + (center2_coef.at<float>(1, 0) * i) + center2_coef.at<float>(0, 0));
      temp_center2_point.y = (int)i;

      left_point.push_back(temp_left_point);
      left_point_f.push_back(temp_left_point);
      right_point.push_back(temp_right_point);
      right_point_f.push_back(temp_right_point);
      extra_point.push_back(temp_extra_point);
      extra_point_f.push_back(temp_extra_point);
      center_point.push_back(temp_center_point);
      center_point_f.push_back(temp_center_point);
      center2_point.push_back(temp_center2_point);
      center2_point_f.push_back(temp_center2_point);
    }
    const Point* left_points_point_ = (const cv::Point*) Mat(left_point).data;
    int left_points_number_ = Mat(left_point).rows;
    const Point* right_points_point_ = (const cv::Point*) Mat(right_point).data;
    int right_points_number_ = Mat(right_point).rows;
    const Point* extra_points_point_ = (const cv::Point*) Mat(extra_point).data;
    int extra_points_number_ = Mat(extra_point).rows;
    const Point* center_points_point_ = (const cv::Point*) Mat(center_point).data;
    int center_points_number_ = Mat(center_point).rows;
    const Point* center2_points_point_ = (const cv::Point*) Mat(center2_point).data;
    int center2_points_number_ = Mat(center2_point).rows;

    polylines(_sliding_frame, &left_points_point_, &left_points_number_, 1, false, Scalar(255, 200, 200), 5);
    polylines(_sliding_frame, &right_points_point_, &right_points_number_, 1, false, Scalar(200, 200, 255), 5);
    polylines(_sliding_frame, &extra_points_point_, &extra_points_number_, 1, false, Scalar(200, 255, 255), 5);
    polylines(_sliding_frame, &center_points_point_, &center_points_number_, 1, false, Scalar(200, 255, 200), 5);
    polylines(_sliding_frame, &center2_points_point_, &center2_points_number_, 1, false, Scalar(200, 255, 200), 5);
    
    perspectiveTransform(left_point_f, warped_left_point, trans);
    perspectiveTransform(right_point_f, warped_right_point, trans);
    perspectiveTransform(extra_point_f, warped_extra_point, trans);
    perspectiveTransform(center_point_f, warped_center_point, trans);
    perspectiveTransform(center2_point_f, warped_center2_point, trans);

    for (int i = 0; i <= height_; i++) {
      Point temp_left_point;
      Point temp_right_point;
      Point temp_extra_point;
      Point temp_center_point;
      Point temp_center2_point;

      temp_left_point.x = (int)warped_left_point[i].x;
      temp_left_point.y = (int)warped_left_point[i].y;
      temp_right_point.x = (int)warped_right_point[i].x;
      temp_right_point.y = (int)warped_right_point[i].y;
      temp_extra_point.x = (int)warped_extra_point[i].x;
      temp_extra_point.y = (int)warped_extra_point[i].y;
      temp_center_point.x = (int)warped_center_point[i].x;
      temp_center_point.y = (int)warped_center_point[i].y;
      temp_center2_point.x = (int)warped_center2_point[i].x;
      temp_center2_point.y = (int)warped_center2_point[i].y;

      left_points.push_back(temp_left_point);
      right_points.push_back(temp_right_point);
      extra_points.push_back(temp_extra_point);
      center_points.push_back(temp_center_point);
      center2_points.push_back(temp_center2_point);
    }

    const Point* left_points_point = (const cv::Point*) Mat(left_points).data;
    int left_points_number = Mat(left_points).rows;
    const Point* right_points_point = (const cv::Point*) Mat(right_points).data;
    int right_points_number = Mat(right_points).rows;
    const Point* extra_points_point = (const cv::Point*) Mat(extra_points).data;
    int extra_points_number = Mat(extra_points).rows;
    const Point* center_points_point = (const cv::Point*) Mat(center_points).data;
    int center_points_number = Mat(center_points).rows;
    const Point* center2_points_point = (const cv::Point*) Mat(center2_points).data;
    int center2_points_number = Mat(center2_points).rows;

    Point lane_center = *(center_points_point + center_points_number - 10);
    Point lane_center2 = *(center2_points_point + center2_points_number - 10);
    static Point prev_lane_center;
    static Point prev_lane_center2;
    gettimeofday(&endTime, NULL);
    if (!flag){
      diffTime = (endTime.tv_sec - start_.tv_sec) + (endTime.tv_usec - start_.tv_usec)/1000000.0;
      flag = true;
    }
    else{
      diffTime = (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec)/1000000.0;
      startTime = endTime;
    }
    lane_center.x = lowPassFilter(diffTime, lane_center.x, prev_lane_center.x);
    lane_center.y = lowPassFilter(diffTime, lane_center.y, prev_lane_center.y);
    lane_center2.x = lowPassFilter(diffTime, lane_center2.x, prev_lane_center2.x);
    lane_center2.y = lowPassFilter(diffTime, lane_center2.y, prev_lane_center2.y);

    prev_lane_center = lane_center;
    prev_lane_center2 = lane_center2;
    
    polylines(new_frame, &left_points_point, &left_points_number, 1, false, Scalar(255, 100, 100), 5);
    polylines(new_frame, &right_points_point, &right_points_number, 1, false, Scalar(100, 100, 255), 5);
    polylines(new_frame, &extra_points_point, &extra_points_number, 1, false, Scalar(100, 255, 255), 5);
    polylines(new_frame, &center_points_point, &center_points_number, 1, false, Scalar(100, 255, 100), 5);
    polylines(new_frame, &center2_points_point, &center2_points_number, 1, false, Scalar(100, 255, 100), 5);
    
    left_point.clear();
    right_point.clear();
    extra_point.clear();
    center_point.clear();
    center2_point.clear();

    /***************/
    /* Dynamic ROI */
    /***************/

    Point temp_roi_point;
    Point temp_droi_point;
    vector<Point2f> droi_point_f;
    vector<Point2f> warped_droi_point;
    vector<Point> roi_points;
    vector<Point> droi_points;
    
    temp_droi_point.y = (int)height_;
    temp_droi_point.x = 0;
    droi_point_f.push_back(temp_droi_point);
    temp_droi_point.x = (int)width_;
    droi_point_f.push_back(temp_droi_point);
    
    temp_droi_point.y = distance_;
    temp_droi_point.x = (int)width_;
    droi_point_f.push_back(temp_droi_point);
    temp_droi_point.x = 0;
    droi_point_f.push_back(temp_droi_point);
    
    perspectiveTransform(droi_point_f, warped_droi_point, trans);
    
    int droi_num[5] = {0, 1, 2, 3, 0};
    int roi_num[5] = {0, 1, 3, 2, 0};
    
    for (int i = 0; i < 5; i++) {
      temp_droi_point.x = (int)warped_droi_point[droi_num[i]].x;
      temp_droi_point.y = (int)warped_droi_point[droi_num[i]].y;
      
      droi_points.push_back(temp_droi_point);
      
      temp_roi_point.x = (int)corners_[roi_num[i]].x;
      temp_roi_point.y = (int)corners_[roi_num[i]].y;
      
      roi_points.push_back(temp_roi_point);
    }

    const Point* roi_points_point = (const cv::Point*) Mat(roi_points).data;
    int roi_points_number = Mat(roi_points).rows;
    const Point* droi_points_point = (const cv::Point*) Mat(droi_points).data;
    int droi_points_number = Mat(droi_points).rows;

    polylines(_frame, &roi_points_point, &roi_points_number, 1, false, Scalar(0, 0, 255), 5);
    polylines(_frame, &droi_points_point, &droi_points_number, 1, false, Scalar(0, 255, 0), 5);

    string TEXT = "ROI";
    Point2f T_pos(Point2f(270, _frame.rows-120));
    putText(_frame, TEXT, T_pos, FONT_HERSHEY_DUPLEX, 2, Scalar(0, 0, 255), 5, 8);
    
    return new_frame;
  }
  return _frame;
}

void LaneDetector::clear_release() {
  left_lane_inds_.clear();
  right_lane_inds_.clear();
  left_x_.clear();
  left_y_.clear();
  right_x_.clear();
  right_y_.clear();
  extra_x_.clear();
  extra_y_.clear();
  center_x_.clear();
  center_y_.clear();
  center2_x_.clear();
  center2_y_.clear();
}

void LaneDetector::get_steer_coef(float vel){
  float value;
  if (vel > 1.2f)
    value = 1.2f;
  else
    value = vel;

  if (value < 0.65f){
    K1_ = K2_ =  K_;
  }
  else{
    K1_ = (a_[0] * pow(value, 4)) + (a_[1] * pow(value, 3)) + (a_[2] * pow(value, 2)) + (a_[3] * value) + a_[4];
    K2_ = (b_[0] * pow(value, 4)) + (b_[1] * pow(value, 3)) + (b_[2] * pow(value, 2)) + (b_[3] * value) + b_[4];
  }
  
}

void LaneDetector::controlSteer() {
  Mat l_fit(left_coef_), r_fit(right_coef_), c_fit(center_coef_);
  float car_position = width_ / 2;
  float l1 = 0, l2 = 0;
  
  lane_coef_.coef.resize(3);
  if (!l_fit.empty() && !r_fit.empty()) {
    lane_coef_.coef[0].a = l_fit.at<float>(2, 0);
    lane_coef_.coef[0].b = l_fit.at<float>(1, 0);
    lane_coef_.coef[0].c = l_fit.at<float>(0, 0);

    lane_coef_.coef[1].a = r_fit.at<float>(2, 0);
    lane_coef_.coef[1].b = r_fit.at<float>(1, 0);
    lane_coef_.coef[1].c = r_fit.at<float>(0, 0);

    lane_coef_.coef[2].a = c_fit.at<float>(2, 0);
    lane_coef_.coef[2].b = c_fit.at<float>(1, 0);
    lane_coef_.coef[2].c = c_fit.at<float>(0, 0);
    

    float i = ((float)height_) * eL_height_;  
    float j = ((float)height_) * trust_height_;
    float k = ((float)height_) * e1_height_;

    l1 =  j - i;
    l2 = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(j, 2)) + (lane_coef_.coef[2].b * j) + lane_coef_.coef[2].c);

    e_values_[0] = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));  //trust_e1
    e_values_[2] = ((lane_coef_.coef[2].a * pow(k, 2)) + (lane_coef_.coef[2].b * k) + lane_coef_.coef[2].c) - car_position;  //e1
    SteerAngle_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
  
  }
}

float LaneDetector::display_img(Mat _frame, int _delay, bool _view) {		
  Mat new_frame, gray_frame, edge_frame, binary_frame, sliding_frame, resized_frame, lc_frame;
  
  if(!_frame.empty()) resize(_frame, new_frame, Size(width_, height_));
  Mat trans = getPerspectiveTransform(corners_, warpCorners_); /* apply ROI setting */
  
  cuda::GpuMat gpu_map1, gpu_map2;
  gpu_map1.upload(map1_);
  gpu_map2.upload(map2_);
  
  cuda::GpuMat gpu_frame, gpu_remap_frame, gpu_warped_frame, gpu_blur_frame, gpu_gray_frame, gpu_binary_frame;
  
  gpu_frame.upload(new_frame);
  cuda::remap(gpu_frame, gpu_remap_frame, gpu_map1, gpu_map2, INTER_LINEAR);
  gpu_remap_frame.download(new_frame);  /* apply camera matrix to new_frame */

  cuda::warpPerspective(gpu_remap_frame, gpu_warped_frame, trans, Size(width_, height_)); /* ROI apply frame */

  static cv::Ptr< cv::cuda::Filter > filters;
  filters = cv::cuda::createGaussianFilter(gpu_warped_frame.type(), gpu_blur_frame.type(), cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT);
  filters->apply(gpu_warped_frame, gpu_blur_frame);
  cuda::cvtColor(gpu_blur_frame, gpu_gray_frame, COLOR_BGR2GRAY);
  gpu_gray_frame.download(gray_frame);
  adaptiveThreshold(gray_frame, binary_frame, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, Threshold_box_size_, -(Threshold_box_offset_));
//  cuda::threshold(gpu_gray_frame, gpu_binary_frame, threshold_, 255, THRESH_BINARY);
//  gpu_binary_frame.download(binary_frame);


  sliding_frame = detect_lines_sliding_window(binary_frame, _view);
  controlSteer();

  if (_view) {
    resized_frame = draw_lane(sliding_frame, new_frame);

    namedWindow("Window1");
    moveWindow("Window1", 0, 0);
    namedWindow("Window2");
    moveWindow("Window2", 720, 0);
    namedWindow("Window3");
    moveWindow("Window3", 1380, 0);

    if(!new_frame.empty()) {
      resize(new_frame, new_frame, Size(640, 480));
      imshow("Window1", new_frame);
    }
    if(!sliding_frame.empty()) {
      resize(sliding_frame, sliding_frame, Size(640, 480));
      imshow("Window2", sliding_frame);
    }
    if(!resized_frame.empty()){
      resize(resized_frame, resized_frame, Size(640, 480));
      imshow("Window3", resized_frame);
    }

    waitKey(_delay);
  }
  clear_release();

  return SteerAngle_;
}

} /* namespace lane_detect */
