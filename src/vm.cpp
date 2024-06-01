#include "avoid_vm/vm.hpp"
VM::VM() : Node("mysub")
{
  writer1.open("avoid123.mp4", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 30, cv::Size(500, 500));
  lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&VM::scanCb, this, _1));

  pub_ = this->create_publisher<std_msgs::msg::Int32>("err", rclcpp::SensorDataQoS());
  timer_ = this->create_wall_timer(100ms, std::bind(&VM::publish_msg, this));
}

void VM::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  int count = scan->scan_time / scan->time_increment;
  int x, y, rmindex, lmindex;
  float rx, ry, lx, ly, rw, rh, lw, lh, clx, cly, crx, cry, rdmin, ldmin, rtheta, ltheta;
  cv::Mat img(LENGTH, LENGTH, CV_8UC3, cv::Scalar(255, 255, 255));
  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    // printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
    x = LENGTH/2 + scan->ranges[i]*XXX * sin(degree*M_PI/180);
    y = LENGTH/2 + scan->ranges[i]*XXX * cos(degree*M_PI/180);
    cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), 2);
  }
  cv::Mat grayl, grayr;
  cv::Mat ROIL = img(cv::Rect(ROI, ROI, LENGTH/2-ROI, LENGTH/2-ROI));
  cv::Mat ROIR = img(cv::Rect(LENGTH/2-1, ROI, LENGTH/2-ROI-1, LENGTH/2-ROI));
	cv::cvtColor(ROIL, grayl, cv::COLOR_BGR2GRAY);
  cv::cvtColor(ROIR, grayr, cv::COLOR_BGR2GRAY);
  cv::Mat binl, binr;
	cv::threshold(grayl, binl, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
  cv::threshold(grayr, binr, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
	cv::Mat labelsl, statsl, centroidsl;
  cv::Mat labelsr, statsr, centroidsr;
	int cntl = cv::connectedComponentsWithStats(binl, labelsl, statsl, centroidsl); // 레이블 영역의 통계, 무게 중심 좌표 정보
  int cntr = cv::connectedComponentsWithStats(binr, labelsr, statsr, centroidsr); // 레이블 영역의 통계, 무게 중심 좌표 정보

	cv::Mat dstl, dstr;
	cv::cvtColor(binl, dstl, cv::COLOR_GRAY2BGR);
  cv::cvtColor(binr, dstr, cv::COLOR_GRAY2BGR);
	// 각 객체 영역에 바운딩 박스 표시하기
  cv::circle(img, cv::Point(LENGTH/2, LENGTH/2), 2, cv::Scalar(0, 0, 0), 2);
  ldmin = sqrt((LENGTH/2-ROI-statsl.at<int>(1, 0)-statsl.at<int>(1, 2))*(LENGTH/2-ROI-statsl.at<int>(1, 0)-statsl.at<int>(1, 2))+(LENGTH/2-ROI-statsl.at<int>(1, 1)-statsl.at<int>(1, 3))*(LENGTH/2-ROI-statsl.at<int>(1, 1)-statsl.at<int>(1, 3)));
  rdmin = sqrt(statsr.at<int>(1, 0)*(statsr.at<int>(1, 0))+(LENGTH/2-ROI-statsr.at<int>(1, 1)-statsr.at<int>(1, 3))*(LENGTH/2-ROI-statsr.at<int>(1, 1)-statsr.at<int>(1, 3)));
	for (int i = 1;i < cntl;i++) {
		int* p = statsl.ptr<int>(i);
    if (p[4] < 20) continue; 
		cv::rectangle(img, cv::Rect(p[0]+ROI, p[1]+ROI, p[2], p[3]), cv::Scalar(255, 0, 0)), 2; // x,y,가로,세로 크기
    if(ldmin > sqrt((LENGTH/2-ROI-p[0]-p[2])*(LENGTH/2-ROI-p[0]-p[2])+(LENGTH/2-ROI-p[1]-p[3])*(LENGTH/2-ROI-p[1]-p[3]))){
      ldmin = sqrt((LENGTH/2-ROI-p[0]-p[2])*(LENGTH/2-ROI-p[0]-p[2])+(LENGTH/2-ROI-p[1]-p[3])*(LENGTH/2-ROI-p[1]-p[3]));
      lx = p[0];
      ly = p[1];
      lw = p[2];
      lh = p[3];
    }  
    
	}
  for (int j = 1;j < cntr;j++) {
		int* p = statsr.ptr<int>(j);
    if (p[4] < 20) continue; 
		cv::rectangle(img, cv::Rect(p[0]+LENGTH/2, p[1]+ROI, p[2], p[3]), cv::Scalar(255, 0, 255)), 2; // x,y,가로,세로 크기
    if(rdmin > sqrt(p[0]*p[0]+(LENGTH/2-ROI-p[1]-p[3])*(LENGTH/2-ROI-p[1]-p[3]))){
      rdmin = sqrt(p[0]*p[0]+(LENGTH/2-ROI-p[1]-p[3])*(LENGTH/2-ROI-p[1]-p[3]));
      rx = p[0];
      ry = p[1];
      rw = p[2];
      rh = p[3];
    } 
	}
  if(cntl == 1&&cntr != 1){
    cv::line(img, cv::Point(LENGTH/2, LENGTH/2), cv::Point(200, LENGTH/2), cv::Scalar(255, 0, 0), 3);
    cv::line(img, cv::Point(LENGTH/2, LENGTH/2), cv::Point(rx+LENGTH/2, ry+rh+ROI), cv::Scalar(255, 255, 0), 3);
    ltheta = 0;
    rtheta = atan((400-ROI-ry-rh)/(rx+0.0000001));
    ldmin = 200;
  }
  else if(cntr == 1&&cntl != 1){
    cv::line(img, cv::Point(LENGTH/2, LENGTH/2), cv::Point(600, LENGTH/2), cv::Scalar(255, 255, 0), 3);
    cv::line(img, cv::Point(LENGTH/2, LENGTH/2), cv::Point(lx+lw+ROI, ly+lh+ROI), cv::Scalar(255, 0, 0), 3);
    ltheta = atan((400-ROI-ly-lh)/(400-ROI-lx-lw+0.0000001));
    rtheta = 0;
    rdmin = 200;
  }
  else if(cntl == 1&&cntr == 1){
    cv::line(img, cv::Point(LENGTH/2, LENGTH/2), cv::Point(200, LENGTH/2), cv::Scalar(255, 0, 0), 3);
    cv::line(img, cv::Point(LENGTH/2, LENGTH/2), cv::Point(600, LENGTH/2), cv::Scalar(255, 255, 0), 3);
    ltheta = 0;
    rtheta = 0;
    ldmin = 200;
    rdmin = 200;
  }
  else{
    cv::line(img, cv::Point(LENGTH/2, LENGTH/2), cv::Point(rx+LENGTH/2, ry+rh+ROI), cv::Scalar(255, 255, 0), 3);
    cv::line(img, cv::Point(LENGTH/2, LENGTH/2), cv::Point(lx+lw+ROI, ly+lh+ROI), cv::Scalar(255, 0, 0), 3);
    ltheta = atan((400-ROI-ly-lh)/(400-ROI-lx-lw+0.0000001));
    rtheta = atan((400-ROI-ry-rh)/(rx+0.0000001));
  }
  // printf("rx = %f, ry = %f\n", rx+LENGTH/2,ry+rh+50);
  // cv::line(img, cv::Point(LENGTH/2, LENGTH/2), cv::Point(400-200*sin(rtheta/2)+200*sin(ltheta/2), 200+abs(200*sin(rtheta/2)-sin(ltheta/2))), cv::Scalar(0, 0, 0), 2);
  cv::arrowedLine(img, cv::Point(LENGTH/2, LENGTH/2), cv::Point(400-200*sin(rtheta/2-ltheta/2), 400-200*cos(rtheta/2-ltheta/2)), cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
  cv::drawMarker(img, cv::Point(400, 400), cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 40);
  //cv::circle(img, cv::Point(400, 400), 200, cv::Scalar(0, 0, 255), 2);
  printf("ltheta = %f, rtheta = %f\n", RAD2DEG(ltheta), RAD2DEG(rtheta));
  printf("ldmin = %f, rdmin = %f\n", ldmin, rdmin);
  //printf("rx = %f, ry = %f\n", 400-ROI-lx-lw, rx);
  printf("cntl = %d, cntr = %d\n", cntl, cntr);
  err = (RAD2DEG(ltheta)-RAD2DEG(rtheta))/2;
  //if((400-ROI-lx-lw) == 0 && rx == 0) err = 0;
  //if(180-RAD2DEG(ltheta)-RAD2DEG(rtheta)<15&&ldmin <= 50&&rdmin <= 50)err = 100;
  if(ldmin <= 50&&rdmin <= 50&&RAD2DEG(ltheta)>80&&RAD2DEG(rtheta)>80)err = 100;
  else if(ldmin <= 30&&rdmin > 30)err = 200;
  else if(rdmin <= 30&&ldmin > 30)err = 300;
  else if(ldmin <= 30&&rdmin <= 30&&RAD2DEG(ltheta)>RAD2DEG(rtheta))err = 400;
  else if(ldmin <= 30&&rdmin <= 30&&RAD2DEG(ltheta)<RAD2DEG(rtheta))err = 500;
  //err1=RAD2DEG(acos(200*cos(rtheta/2-ltheta/2)/200));
  printf("err = %d\n",err);
  writer1 << img;
  cv::imshow("img", img);
  //cv::imshow("img1", binl);
  //cv::imshow("img2", binr);
  cv::waitKey(1);
}
void VM::publish_msg()
{
  intmsg.data = err;
	//RCLCPP_INFO(this->get_logger(), "Publish: %d", intmsg.data);
	pub_->publish(intmsg);
}