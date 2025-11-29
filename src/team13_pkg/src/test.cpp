#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace rclcpp;
using namespace cv;

class TestNode : public rclcpp::Node {
 public:
  TestNode(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Initializing TestNode");

    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&TestNode::callback_camera, this, std::placeholders::_1));

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);

    RCLCPP_INFO(this->get_logger(), "TestNode initialized successfully");
  }

  ~TestNode() { cv::destroyWindow("Detection Result"); }

private:
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);

  // 稳定的圆环点计算方法 - 逆时针方向：左→下→右→上
  vector<Point2f> calculateRingPoints(Point2f center, float radius);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  vector<Point2f> outer_ring_points;  // 外圆点
  vector<Point2f> inner_ring_points;  // 内圆点
};

vector<Point2f> TestNode::calculateRingPoints(Point2f center, float radius) {
    vector<Point2f> points;
    // 逆时针方向：左→下→右→上
    points.push_back(Point2f(center.x - radius, center.y));   // 左
    points.push_back(Point2f(center.x, center.y + radius));   // 下  
    points.push_back(Point2f(center.x + radius, center.y));   // 右
    points.push_back(Point2f(center.x, center.y - radius));   // 上
    return points;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>("TestNode");
  RCLCPP_INFO(node->get_logger(), "Starting TestNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

void TestNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
  try {
     cv_bridge::CvImagePtr cv_ptr;

    if (msg->encoding == "rgb8" || msg->encoding == "R8G8B8") {
      cv::Mat image(msg->height, msg->width, CV_8UC3,
                    const_cast<unsigned char *>(msg->data.data()));
      cv::Mat bgr_image;
      cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
      cv_ptr = std::make_shared<cv_bridge::CvImage>();
      cv_ptr->header = msg->header;
      cv_ptr->encoding = "bgr8";
      cv_ptr->image = bgr_image;
    } else {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    cv::Mat image = cv_ptr->image;

    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image");
      return;
    }
    
    Mat result_image=image.clone();
    Mat imagecopy=image;
    
    // 清空点集
    outer_ring_points.clear();
    inner_ring_points.clear();
    
    // 转换到 HSV 空间
    Mat hsv,maskto,masktoo,imgGray;
    cvtColor(image, hsv, COLOR_BGR2HSV);
  
    cvtColor(imagecopy,imgGray,COLOR_BGR2GRAY);
    
    Mat mask1, mask2, mask,resized_image;
    inRange(hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), mask1);
    inRange(hsv, Scalar(170, 120, 70), Scalar(180, 255, 255),mask2);
    mask = mask1 | mask2;

    // 适度的形态学操作
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(mask, maskto, MORPH_CLOSE, kernel);
    morphologyEx(maskto, masktoo, MORPH_OPEN, kernel);

    // 找轮廓
    vector<vector<Point>> contours;
    findContours(masktoo, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Point2f outer_center;
    float outer_radius = 0;
    bool outer_ring_found = false;

    for (size_t i = 0; i < contours.size(); i++) {
      double area = contourArea(contours[i]);
      if (area < 500) continue;

      // 计算最小外接圆
      Point2f center;
      float radius = 0;
      minEnclosingCircle(contours[i], center, radius);

      // 计算圆形度
      double perimeter = arcLength(contours[i], true);
      double circularity = 4 * CV_PI * area / (perimeter * perimeter);

      if (circularity > 0.85 && area > 50 && area < 20000) {
        outer_center = center;
        outer_radius = radius;
        outer_ring_points = calculateRingPoints(center, radius);
        outer_ring_found = true;

        // 绘制检测到的外圆
        circle(result_image, center, static_cast<int>(radius), Scalar(255, 255, 0), 2);  // 青色圆圈
        circle(result_image, center, 3, Scalar(0, 0, 255), -1);  // 红色圆心

        // 绘制外圆上的四个点
        vector<string> point_names = {"左", "下", "右", "上"};
        vector<cv::Scalar> point_colors = {
            cv::Scalar(255, 0, 0),    // 蓝色 - 左
            cv::Scalar(0, 255, 0),    // 绿色 - 下
            cv::Scalar(0, 255, 255),  // 黄色 - 右
            cv::Scalar(255, 0, 255)   // 紫色 - 上
        };

        for (int j = 0; j < 4; j++) {
          cv::circle(result_image, outer_ring_points[j], 2, point_colors[j], -1);
          cv::circle(result_image, outer_ring_points[j], 2, cv::Scalar(0, 0, 0), 2);

          RCLCPP_INFO(this->get_logger(),
                      "ring_red, point %d (%s): (%.1f, %.1f)",
                      j + 1, point_names[j].c_str(),
                      outer_ring_points[j].x, outer_ring_points[j].y);
        }

        string info_text = "R:" + to_string((int)radius);
        cv::putText(result_image, info_text, cv::Point(center.x - 15, center.y + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

        
      }
    }

   
    Mat maskg,maskg1,maskg2,imgBlur,maskgto,maskgtoo;
                    
    int hmin2=166,smin2=0,vmin2=0;         
    int hmax2=179,smax2=255,vmax2=255; 

    Scalar lower(hmin2,smin2,vmin2);          
    Scalar upper(hmax2,smax2,vmax2);          
    inRange(imagecopy,lower,upper,maskg);   

    Mat kernel2 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(maskg, maskgto, MORPH_CLOSE, kernel2);
    morphologyEx(maskgto, maskgtoo, MORPH_OPEN, kernel2);

    // 找轮廓
    vector<vector<Point>> contoursg;
    findContours(maskgtoo, contoursg, RETR_TREE, CHAIN_APPROX_SIMPLE);

    Point2f inner_center;
    float inner_radius = 0;
    bool inner_ring_found = false;

    for (size_t i = 0; i < contoursg.size(); i++) {
      double areag = contourArea(contoursg[i]);
      if (areag < 1000 || areag > 4000) continue;

      // 计算最小外接圆
      Point2f centerg;
      float radiusg = 0;
      minEnclosingCircle(contoursg[i], centerg, radiusg);

      // 计算圆形度
      double perimeterg = arcLength(contoursg[i], true);
      double circularityg = 4 * CV_PI * areag / (perimeterg * perimeterg);
      
      if (circularityg > 0.6 && areag > 1000 && areag < 4000) {
        inner_center = centerg;
        inner_radius = radiusg;
        inner_ring_points = calculateRingPoints(centerg, radiusg);
        inner_ring_found = true;

        // 绘制检测到的内圆
        circle(result_image, centerg, static_cast<int>(radiusg), Scalar(0, 255, 0), 2);  // 绿色圆圈
        circle(result_image, centerg, 3, Scalar(0, 0, 255), -1);  // 红色圆心

        // 绘制内圆上的四个点
        vector<string> point_namesg = {"左", "下", "右", "上"};
        vector<cv::Scalar> point_colorsg = {
            cv::Scalar(255, 0, 0),    // 蓝色 - 左
            cv::Scalar(0, 255, 0),    // 绿色 - 下
            cv::Scalar(0, 255, 255),  // 黄色 - 右
            cv::Scalar(255, 0, 255)   // 紫色 - 上
        };
        
        for (int j = 0; j < 4; j++) {
          cv::circle(result_image, inner_ring_points[j], 2, point_colorsg[j], -1);
          cv::circle(result_image, inner_ring_points[j], 2, cv::Scalar(0, 0, 0), 2);

          RCLCPP_INFO(this->get_logger(),
                      "ring_red, point %d (%s): (%.1f, %.1f)",
                      j + 1, point_namesg[j].c_str(),
                      inner_ring_points[j].x, inner_ring_points[j].y);
        }                                                  
      
        string info_text = "R:" + to_string((int)radiusg);
        cv::putText(result_image, info_text, cv::Point(centerg.x - 15, centerg.y + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

       
      }
    }
    
    imshow("Detection Result", result_image);
    cv::waitKey(1);

    // 发布消息 - 按照外圆→内圆的顺序，每个圆4个点（左→下→右→上）
    referee_pkg::msg::MultiObject msg_object;
    msg_object.header = msg->header;  // 使用图像消息的时间戳
    msg_object.num_objects = 0;

    // 外圆
    if (outer_ring_found) {
      referee_pkg::msg::Object outer_obj;
      outer_obj.target_type = "Ring_red";
      
      for (int j = 0; j < 4; j++) {
        geometry_msgs::msg::Point corner;
        corner.x = outer_ring_points[j].x;
        corner.y = outer_ring_points[j].y;
        corner.z = 0.0;
        outer_obj.corners.push_back(corner);
      }
      
      msg_object.objects.push_back(outer_obj);
      msg_object.num_objects++;
    }

    // 内圆
    if (inner_ring_found) {
      referee_pkg::msg::Object inner_obj;
      inner_obj.target_type = "Ring_red";
      
      for (int j = 0; j < 4; j++) {
        geometry_msgs::msg::Point corner;
        corner.x = inner_ring_points[j].x;
        corner.y = inner_ring_points[j].y;
        corner.z = 0.0;
        inner_obj.corners.push_back(corner);
      }
      
      msg_object.objects.push_back(inner_obj);
      msg_object.num_objects++;
    }

    if (msg_object.num_objects > 0) {
      Target_pub->publish(msg_object);
      RCLCPP_INFO(this->get_logger(), "Published %d ring_red targets",
                  msg_object.num_objects);
    }

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}
