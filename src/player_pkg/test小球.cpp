#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
using namespace std;
using namespace cv;

vector<Point2f> calculateStableSpherePoints(Point2f center, float radius) {
    vector<Point2f> points;
    points.push_back(Point2f(center.x - radius, center.y));   // 左
    points.push_back(Point2f(center.x, center.y + radius));   // 下
    points.push_back(Point2f(center.x + radius, center.y));   // 右
    points.push_back(Point2f(center.x, center.y - radius));   // 上
    return points;}
int main(){
    string path="1764246172194.png";
    Mat image=imread(path);
    Mat result_image=image.clone();
// 转换到 HSV 空间
    Mat hsv,maskto,masktoo;
    cvtColor(image, hsv, COLOR_BGR2HSV);

    // 红色检测 - 使用稳定的范围
    Mat mask1, mask2, mask,resized_image;
    inRange(hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), mask1);
    inRange(hsv, Scalar(170, 120, 70), Scalar(180, 255, 255),mask2);
    mask = mask1 | mask2;


    // 适度的形态学操作
    Mat kernel =
        getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(mask, maskto, MORPH_CLOSE, kernel);
    morphologyEx(maskto, masktoo, MORPH_OPEN, kernel);

    // 找轮廓
    vector<vector<Point>> contours;
    findContours(masktoo, contours, RETR_EXTERNAL,
                     CHAIN_APPROX_SIMPLE);

    //Point_V.clear();
    int valid_spheres = 0;

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

      if (circularity > 0.85 && radius > 50 && radius < 20000) {
        vector<Point2f> sphere_points =
            calculateStableSpherePoints(center, radius);

        // 绘制检测到的球体
        circle(result_image, center, static_cast<int>(radius),
                   Scalar(0, 255, 0), 2);  // 绿色圆圈
        circle(result_image, center, 3, Scalar(0, 0, 255),
                   -1);  // 红色圆心

        // 绘制球体上的四个点
        vector<string> point_names = {"左", "下", "右", "上"};
        vector<cv::Scalar> point_colors = {
            cv::Scalar(255, 0, 0),    // 蓝色 - 左
            cv::Scalar(0, 255, 0),    // 绿色 - 下
            cv::Scalar(0, 255, 255),  // 黄色 - 右
            cv::Scalar(255, 0, 255)   // 紫色 - 上
        };

        for (int j = 0; j < 4; j++) {
          cv::circle(result_image, sphere_points[j], 18, point_colors[j], -1);
          cv::circle(result_image, sphere_points[j], 18, cv::Scalar(0, 0, 0), 2);

          // 标注序号
          string point_text = to_string(j + 1);
          cv::putText(
              result_image, point_text,
              cv::Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
              cv::FONT_HERSHEY_SIMPLEX, 3.6, cv::Scalar(255, 255, 255), 3);
          cv::putText(
              result_image, point_text,
              cv::Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
              cv::FONT_HERSHEY_SIMPLEX, 3.6, point_colors[j], 2);}
              valid_spheres++;
              
    
      }
    }

    // 显示结果图像
    cv::resize(result_image,resized_image,Size(960,480));
    cv::imshow("Detection Result", resized_image);
    cv::waitKey(0);
       return 0;
    }