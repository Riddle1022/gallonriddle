#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <string> // 包含字符串处理头文件
using namespace cv;
using namespace std;
// 计算两点间欧氏距离
double getDistance(const Point2f& p1, const Point2f& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}
// 提取矩形的四个角点（基于轮廓检测）
vector<Point2f> getRectCorners(const Mat& binary) {
    vector<vector<Point>> contours;
    // 提取最外层轮廓
    findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    vector<Point2f> rect_corners;
    if (!contours.empty()) {
        // 取面积最大的轮廓（目标矩形）
        int max_idx = 0;
        double max_area = contourArea(contours[0]);
        for (int i = 1; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                max_idx = i;
            }
        }
        // 多边形逼近，获取矩形的四个角点
        vector<Point> approx;
        approxPolyDP(contours[max_idx], approx, arcLength(contours[max_idx], true)*0.02, true);
        // 转换为Point2f并保留前4个点
        for (int i = 0; i < min((int)approx.size(), 4); i++) {
            rect_corners.push_back(Point2f(approx[i].x, approx[i].y));
        }
    }
    return rect_corners;
}
// 对矩形角点按左下→右下→右上→左上排序
void sortRectCorners(vector<Point2f>& corners) {
    if (corners.size() != 4) return;
    // 计算中心点
    Point2f center(0, 0);
    for (auto& p : corners) {
        center.x += p.x;
        center.y += p.y;
    }
    center.x /= 4;
    center.y /= 4;
    // 按象限和角度排序
    sort(corners.begin(), corners.end(), [&center](const Point2f& a, const Point2f& b) {
        // 计算相对于中心点的角度
        double angle_a = atan2(a.y - center.y, a.x - center.x);
        double angle_b = atan2(b.y - center.y, b.x - center.x);
        // 角度排序（左下→右下→右上→左上）
        return angle_a > angle_b;
    });
}
int main(int argc, char** argv) { // 添加命令行参数解析
    // 固定路径（直接修改此处路径即可）
    string image_path = "red_rectangle.png"; // 这里替换为你的图片实际路径
    // 打印当前使用的图片路径，方便调试
    cout << "正在处理图片：" << image_path << endl;
    // 读取图片（图一：红色矩形）
    Mat image = imread(image_path);
    if (image.empty()) {
        printf("无法读取图片！请检查路径是否正确：%s\n", image_path.c_str());
        return -1;
    }
    // 创建结果图像
    Mat result = image.clone();
    // 步骤1：颜色空间转换，提取红色区域
    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);
    // 红色的HSV范围（分两段，因为红色在HSV中是环形的）
    Scalar lower_red1(0, 120, 70);
    Scalar upper_red1(10, 255, 255);
    Scalar lower_red2(170, 120, 70);
    Scalar upper_red2(180, 255, 255);
    Mat mask1, mask2, mask;
    inRange(hsv, lower_red1, upper_red1, mask1);
    inRange(hsv, lower_red2, upper_red2, mask2);
    mask = mask1 | mask2; // 合并两个红色区域的掩码
    // 形态学操作，去除噪声
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    // 步骤2：提取矩形角点
    vector<Point2f> rect_corners = getRectCorners(mask);
    // 步骤3：角点排序
    sortRectCorners(rect_corners);
    // 步骤4：绘制角点和编号（模仿图二的标记方式）
    if (rect_corners.size() == 4) {
        // 定义颜色数组（对应图二的颜色：1-紫色，2-绿色，3-黄色，4-蓝色）
        vector<Scalar> colors = {Scalar(255, 0, 255), Scalar(0, 255, 0), Scalar(0, 255, 255), Scalar(255, 0, 0)};
        for (int i = 0; i < 4; i++) {
            // 绘制实心圆标记角点
            circle(result, rect_corners[i], 10, colors[i], -1);
            // 绘制黑色外圈
            circle(result, rect_corners[i], 10, Scalar(0, 0, 0), 2);
            // 绘制数字编号
            putText(result, to_string(i+1), 
                    Point(rect_corners[i].x - 15, rect_corners[i].y + 20),
                    FONT_HERSHEY_SIMPLEX, 1.2, Scalar(255, 255, 255), 3);
        }
        // 绘制矩形边框
        for (int i = 0; i < 4; i++) {
            line(result, rect_corners[i], rect_corners[(i+1)%4], Scalar(0, 0, 255), 2);
        }
    } else {
        printf("未检测到完整的矩形角点，仅识别到 %d 个点！\n", (int)rect_corners.size());
    }
    // 显示结果
    imshow("Original Image", image);
    imshow("Red Mask", mask);
    imshow("Detection Result", result);
     // 保存结果（保存路径与原图同目录，后缀加_result）
    string save_path = image_path.substr(0, image_path.find_last_of(".")) + "_result.png";
    imwrite(save_path, result);
    cout << "结果已保存至：" << save_path << endl;
    waitKey(0);
    destroyAllWindows();
    return 0;
}