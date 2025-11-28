// #include <opencv2/opencv.hpp>
// #include <vector>

// using namespace cv;
// using namespace std;

// int main() {
//     // 1. 读取图像（替换为你的图像路径）
//     Mat img = imread("1.png");
//     if (img.empty()) {
//         cout << "图像读取失败！" << endl;
//         return -1;
//     }

//     // 2. 转换到HSV空间（便于红色区域分割）
//     Mat hsv;
//     cvtColor(img, hsv, COLOR_BGR2HSV);

//     // 3. 定义红色的HSV阈值范围（红色分两个区间，可根据实际颜色微调）
//     Scalar lower_red1(0, 120, 70);
//     Scalar upper_red1(10, 255, 255);
//     Scalar lower_red2(160, 120, 70);
//     Scalar upper_red2(180, 255, 255);

//     // 4. 二值化得到红色区域掩码
//     Mat mask1, mask2, mask;
//     inRange(hsv, lower_red1, upper_red1, mask1);
//     inRange(hsv, lower_red2, upper_red2, mask2);
//     mask = mask1 | mask2;

//     // 5. 形态学操作（去除噪声）
//     Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
//     morphologyEx(mask, mask, MORPH_OPEN, kernel);
//     morphologyEx(mask, mask, MORPH_CLOSE, kernel);

//     // 6. 查找红色区域的轮廓
//     vector<vector<Point>> contours;
//     findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

//     // 存储四个端点：左下角(1)、左上角(2)、右上角(3)、右下角(4)
//     vector<Point> markerPoints(4);
//     int validContourCount = 0; // 记录有效红色条数量（预期为2：左右各1条）

//     // 7. 遍历轮廓，获取左右红色条的“上下端点”
//     for (const auto& contour : contours) {
//         if (contourArea(contour) < 100) continue; // 过滤小噪声

//         Point topPoint = contour[0]; // 初始化为轮廓第一个点
//         Point bottomPoint = contour[0];
//         for (const auto& p : contour) {
//             if (p.y < topPoint.y) topPoint = p;     // 找“最上端点”（y坐标最小）
//             if (p.y > bottomPoint.y) bottomPoint = p; // 找“最下端点”（y坐标最大）
//         }

//         // 计算轮廓中心点，判断是“左侧”还是“右侧”红色条
//         Moments mom = moments(contour);
//         Point center(mom.m10 / mom.m00, mom.m01 / mom.m00);

//         if (center.x < img.cols / 2) { // 左侧红色条
//             markerPoints[0] = bottomPoint; // 数字1：左下端（左下角）
//             markerPoints[1] = topPoint;    // 数字2：左上端（左上角）
//         } else { // 右侧红色条
//             markerPoints[2] = topPoint;    // 数字3：右上端（右上角）
//             markerPoints[3] = bottomPoint; // 数字4：右下端（右下角）
//         }
//         validContourCount++;
//     }

//     // 8. 绘制“空心圆+数字标注”（仅当检测到2条有效红色条时执行）
//     if (validContourCount == 2) {
//         string nums[] = {"1", "4", "3", "2"}; // 逆时针标注的数字
//         for (int i = 0; i < 4; ++i) {
//             // 绘制“空心圆”：半径10，线条粗细2
//             circle(img, markerPoints[i], 10, Scalar(0, 255, 0), 2);
//             // 标注“数字”：位置在圆的右下方（偏移量可微调）
//             putText(
//                 img,                  // 目标图像
//                 nums[i],              // 要显示的数字
//                 Point(markerPoints[i].x + 15, markerPoints[i].y + 10), // 文字位置
//                 FONT_HERSHEY_SIMPLEX, // 字体样式
//                 0.8,                  // 字体大小
//                 Scalar(255, 255, 255),// 文字颜色（白色）
//                 2                     // 文字线条粗细
//             );
//         }
//     } else {
//         cout << "未检测到2条有效红色条，标注逻辑未执行！" << endl;
//     }

//     // 9. 显示结果
//     imshow("红色条端点标注结果", img);
//     waitKey(0);
//     destroyAllWindows();

//     return 0;
// }
