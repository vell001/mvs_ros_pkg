#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

string image_file = "../distorted2.jpg";   // 请确保路径正确

double camera_matrix[9] = {
856.26773,   0.     , 634.35543,
           0.     , 856.78936, 502.39862,
           0.     ,   0.     ,   1.     
        };

double distortion_coefficients[5] = {
         -0.131818, 0.103490, -0.004228, 0.005060, 0.000000
        };

int main(int argc, char **argv) {

  // 本程序实现去畸变部分的代码。尽管我们可以调用OpenCV的去畸变，但自己实现一遍有助于理解。
  // 畸变参数
  double k1 =distortion_coefficients[0], k2 =distortion_coefficients[1], p1 =distortion_coefficients[2], p2 = distortion_coefficients[3];
  // 内参
  double fx = camera_matrix[0], fy =  camera_matrix[4], cx =  camera_matrix[2], cy =  camera_matrix[5];

  cv::Mat image = cv::imread(image_file, 0);   // 图像是灰度图
  int rows = image.rows, cols = image.cols;
  cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);   // 去畸变以后的图  
  cout << "col = " << cols << "row = " << rows << endl;

  // 计算去畸变后图像的内容
  for (int v = 0; v < rows; v++) {
    for (int u = 0; u < cols; u++) {
      // 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted, v_distorted)
      double x = (u - cx) / fx, y = (v - cy) / fy;
      double r = sqrt(x * x + y * y);
      double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
      double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
      double u_distorted = fx * x_distorted + cx;
      double v_distorted = fy * y_distorted + cy;

      // 赋值 (最近邻插值)
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
        image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
      } else {
        image_undistort.at<uchar>(v, u) = 0;
      }
    }
  }

  // 画图去畸变后图像
  cv::imshow("distorted", image);
  cv::imshow("undistorted", image_undistort);
  cv::waitKey();
  return 0;
}
