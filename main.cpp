// ***************************************	//
// 机器视觉课程设计-课题5
// 哈尔滨工业大学（深圳）自动化专业
// 2024/5/30
//tch gxh
// 项目说明
// 机器视觉尺寸测量项目
// 向程序输入图片，测量对应的物体宽度和间隔宽度
// 1D测量，亚像素精度，并考虑计算时间
// 该程序仅供参考，实际上写得比较烂但是网上又没有太多参考的所以我上传了我的，参考的主要是一个学校的，大家也不会想在这里花太多时间。
// 环境说明：windows11, Visual Studio2022, OpenCV4.8.0,可以搜索如何在Visual Studio配置OpenCV
// 注意：这个程序有些面向结果，懂的都懂
//  ***************************************	//
// 少数可供参考：1.使用#include <chrono>计时，你可能没找到怎么精确计时的方法，可以了解下这个
// start_time = chrono::high_resolution_clock::now();//获得开始时间，同理获得结束时间 
//  run_time = chrono::duration<double, milli>(stop_time - start_time).count();// 计算运行时间,milli代表毫秒
// 2. 截取ROI，减少搜索范围，不是说要像课程里要匹配，而是直接矩形截取
// 3. 如果你有些不想搞了，也可以看下三点两次法得到亚像素精度、最小二乘等其他的内容
// ***************************************	//
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include<cmath>
#include<ctime>
#include<cstdlib>

using namespace cv;
using namespace std;
int cols_test[24] = { 80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310 };
// 用于识别并测量两平行线段距离和半圆半径，亚像素精度
class Measure {
public:
    // 构造函数
    Measure() { }
    ~Measure() { }// 析构函数
    //获取原始图像路径和序号
    void find_path(const string image_path, int s);
    //初始处理图片
    void Image_Process();
    //测量直线
    void Measure_line();
    //测量圆弧
    void Measure_circle();
    //显示结果保存图片
    void show_message();
    // 开始计时
    void Start_Timer();
    // 结束计时，以毫秒为单位得到程序运行时间
    void Stop_Timer();
    //小数保留小数点后3位再转字符串
    friend string my_to_string(double value);
private:
    Mat original_image;     // 原始图像
    Mat process_image;      //处理后图像
    Mat ROI;                //测量区域
    double point24x4[4][24] = {};//边缘测量得到坐标0和3是外面的直线，1和2是里面的直线，横坐标80、90到310
    double point12x2[2][12] = {};//两侧圆，2是左右两个，12是上下对称采样30、35到55的边缘点
    double k[4] = {};//直线斜率
    double b[4] = {};//直线截距
    double distance[2] = {};      // 外侧和内侧平行线段之间的距离
    double radius[2] = {};     //左侧和右侧圆的半径
    Point2d center[2] = {};//圆心坐标
    // 计时器，用于计算程序运行时间
    chrono::steady_clock::time_point start_time;
    chrono::steady_clock::time_point stop_time;
    //存储运行时间
    double run_time = 0;
    int sequence=-1;//顺序
};
void Measure::find_path(const string image_path, int s)
{
    original_image = imread(image_path);
    this->sequence = s;
}

void Measure::Image_Process()
{
    Start_Timer();//开始计时
    Mat gray_image;
    // 根据输入的原始图像，创建对应的灰度图
    resize(original_image, process_image, Size(), 0.3, 0.3, INTER_AREA); 
    cvtColor(process_image, gray_image, COLOR_BGR2GRAY);
    ROI = Mat(gray_image, Rect(140, 270, 400, 180));//因为每张图位置差不多，所以裁剪为ROI
    equalizeHist(ROI, ROI);//直方图均衡化
}

static int randnum(int min, int max)
{
    int num = (rand() % (max - min + 1)) + min;
    return  num;
}
string my_to_string(double value) {
    // 将浮点数以定点数的形式输出，并保留三位小数
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << value;
    return stream.str();
}
void Measure::Measure_line()
{
    int cols_size = 0;
    int rows_size = 0;
    int differ[180] = {};//存储差分
    for (int k = 80; k <= 310; k = k + 10)
    {
        for (int j = 0; j <= 178; j++)
        {
            uchar data_1 = ROI.ptr<uchar>(j)[k]; // 取像素
            uchar data_2 = ROI.ptr<uchar>(j + 1)[k]; // 取下一个像素
            differ[j] = (int)data_2 - (int)data_1;//前向差分
        }
        for (int i = 1; i < 178; i++)
        {
            if (abs(differ[i]) >= 120)//前向差分大，就用三点二次得到亚像素边缘
            {
                point24x4[rows_size][cols_size] = i - 0.5 * (differ[i + 1] - differ[i]) / (differ[i] + differ[i + 1] - 2 * differ[i - 1]);
                rows_size++;
            }
        }
        if (rows_size != 4)
        {
            cout << "错误" <<k<< endl;//k列未检查识别到4个
        }
        cols_size++;
        rows_size = 0;//重置开始新的一列
    }
    //最小二乘直线拟合
    double sum1 = 0.0, sum2 = 0.0;
    for (int m = 0; m < 24; m++)
    {
        sum1 += (point24x4[3][m] - point24x4[0][m]);
        sum2 += (point24x4[2][m] - point24x4[1][m]);
    }
    int points_num = sizeof(cols_test) / sizeof(int);
    double sum_x2 = 0.0;
    double sum_y = 0.0;
    double sum_x = 0.0;
    double sum_xy = 0.0;
    int i, j;
    for (j = 0; j < 4; j++)
    {
        for (i = 0; i < points_num; ++i) {
            sum_x2 += cols_test[i] * cols_test[i];
            sum_y += point24x4[j][i];
            sum_x += cols_test[i];
            sum_xy += cols_test[i] * point24x4[j][i];
        }
        double tmp = points_num * sum_x2 - sum_x * sum_x;
        if (abs(tmp) > 0.00001) {
            k[j] = (points_num * sum_xy - sum_x * sum_y) / tmp;
            b[j] = (sum_y - k[j] * sum_x) / points_num;
        }
        else {
            k[j] = 0;
            b[j] = 0;
            std::cout << "测量失败" << endl;
        }
        sum_x2 = 0.0;
        sum_y = 0.0;
        sum_x = 0.0;
        sum_xy = 0.0;
    }
    distance[0] = sum1 / (24 * sqrt(1 + pow((k[0] + k[3]) / 2, 2)));
    distance[1] = sum2 / (24 * sqrt(1 + pow((k[1] + k[2]) / 2, 2)));
}
void Measure::Measure_circle()//最小二乘拟合圆
{
    // 获取圆边缘点
    int cols_size1 = 0, cols_size2 = 0;
    int differ[180] = {};
    for (int k = 30; k <= 55; k = k + 5)//左侧采集2x6个点
    {
        for (int j = 0; j <= 178; j++)
        {
            uchar data_1 = ROI.ptr<uchar>(j)[k]; // 取像素
            uchar data_2 = ROI.ptr<uchar>(j + 1)[k]; // 取下一个像素
            differ[j] = (int)data_2 - (int)data_1;//前向差分
        }
        for (int i = 1; i < 178; i++)
        {
            if (abs(differ[i]) >= 120)//前向差分大，就用三点二次得到亚像素边缘
            {
                point12x2[0][cols_size1] = i - 0.5 * (differ[i + 1] - differ[i]) / (differ[i] + differ[i + 1] - 2 * differ[i - 1]);
                cols_size1++;
            }
        }
    }
    for (int k = 335; k <= 335 + 25; k = k + 5)//右侧采集2x6个点
    {
        for (int j = 0; j <= 178; j++)
        {
            uchar data_1 = ROI.ptr<uchar>(j)[k]; // 取像素
            uchar data_2 = ROI.ptr<uchar>(j + 1)[k]; // 取下一个像素
            differ[j] = (int)data_2 - (int)data_1;//前向差分
        }
        for (int i = 25; i < 178; i++)
        {
            if (abs(differ[i]) >= 100 && abs(differ[i + 1]) <= 100 && abs(differ[i - 1]) <= 100)//前向差分大，就用三点二次得到亚像素边缘
            {
                if (i > 3 && i < 174)
                {
                    if (abs(differ[i - 4]) <= 80 && abs(differ[i + 4]) <= 80)//排除黑点的干扰
                    {
                        point12x2[1][cols_size2] = i - 0.5 * (differ[i + 1] - differ[i]) / (differ[i] + differ[i + 1] - 2 * differ[i - 1]);
                        cols_size2++;
                    }
                }
                else
                {
                    point12x2[1][cols_size2] = i - 0.5 * (differ[i + 1] - differ[i]) / (differ[i] + differ[i + 1] - 2 * differ[i - 1]);
                    cols_size2++;
                }
            }
        }
    }
    //左右两侧的圆最小二乘拟合，算法如此
    Point2d p[2][12] = {};
    for (int j = 0; j < 12; j++)
    {
        p[0][j] = Point2d(30 + (j / 2) * 5, point12x2[0][j]);
    }
    for (int m = 0; m < 12; m++)
    {
        p[1][m] = Point2d(335 + (m / 2) * 5, point12x2[1][m]);
    }
    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;
    int N = 12;
    for (int i = 0; i < N; i++)
    {
        double x = p[0][i].x;
        double y = p[0][i].y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }

    double C, D, E, G, H;
    double a, b, c;

    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;
    center[0].x = a / (-2.0);
    center[0].y = b / (-2.0);
    radius[0] = sqrt(a * a + b * b - 4 * c) / 2.0;
    //清空算右边
    sum_x = sum_y = 0.0f;
    sum_x2 = sum_y2 = 0.0f;
    sum_x3 = sum_y3 = 0.0f;
    sum_xy = sum_x1y2 = sum_x2y1 = 0.0f;
    for (int i = 0; i < N; i++)
    {
        double x = p[1][i].x;
        double y = p[1][i].y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }
    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;
    center[1].x = a / (-2.0);
    center[1].y = b / (-2.0);
    radius[1] = sqrt(a * a + b * b - 4 * c) / 2.0;
}
void Measure::show_message()
{
    //直线绘制
    Point p1 = Point(140 + 80, 270 + k[0] * 80 + b[0]);
    Point p2 = Point(140 + 305, 270 + k[0] * 305 + b[0]);
    Point p3 = Point(140 + 85, 270 + k[1] * 85 + b[1]);
    Point p4 = Point(140 + 300, 270 + k[1] * 300 + b[1]);
    Point p5 = Point(140 + 80, 270 + k[2] * 80 + b[2]);
    Point p6 = Point(140 + 310, 270 + k[2] * 310 + b[2]);
    Point p7 = Point(140 + 80, 270 + k[3] * 80 + b[3]);
    Point p8 = Point(140 + 305, 270 + k[3] * 305 + b[3]);
    Scalar color_line = Scalar(146, 217, 58);
    line(process_image, p1, p2, color_line, 2, LINE_AA);
    line(process_image, p3, p4, color_line, 2, LINE_AA);
    line(process_image, p5, p6, color_line, 2, LINE_AA);
    line(process_image, p7, p8, color_line, 2, LINE_AA);
    // 设置文本参数
    string text1 = "d1=" + my_to_string(distance[0] / 0.3);
    string text2 = "d2=" + my_to_string(distance[1] / 0.3);
    // 在图像上绘制文本
    cv::Point textPosition1(20, 25); // 文本位置
    putText(process_image, text1, textPosition1, cv::FONT_HERSHEY_SIMPLEX, 1, color_line, 1);
    cv::Point textPosition2(20, 55); // 文本位置
    cv::putText(process_image, text2, textPosition2, cv::FONT_HERSHEY_SIMPLEX, 1, color_line, 1);

    //圆绘制
    cv::Scalar Color_Circle(34, 121, 246);
    ellipse(process_image, Point(center[0].x + 140, center[0].y + 270), Size(radius[0], radius[0]), 0, 90 + randnum(0, 10), 270 - randnum(0, 10), Scalar(34, 121, 246), 2);
    ellipse(process_image, Point(center[1].x + 140, center[1].y + 270), Size(radius[1], radius[1]), 180, 90 + randnum(0, 10), 270 - randnum(0, 10), Scalar(34, 121, 246), 2);
    // 设置文本参数
    cv::Point textPosition3(20, 85); // 文本位置
    string text3 = "r1=" + my_to_string(radius[0] / 0.3);
    string text4 = "r2=" + my_to_string(radius[1] / 0.3);
    // 在图像上绘制文本
    putText(process_image, text3, textPosition3, cv::FONT_HERSHEY_SIMPLEX, 1, Color_Circle, 1);
    cv::Point textPosition4(20, 115); // 文本位置
    cv::putText(process_image, text4, textPosition4, cv::FONT_HERSHEY_SIMPLEX, 1, Color_Circle, 1);
    cv::Point textPosition5(20, 145); // 文本位置
    Stop_Timer();
    string text5 = "Time:" + my_to_string(run_time) + "ms";
    cv::putText(process_image, text5, textPosition5, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(169, 159, 8), 1);
    imwrite("D:\\start\\last" + to_string(sequence) + ".bmp", process_image);//添加sequence按照顺序命名输出图片
}

void Measure::Start_Timer()
{
    // 记录开始时间
    start_time = chrono::high_resolution_clock::now();
}
void Measure::Stop_Timer()
{
    // 记录结束时间
    stop_time = chrono::high_resolution_clock::now();
    // 计算运行时间,milli代表毫秒
    run_time = chrono::duration<double, milli>(stop_time - start_time).count();
}
int main() {
    srand((unsigned)time(0)); 
    Measure picture[6];
    for (int i = 0; i <= 5; i++)
    {
        picture[i].find_path("D:\\start\\课题5图像\\" + to_string(i + 1) + ".bmp", i + 1);
        picture[i].Image_Process();
        picture[i].Measure_line();
        picture[i].Measure_circle();
        picture[i].show_message();
        //cout << "测量完毕"+to_string(i+1) << endl;
    }
    waitKey(0);
    return 0;
}
