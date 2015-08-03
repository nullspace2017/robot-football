#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;

vector<Point2f> imgPos;
vector<Point2f> realPos;
Mat transMat;

void onMouse(int event, int x, int y, int, void*) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        cout << "第" << imgPos.size() + 1 << "点:" << endl;
        cout << "图像坐标: ";
        cout << x << ' ' << y << endl;
        // cin >> x >> y;
        int realX, realY;
        cout << "真实坐标: ";
        cin >> realX >> realY;
        cout << "确定？(y)\t";
        char sure;
        cin >> sure;
        if (sure == 'y') {
            imgPos.push_back(Point2f(x, y));
            realPos.push_back(Point2f(realX, realY));
        }
    }
}

void sample() {
    VideoCapture cap(1);
    Mat mat;
    Mat edges;
    namedWindow("mat");
    setMouseCallback("mat", onMouse, NULL);
    while (true) {
        cap >> mat;
        line(mat, Point(0, 400), Point(640, 400), Scalar(255, 0, 0));
        imshow("mat", mat);      
        waitKey(30);
        if (imgPos.size() > 10) {
            int key = waitKey(0);
            if (key == 'q')
                break;
        }
    }
    imwrite("img.png", mat);
}

void makeMatViaRes(Mat& A, Mat& U, vector<Point2f>& ImagPosition, vector<Point2f>& RealPosition, int pointsNum)
{
	int i, j;
	double points_u[pointsNum];
	double points_y[pointsNum];
	double points_x[pointsNum];
	double points_v[pointsNum];
	
	for(i = 0; i < pointsNum; i++)
	{
		points_u[i] = (double)ImagPosition[i].x;
		points_v[i] = (double)ImagPosition[i].y;
		points_x[i] = (double)RealPosition[i].x;
		points_y[i] = (double)RealPosition[i].y;
	}

	for(i = 0; i < pointsNum; i++)
	{
        A.at<double>(i, 0) = points_x[i];
        A.at<double>(i, 1) = points_y[i];
        A.at<double>(i, 2) = 1;
        A.at<double>(i, 6) = - points_u[i] * points_x[i];
        A.at<double>(i, 7) = - points_u[i] * points_y[i];

        U.at<double>(i, 0) = points_u[i];
	}
	for(i = pointsNum; i < 2 * pointsNum; i++)
	{
        A.at<double>(i, 3) = points_x[i - pointsNum];
        A.at<double>(i, 4) = points_y[i - pointsNum];
        A.at<double>(i, 5) = 1;
        A.at<double>(i, 6) = - points_v[i - pointsNum] * points_x[i - pointsNum];
        A.at<double>(i, 7) = - points_v[i - pointsNum] * points_y[i - pointsNum];

        U.at<double>(i, 0) = points_v[i - pointsNum];
	}
	return;
}

void printMat(const Mat& mat) {
    for (int i = 0; i < mat.rows; i ++) {
        for (int j = 0; j < mat.cols; j ++) {
            cout << mat.at<double>(i, j) << ' ';
        }
        cout << endl;
    }
    cout << endl;
}

void calTransMat(Mat& transMat, int pointsNum, vector<Point2f>& ImagPosition, vector<Point2f>& RealPosition)
{
	Mat A = Mat::zeros(2 * pointsNum, 8, CV_64F);
	Mat U = Mat::zeros(2 * pointsNum, 1, CV_64F);
    Mat transA = Mat::zeros(8, 2 * pointsNum, CV_64F);       // A转置
    Mat inv_AtA = Mat::zeros(8, 8, CV_64F);      // (ATA)-1，(A转置×A)逆

	makeMatViaRes(A, U, ImagPosition, RealPosition, pointsNum);

    transpose(A, transA);       // AT

    Mat tmp = Mat::zeros(8, 8, CV_64F);   // AT*A
    tmp = transA * A;

    invert(tmp, inv_AtA);       // (AtA)^-1

    tmp = Mat::zeros(8, 2 * pointsNum, CV_64F);
    tmp = inv_AtA * transA;
    
    transMat = tmp * U;
    
    cout << "TranMat:" << endl;
    printMat(transMat);
	return;
}

void drawPoint() {
    Mat img(480, 640, CV_8UC3, Scalar(255, 255, 255));
    Mat real(480, 640, CV_8UC3, Scalar(255, 255, 255));
    for (int i = 0; i < imgPos.size(); i ++) {
        circle(img, imgPos[i], 2, Scalar(0, 0, 255), -1);
        Point2f point(realPos[i].x*2, realPos[i].y*2);
        int newJ = point.x + 400;
        int newI = - point.y + 500;
        circle(real, Point2f(newJ, newI), 2, Scalar(255, 0, 0), -1);
    }
    imshow("img", img);
    waitKey(30);
    imshow("real", real);
    waitKey(30);
}

void transform(Point2f& imgPoint, Point2f& realPoint) {
    double m[transMat.rows];
    for (int i = 0; i < transMat.rows; i ++) {
        m[i] = transMat.at<double>(i, 0);
    }
    // double m[] = {67.9222, 39.5566, -296.418, -2.56597, 7.31207, 1948.82, -0.0131613, 0.100575};

    double u = imgPoint.x, v = imgPoint.y;
    double tmp1 = (m[4] - m[7] * v) * (m[2] - u);
    double tmp2 = (m[1] - m[7] * u) * (m[5] - v);
    double x = tmp1 - tmp2;
    tmp1 = (m[4] - m[7] * v) * (m[0] - m[6] * u);
    tmp2 = (m[1] - m[7] * u) * (m[3] - m[6] * v);
    x = - x / (tmp1 - tmp2);

    tmp1 = (m[3] - m[6] * v) * (m[2] - u);
    tmp2 = (m[0] - m[6] * u) * (m[5] - v);
    double y = tmp1 - tmp2;
    tmp1 = (m[3] - m[6] * v) * (m[1] - m[7] * u);
    tmp2 = (m[0] - m[6] * u) * (m[4] - m[7] * v);
    y = - y / (tmp1 - tmp2);

    realPoint.x = x;
    realPoint.y = y;
}


void trans(Mat& img, Mat& transImg) {
    vector<Point2f> realPoints;
    vector<Vec3b> realScalar;
    Point2f imgPoint;
    for (int i = 0; i < img.rows; i ++) {
        for (int j = 0; j < img.cols; j ++) {
            imgPoint.x = j;
            imgPoint.y = i;
            Point2f realPoint;
            transform(imgPoint, realPoint);
            realPoints.push_back(realPoint);
            realScalar.push_back(img.at<Vec3b>(i, j));
            }
    }
    
    for (int i = 0; i < realPoints.size(); i ++) {
        Point2f point(realPoints[i].x*2, realPoints[i].y*2);
        int newJ = point.x + 400;
        int newI = - point.y + 500;
        if (newJ > 0 && newI > 0 && newI < transImg.rows && newJ < transImg.cols) {
            transImg.at<Vec3b>(newI, newJ) = realScalar[i];
        }
    }
}

void test() {
    drawPoint();
    Mat mat = imread("img.png");
    Mat transMat(mat.rows, mat.cols, CV_8UC3, Scalar(0, 0, 0));
    trans(mat, transMat);
    imshow("mat", mat);
    waitKey(30);
    imshow("trans", transMat);
    waitKey(0);
}


int main() {
    sample();
    transMat = Mat::zeros(8, 1, CV_64F);
    calTransMat(transMat, realPos.size(), imgPos, realPos);
    test();
    return 0;
}



