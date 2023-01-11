#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include <iostream>
#include <sstream>

const int FRAME_WIDTH = 2560;
const int FRAME_HEIGHT = 720;

const cv::Size displaySize(1280, 480);
const cv::Size singleDisplaySize(320, 240);

const std::string intrinsic_filename = "intrinsics.yml";
const std::string extrinsic_filename = "extrinsics.yml";

const  enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4, STEREO_HH4=5 };


cv::Mat xyz;
cv::Mat floatDisp;


static void stereo_match(int algorithm, cv::Mat img_left, cv::Mat img_right, cv::Mat& out_stereo_frame) {
    cv::Mat m_img_left;
    cv::Mat m_img_right;
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16,9);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);

    int numberOfDisparities = 256;  // max-disparity
    int SADWindowSize = 0;          // blocksize
    // int color_mode = algorithm == STEREO_BM ? 0 : -1;

    cv::cvtColor(img_left, m_img_left, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_right, m_img_right, cv::COLOR_BGR2GRAY);

    float scale = 1;
    cv::Size img_size = m_img_left.size();
    cv::Rect roi1, roi2;
    cv::Mat Q;  // 深度视差映射矩阵


    // load intrinsic parameters
    cv::FileStorage fs(intrinsic_filename, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", intrinsic_filename.c_str());
        return;
    }

    cv::Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    M1 *= scale;
    M2 *= scale;

    // load extrinsic parameters
    fs.open(extrinsic_filename, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", extrinsic_filename.c_str());
        return;
    }

    cv::Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

    cv::Mat map11, map12, map21, map22;
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

    cv::Mat img1r, img2r;
    remap(m_img_left, img1r, map11, map12, cv::INTER_LINEAR);
    remap(m_img_right, img2r, map21, map22, cv::INTER_LINEAR);

    m_img_left = img1r;
    m_img_right = img2r;

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = m_img_left.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    if(algorithm==STEREO_HH)
        sgbm->setMode(cv::StereoSGBM::MODE_HH);
    else if(algorithm==STEREO_SGBM)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
    else if(algorithm==STEREO_HH4)
        sgbm->setMode(cv::StereoSGBM::MODE_HH4);
    else if(algorithm==STEREO_3WAY)
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    cv::Mat disp, disp8;

    int64 t = cv::getTickCount();
    float disparity_multiplier = 1.0f;
    if( algorithm == STEREO_BM )
    {
        bm->compute(m_img_left, m_img_right, disp);
        if (disp.type() == CV_16S)
            disparity_multiplier = 16.0f;
    }
    else if( algorithm == STEREO_SGBM || algorithm == STEREO_HH || algorithm == STEREO_HH4 || algorithm == STEREO_3WAY )
    {
        sgbm->compute(m_img_left, m_img_right, disp);
        if (disp.type() == CV_16S)
            disparity_multiplier = 16.0f;
    }
    t = cv::getTickCount() - t;
    // printf("Time elapsed: %fms\n", t*1000/cv::getTickFrequency());

    if( algorithm != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);

    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
    cv::Mat disp8_3c;
    cv::applyColorMap(disp8, disp8_3c, cv::COLORMAP_TURBO);


    disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
    reprojectImageTo3D(floatDisp, xyz, Q, true);

    out_stereo_frame = disp8_3c;
}


cv::Vec3f point3;
float d;
cv::Point origin;   // 鼠标按下起始点
cv::Rect selection;     // 自定义矩形
bool selectObject = false; // 是否选择对象

/*鼠标点击事件处理*/
static void onMouse(int event, int x, int y, int, void*) {
    if (selectObject) {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
    }

    switch (event) {
    case cv::EVENT_LBUTTONDOWN:
        origin = cv::Point(x, y);
        selection = cv::Rect(x, y, 0, 0);
        selectObject = true;
        point3 = xyz.at<cv::Vec3f>(origin);
        point3[0];

        std::cout << "============================" << std::endl;
        std::cout << "world coordinate: " << std::endl;
        std::cout << "x: " << point3[0] << "y: " << point3[1] << "z: " << point3[2] << std::endl;
        d = point3[0] * point3[0] + point3[1] * point3[1] + point3[2] * point3[2];
        d = sqrt(d);    // mm

        d = d / 10.0;   // cm
        std::cout << "range: " << d << "cm" << std::endl;
        std::cout << "============================" << std::endl;

        break;

    case cv::EVENT_LBUTTONUP:
        selectObject = false;
        if (selection.width > 0 && selection.height > 0)
            break;
    }
}

int main(int argc, char** argv)
{
    cv::Mat frame_left;
    cv::Mat frame_right;
    cv::Mat frame;
    cv::VideoCapture cap;
    cap.open(1);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    if (!cap.isOpened())
        return -1;

    cv::Mat resize_frame_left;
    cv::Mat resize_frame_right;
    cv::Mat resizedFrame;

    cv::Mat stereo_match_frame; // 实时深度视差图

    while (cap.read(frame)) {
        frame_left = frame(cv::Rect(0, 0, int(FRAME_WIDTH/2), FRAME_HEIGHT));
        frame_right = frame(cv::Rect(int(FRAME_WIDTH/2), 0, int(FRAME_WIDTH/2), FRAME_HEIGHT));
        cv::resize(frame_left, resize_frame_left, singleDisplaySize);
        cv::resize(frame_right, resize_frame_right, singleDisplaySize);

        stereo_match(STEREO_3WAY, frame_left, frame_right, stereo_match_frame); // 立体匹配

        cv::resize(frame, resizedFrame, displaySize);
        // cv::imshow("setero_match", resizedFrame);
        cv::imshow("stereo", stereo_match_frame);
        cv::imshow("left", resize_frame_left);
        cv::imshow("right", resize_frame_right);

        cv::setMouseCallback("stereo", onMouse, 0);

        if (cv::waitKey(30) == 27)
            break;
    }

    cv::destroyAllWindows();
    cap.release();

    return 0;
}