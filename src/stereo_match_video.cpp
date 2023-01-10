#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

const std::string intrinsic_filename = "intrinsics.yml";
const std::string extrinsic_filename = "extrinsics.yml";

const  enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4, STEREO_HH4=5 };

int main(int argc, char** argv)
{
    cv::Mat frame;
    cv::VideoCapture cap;
    cap.open(1);

    if (!cap.isOpened())
        return -1;

    while (cap.read(frame)) {
        cv::imshow("setero_match", frame);

        if (cv::waitKey(30) == 27)
            break;
    }

    return 0;
}