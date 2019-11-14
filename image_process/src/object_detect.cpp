#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

static const string origin_image = "origin_image";
static const string result_image = "result_image";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/object_detect/output_video", 1);
    }

    ~ImageConverter()
    {
        // cv::destroyWindow(origin_image);
        // cv::destroyWindow(result_image);
    }

    void saveImage(cv::Mat image, int index)
    {
        //定义保存图像的名字
        string strSaveName;
        char buffer[256];
        sprintf(buffer, "D%04d", index);
        strSaveName = buffer;

        //定义保存图像的完整路径
        string strImgSavePath = strSaveName;
        //定义保存图像的格式
        strImgSavePath += ".jpg";

        //保存操作
        imwrite(strImgSavePath.c_str(), image);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        static long i = 0;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw an example circle on the video stream
        // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        //     cv::circle(cv_ptr->image, cv::Point(50, 50), 30, CV_RGB(255, 0, 0), -1);

        Mat edge_image, gray_image;
        namedWindow(origin_image);
        imshow(origin_image, cv_ptr->image);
        saveImage(cv_ptr->image, i);
        i++;
        //imwrite("1.jpg", cv_ptr->image);
        // GaussianBlur(cv_ptr->image, gray_image, Size(3, 3), 0, 0); //高斯模糊，使平滑

        // cvtColor(gray_image, gray_image, CV_BGR2GRAY);

        // Canny(gray_image, edge_image, 50, 150);

        //Laplacian(gray_image, edge_image, CV_16S, 3);

        //convertScaleAbs(edge_image, edge_image);

        //threshold(edge_image, edge_image, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);

        // Update GUI Window
        // namedWindow(result_image);
        // imshow(result_image, edge_image);
        waitKey(3);
        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detect");
    ImageConverter ic;
    ros::spin();
    return 0;
}
