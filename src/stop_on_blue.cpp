#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Includes for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <stop_on_blue/StopOnBlueConfig.h>
#include <dynamic_reconfigure/server.h>

// Includes for working with images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

class StopOnBlue {
public:
    StopOnBlue();
    ~StopOnBlue();

    // Called every time a new camera frame is recieved
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    // Called every time the dynamic reconfigure is updated
    void configCallback(stop_on_blue::StopOnBlueConfig &config, uint32_t level);

private:
    // Count the number of white pixels in a region
    float whiteAmount(const cv::Mat& image);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pub_;

    // The dynamic reconfigure server
    dynamic_reconfigure::Server<stop_on_blue::StopOnBlueConfig> server_;
    stop_on_blue::StopOnBlueConfig config_;
};

// Constructor
StopOnBlue::StopOnBlue() : nh_{"~"}, it_{nh_}
{
    // Subscribe to the camera publisher node
    image_sub_ = it_.subscribe("/camera_view", 1, &StopOnBlue::imageCb, this); // cam_pub pubs

    // To publish your message
    pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // Dynamic Reconfigure
    server_.setCallback(boost::bind(&StopOnBlue::configCallback, this, _1, _2));
}

StopOnBlue::~StopOnBlue() // Destructor
{
    cv::destroyWindow("HSV image");
    cv::destroyWindow("dilated mask");
}

/**
 * Dynamic Reconfigure Callback
 * ============================
 * Called every time the Dynamic Reconfigure UI is updated by the user.
 */
void StopOnBlue::configCallback(stop_on_blue::StopOnBlueConfig &config, uint32_t level)
{
    config_ = config;
}


// Image Callback function
void StopOnBlue::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    // 1. Convert to cv image in BGR
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 2. Blur the image to reduce noise (kernel must be odd). See OpenCV Lecture 1
    if (config_.use_median_blur)
    {
        cv::medianBlur(cv_ptr->image, cv_ptr->image, 2*config_.median_blur_amount + 1);
    }

    // 3. Convert to HSV
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
    //cv::imshow("HSV image", hsv_image);
    
    // 4. Masking. The Hue angle of pure Blue is 240
    cv::Mat mask;
    cv::inRange(hsv_image,
        cv::Scalar(config_.mask_l_hue/2.0, config_.mask_l_sat, config_.mask_l_lum),
        cv::Scalar(config_.mask_h_hue/2.0, config_.mask_h_sat, config_.mask_h_lum),
        mask);
    // cv::imshow("HSV mask", mask);

    // 5. Dilate the mask
    cv::Mat dilated_mask;
    cv::Mat element_d = cv::getStructuringElement(cv::MORPH_RECT, // MORPH_ELLIPSE
                            cv::Size(2*config_.mask_dilate + 1, 2*config_.mask_dilate+1),
                            cv::Point(config_.mask_dilate, config_.mask_dilate));

    cv::dilate(mask, dilated_mask, element_d);
    cv::imshow("dilated mask", dilated_mask);
    
    // stop when blue is over 7%
    float white_amount = whiteAmount(dilated_mask); // get the ratio of white pixels
    ROS_INFO_STREAM("white/all pixel ratio: " << white_amount);

    geometry_msgs::Twist output_message;
    output_message.linear.x = 0;

    if (white_amount > 0.07)
        output_message.linear.x = 0;
    else
        output_message.linear.x = config_.speed;
    pub_.publish(output_message);

    cv::waitKey(3);
}

// Count White Pixels In an area; Return the ratio of white pixels
float StopOnBlue::whiteAmount(const cv::Mat& image)
{
    int white_cnt = cv::countNonZero(image);
    return (float)white_cnt/(image.cols*image.rows);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "stop_on_blue");
    StopOnBlue lf{};
    ROS_INFO_STREAM("stop_on_blue running!");
    ros::spin();
    return 0;
}
// CJ S21
