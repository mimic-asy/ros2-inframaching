#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>


class Image_Subscriber : public rclcpp::Node
{
public :
  Image_Subscriber()
  : Node("image_subscriber")
  {
    ///camera/camera/color/image_rawからデータが送られてきたらimage_callbackを呼び出す
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw",
        10,
        std::bind(&Image_Subscriber::image_callback, this, std::placeholders::_1));

  }

  private:
  void image_callback(const sensor_msgs::msg::Image &msg)
  {
    //送られてきたデータのタイムスタンプを出力する
    RCLCPP_INFO(this->get_logger(), "Message timestamp: '%d'", msg.header.stamp.nanosec);

    //画像を取得・出力
    cv::Mat Image = Get_image(msg);
    //特徴点を取得・出力
    Feature_point(Image);
  }
    //msgから画像を取得する
  cv::Mat Get_image(const sensor_msgs::msg::Image &msg)
  {
    cv::Mat image(msg.height, msg.width, CV_8UC3, const_cast<unsigned char*>(msg.data.data()), msg.step);
    cv::Mat glay_image;
    //画像を白黒にする
    cv::cvtColor(image,glay_image, cv::COLOR_BGR2GRAY);

    return glay_image;
  }
    //特徴点を取得する
  void Feature_point(cv::Mat &glay_image)
  {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Mat outimage;

    //Initiate ORB detector
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create(500,1.2f,8,31,0,2,cv::ORB::HARRIS_SCORE,31,10);

    //find keypoints with orb
    detector->detect(glay_image, keypoints, cv::noArray());

    //compute the descriptors with ORB
    detector->compute(glay_image,keypoints,descriptors);

    //draw only keypoints location,not size and orientation
    cv::Scalar red_color(0, 0, 255);
    cv::drawKeypoints(glay_image,keypoints,outimage,red_color,cv::DrawMatchesFlags::DEFAULT );

    // 取得した画像を表示
    cv::imshow("Image", outimage);
    cv::waitKey(1);

  }

//subscription_の宣言
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Image_Subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
