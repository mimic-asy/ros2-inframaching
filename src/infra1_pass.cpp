#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

class Infra_Subscriber : public rclcpp::Node
{
public:
  // Subscriberを作成する
  message_filters::Subscriber<sensor_msgs::msg::Image> infr1_;
  message_filters::Subscriber<sensor_msgs::msg::Image> infr2_;

  // Publisherを作成する
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr combined_image_pub_;

  // message_filtersの長い型の名前をexacttime_policyに置き換える
  // 2つのsensor_msgs::msg::Imageの同期の定義を定める
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> exacttime_policy;

  // 定めた定義をSynchronizeに適応させたインスタンスを作成する
  //sync_の引数には(同期の定義のキューサイズ,同期の定義の引数１, 同期の定義の引数2)が入る
  message_filters::Synchronizer<exacttime_policy> sync_;

  // クラスのコンストラクタ ImageCombiner()を作成
  Infra_Subscriber()
      : Node("image_combiner_node"), sync_(exacttime_policy(10),infr1_, infr2_)
  {
    //infrn_構造体のsubscribeに対してtopicを代入する
    infr1_.subscribe(this, "/camera/infra1/image_rect_raw");
    infr2_.subscribe(this, "/camera/infra2/image_rect_raw");

    //publisherで発信するデータの型とキューのサイズを決める
    combined_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/output_image", 10);

    //定めた定義に適合した際にTopic_callbackを実行する
    sync_.registerCallback(&Infra_Subscriber::topic_callback, this);

  }

  public:
  void topic_callback(
        const sensor_msgs::msg::Image::SharedPtr infr1_image,
        const sensor_msgs::msg::Image::SharedPtr infr2_image)
  {
    //画像にする
    cv::Mat infr1_cv_image = cv_bridge::toCvShare(infr1_image,"mono8")->image;
    cv::Mat infr2_cv_image = cv_bridge::toCvShare(infr2_image,"mono8")->image;

    //cv::imshow("Image", infr1_cv_image);
    //cv::imshow("Image", infr2_cv_image);

    //cv::waitKey(1);

  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Infra_Subscriber>());
  rclcpp::shutdown();
  return 0;
}