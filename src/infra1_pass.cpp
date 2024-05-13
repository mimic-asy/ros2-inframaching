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

  struct kp_and_img
  {
    std::vector<cv::KeyPoint> kp;
    cv::Mat img;
    cv::Mat ds;

  };


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
    struct kp_and_img feature_infr1;
    struct kp_and_img feature_infr2;

    //画像にする
    cv::Mat infr1_cv_image = cv_bridge::toCvShare(infr1_image,"mono8")->image;
    cv::Mat infr2_cv_image = cv_bridge::toCvShare(infr2_image,"mono8")->image;

    //画像と特徴点を取得する
    feature_infr1 = Feature_point(infr1_cv_image);
    feature_infr2 = Feature_point(infr2_cv_image);


    RCLCPP_INFO(this->get_logger(), "infr1 type is: '%d'", feature_infr1.img.type());
    RCLCPP_INFO(this->get_logger(), "infr2 type is: '%d'", feature_infr2.img.type());

    //取得した特徴点をマッチングして出力
    matching_descriptor(feature_infr1,feature_infr2);

  }

  struct kp_and_img Feature_point(cv::Mat &grey_image)
  {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Mat outimage;
    struct kp_and_img output;

    //Initiate ORB detector
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create(500,1.2f,8,31,0,2,cv::ORB::HARRIS_SCORE,31,10);

    //find keypoints with orb
    detector->detect(grey_image, keypoints, cv::noArray());

    //compute the descriptors with ORB
    detector->compute(grey_image,keypoints,descriptors);

    //draw only keypoints location,not size and orientation
    cv::Scalar red_color(0, 0, 255);
    cv::drawKeypoints(grey_image,keypoints,outimage,red_color,cv::DrawMatchesFlags::DEFAULT);

    output.kp = keypoints;
    output.img = outimage;
    output.ds = descriptors;

    return output;

  }

  //画像の特徴点をマッチングさせる
  void matching_descriptor(struct kp_and_img descriptor1,struct kp_and_img descriptor2)
  {

    cv::Mat img_match;
    std::vector<std::vector<cv::DMatch>> matches;
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );

    //ハミング距離を使用してマッチングを行う
    matcher->knnMatch(descriptor1.ds, descriptor2.ds, matches, 2, cv::noArray(), false);

    //マッチングを描写
    drawMatches ( descriptor1.img,descriptor1.kp, descriptor2.img, descriptor2.kp, matches, img_match );

    cv::imshow("Image", img_match);
    cv::waitKey(1);

  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Infra_Subscriber>());
  rclcpp::shutdown();
  return 0;
}