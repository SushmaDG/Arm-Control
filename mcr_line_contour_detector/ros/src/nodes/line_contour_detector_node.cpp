#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "vector"
#include "opencv2/core/version.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_contour_detector_node");
  cv::Mat image;
  //image = cv::imread("/home/emwe/b-it-bots/5.png", CV_LOAD_IMAGE_COLOR);
  image = cv::imread("/home/emwe/b-it-bots/rgb_img/left0035.jpg", CV_LOAD_IMAGE_COLOR);
  cv::Mat gray_img;
  cvtColor( image, gray_img, CV_BGR2GRAY );
  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
  //cv::imshow("Gray image", gray_img);
  //cv::waitKey(0);
  
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(11,11));
  
  cv::Mat close;

  cv::morphologyEx(gray_img, close, 3, kernel);

  cv::Mat dst_div ;//= cv::Mat(gray_img.size(), CV_32FC1);// = (gray_img2)/(close);
  
  //cv::divide(gray_img, close, dst_div, 1, -1);
  dst_div = gray_img/close;
  //std::cout<<dst_div<<"\n";
  //cv::imshow("dst_div", close);
  cv::Mat div2;
  cv::normalize(dst_div, div2, 0, 255, cv::NORM_MINMAX);
  cv::Mat res_8u;
  div2.convertTo(res_8u, CV_8UC1);
  
  //cv::imshow("normalize imgray", res_8u);
  cv::Mat threshold;// = cv::Mat(gray_img.size(), CV_8UC1);
  cv::adaptiveThreshold(res_8u, threshold, 255, 0,1,19,2);
  cv::imshow("threshold", threshold);
  
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
 
  cv::findContours(threshold, contours, hierarchy, 3, 2);
  float max_area = 0.0;
  std::vector<cv::Point> best_contour = contours[0]; 
  for (auto const& contour_: contours) {
    float area = cv::contourArea(contour_);
    if (area > 1000)
    {
      if (area > max_area)
      {
        max_area = area;
        best_contour = contour_;
      }
    }
  }

  std::vector<std::vector<cv::Point> > contours_final;
  contours_final.push_back(best_contour);

  cv::Mat mask = cv::Mat::zeros(gray_img.size(), CV_8UC1);
  mask.setTo(255, mask<=0);
  std::vector<cv::Vec4i> hierarchy_;
  cv::drawContours(mask, contours_final, 0, 0, -1);
  cv::drawContours(mask, contours_final, 0, 255, 1); 
  //cv::imshow("mask", mask);
  cv::Mat res;
  cv::bitwise_or(gray_img, mask, res);
  cv::imshow("res", res);

  //This threshod can be skipped
  cv::Mat new_threshold;
  cv::threshold(res, new_threshold, 200, 255, 0);
  cv::imshow("new_threshold", new_threshold);


  cv::Mat flipped_threshold;
  cv::threshold(new_threshold, flipped_threshold, 50, 255, CV_THRESH_BINARY_INV);
  cv::imshow("flipped_threshold", flipped_threshold);

  //Skeletonization
  cv::threshold(flipped_threshold, flipped_threshold, 127, 255, cv::THRESH_BINARY); 
  cv::Mat dst_8u;
  flipped_threshold.convertTo(dst_8u, CV_8UC1);
  cv::imshow("corner2", dst_8u);
  cv::Mat skel(flipped_threshold.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat temp;
  cv::Mat eroded;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

  bool done;
  do
  {
    cv::erode(flipped_threshold, eroded, element);
    cv::dilate(eroded, temp, element); // temp = open(img)
    cv::subtract(flipped_threshold, temp, temp);
    cv::bitwise_or(skel, temp, skel);
    eroded.copyTo(flipped_threshold);
  
    done = (cv::countNonZero(flipped_threshold) == 0);
  } while (!done); 
  cv::imshow("Skeleton", skel);
  
  std::vector<cv::Point> contour_list;
  for (int i=0; i<skel.rows; i++)
  {
    for (int j=0; j<skel.cols; j++)
    {
      if (skel.at<uchar>(i,j) == 255)
      {
        cv::Point skel_;
        skel_.x = i;
        skel_.y = j;
        contour_list.push_back(skel_);
      }
    }
  }
  std::cout<<contour_list;
  
  
  //ros::spinOnce();
  cv::waitKey(0);
  return 0;
}