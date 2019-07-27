#include <mcr_line_contour_detector/line_contour_finder.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

LineContourFinder::LineContourFinder()
{
}

LineContourFinder::~LineContourFinder()
{
}

std::vector<std::vector<cv::Point> > LineContourFinder::find2DContours(const cv::Mat &image, cv::Mat &debug_image)
{
  //cv::Mat gray_image;
  //cv::cvtColor(image, gray_image, CV_BGR2GRAY);
  cv::Mat gray_img;
  cvtColor( image, gray_img, CV_BGR2GRAY );
  ///cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
  //cv::imshow("Gray image", gray_img);
  //cv::waitKey(0);
  
  //FIND OUTER LINE OF GRID
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(11,11));
  
  cv::Mat close;

  cv::morphologyEx(gray_img, close, 3, kernel);

  cv::Mat dst_div ;
  
  //cv::divide(gray_img, close, dst_div, 1, -1);
  dst_div = gray_img/close;
  //std::cout<<dst_div<<"\n";
  //cv::imshow("dst_div", close);
  cv::Mat div2;
  cv::normalize(dst_div, div2, 0, 255, cv::NORM_MINMAX);
  cv::Mat res_8u;
  div2.convertTo(res_8u, CV_8UC1);
  
  //cv::imshow("normalize imgray", res_8u);
  cv::Mat threshold;
  cv::adaptiveThreshold(res_8u, threshold, 255, 0,1,19,2);
  //cv::imshow("threshold", threshold);
  
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
 
  cv::findContours(threshold, contours, hierarchy, 3, 2);
  float max_area = 0.0;
  std::vector<cv::Point> best_contour = contours[0]; 
  for (int i=0; i<contours.size(); i++) {
    float area = cv::contourArea(contours[i]);
    if (area > 1000)
    {
      if (area > max_area)
      {
        max_area = area;
        best_contour = contours[i];
      }
    }
  }

  //FIND CONTOUR INSIDE THE GRID AND REMOVE THE GRID
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
  //cv::imshow("res", res);
//  res.copyTo(debug_image);

  //Threshold
  cv::Mat new_threshold;
  cv::threshold(res, new_threshold, 200, 255, 0);
  //cv::imshow("new_threshold", new_threshold);
  
  //cv::Mat blur_img;
  //cv::blur(new_threshold, blur_img, cv::Size(2,2));

  //blur_img.copyTo(debug_image);
  
  //DETECT CORNERS
/*   cv::Mat dst_, cdst;
  cv::Canny(new_threshold, dst_, 50, 200, 3);
  cv::cvtColor(dst_, cdst, cv::COLOR_GRAY2BGR);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(dst_, lines, 1, CV_PI / 180, 50, 50, 10);
  for (size_t i = 0; i < lines.size(); i++)
  {
    cv::Vec4i l = lines[i];
    cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, 2);
  }
 
  dst_.copyTo(debug_image); */

/*  cv::Mat dst_harris;
  cv::cornerHarris(new_threshold,dst_harris,2,3,0.04);
  //cv2.dilate(dst,None)
  cv::Mat dilation_dst;
  cv::Mat element_ = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  cv::dilate(dst_harris,dilation_dst,element_);
  double min_, max_;
  cv::minMaxLoc(dst_harris, &min_, &max_);
  //img[dst>0.01*dst.max()]=[0,0,255]
  //new_threshold.copyTo(debug_image);
  for (int i=0; i<dilation_dst.rows; i++)
  {
    for (int j=0; j<dilation_dst.cols; j++)
    {
      if (float(dilation_dst.at<uchar>(i,j)) > 0.01*max_)
      {
        cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(i,j));
        color[0] = 0;
        color[1] = 0;
        color[2] = 255;
        dilation_dst.at<cv::Vec3b>(cv::Point(i,j)) = color;
        std::cout<<"Corner: "<<i<<", "<<j<<std::endl;
      }
    }
  } 
  //dilation_dst.copyTo(debug_image);
*/

  cv::Mat flipped_threshold;
  cv::threshold(new_threshold, flipped_threshold, 50, 255, CV_THRESH_BINARY_INV);
  //cv::imshow("flipped_threshold", flipped_threshold);

  //flipped_threshold.copyTo(debug_image);

  //Skeletonization
  cv::threshold(flipped_threshold, flipped_threshold, 127, 255, cv::THRESH_BINARY); 
  cv::Mat dst_8u;
  flipped_threshold.convertTo(dst_8u, CV_8UC1);
  //cv::imshow("corner2", dst_8u);
  cv::Mat skel(flipped_threshold.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat temp;
  cv::Mat eroded;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

  bool done;
  do
  {
    cv::erode(flipped_threshold, eroded, element);
    cv::dilate(eroded, temp, element);
    cv::subtract(flipped_threshold, temp, temp);
    cv::bitwise_or(skel, temp, skel);
    eroded.copyTo(flipped_threshold);
    done = (cv::countNonZero(flipped_threshold) == 0);
  } while (!done); 
  //cv::imshow("Skeleton", skel);
 skel.copyTo(debug_image);

  std::vector<cv::Point> contour_list;
  for (int i=0; i<skel.rows; i++)
  {
    for (int j=0; j<skel.cols; j++)
    {
      if (skel.at<uchar>(i,j) == 255)
      {
        cv::Point skel_;
        skel_.x =j;
        skel_.y =i;
        contour_list.push_back(skel_);
      }
    }
  }

  std::vector<std::vector<cv::Point> > final_2d_contours;
  final_2d_contours.push_back(contour_list);
  //std::cout<<contour_list;
  return final_2d_contours;
}

std::vector<pcl::PCLPointCloud2::Ptr> LineContourFinder::get3DContours(const std::vector<std::vector<cv::Point> > &contours, pcl::PCLPointCloud2::Ptr input_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*input_cloud, *xyz_input_cloud);

    // loop through points in the 2D contour and find their 3D positions in the given pointcloud
    std::vector<pcl::PCLPointCloud2::Ptr> pcl_contours;
    //pcl::PCLPointCloud2::Ptr pcl_contours;
    
    //std::cout<<"Converting to 3d...";
    
    //for (size_t i = 0; i < contours.size(); i++)
    //{
      pcl::PCLPointCloud2::Ptr pcl_contour(new pcl::PCLPointCloud2);
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_contour(new pcl::PointCloud<pcl::PointXYZ>);
      
      //std::cout<<"Processing conversion...[0]";

      for (size_t j = 0; j < contours[0].size(); j++)
      {
        pcl::PointXYZ pcl_point = xyz_input_cloud->at(contours[0][j].x, contours[0][j].y);

        if ((!pcl_isnan(pcl_point.x)) && (!pcl_isnan(pcl_point.y)) && (!pcl_isnan(pcl_point.z)))
        {
          xyz_contour->points.push_back(pcl_point);
        }
      }

      //std::cout<<"processing conversion...[1]";

      // remove outliers in the pointcloud. this ensures the points are roughly on the same plane
      xyz_contour->header = xyz_input_cloud->header;
      
      //std::cout<<"Precessing conversion...[2]";
      
      if (xyz_contour->points.size() > 0)
      {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(xyz_contour);
        sor.setMeanK(50);
        sor.setStddevMulThresh(3.0);
        sor.filter(*xyz_contour);
        pcl::toPCLPointCloud2(*xyz_contour, *pcl_contour);
        pcl_contours.push_back(pcl_contour);
        //pcl_contours = pcl_contour;
      }
    //}

    return pcl_contours;
}


std::vector<std::vector<cv::Point> > LineContourFinder::find2DContoursTest(const cv::Mat &image, cv::Mat &debug_image)
{
  cv::Mat gray_img;
  cvtColor( image, gray_img, CV_BGR2GRAY );

  cv::Mat gray_img_th;
  cv::threshold(gray_img, gray_img_th, 150, 255, 0);

  std::vector<std::vector<cv::Point> > contours_0;
  std::vector<cv::Vec4i> hierarchy_0;
  cv::findContours(gray_img_th, contours_0, hierarchy_0, 3, 2);

  float max_area_0 = 0.0;
  std::vector<cv::Point> best_contour_0 = contours_0[0]; 
  for (int i=0; i<contours_0.size(); i++) {
    float area = cv::contourArea(contours_0[i]);
    if (area > 1000)
    {
      if (area > max_area_0)
      {
        max_area_0 = area;
        best_contour_0 = contours_0[i];
      }
    }
  }
    //FIND CONTOUR INSIDE THE GRID AND REMOVE THE GRID
  std::vector<std::vector<cv::Point> > contours_final_0;
  contours_final_0.push_back(best_contour_0);

  cv::Mat mask_0 = cv::Mat::zeros(gray_img.size(), CV_8UC1);
  //mask_0.setTo(0, mask_0<=0);
  cv::drawContours(mask_0, contours_final_0, 0, (255,255,255, 255), -1);
  
  cv::Mat dst_0;
  cv::bitwise_and(gray_img, mask_0, dst_0);

  cv::Mat mask_diff = 255 - mask_0;
  cv::Mat roi;
  cv::add(dst_0, mask_diff, roi);
  //roi.copyTo(debug_image);


  //FIND OUTER LINE OF GRID
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(11,11));  
  cv::Mat close;
  cv::morphologyEx(roi, close, 3, kernel);
  cv::Mat dst_div ;
  dst_div = roi/close;
  cv::Mat div2;
  cv::normalize(dst_div, div2, 0, 255, cv::NORM_MINMAX);
  cv::Mat res_8u;
  div2.convertTo(res_8u, CV_8UC1);
  //res_8u.copyTo(debug_image);
  
  cv::Mat threshold;
  cv::adaptiveThreshold(res_8u, threshold, 255, 0,1,19,2);
  //threshold.copyTo(debug_image);

  //Skeletonization
  cv::Mat skel(threshold.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat temp;
  cv::Mat eroded;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

  bool done;
  do
  {
    cv::erode(threshold, eroded, element);
    cv::dilate(eroded, temp, element);
    cv::subtract(threshold, temp, temp);
    cv::bitwise_or(skel, temp, skel);
    eroded.copyTo(threshold);
    done = (cv::countNonZero(threshold) == 0);
  } while (!done); 
 
  skel.copyTo(debug_image);

  std::vector<cv::Point> contour_list;
  for (int i=0; i<skel.rows; i++)
  {
    for (int j=0; j<skel.cols; j++)
    {
      if (skel.at<uchar>(i,j) == 255)
      {
        cv::Point skel_;
        skel_.x =j;
        skel_.y =i;
        contour_list.push_back(skel_);
      }
    }
  }

  std::vector<std::vector<cv::Point> > final_2d_contours;
  final_2d_contours.push_back(contour_list);
  //std::cout<<contour_list;
  return final_2d_contours; 
}
