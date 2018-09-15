// Include the ROS C++ APIs
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
//stuff for reading yaml files
#include <yaml-cpp/yaml.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Pose.h>
#include "spqr_location_service/GetLocation.h"
#include <graph.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <voronoi.h>
#include <draw.h>
#include <math.h>
#include <Eigen/Geometry>
#include "cubic_spline.h"
#include "brezier_curve.h"
#include "astar.h"
Draw d;

typedef Eigen::Matrix<double, 2, 1> Vector2;

struct callBack_userdata{
    float mapResolution;
    cv::Mat map;
    geometry_msgs::Pose mapOrigin;
    Graph g;
};

void rebuildMapFromTresholds(cv::Mat &inputMap, cv::Mat &outputMap, float map_occupied_threshold, float map_free_threshold)
{
  cv::Vec3b not_visited_color(205, 205, 205);
  cv::Vec3b free_cell_color(254, 254, 254);
  cv::Vec3b occupied_cell_color(0, 0, 0); 
  int _map_occupied_threshold = round(map_occupied_threshold);
  int _map_free_threshold = round(map_free_threshold);
  
  //change the outputMap
  for(int r=0; r<inputMap.rows; r++){
    for(int c=0; c<inputMap.cols; c++){
      int pixel_intensity = (int)inputMap.at<signed char>(r,c);
      if(pixel_intensity == -1){
	outputMap.at<cv::Vec3b>(r,c) = not_visited_color;
      }else{
	if(pixel_intensity > _map_occupied_threshold){
	  outputMap.at<cv::Vec3b>(r,c) = occupied_cell_color;
	}else if(pixel_intensity < _map_free_threshold){
	  outputMap.at<cv::Vec3b>(r,c) = free_cell_color;
	}
      }
    }
  }
}


void writeBrezierCurve(std::vector<Vector2> spline_pos, std::vector<Vector2> spline_vel, std::vector<Vector2> spline_acc){
  ofstream myfile;
  myfile.open ("brezier_curve.txt");
  for(int i=0; i< spline_pos.size(); i++){
    std::stringstream ss;
    ss << std::to_string(spline_pos[i].x()) << "," << std::to_string(spline_pos[i].y()) <<  ","; 
    ss << std::to_string(spline_vel[i].x()) << "," << std::to_string(spline_vel[i].y()) <<  ",";
    ss << std::to_string(spline_acc[i].x()) << "," << std::to_string(spline_acc[i].y()) <<  "\n";

    std::string line = ss.str(); 
    myfile << line;
  }  
  myfile.close();
}

void writeCubicSpline(std::vector<Vector2> spline_pos, std::vector<Vector2> spline_vel, std::vector<Vector2> spline_acc){
  ofstream myfile;
  myfile.open ("cubic.txt");
  for(int i=0; i< spline_pos.size(); i++){
    std::stringstream ss;
    ss << std::to_string(spline_pos[i].x()) << "," << std::to_string(spline_pos[i].y()) <<  ","; 
    ss << std::to_string(spline_vel[i].x()) << "," << std::to_string(spline_vel[i].y()) <<  ",";
    ss << std::to_string(spline_acc[i].x()) << "," << std::to_string(spline_acc[i].y()) <<  "\n";

    std::string line = ss.str(); 
    myfile << line;
  }  
  myfile.close();
}

void writeVoronoi(std::vector<int>& path, callBack_userdata& userdata, std::vector<Vector2> velocities){
  
  ofstream myfile_v;
  myfile_v.open ("voronoi_path.txt");
  for(int i = 0; i< path.size(); i++){
    graphNode curr_node = userdata.g.getNode(path[i]);
    Vector2   curr_vel  = velocities[i];
    std::stringstream ss;
    ss << std::to_string(curr_node.pos_x) << "," << std::to_string(curr_node.pos_y) << ","; 
    ss << std::to_string(curr_vel.x()) << "," << std::to_string(curr_vel.y()) << "\n";
    myfile_v << ss.str();
  }  
  myfile_v.close();
  
}
double rad2deg(double rad){
  return rad*180/M_PI;
}

void drawCompletePathBrezierCurve(cv::Mat& map, const std::vector<Vector2>& complete_path, callBack_userdata& userdata){
  
  Scalar black(0,0,0);
  for(int i=0; i < complete_path.size(); i++){
    cv::Point2f p = d.pointToPixel(cv::Point2f(complete_path[i].x(),complete_path[i].y()), userdata.mapResolution, userdata.mapOrigin);
    map.at<Vec3b>(p)[0] = black[0];
    map.at<Vec3b>(p)[1] = black[1];
    map.at<Vec3b>(p)[2] = black[2];
  }
  cv::imshow("brezier curve",map);
}

void drawCompletePathCubicSpline(cv::Mat& map,const std::vector<Vector2>& complete_path, callBack_userdata& userdata){
  
  Scalar red(0,0,255);
  for(int i=0; i < complete_path.size(); i++){
    cv::Point2f p = d.pointToPixel(cv::Point2f(complete_path[i].x(),complete_path[i].y()), userdata.mapResolution, userdata.mapOrigin);
    map.at<Vec3b>(p)[0] = red[0];
    map.at<Vec3b>(p)[1] = red[1];
    map.at<Vec3b>(p)[2] = red[2];
  }
  cv::imshow("cubic spline",map);
}


int main(int argc, char** argv){
  // Announce this program to the ROS master as a "node" called "spqr_build_topological_graph_node2"
  ros::init(argc, argv, "spqr_build_topological_graph_node2");

  // Start the node resource managers (communication, time, etc)
  ros::start();

  // ROS NodeHandle
  ros::NodeHandle nh;
  cv::Mat map_img;
    
  geometry_msgs::Pose map_origin;
  float map_resolution;
  float map_occupied_threshold = 65.0;
  float map_free_threshold = 19.6;
  int map_height;
  int map_width;
  nav_msgs::OccupancyGrid map;
  ros::ServiceClient map_service_client_ = nh.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap srv_map;
  if (map_service_client_.call(srv_map)){
    ROS_INFO("Map service called successfully");
    map = nav_msgs::OccupancyGrid(srv_map.response.map);
    map_resolution = map.info.resolution;
    map_height = map.info.height;
    map_width = map.info.width;
    map_origin = map.info.origin;
    cv::Mat map_img_tmp = cv::Mat(cv::Size(map_width, map_height), CV_8SC1, (signed char*)map.data.data());
    map_img = cv::Mat(cv::Size(map_width, map_height), CV_8UC3);
    rebuildMapFromTresholds(map_img_tmp, map_img, map_occupied_threshold, map_free_threshold);
  }else{
    ROS_ERROR("Failed to call map service");
    return 0;
  }

  // visualize the map
  cv::namedWindow("Map");
  callBack_userdata userdata;
  cv::Mat costMap = map_img.clone();
  userdata.map = map_img;
  userdata.mapResolution = map_resolution;
  userdata.mapOrigin = map_origin;

  //Show the map
  cv::rectangle(userdata.map, cv::Point(0,0), cv::Point(map_img.cols, 50),  cv::Scalar(200, 0, 0), -1);
  cv::putText(userdata.map, "Press h for help", cv::Point(5, 35),CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2, CV_AA);
  
  
  cv::imshow("Map", userdata.map);
  
  
  //wv import graph from file
  userdata.g.importGraph();
  d.drawNode(userdata.map.clone(), userdata.g.getNodeList(), userdata.mapResolution, userdata.mapOrigin);
  std::cout << "GRAPH IMPORTED SUCCESFULLY." << std::endl <<	std::flush;
  
  
  //########### ASTAR ####################
  string name_start = "WS01";
  string name_goal  = "WS03";
  
  // get ids of start and goal
  int id_start = userdata.g.getIdFromLabel(name_start);
  std::cout << name_start << " = " << id_start << std::endl;
  int id_goal  = userdata.g.getIdFromLabel(name_goal);
  std::cout << name_goal << " = " << id_goal << std::endl;
  
  //wv compute path from start to goal
  Astar myAstar(userdata.g.getNodeList());
  std::vector<int> path_ids = myAstar.compute(id_start,id_goal);
  
  std::vector<Vector2> points;
  for(int i=0; i<path_ids.size();i++){
      graphNode node = userdata.g.getNode(path_ids[i]);
      points.push_back(Vector2(node.pos_x, node.pos_y));
  }
  
  //########### CUBIC SPLINES ####################
  Row2 vi,vf;
  vi << 0.0, 0.0;
  vf << 0.0, 0.0;
  CubicSpline* cubic_spline = new CubicSpline(points,vi,vf);
  cubic_spline->setTimeInterval(0.01);
  cubic_spline->compute();

  std::vector<Vector2> cubic_spline_pos = cubic_spline->getPositions();
  std::vector<Vector2> cubic_spline_vel = cubic_spline->getVelocities();
  std::vector<Vector2> cubic_spline_acc = cubic_spline->getAccelerations();
  
  writeCubicSpline(cubic_spline_pos, cubic_spline_vel, cubic_spline_acc);
  
  //########### BREZIER CURVES ####################
  Vector2 b_vi,b_vf;
  b_vi = vi.transpose();
  b_vf = vf.transpose();
  BrezierCurve* brezier = new BrezierCurve(points, cubic_spline_pos, cubic_spline_acc, b_vi, b_vf, 0.01);
  brezier->compute();
  std::vector<Vector2> brezier_pos = brezier->getPositions();
  std::vector<Vector2> brezier_vel = brezier->getVelocities();
  std::vector<Vector2> brezier_acc = brezier->getAccelerations(); 
  
  writeBrezierCurve(brezier_pos, brezier_vel, brezier_acc);
  writeVoronoi(path_ids, userdata, brezier->getVelocitiesAtWaypoints());
  
  //########### DRAW RESULTS ####################
  Mat cc = map_img.clone();
  drawCompletePathCubicSpline(cc, cubic_spline_pos, userdata);
  
  Mat cc1 = map_img.clone();
  drawCompletePathBrezierCurve(cc1, brezier_pos , userdata);

  Mat cc2 = map_img.clone();
  d.drawShortestPath(cc2, userdata.g.getNodeList(), userdata.mapResolution, userdata.mapOrigin, path_ids);

  
  while(1)
  {
    // Rosspin
    //cv::imshow("test", blackImg);
    char key=cv::waitKey(0);
    switch(key)
    {
      
      case 't':
      {
	navNode navNode = userdata.g.generateNavigationGraph(costMap, userdata.g.getNodeList(), userdata.mapResolution, userdata.mapOrigin);
	bool check = userdata.g.checkNavigationConnectivity(navNode);
	break;
      }

      case 'j':
      {
	int inputId;
	userdata.g.getLabelFromId(inputId);
	break;
      }
    }
    ros::spinOnce();
    cv::waitKey(10);
  }
  // Stop the node's resources
  ros::shutdown();
  // Exit tranquilly
  return 0;
}