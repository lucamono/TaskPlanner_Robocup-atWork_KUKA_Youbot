//created by Wilson Villa

#ifndef VORONOI_H
#define VORONOI_H
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <ctime>

using namespace std;
using namespace cv;


struct connected{
  Point2f from;
  Point2f to;
  string name_from;
  string name_to;
  connected(Point2f& from_,Point2f& to_, string& name_from_, string& name_to_){
    from = from_;
    to = to_;
    name_from = name_from_;
    name_to = name_to_;
  }
};



class Voronoi{
private:
  vector<Point2f> _paths;
  vector<Point2f> _vertex;
  vector<Point2f> _vertex_locations;
  vector<string> _locations_names;
  
  vector<Point2f> _sampled_path;
  vector<connected> _connected_points;

  Mat _gray;
  Mat _color;
  float _robot_size;
  float _distance_threshold;





  void setColor(Point2f v, Mat& map, const Scalar& color);
  bool isNearWall(const Point2f& v);
  bool isColor(const Point2f v, const Mat& map, const Scalar& color);
  bool isVertex(const Point2f& point);

  bool isNearVertex(const Point2f& v);
  void getNeighbourPoints(Point2f point, vector<Point2f>& neighbours);
  vector<Point2f> getRect2(Point2f v, Mat& map);

  float getDistance(Point2f p1, Point2f p2);


  //wv insert in order the given pair p1 into the ordered vector 
  void insert_ordered(pair<float,Point2f> p1, vector<pair<float,Point2f> >& vector_ord);

  void followPathGoal2(Point2f v_curr, Point2f v_goal, Mat& map, vector<Point2f>& path );
  void sampledPath();


private:
  void computeCObstacle();
  void addIdColor(int id_color, vector<int>& colors);

  void computeVoronoi();
public:
  
  Voronoi(Mat& color_, float robot_size, float distance_threshold_);
  bool isOut(const Point2f& v);
  
  void addLocationToPath(Point2f& location_, string& name_);
  void addLocationsToPath(vector<Point2f>& locations_, vector<string>& names_);


  
  string getAssociatedName(Point2f& point);
  
public:
  void drawVertex(Mat& map);
  void drawLocationVertex(Mat& map);
  void drawPaths(Mat& map);
  void drawSampledPaths(Mat& map);
  void drawPathsE(Mat& map,vector<Point2f>& path);
  
  void compute(vector<Point2f>& points,vector<string>& points_names);
  void connectPoints(Point2f& curr_point, Mat& map);
  inline vector<Point2f> getSampledPath() const {return _sampled_path;}
  inline vector<connected> getConnectedPoints() const {return _connected_points;}
  
  
    
};
#endif
