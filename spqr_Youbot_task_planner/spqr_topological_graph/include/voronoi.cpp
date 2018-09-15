#include "voronoi.h"

Scalar blue(255,0,0);
Scalar red(0,0,255);
Scalar green(0,255,0);
Scalar black(0,0,0);
Scalar white(255,255,255);
Scalar yellow(255,255,0);



void Voronoi::setColor(Point2f v, Mat& map,const Scalar& color){  
  map.at<Vec3b>(v)[0] = color[0];
  map.at<Vec3b>(v)[1] = color[1];
  map.at<Vec3b>(v)[2] = color[2];
}



bool Voronoi::isColor(const Point2f v, const Mat& map, const Scalar& color){
  return map.at<Vec3b>(v)[0] == color[0] &&
	 map.at<Vec3b>(v)[1] == color[1] &&
	 map.at<Vec3b>(v)[2] == color[2];
}


float Voronoi::getDistance(Point2f p1, Point2f p2){
  return sqrt(pow((p1.x -p2.x),2) + pow((p1.y - p2.y),2));
}

void Voronoi::computeVoronoi(){
  
  int edgeThresh = 100;
  int distType = DIST_L2;
  int maskSize = DIST_MASK_5;
  int voronoiType = 0;
  

  Mat edge = _gray >= edgeThresh, dist, labels, dist8u;
  distanceTransform( edge, dist, labels, distType, maskSize, voronoiType );

  dist8u.create(labels.size(), CV_8UC3);
  int last_idx = -1;
	
  for( int i = 1; i < labels.rows-1; i++ ){
    const int* ll = (const int*)labels.ptr(i);
    
    for( int j = 1; j < labels.cols-1; j++ ){
      const int* tt = (const int*)labels.ptr(i+1);
      
      Point2f pp(j, i);
      //wv compute only for pixels that are no gray
      if(!isOut(pp)){
	//vector that contains the id of the colors
	vector<int> id_colors;

	//wv add the id-color of the current pixel
	int idx = (ll[j]-1)%8 + 1;
	id_colors.push_back(idx);
	
	//wv id-color of the pixel-right
	int idx_r = (ll[j+1]-1)%8 + 1;
	//wv id-color of the pixel-right-down
	int idx_rd = (tt[j+1]-1)%8 + 1;
	//wv id-color of the pixel-down
	int idx_d = (tt[j]-1)%8 + 1;
	
	//wv add the id if it is not already present
	addIdColor(idx_r , id_colors);
	addIdColor(idx_rd, id_colors);
	addIdColor(idx_d , id_colors);

	//wv if there are at least two colors then it is a point of the voronoi diagram
	//wv if there are more than two coloros it is a vertex
	int num_diff_colors = id_colors.size();
	if(num_diff_colors>1){
	  _paths.push_back(pp);
	  if(num_diff_colors > 2 && !isNearVertex(pp)){
	    _vertex.push_back(pp);
	  }
	}
      }      
    }
  }  
}

void Voronoi::getNeighbourPoints(Point2f point, vector<Point2f>& neighbours){
  neighbours.push_back( Point2f(point.x-1, point.y-1)); //up-left
  neighbours.push_back( Point2f(point.x  , point.y-1)); //up-center
  neighbours.push_back( Point2f(point.x+1, point.y-1)); //up-right
  neighbours.push_back( Point2f(point.x+1, point.y  )); //right-center
  neighbours.push_back( Point2f(point.x+1, point.y+1)); //right-bottom
  neighbours.push_back( Point2f(point.x  , point.y+1)); //bottom-center
  neighbours.push_back( Point2f(point.x-1, point.y+1)); //bottom-left
  neighbours.push_back( Point2f(point.x-1, point.y  )); //left-center 
}

vector<Point2f> Voronoi::getRect2(Point2f v, Mat& map){
  vector<Point2f> result;
  //corner up-left
  vector<Point2f> candidates;

  getNeighbourPoints(v, candidates);

  for(int i=0; i < candidates.size(); i++){
    if(isColor(candidates[i],map, red)){
      result.push_back(candidates[i]);
    }
  }
  return result;
}
// wv check if the point v is in the proximity of a wall
bool Voronoi::isNearWall(const Point2f& v){
  vector<Point2f> candidates;
  getNeighbourPoints(v, candidates);

  for(int i=0; i < candidates.size(); i++){
    if(isColor(candidates[i], _color, black)){
      return true;
    }
  }
  return false;
}

bool Voronoi::isVertex(const Point2f& point){
  return find(_vertex.begin(),_vertex.end(), point) != _vertex.end();
}

// wv check if the point v is in the proximity of a vertex
bool Voronoi::isNearVertex(const Point2f& v){
  vector<Point2f> candidates;
  getNeighbourPoints(v, candidates);

  for(int i=0; i < candidates.size(); i++){
    if(isVertex(candidates[i])){
      return true;
    }
  }
  return false;
}

void Voronoi::followPathGoal2(Point2f v_curr, Point2f v_goal, Mat& map, vector<Point2f>& path ){
  setColor(v_curr, map, green);
  
  if(v_curr == v_goal){
    path.push_back(v_curr);
    vector<Point2f> sampled_path;

    Point2f last = path[0];
    for(int i=0; i < path.size(); i++){
      if( path[i] != v_goal ){
	sampled_path.push_back(path[i]);
	last = path[i];
      }
    }
    path = sampled_path;
    return;
    
  }else if(isNearWall(v_curr)){
    path.clear();
    return;  
  }else{
    vector<Point2f> asd1 = getRect2(v_curr, map);
    for(int j=0; j < asd1.size(); j++){
	path.push_back(v_curr);
	followPathGoal2(asd1[j], v_goal, map, path);
    }    
  } 
}

//wv insert in order the given pair p1(distance, point) into the ordered vector 
void Voronoi::insert_ordered(pair<float, Point2f> p1, vector<pair<float,Point2f> >& vector_ord){

    for(int j=0; j<vector_ord.size();j++){
      if(p1.first < vector_ord[j].first){
	vector_ord.insert(vector_ord.begin() + j, p1);
	return;
      }
    }
    //wv the point has the maximum distance of the ordered vector
    vector_ord.push_back(p1);    
}

Voronoi::Voronoi(Mat& color_, float robot_size_, float distance_threshold_){
  _color = color_.clone();
  _robot_size = robot_size_;
  _distance_threshold = distance_threshold_;
  
  cvtColor(_color, _gray, COLOR_RGB2GRAY);
  //add c-obstacle to gray map
  computeCObstacle();
}

void Voronoi::compute(vector<Point2f>& points,vector<string>& points_names){
  cout << "Voronoi created test" << endl;
  computeVoronoi();
  
  addLocationsToPath(points, points_names);

  sampledPath();
  
  Mat test2 = _color.clone();
  drawSampledPaths(test2); 
  
  for(int i=0; i< _sampled_path.size(); i++){  
    connectPoints(_sampled_path[i],test2);
  }
}


void Voronoi::computeCObstacle(){
  Mat kernel1 = Mat::ones(Size(_robot_size,_robot_size), CV_8UC1);
  erode(_gray, _gray, kernel1);
}

bool Voronoi::isOut(const Point2f& v){
  return _color.at<Vec3b>(v)[0] == 205 &&
	 _color.at<Vec3b>(v)[1] == 205 &&
	 _color.at<Vec3b>(v)[2] == 205;
}

void Voronoi::addIdColor(int id_color, vector<int>& colors){
  if(!(find(colors.begin(), colors.end(),id_color) != colors.end())){
    colors.push_back(id_color);
  }
}

void Voronoi::addLocationToPath(Point2f& location_, string& name_){
  
  vector<pair<float,Point2f> > vector_ord;
  for(int i=0; i < _paths.size(); i++){
    float dist = getDistance(location_, _paths[i]);
    insert_ordered(pair<float,Point2f>(dist, _paths[i]), vector_ord);
  }
  
  for(int j=0; j<vector_ord.size(); j++){
    Mat pto;
    _color.copyTo(pto);
    vector<Point2f> path2goal;
    Point2f point_from = vector_ord[j].second;
    
    line(pto, location_, point_from, red, 1);
    followPathGoal2(point_from, location_, pto, path2goal); 
    if(path2goal.size()!=0){
      _paths.insert(_paths.end(),path2goal.begin(),path2goal.end());
      break;
    }     
  }
  _vertex_locations.push_back(location_);
  _locations_names.push_back(name_);
}

void Voronoi::addLocationsToPath(vector<Point2f>& locations_, vector<string>& names_){
  for(int i=0; i < locations_.size(); i++){
    addLocationToPath(locations_[i], names_[i]);  
  } 
}



//draw all voronoi vertex and all locations
void Voronoi::drawVertex(Mat& map){
  for( int i=0; i < _vertex.size(); i++){
    setColor(_vertex[i], map, green); 
  }
}

//draw all voronoi vertex and all locations
void Voronoi::drawLocationVertex(Mat& map){
  for( int i=0; i < _vertex_locations.size(); i++){
    setColor(_vertex_locations[i], map, black); 
  }
}

//draw all points of the path
void Voronoi::drawPaths(Mat& map){ 
  for(int i =0; i < _paths.size(); i++){
    setColor(_paths[i],map, red);
  }
}


//draw all points of the sampled path
void Voronoi::drawSampledPaths(Mat& map){ 
  for(int i =0; i < _sampled_path.size(); i++){
    setColor(_sampled_path[i],map, red);
  }
}

void Voronoi::sampledPath(){
  //init sampled path with all the _vertex_locations
  _sampled_path = _vertex_locations;
  
  for(int i=0; i < _paths.size(); i++){
    Point2f curr_node = _paths[i];
    bool flag = true;
    int l = 0;
    int r = _sampled_path.size()-1;
    while(r >= l){
      if(getDistance(curr_node, _sampled_path[l]) < _distance_threshold || getDistance(curr_node, _sampled_path[r]) < _distance_threshold){
	flag = false;
	break;
      }
      l++;
      r--;
    }
    if(flag == true){
      _sampled_path.push_back(curr_node);
    }  
  }
}

string Voronoi::getAssociatedName(Point2f& point){
  ptrdiff_t pos = find(_vertex_locations.begin(), _vertex_locations.end(), point) - _vertex_locations.begin();

  if(pos >= _vertex_locations.size()) {
      return "";
  }else{
    return _locations_names[pos];
  }
}


void Voronoi::connectPoints(Point2f& curr_point, Mat& map){
//   setColor(curr_point,map,'g');
  int w_min = 0;
  int w_max = map.cols-1;
  int h_min = 0;
  int h_max = map.rows-1;
  
  int size = _distance_threshold*2;
  string name_from = getAssociatedName(curr_point);
  string name_to = "";
  if(name_from == ""){
    size= _distance_threshold*1.6;
  }
  int x_l = curr_point.x - size;
  int y_l = curr_point.y - size;
  int x_r = size*2;
  int y_r = size*2;
  
  int x_min = x_l < w_min ? w_min : x_l;
  int y_min = y_l < h_min ? h_min : y_l;
  
  int x_max = x_r > w_max ? w_max : x_r;
  int y_max = y_r > h_max ? h_max : y_r;
  
  cv::Rect ROI(x_min,y_min,x_max,y_max);
  Mat sss = map(ROI);
  

  for(int i=0; i < (ROI.height-1)/2; i++){
    for(int j=i; j < ROI.width-i; j++){
      Point2f curr_neighT(j,i);
      Point2f curr_neighD(j,ROI.height-1-i);
      Point2f curr_neighL(i,j);
      Point2f curr_neighR(ROI.height-1-i,j);
      
      
      if(isColor(curr_neighT,sss,red)){
	curr_neighT.x = x_l + curr_neighT.x;
	curr_neighT.y = y_l + curr_neighT.y;
	name_to = getAssociatedName(curr_neighT);
	_connected_points.push_back(connected(curr_point, curr_neighT, name_from, name_to));
      }else{
	setColor(curr_neighT,sss,yellow);
      }

      
      if(isColor(curr_neighD,sss,red)){
	curr_neighD.x = x_l + curr_neighD.x;
	curr_neighD.y = y_l + curr_neighD.y;
	name_to = getAssociatedName(curr_neighD);
	_connected_points.push_back(connected(curr_point, curr_neighD, name_from, name_to));
      }else{
	setColor(curr_neighD,sss,yellow);
      }
      
      
      if(isColor(curr_neighL,sss,red)){
	curr_neighL.x = x_l + curr_neighL.x;
	curr_neighL.y = y_l + curr_neighL.y;
	name_to = getAssociatedName(curr_neighL);
	_connected_points.push_back(connected(curr_point, curr_neighL, name_from, name_to));
      }else{
	setColor(curr_neighL,sss,yellow);
      }

      if(isColor(curr_neighR,sss,red)){
	curr_neighR.x = x_l + curr_neighR.x;
	curr_neighR.y = y_l + curr_neighR.y;
	name_to = getAssociatedName(curr_neighR);
	_connected_points.push_back(connected(curr_point, curr_neighR, name_from, name_to));
      }else{
	setColor(curr_neighR,sss,yellow);
      }
      
    }
  } 
}

