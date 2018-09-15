// Include the ROS C++ APIs
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Pose.h>
#include "location_service/GetLocation.h"
#include <graph.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;

///********slot dimension parameters********
int scale = 3;
int startX = 450*scale;
int startY = 20*scale;
int width = 160;
int height = 50;
int dist = 10;
vector<pair<cv::Scalar,string>> slots_info;



struct action{
  string type,from,goal, slot;
  int from_index;
  int goal_index;
  action(string tp, string f, string g, string sl="", int fi=0, int gi=0){
    type=tp;
    from=f;//position of the robot
    goal=g;//take/drop -> goal= object; move -> goal=destination;
    slot=sl;
    from_index=fi;
    goal_index=gi;
  }
};
struct callBack_userdata{
	float mapResolution;
	cv::Mat map;
	geometry_msgs::Pose mapOrigin;
	Graph g;
	int sizeNodeLoader;	
};
int action_showed = 0;

vector<action> actions;
map<string,int> pose_index;

bool locatServ=true;

void rebuildMapFromTresholds(cv::Mat &inputMap, cv::Mat &outputMap, float map_occupied_threshold, float map_free_threshold){
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
      }
      else{
	if(pixel_intensity > _map_occupied_threshold){
	  //std::cout<<r<<" , "<<c<<std::endl;
	  outputMap.at<cv::Vec3b>(r,c) = occupied_cell_color;
	}
	else if(pixel_intensity < _map_free_threshold){
	  outputMap.at<cv::Vec3b>(r,c) = free_cell_color;
	}
      }
    }
  }
}

cv::Point2f pixelToPoint(int x, int y, float map_resolution, geometry_msgs::Pose map_origin){
  return cv::Point2f(x*map_resolution + map_origin.position.x, y*map_resolution + map_origin.position.y);
}

cv::Point2f pointToPixel(cv::Point2f pt, float map_resolution, geometry_msgs::Pose map_origin){
  return cv::Point2f((pt.x - map_origin.position.x)/map_resolution,(pt.y - map_origin.position.y)/map_resolution);
}

void drawNode(callBack_userdata*_userdata){

  cv::Mat inputMap = _userdata->map;
  float mapRes = _userdata->mapResolution;
  geometry_msgs::Pose mapOrig = _userdata->mapOrigin;
  int Localizer_size = _userdata->sizeNodeLoader;
  Graph gra = _userdata->g;
  // Setup a rectangle to define your region of interest
  cv::Rect myROI(150, 250, 630, 300);//map in 430, 430-630: leggenda


  inputMap = inputMap(myROI);
  resize(inputMap,inputMap,cv::Size(inputMap.cols*scale,inputMap.rows*scale));
  cv::flip(inputMap,inputMap,0);


  cv::putText(inputMap, "slot 1", cv::Point(startX+20, startY-10),
              CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(100, 100, 100), 1, CV_AA);
  cv::putText(inputMap, "slot 2", cv::Point(startX+23+width+5, startY-10),
              CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(100, 100, 100), 1, CV_AA);
  cv::putText(inputMap, "slot 3", cv::Point(startX+25+2*width+10, startY-10),
              CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(100, 100, 100), 1, CV_AA);
  cv::rectangle(inputMap, cv::Point(startX, startY), cv::Point(startX+width, startY+height), slots_info.at(0).first, 3);//slot1
  cv::rectangle(inputMap, cv::Point(startX+width+dist, startY), cv::Point(startX+2*width+dist, startY+height), slots_info.at(1).first, 3);//slot2
  cv::rectangle(inputMap, cv::Point(startX+2*width+2*dist, startY), cv::Point(startX+3*width+2*dist, startY+height), slots_info.at(2).first, 3);//slot3

  cv::putText(inputMap, slots_info.at(0).second, cv::Point(startX+20, startY+35),
              CV_FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(100, 100, 100), 1, CV_AA);
  cv::putText(inputMap, slots_info.at(1).second, cv::Point(startX+23+width+5, startY+35),
              CV_FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(100, 100, 100), 1, CV_AA);
  cv::putText(inputMap, slots_info.at(2).second, cv::Point(startX+25+2*width+10, startY+35),
              CV_FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(100, 100, 100), 1, CV_AA);

  cv::imshow("Map", inputMap);
}
void draw(int event, callBack_userdata*_userdata , int action_showed, int dim_circle, float dim_text, int gap, int event_bk){

  Graph g = _userdata->g;
  float mapRes = _userdata->mapResolution;
  geometry_msgs::Pose mapOrigin = _userdata->mapOrigin;
  action action = actions.at(action_showed);

  cout << "action: " << action.type;
  if (action.type == "move") {
    cout << " from " << action.from << " to " << action.goal << endl;
    action.from_index = pose_index.at(action.from);
    action.goal_index = pose_index.at(action.goal);

    cv::Point2f p_from = pointToPixel(cv::Point2f(g.getNodeList().at(action.from_index).pos_x, mapRes-g.getNodeList().at(action.from_index).pos_y),mapRes,mapOrigin);
    cv::Point2f p_to = pointToPixel(cv::Point2f(g.getNodeList().at(action.goal_index).pos_x, mapRes-g.getNodeList().at(action.goal_index).pos_y),mapRes,mapOrigin);
    p_from.x-=7/2;
    p_from.x+=3*gap;
    p_from.y+=1;
    p_to.x-=7/2;
    p_to.x+=3*gap;
    p_to.y+=1;


    circle(_userdata->map,
           pointToPixel(cv::Point2f(g.getNodeList().at(action.goal_index).pos_x, g.getNodeList().at(action.goal_index).pos_y),
                        _userdata->mapResolution, _userdata->mapOrigin), dim_circle, cv::Scalar(255, 0, 0), -1);
    if(action_showed)
      circle(_userdata->map,
             pointToPixel(cv::Point2f(g.getNodeList().at(action.from_index).pos_x, g.getNodeList().at(action.from_index).pos_y),
                          _userdata->mapResolution, _userdata->mapOrigin), dim_circle, cv::Scalar(205, 205, 205), -1);

    cv::flip(_userdata->map,_userdata->map,0);

    if(action_showed)
      cv::putText(_userdata->map, to_string(action_showed-1), p_from,
                  CV_FONT_HERSHEY_SIMPLEX, dim_text, cv::Scalar(255, 255, 255), 1, CV_AA);


    cv::putText(_userdata->map, to_string(action_showed), p_to,
                CV_FONT_HERSHEY_SIMPLEX, dim_text, cv::Scalar(255, 255, 255), 1, CV_AA);
    cv::flip(_userdata->map,_userdata->map,0);

  }
  else{

    cout << " object " << action.goal << " at "<< action.from<<" from "<<action.slot << endl;

    action.from_index = pose_index.at(action.from);

    cv::Point2f p_to = pointToPixel(cv::Point2f(g.getNodeList().at(action.from_index).pos_x, mapRes-g.getNodeList().at(action.from_index).pos_y),mapRes,mapOrigin);
    p_to.x-=7/2;
    p_to.x+=3*gap;
    p_to.y+=1;

    cv::Scalar color;
    string in_slot;
    if (action.type == "take"){
      circle(_userdata->map,
             pointToPixel(cv::Point2f(g.getNodeList().at(action.from_index).pos_x, g.getNodeList().at(action.from_index).pos_y),
                          _userdata->mapResolution, _userdata->mapOrigin), dim_circle, cv::Scalar(32, 128, 0), -1);
      color= cv::Scalar(0,0,255);
      in_slot=action.goal;
    }
    else{
      circle(_userdata->map,
             pointToPixel(cv::Point2f(g.getNodeList().at(action.from_index).pos_x, g.getNodeList().at(action.from_index).pos_y),
                          _userdata->mapResolution, _userdata->mapOrigin), dim_circle, cv::Scalar(0, 0, 255), -1);
      color= cv::Scalar(32, 128, 0);
      in_slot = "empty";
    }

    if(action.slot=="slot1"){
      slots_info.at(0).first=color;
      slots_info.at(0).second=in_slot;
    }
    else if(action.slot=="slot2"){
      slots_info.at(1).first=color;
      slots_info.at(1).second=in_slot;
    }
    else {
      slots_info.at(2).first=color;
      slots_info.at(2).second=in_slot;
    }

    cv::flip(_userdata->map,_userdata->map,0);

    cv::putText(_userdata->map, to_string(action_showed), p_to,
                CV_FONT_HERSHEY_SIMPLEX, dim_text, cv::Scalar(255, 255, 255), 1, CV_AA);
    cv::flip(_userdata->map,_userdata->map,0);

  }
}
// OpenCV MouseClick Event Callback
void MouseClickEventCallback(int event, int x, int y, int flags, void* userdata){
  int dim_circle = 8;
  float dim_text = .3;
  int gap;
  int event_bk = cv::EVENT_LBUTTONDOWN;
  action_showed<10 ? gap=0 :  gap=-1;

  callBack_userdata*_userdata = (callBack_userdata*)(userdata);

  if (event == cv::EVENT_MBUTTONDOWN ) {
    if(event_bk != event){
      --action_showed;
      event_bk = cv::EVENT_MBUTTONDOWN;
    }
    if(action_showed>1){
//      --action_showed;

      Graph g = _userdata->g;
      float mapRes = _userdata->mapResolution;
      geometry_msgs::Pose mapOrigin = _userdata->mapOrigin;

      int last_to_id = pose_index.at(actions.at(action_showed).from);

      draw(event, _userdata , action_showed-1, dim_circle, dim_text, gap, event_bk);

      circle(_userdata->map,
             pointToPixel(cv::Point2f(g.getNodeList().at(last_to_id).pos_x, g.getNodeList().at(last_to_id).pos_y),
                          _userdata->mapResolution, _userdata->mapOrigin), dim_circle, cv::Scalar(255, 255, 255), -1);
    }

  }
    if (event == cv::EVENT_LBUTTONDOWN ) {
      if(event_bk != event){
        ++action_showed;
        event_bk = cv::EVENT_LBUTTONDOWN;
      }
      if (action_showed < actions.size()) {
        draw(event, _userdata , action_showed, dim_circle, dim_text, gap, event_bk);
        ++action_showed;
      }
  }

  drawNode(_userdata);
}

int getLocations(ros::NodeHandle &n, Graph *graph) {
    ros::ServiceClient clientGetLocations = n.serviceClient<location_service::GetLocation>("location_service/get_location");
    location_service::GetLocation srvGet;
    srvGet.request.name = "";
    //get location
    if (clientGetLocations.call(srvGet)) {
        //if the location alreay exists
        int size = srvGet.response.locations.size();
        for (int i = 0; i < size; i++){   
	        graphNode nodeTemp;
		std::vector<int> list_id;
	        std::string name = srvGet.response.locations[i].name;
		//open file graph
		std::string package_path = ros::package::getPath("spqr_viewer_planner");  
	        std::string path_config_loc = package_path + "/config/configTopologic.txt";
		std::ifstream Configuration_file(path_config_loc.c_str());
		if (Configuration_file){
			std::string line;
			bool notFound = true;			
			while (getline (Configuration_file,line) && notFound){
				Configuration_file.ignore(0);
				if((name[0] == line[0]) && (name[1] == line[1])){
	 	  			float x = srvGet.response.locations[i].pose.position.x;
	    	    			float y = srvGet.response.locations[i].pose.position.y;
		    			nodeTemp.pos_x = x;
		    			nodeTemp.pos_y = y;
		    			graph->addNode(nodeTemp,list_id,1,name);
					notFound = false;    
	        		}
	  		}
		}
	}
	return graph->getNodeList().size();
    } else {
        std::cout <<"Failed to call service location_service/GetLocation" << std::endl;
    }
}


// Standard C++ entry point
int main(int argc, char** argv) {

  // Announce this program to the ROS master as a "node" called "spqr_build_topological_graph_node"
  ros::init(argc, argv, "spqr_path_viewe_node");

  // Start the node resource managers (communication, time, etc)
  ros::start();

  // ROS NodeHandle
  ros::NodeHandle nh;
  // load the map
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
    }
    else
    {
      ROS_ERROR("Failed to call map service");
      return 0;
    }
  if(argv[2])
  {
    map_img = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
    cv::flip(map_img,map_img,0);
  }

  ///read pddl output
  vector<string> pddl_out;
  ifstream pddl_out_file(argv[1]);
  copy(istream_iterator<string>(pddl_out_file),
       istream_iterator<string>(),
       back_inserter(pddl_out));
  pddl_out_file.close();

  ///parse the pddl actions
  for(int i=0; i<pddl_out.size(); ++i){
    if(pddl_out[i]=="(take-obj-from-location"){
      transform(pddl_out[i+2].begin(), pddl_out[i+2].end(), pddl_out[i+2].begin(), ::toupper);
      transform(pddl_out[i+3].begin(), pddl_out[i+3].end(), pddl_out[i+3].begin(), ::toupper);
      string slot = pddl_out[i+4].substr(0, pddl_out[i+4].size()-1);

      action a = action("take",pddl_out[i+3],pddl_out[i+2],slot);
      actions.push_back(a);
    }
    else if(pddl_out[i]=="(drop-object"){

      transform(pddl_out[i+2].begin(), pddl_out[i+2].end(), pddl_out[i+2].begin(), ::toupper);
      transform(pddl_out[i+3].begin(), pddl_out[i+3].end(), pddl_out[i+3].begin(), ::toupper);
      string slot = pddl_out[i+4].substr(0, pddl_out[i+4].size()-1);

      action a = action("drop",pddl_out[i+3],pddl_out[i+2],slot);
      actions.push_back(a);
    }
    else if(pddl_out[i]=="(move"){
      transform(pddl_out[i+2].begin(), pddl_out[i+2].end(), pddl_out[i+2].begin(), ::toupper);
      string from = pddl_out[i+2];
      while(pddl_out[i+3].find("_")!= std::string::npos)
        i+=4;

      string dest = pddl_out[i+3].substr(0, pddl_out[i+3].size()-1);

      transform(dest.begin(), dest.end(), dest.begin(), ::toupper);

      action a = action("move",from,dest);
      actions.push_back(a);
    }
  }
  // visualize the map
  cv::namedWindow("Map");
  callBack_userdata userdata;
  userdata.map = map_img;
  userdata.mapResolution = map_resolution;
  userdata.mapOrigin = map_origin;

  userdata.sizeNodeLoader=getLocations(nh,&userdata.g);

//vector<pair<cv::Scalar,string>> slots_info;

  for(int i=0; i<3; i++)
    slots_info.push_back(pair<cv::Scalar,string>(cv::Scalar(32, 128, 0),"empty"));

  cv::Mat inputMap = userdata.map;
  float mapRes = userdata.mapResolution;
  geometry_msgs::Pose mapOrig = userdata.mapOrigin;
  int Localizer_size = userdata.sizeNodeLoader;
  Graph gra = userdata.g;
  action action = actions.at(action_showed);
  int from_id = 0;

  for(int i;i<actions.size();++i){
    if(actions.at(i).type == "move"){
      cout<<"robot start in: "<<actions.at(i).from<<" to "<<actions.at(i).goal<<endl;
      break;
    }
  }

    for(int i = 0; i< gra.getNodeList().size(); i++)
      pose_index.insert(pair<string,int>(gra.getNodeList().at(i).label,i));

//  for(auto& it : pose_index)
//    cout<<it.first<<" at: "<<it.second<<endl;


  cv::setMouseCallback("Map", MouseClickEventCallback, &userdata);


  while(1){
    // Rosspin
    //cv::imshow("test", blackImg);
    char key=cv::waitKey(0);
    if(key == 'q'){
        cv::destroyAllWindows();
        return 0;
      }
  }
}

