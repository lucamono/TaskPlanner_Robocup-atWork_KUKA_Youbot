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
#include <astar.h>

struct callBack_userdata{
    float mapResolution;
    cv::Mat map;
    geometry_msgs::Pose mapOrigin;
    Graph g;
};

int idLastSelected=-1;

bool locatServ=true;

std::vector<cv::Point2f> locationsFromService;
std::vector<std::string> namesFromService;
// String splitter
std::vector<std::string> explode(const std::string& str, const char& ch)
{
    std::string next;
    std::vector<std::string> result;
    // For each character in the string
    for (std::string::const_iterator it = str.begin(); it != str.end(); it++)
    {
	// If we've hit the terminal character
	if (*it == ch)
	{
	    // If we have some characters accumulated
	    if (!next.empty())
	    {
		// Add them to the result vector
		result.push_back(next);
		next.clear();
	    }
	} 
	else
	{
	  // Accumulate the next character into the sequence
	  next += *it;
	}
    }
  if (!next.empty())
      result.push_back(next);
  return result;
}

std::pair<std::string, std::string> getMapImgInfos(std::string config_file_path)
{
    YAML::Node map_config_file = YAML::LoadFile(config_file_path);
    std::string map_file_name = map_config_file["image"].as<std::string>();
    std::vector<std::string> map_file_path_splitted = explode(config_file_path, '/');
    std::string map_img_file_path = "/";
    for(int i=0; i<map_file_path_splitted.size()-1; i++)
    {
	map_img_file_path += map_file_path_splitted[i] + "/";
    }
    map_img_file_path += map_file_name;
    return std::pair<std::string, std::string>(map_img_file_path, map_config_file["resolution"].as<std::string>());
}

cv::Mat manuallyConvertMapToImg(const nav_msgs::OccupancyGrid& map, int &map_height, int &map_width)
{
    cv::Mat mat;
    mat.create(map_height, map_width, CV_8SC1);
    int rows = mat.rows;
    int cols = mat.cols;
    if (mat.isContinuous())
    {
	cols = rows*cols;
	rows = 1;
    } 
    for (int r = 0; r < rows; ++r)
    {
	uchar *pOutput = mat.ptr<uchar>(r);
	for (int c = 0; c < cols; ++c)
	{
	    *pOutput = (uchar)map.data.at(c);
	    ++pOutput;
	}
    }
    return mat;
}

void rebuildMapFromTresholds(cv::Mat &inputMap, cv::Mat &outputMap, float map_occupied_threshold, float map_free_threshold)
{
    cv::Vec3b not_visited_color(205, 205, 205);
    cv::Vec3b free_cell_color(254, 254, 254);
    cv::Vec3b occupied_cell_color(0, 0, 0); 
    int _map_occupied_threshold = round(map_occupied_threshold);
    int _map_free_threshold = round(map_free_threshold);
    //change the outputMap
    for(int r=0; r<inputMap.rows; r++)
    {
	for(int c=0; c<inputMap.cols; c++)
	{
	    int pixel_intensity = (int)inputMap.at<signed char>(r,c);
	    if(pixel_intensity == -1)
	    {
		outputMap.at<cv::Vec3b>(r,c) = not_visited_color;
	    }
	    else
	    {
		if(pixel_intensity > _map_occupied_threshold)
		{
		    //std::cout<<r<<" , "<<c<<std::endl;
		    outputMap.at<cv::Vec3b>(r,c) = occupied_cell_color;
		}
		else if(pixel_intensity < _map_free_threshold)
		{
		    outputMap.at<cv::Vec3b>(r,c) = free_cell_color;
		}
	    }
	}
    }
}



// OpenCV MouseClick Event Callback
void MouseClickEventCallback(int event, int x, int y, int flags, void* userdata)
{
    callBack_userdata*_userdata = (callBack_userdata*)(userdata);
    float mapRes = _userdata->mapResolution;
    geometry_msgs::Pose mapOrig = _userdata->mapOrigin;
    //the list of nodes in the graph
    pGraph nodelist = _userdata->g.getNodeList();
    Draw d;
    if ( event == cv::EVENT_LBUTTONDBLCLK )
    {
	std::cout << "Left button of the mouse is clicked - position " << d.pixelToPoint(x, y, mapRes, mapOrig) << std::endl;
	std::cout << "#######NEXT STEP########" << std::endl;
	//create the node
	if(nodelist.size()>0)
	{
	    std::string input;
	    std::vector<int> adjacency;
	    std::cout << "insert the id marker of adjacency" << std::endl;
	    //input from keyboard
	    std::cin >> input;
	    std::string delimiter = ",";
	    int pos = input.find(delimiter);
	    std::string token;
	    int numb;
	    while (pos !=-1)
	    {
		token = input.substr(0, pos);
		std::istringstream ( token ) >> numb;	
		adjacency.push_back(numb);
		input.erase(0, pos + delimiter.length());  
		pos = input.find(delimiter);
	    }
	    std::istringstream ( input ) >> numb;
	    adjacency.push_back(numb);
	    //update the node
	    graphNode newNode;     
	    newNode.pos_x = d.pixelToPoint(x, y, mapRes, mapOrig).x;
	    newNode.pos_y = d.pixelToPoint(x, y, mapRes, mapOrig).y;
	    newNode.list_id = adjacency;
	    _userdata->g.addNode(newNode, adjacency,0,"");
	}
	else
	{
	    std::vector<int> adjacency;
	    std::cout << "First Node of the Map added" << std::endl;
	    graphNode newNode;
	    newNode.pos_x = d.pixelToPoint(x, y, mapRes, mapOrig).x;
	    newNode.pos_y = d.pixelToPoint(x, y, mapRes, mapOrig).y;
	    _userdata->g.addNode(newNode, adjacency,0,"");
	}
	//cv::Mat clone_map = _userdata->map.clone();
	for(int y=0; y < nodelist.size(); y++)
	{
	    std::cout << nodelist.at(y).id << std::endl;
	}
	//drawNode(clone_map,_userdata->g,_userdata->mapResolution,_userdata->mapOrigin,_userdata->sizeNodeLoader);
	cv::Mat clone_map = _userdata->map.clone();
	d.drawNode(clone_map,_userdata->g.getNodeList(), mapRes, mapOrig);
    }
    else if  ( event == cv::EVENT_LBUTTONDOWN )
    {
	std::cout << endl;
	//update the colored matrix
	cv::Mat clone_map = _userdata->map.clone();
	d.drawNode(clone_map,_userdata->g.getNodeList(), mapRes, mapOrig);
	//if color are blue or red
	bool blue = (((int)clone_map.at<Vec3b>(Point(x, y))[0] == 255)&&((int)clone_map.at<Vec3b>(Point(x, y))[1] == 0)&&((int)clone_map.at<Vec3b>(Point(x, y))[2] == 0));
	bool red = (((int)clone_map.at<Vec3b>(Point(x, y))[0] == 0)&&((int)clone_map.at<Vec3b>(Point(x, y))[1] == 0)&&((int)clone_map.at<Vec3b>(Point(x, y))[2] == 255));
	if(blue || red)
	{
	    //generate legends for each node
	    cv::Point p1;
	    bool found=false;
	    int i=0;
	    int th = 0;
	    while(i < nodelist.size() && !found)
	    {
		p1 = (cv::Point)d.pointToPixel(cv::Point2f(nodelist.at(i).pos_x, nodelist.at(i).pos_y), mapRes, mapOrig);
		//find node inside the bounding box
		if(x-th <= p1.x  && x+th >= p1.x && y-th <= p1.y && y+th >= p1.y)
		{
		    std::cout << "id Node: " << nodelist.at(i).id << std::endl;
		    std::cout << "label Node: " << nodelist.at(i).label << std::endl;
		    std::cout << "adjacency list: ";
		    for(int j=0; j < nodelist.at(i).list_id.size(); j++)
			std::cout << nodelist.at(i).list_id.at(j) << " | ";
		    std::cout << std::endl;  
		    clone_map = _userdata->map.clone();
		    d.drawInfoNode(clone_map, _userdata->g.getNodeList(), nodelist.at(i).id, mapRes, mapOrig);
		    cv::imshow("Map", clone_map);
		    found = true;
		    idLastSelected = nodelist.at(i).id;
		}
		i++;
	    }
	}
	else idLastSelected =-1;
    }
}

void getLocations(ros::NodeHandle &n, Graph *graph,float res, geometry_msgs::Pose orig )
{   Draw d;
    ros::ServiceClient clientGetLocations = n.serviceClient<spqr_location_service::GetLocation>("location_service/get_location");
    spqr_location_service::GetLocation srvGet;
    srvGet.request.name = "";
    //get location
    if (clientGetLocations.call(srvGet))
    {
        //if the location alreay exists
        int size = srvGet.response.locations.size();
        for (int i = 0; i < size; i++)
	{   
	    graphNode nodeTemp;
	    std::vector<int> list_id;
	    std::string name = srvGet.response.locations[i].name;
	    //open file graph
	    std::string package_path = ros::package::getPath("spqr_topological_graph");  
	    std::string path_config_loc = package_path + "/config/configTopologic.txt";
	    std::ifstream Configuration_file(path_config_loc.c_str());
	    if (Configuration_file)
	    {
		std::string line;
		bool notFound = true;			
		while (getline (Configuration_file,line) && notFound)
		{
		    Configuration_file.ignore(0);
		    if((name[0] == line[0]) && (name[1] == line[1]))
		    {
			float x = srvGet.response.locations[i].pose.position.x;
			float y = srvGet.response.locations[i].pose.position.y;
			nodeTemp.pos_x = x;
			nodeTemp.pos_y = y;
			notFound = false;
			cv::Point p1 = (cv::Point)d.pointToPixel(cv::Point2f(x,y), res, orig);
			locationsFromService.push_back(p1);
			namesFromService.push_back(name);
		    }
		}
	    }
	}
    }
    else
    {
        std::cout <<"Failed to call service location_service/GetLocation" << std::endl;
    }
}

//wv checks if the given point is present into the pair vector, if true returns its id else  returns -1
int isAlreadyPresent(Point2f point, std::vector<std::pair<int, Point2f> >& id_point_pairs){
  int result = -1;
  int size = id_point_pairs.size();
  for(int i=0; i < size; i++){
    if(point == id_point_pairs[i].second){
      result = id_point_pairs[i].first;
      break;
    }
  }
  return result;
}

//wv adds a new node on the graph only if it is not already present on it and returns the id of the new/old node
int addNodeFromVornoi(Point2f node, std::string label, std::vector<std::pair<int, Point2f> >& already_added_points, callBack_userdata& userdata){
  Draw d;
  Point2f node_transformed = d.pixelToPoint(node.x, node.y, userdata.mapResolution, userdata.mapOrigin);
  std::vector<int> list_id;
  graphNode nodeTemp(node_transformed.x, node_transformed.y);
  int id = isAlreadyPresent(node, already_added_points);

  //wv the point is not present
  if(id == -1){
      userdata.g.addNode(nodeTemp, list_id, !label.empty(), label);
      already_added_points.push_back(std::pair<int,Point2f>(nodeTemp.id, node));
      id = nodeTemp.id;
  }
  return id; 
}
// Standard C++ entry point
int main(int argc, char** argv)
{
    // Announce this program to the ROS master as a "node" called "spqr_build_topological_graph_node"
    ros::init(argc, argv, "spqr_build_topological_graph_node");

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
    if (map_service_client_.call(srv_map))
    {
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
    // visualize the map
    cv::namedWindow("Map");
    callBack_userdata userdata;
    cv::Mat costMap = map_img.clone();
    userdata.map = map_img;
    userdata.mapResolution = map_resolution;
    userdata.mapOrigin = map_origin;
    getLocations(nh,&userdata.g,userdata.mapResolution,userdata.mapOrigin); 
    cv::setMouseCallback("Map", MouseClickEventCallback, &userdata);
    //Show the map
    cv::rectangle(userdata.map, cv::Point(0,0), cv::Point(map_img.cols, 50),  cv::Scalar(200, 0, 0), -1);
    cv::putText(userdata.map, "Press h for help", cv::Point(5, 35),CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2, CV_AA);
    cv::imshow("Map", userdata.map);
    Draw d;
    while(1)
    {
	// Rosspin
	//cv::imshow("test", blackImg);
	char key=cv::waitKey(0);
	switch(key)
	{
	    case 'q':
	    {
		cv::destroyAllWindows();
		return 0;
	    }
	    case 'a':
	    {
		std::string input;
		std::string curr_node;
		std::cout << "Remove arc from Graph, insert the current node: " << std::endl;
		//input from keyboard
		std::cin >> curr_node;    
		int node;
		std::istringstream ( curr_node ) >> node;
		std::cout << "insert the other nodes: " << std::endl;		    
		//input from keyboard
		std::cin >> input;
		std::string delimiter = ",";
		int pos = input.find(delimiter);
		std::string token;
		int numb;
		while (pos !=-1)
		{
		    token = input.substr(0, pos);
		    std::istringstream ( token ) >> numb;	
		    userdata.g.removeArc(userdata.g.getNodeList().at(node),userdata.g.getNodeList().at(numb));
		    input.erase(0, pos + delimiter.length());  
		    pos = input.find(delimiter);
		}
		std::istringstream ( input ) >> numb;
		userdata.g.removeArc(userdata.g.getNodeList().at(node),userdata.g.getNodeList().at(numb));
		cv::Mat clone_map = userdata.map.clone();
		d.drawNode(clone_map,userdata.g.getNodeList(),userdata.mapResolution,userdata.mapOrigin);
		break;
	    }
	    case 'n':
	    {	
		/*
		std::cout << "Remove node from Graph, ";
		int node_idx;
		std::cout << "insert the node to delete: ";
		std::cin >> node_idx; */
		if(idLastSelected>=0){
		    std::cout << "Remove node from Graph, ";
		    userdata.g.removeNode(idLastSelected);
		    cv::Mat clone_map = userdata.map.clone();
		    d.drawNode(clone_map,userdata.g.getNodeList(),userdata.mapResolution,userdata.mapOrigin);
		    idLastSelected=-1;
		}
		break;
	    }
	    
	    case 'z':
	    {
	        std::cout << "AStar computation for minimal paths: " << std::endl;
		if(userdata.g.getNodeList().size()>0){
		  
		    std::vector<int> path;
		    int start;
		    std::cout << "Insert the id Start: ";
		    std::cin >> start;
		    std::cout << endl;
		    int goal;
		    std::cout << "Insert the id goal: ";
		    std::cin >> goal;
		    std::cout << endl;
		   
		    Astar *g = new Astar(userdata.g.getNodeList());
		    path = g->compute(start , goal);

		    //print the shortest path
		    std::cout << "ASTAR Shortest path: " << std::endl; 
		    for(int i = 0; i< path.size(); i++){
			std::cout << path[i] <<" | "; 
		    }
		    std::cout << std::endl;
		    //draw path on image
		    cv::Mat clone_map = userdata.map.clone();
		    d.drawShortestPath(clone_map, userdata.g.getNodeList(),userdata.mapResolution,userdata.mapOrigin, path);
		}
		else std::cout << "No graph exist." << std::endl;
		break;
	    }
	    
	    case 'r':
	    {
// 		std::cout << "Remove node from Graph, ";
// 		int node_idx;
// 		std::cout << "insert the node to delete: ";
// 		std::cin >> node_idx;
		std::vector<int> nodes_to_remove;
		if(idLastSelected>=0){  
		    nodes_to_remove.push_back(idLastSelected);
		    while(!nodes_to_remove.empty()){
		      std::vector<int> neighbours = userdata.g.getNodeList().at(nodes_to_remove[0]).list_id;
		      for (int j=0; j< neighbours.size(); j++){
			if(!(find(nodes_to_remove.begin(),nodes_to_remove.end(),neighbours[j]) != nodes_to_remove.end())){
			  nodes_to_remove.push_back(neighbours[j]);
			}
		      }
		      int id_to_remove =nodes_to_remove[0];
		      userdata.g.removeNode(id_to_remove);
		      nodes_to_remove.erase(nodes_to_remove.begin());
		      for (int j=0; j< nodes_to_remove.size(); j++){
			if(nodes_to_remove[j] > id_to_remove){
			  nodes_to_remove[j] -=1;
			}
		      }     
		    }
		    cv::Mat clone_map = userdata.map.clone();
		    d.drawNode(clone_map,userdata.g.getNodeList(),userdata.mapResolution,userdata.mapOrigin);
		    idLastSelected=-1;
		}
		break;
	    }
	    case 's':
	    {
		std::cout << "Insert arc from Node to another Node. ";
		int node_idx;
		int node_idx2;
		std::cout << "insert the first node: ";
		std::cin >> node_idx;
		std::cout << "insert the second node: ";
		std::cin >> node_idx2;
		userdata.g.addArc(node_idx,node_idx2);
		cv::Mat clone_map = userdata.map.clone();
		d.drawNode(clone_map,userdata.g.getNodeList(),userdata.mapResolution,userdata.mapOrigin);
		break;
	    }
	    case 'e':
	    {
		userdata.g.exportGraph();	
		std::cout << "SAVED ON FILE SUCCESFULLY." <<	std::flush;
		break;
	    }
	    
	    case 'f':
	    {   
	        cv::Mat clone_map = userdata.map.clone();
		int inputId;
		std::cout << "Insert the id node to found: " << std::endl;
		std::cin >> inputId;
	        d.drawFindNode(clone_map,userdata.g.getNodeList(), userdata.mapResolution,userdata.mapOrigin, inputId);
		break;
	    }
	    
	    case 't':
	    {
		navNode navNode = userdata.g.generateNavigationGraph(costMap, userdata.g.getNodeList(), userdata.mapResolution, userdata.mapOrigin);
		bool check = userdata.g.checkNavigationConnectivity(navNode);
		break;
	    }
	    
	    case 'i':
	    {
		pGraph impGraph=userdata.g.importGraph();
		cv::Mat clone_map = userdata.map.clone();
		d.drawNode(clone_map,userdata.g.getNodeList(),userdata.mapResolution,userdata.mapOrigin);
	    std::cout << "GRAPH IMPORTED SUCCESFULLY." << std::endl <<	std::flush;
	    break;
	    }
	    case 'm':
	    {
		std::cout << "Label to id correspondences. Insert the label of the node:" << std::endl;
		std::string inputLabel;
		std::cin >> inputLabel;
		std::cout << "ID: " << userdata.g.getIdFromLabel(inputLabel) << std::endl;
		break;
	    }
	    
	    case 'j':
	    {
		std::cout << "id to label correspondences. Insert the id of the node:" << std::endl;
		int inputId;
		std::cin >> inputId;
		std::cout << "Label: " << userdata.g.getLabelFromId(inputId) << std::endl;
		break;
	    }
	    
	    case 'p':
	    {
		userdata.g.exportGraph();	
		std::cout << "SAVED ON FILE SUCCESFULLY." <<	std::flush;
		break;
	    }
	    case 'h':
	    {   std::cout << "List of commands:" << std::endl;
	        std::cout << std::endl;
	        std::cout << "Mouse Device:" << std::endl;
		std::cout << "LEFT DOUBLE CLICK FOR ADD NODE" << std::endl;
		std::cout << "LEFT CLICK ON DRAWN NODE TO GET INFO" << std::endl;
		std::cout << std::endl;
		std::cout << "Keyboard Device" << std::endl;
	        std::cout << "PRESS v FOR GENERATE TOPOLOGICAL FROM VORONOI . " << std::endl;
		std::cout << "PRESS e FOR EXPORT GRAPH TO FILE. " << std::endl;
		std::cout << "PRESS i FOR IMPORT GRAPH FROM FILE. " << std::endl;
		
		std::cout << "PRESS n FOR REMOVE NODE. " << std::endl;
		std::cout << "PRESS s FOR ADD ARC. " << std::endl;
		std::cout << "PRESS a FOR REMOVE ARC. " << std::endl;
		std::cout << "PRESS m FOR FIND ID NODE FROM LABEL. " << std::endl;
		std::cout << "PRESS r FOR RECURSIVELY DELETE ALL NODES IN THE GRAPH CONNECTED TO THE NODE SELECTED. " << std::endl;
		std::cout << "PRESS f FOR FIND A NODE GIVEN THE ID. " << std::endl;
		std::cout << "PRESS z FOR COMPUTE PATH FROM START TO GOAL. " << std::endl;
		break;
	    }
	    case 'v':
	    {
		cv::Mat clone_map = userdata.map.clone();
		float robot_size = 15;
		float dist_thr   = 15;
		Voronoi v(clone_map, robot_size, dist_thr);
		v.compute(locationsFromService, namesFromService);
		std::vector<connected> cp =v.getConnectedPoints();
		
		std::vector<std::pair<int, Point2f>> already_added_points;

		for(int i=0; i < cp.size(); i++){		      
		  int id_from = addNodeFromVornoi(cp[i].from, cp[i].name_from, already_added_points, userdata);
		  int id_to   = addNodeFromVornoi(cp[i].to  , cp[i].name_to, already_added_points, userdata);
		  userdata.g.addArc(id_from, id_to);
		}
		d.drawNode(clone_map, userdata.g.getNodeList(), userdata.mapResolution,userdata.mapOrigin);
		break;
	    }
	    default:
	    //std::cout<<key<<std::endl;
	    break;
	}
	ros::spinOnce();
	cv::waitKey(10);
    }
    // Stop the node's resources
    ros::shutdown();
    // Exit tranquilly
    return 0;
}