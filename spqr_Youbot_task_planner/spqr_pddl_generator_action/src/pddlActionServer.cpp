#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <spqr_pddl_generator_action/spqr_planAction.h>

#include <std_msgs/Float64.h>

#include <graph.h>
#include <DistanceMatrix.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>
#include <algorithm> 
#include "opencv2/opencv.hpp"
#include <pddlGenerator.h>
#include <ros/package.h>

using namespace cv; 

struct obj
{
    string name;
    string src;
    string dest;
    string container;

    obj(){};
    obj(string n, string s, string d, string c)
    {
	name= std::move(n);
	src= std::move(s);
	dest= std::move(d);
	container= std::move(c);
    }
};

class pddlAction
{
protected:
  
ros::NodeHandle nh_;
// NodeHandle instance must be created before this line. Otherwise strange error may occur.
//actionlib::SimpleActionServer<spqr_pddl_generator_action::pddlAction> as_;
actionlib::SimpleActionServer<spqr_pddl_generator_action::spqr_planAction> as_;
std::string action_name_;
// create messages that are used to published feedback/result
spqr_pddl_generator_action::spqr_planFeedback feedback_;
spqr_pddl_generator_action::spqr_planResult result_;

  
public:
  
pddlAction(std::string name) :
    as_(nh_, name, boost::bind(&pddlAction::executeCBinit, this, _1), false),
    action_name_(name)
    {
	as_.start();
    }

    ~pddlAction(void){}
 
 
 
    bool elementFounded(std::string element, std::vector <std::string> list){
	bool check=false;
	if(std::find(list.begin(), list.end(), element) != list.end())
	    check=true;		
	    return check;
    }

    //iterator for parse
    template<class InputIterator, class T>InputIterator find (InputIterator first, InputIterator last, const T& val)
    {
	while (first!=last)
	{
	    if (*first==val) return first;
	    ++first;
	}
	return last;
    }
	
	std::string convert_to_ObjCorrectName(std::string obj_to_convert) {
		std::string correct_obj = "";
		if(obj_to_convert.find("f20_20_b") != std::string::npos)
			correct_obj = "F20_20_B"; 
		else if(obj_to_convert.find("f20_20_g") != std::string::npos)
			correct_obj = "F20_20_G";
		else if(obj_to_convert.find("s40_40_b") != std::string::npos)
			correct_obj = "S40_40_B";
		else if(obj_to_convert.find("s40_40_g") != std::string::npos)
			correct_obj = "S40_40_G";
		else if(obj_to_convert.find("m20_100") != std::string::npos)
			correct_obj = "M20_100";
		else if(obj_to_convert.find("m20") != std::string::npos)
			correct_obj = "M20";
		else if(obj_to_convert.find("m30") != std::string::npos)
			correct_obj = "M30";
		else if(obj_to_convert.find("r20") != std::string::npos)
			correct_obj = "R20";
		else if(obj_to_convert.find("bearing_box") != std::string::npos)
			correct_obj = "Bearing_box";
		else if(obj_to_convert.find("bearing") != std::string::npos)
			correct_obj = "Bearing";
		else if(obj_to_convert.find("axis") != std::string::npos)
			correct_obj = "Axis";
		else if(obj_to_convert.find("distance_tube") != std::string::npos)
			correct_obj = "Distance_tube";
		else if(obj_to_convert.find("motor") != std::string::npos)
			correct_obj = "Motor";
		return correct_obj;
		//else if(obj_to_convert.find("f20_20_b") != std::string::npos)
		//else if(obj_to_convert.find("f20_20_b") != std::string::npos)
	}


	std::vector<spqr_pddl_generator_action::spqr_action> convert_tospqrPlan(std::vector<std::string> final_action_toConvert) {
		
		std::vector<spqr_pddl_generator_action::spqr_action> final_converted;
		//spqr_pddl_generator_action::spqr_action actual_action;		
		//da fare
		for(int i = 0; i < final_action_toConvert.size();i++) {
		  spqr_pddl_generator_action::spqr_action actual_action;
			std::cout << "entro " << std::endl;
			std::vector<std::string> strs;
      boost::split(strs,final_action_toConvert[i],boost::is_any_of(" "));
            
			if (final_action_toConvert[i].find("drop") != std::string::npos) {
				actual_action.place = strs[strs.size() -1];
				std::cout << "D " << std::endl;
				std::cout << "dove " << strs[strs.size() -1] << std::endl;
				int n_object = 0;
				for(int j = 1; j < (strs.size() -1);j++) {
					actual_action.objects.push_back(convert_to_ObjCorrectName(strs[j]));
					std::cout << "cosa " << strs[j] << std::endl;
					std::cout << "convertito " << convert_to_ObjCorrectName(strs[j]) << std::endl;
					n_object++;
				}
				if(n_object == 1)
					actual_action.type = "D";
				else if (n_object == 2)
					actual_action.type = "DD";
				else 
					actual_action.type = "DDD";
				std::cout << "quanti " << actual_action.type << std::endl;
				final_converted.push_back(actual_action);
				std::cout << "pusho " << std::endl;
			}
			else if(final_action_toConvert[i].find("take") != std::string::npos) {
				actual_action.type = "P";
				actual_action.place = strs[strs.size() -1];
				std::cout << "T " << std::endl;
				std::cout << "dove " << strs[strs.size() -1] << std::endl;
				int n_object = 0;
				for(int j = 1; j < (strs.size() -1);j++) {
					actual_action.objects.push_back(convert_to_ObjCorrectName(strs[j]));
					std::cout << "cosa " << strs[j] << std::endl;
					std::cout << "convertito " << convert_to_ObjCorrectName(strs[j]) << std::endl;
					n_object++;
				}
				if(n_object == 1)
					actual_action.type = "P";
				else if (n_object == 2)
					actual_action.type = "PP";
				else 
					actual_action.type = "PPP";
				std::cout << "quanti " << actual_action.type << std::endl;	
				final_converted.push_back(actual_action);
			}
		}
		std::cout << "piano finale " << std::endl;
		for(int i = 0; i < final_converted.size();i++) {
		  std::cout << final_converted[i] << std::endl;
		}
		return final_converted;		
	}


    void executeCBinit(const spqr_pddl_generator_action::spqr_planGoalConstPtr &goal)
    {
	feedback_.goal = 0.0; 
	std::string package_path = ros::package::getPath("spqr_pddl_generator_action");  
	std::string path_yaml = package_path + "/config/CFH_yaml/plan_bttHARD.yaml";
	std::cout << "apro " << path_yaml << std::endl;
	//init the DistanceMatrix 
	DistanceMatrix dm;
	Graph g = dm.getGraphImported();
	//evaluate all possible shortest paths 
	int res;
	std::vector <std::string> pddlLocations;
	std::vector <std::string> pddlEdges;

	//load CFH parsed subset-graph's data
	FileStorage fs;
	std::cout << "path " << goal->filename << std::endl;
	//fs.open(path_yaml, FileStorage::READ);
	fs.open(goal->filename, FileStorage::READ);

	FileNode n= fs["objects"];
	vector<obj> data;
	//evaluate all possible shortest paths   
	for (auto &&it : n)
	    data.push_back(obj({(string) it["name"],
				(string) it["source"],
				(string) it["dest"],
				(string) it["container"]}));
	fs.release();
	//for all elements in data parses

	std::map<std::pair<std::string,std::string>,int> edges;
	for(int i=0; i<data.size();++i)
	{
	    std::string source = data.at(i).src;
	    std::string dest = data.at(i).dest;
		      
	    if(!elementFounded(source, pddlLocations))
	    {  					
		pddlLocations.push_back(source); 
		for(int j=0;j<data.size();++j)
		{
		    if(source != data.at(j).dest)
		    {
			//evaluate minimal distace between the relative workstations
			res=dm.getDistance(source,data.at(j).dest );
			edges.insert(std::pair<std::pair<std::string,std::string>,int>(std::pair<std::string,std::string>(source,data.at(j).dest ),res));
			edges.insert(std::pair<std::pair<std::string,std::string>,int>(std::pair<std::string,std::string>(data.at(j).dest, source ),res));
		    }
		}
	    }	
	    if(!elementFounded(dest, pddlLocations))
	    {  					 
		pddlLocations.push_back(dest); 
		for(int j=0;j<data.size();++j)
		{
		    if(dest != data.at(j).src)
		    {
			//evaluate minimal distace between the relative workstations
			res=dm.getDistance(data.at(j).src,dest );
			edges.insert(std::pair<std::pair<std::string,std::string>,int>(std::pair<std::string,std::string>(data.at(j).src,dest ),res));
			edges.insert(std::pair<std::pair<std::string,std::string>,int>(std::pair<std::string,std::string>(dest, data.at(j).src ),res));
			res=dm.getDistance(data.at(j).src, "WPS");
			edges.insert(std::pair<std::pair<std::string,std::string>,int>(std::pair<std::string,std::string>("WPS",data.at(j).src),res));
		    }
		}
	    }	
	}
	pddlLocations.push_back("WPS");
	int dist;
	int n_temp_nodes;
	int k=0;
	std::string temp_node;
	for (auto &it:edges)
	{
	    k++;
	    //the distance between the nodes
	    dist=it.second;
	  
	    n_temp_nodes=dist-1;
	  
	    //n_temp_nodes=0;
	    if(n_temp_nodes==0)
	    {
		pddlEdges.push_back("(edge " + it.first.first + " " + it.first.second + ")"); 
	    }
	    else
	    {
		for(int i=0;i < n_temp_nodes;i++)
		{
		    //case with more intermediate nodes
		    if(n_temp_nodes > 1)
		    {
			//the first node adjacency
			if(i==0)
			{
			    pddlEdges.push_back("(edge " + it.first.first + " " + it.first.first + "_" + std::to_string(k*(i+1)) + ")"); 
			    if(!elementFounded(it.first.first + "_" + std::to_string(k*(i+1)), pddlLocations))  					
				pddlLocations.push_back(it.first.first + "_" + std::to_string(k*(i+1)));
			}
			//the last node adjacency
			else
			{
			    if(i==(n_temp_nodes-1))
			    {
				pddlEdges.push_back("(edge " + it.first.first  + "_" + std::to_string(k*i) + " " + it.first.second + ")");
				if(!elementFounded(it.first.first  + "_" + std::to_string(k*i), pddlLocations))  					
				    pddlLocations.push_back(it.first.first  + "_" + std::to_string(k*i));
			    }
			    else
			    {
				pddlEdges.push_back("(edge " + it.first.first  + "_" + std::to_string(k*i) + " " + it.first.first  + "_" + std::to_string(k*(i+1)) + ")");
				if(!elementFounded(it.first.first  + "_" + std::to_string(k*i), pddlLocations))  					
				    pddlLocations.push_back(it.first.first  + "_" + std::to_string(k*i));
				if(!elementFounded(it.first.first  + "_" + std::to_string(k*(i+1)), pddlLocations))  					
				    pddlLocations.push_back(it.first.first  + "_" + std::to_string(k*(i+1)));
			    }
			}
		    }
		    //case with only one intermediate node
		    else
		    {
			pddlEdges.push_back("(edge " + it.first.first + " " + it.first.first + "_" + std::to_string(k*(i+1)) + ")");
			pddlEdges.push_back("(edge " + it.first.first + "_" + std::to_string(k*(i+1)) + " " + it.first.second + ")");
			if(!elementFounded(it.first.first  + "_" + std::to_string(k*(i+1)), pddlLocations))  					
			    pddlLocations.push_back(it.first.first  + "_" + std::to_string(k*(i+1)));
		    }
		}
	    }
	    k++;
	}  
	//take the list of objects from CFH, rename same objects and compute the distances between same objects (heuristic function)
	std::vector<std::string> objects;
	int l=1;
	float distance_i,distance_j;
	//tuple of distance(float) between first object (i), second object (j). the reference are data.at(i).name, data.at(j).name
	std::vector <std::tuple<int,int,float> > allDistances;
	bool found;
	for(int i=0; i< data.size();++i)
	{
	    found=false;
	    for(int j=i+1; j< data.size();++j)
	    {
		if(data.at(i).name==data.at(j).name)
		{
		    if(!found)
		    {
			allDistances.push_back(std::make_tuple(i,i,dm.getDistance(data.at(i).src, data.at(i).dest)));
			found=true;
		    }
		    allDistances.push_back(std::make_tuple(i,j,dm.getDistance(data.at(i).src, data.at(j).dest)));
		    data.at(i).name = data.at(i).name + std::to_string(l++);
		}
	    }
	    //compute and assign priority distances between same nodes
	    if(!allDistances.empty())
	    {
		float distance_i;
		float distance_j;
		distance_i=std::get<2>(allDistances.at(0));
		for(int k=1; k < allDistances.size();++k)
		{
		    distance_j=std::get<2>(allDistances.at(k));
		    //reassign the goal between the objects
		    if(distance_i > distance_j)
		    {
			data.at(std::get<0>(allDistances.at(0))).dest=data.at(std::get<0>(allDistances.at(k))).dest;
			data.at(std::get<0>(allDistances.at(k))).dest=data.at(std::get<0>(allDistances.at(0))).dest;
		    }
		}
	    }
	    allDistances.clear();
	}
	
	for(int i=0; i< data.size(); i++)
	    objects.push_back(data.at(i).name);
      
	//take the start location of the objects from CFH
	std::vector<std::pair<std::string,std::string> > initObjLocation;
	for (int i=0; i< data.size(); i++)
	    initObjLocation.push_back(std::pair<std::string,std::string>(data.at(i).name,data.at(i).src));
	
	//take the goal location of the objects from CFH
	std::vector<std::pair<std::string,std::string> > goalObjLocation; 
	for (int i=0; i< data.size(); i++)
	    goalObjLocation.push_back(std::pair<std::string,std::string>(data.at(i).name,data.at(i).dest));
	
	//update feedback
	feedback_.goal = 0.5; 
	
	//generate the pddl problem file
	PddlProblemGenerator pddl_file;
	std::vector<std::string> final_action_toConvert = pddl_file.createProblemPddl(pddlLocations, objects, pddlEdges,initObjLocation,goalObjLocation);
	std::vector<spqr_pddl_generator_action::spqr_action> final_converted = convert_tospqrPlan(final_action_toConvert);
	
	
	//feedback_.goal = goal->goal; 
	result_.plan = final_converted;
	as_.setSucceeded(result_);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pddl");
    pddlAction pddl(ros::this_node::getName());
    ros::spin();
    return 0;
}
