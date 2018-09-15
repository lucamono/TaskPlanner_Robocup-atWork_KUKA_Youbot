#include "pddlGenerator.h"
#include <stdio.h>

void PddlProblemGenerator::createProblemPddl(std::vector <std::string> objLocations, std::vector <std::string> objObjects){
  //write mode file
  std::fstream myfile;
  myfile.open ("/home/luca/catkin_ws/src/Robocup@Work/spqr_topological_navigation/config/pddl/problem.pddl",std::fstream::out);
  myfile << "(define (problem spqr-planning)" << std::endl;
  myfile << "   (:domain spqr-task-planning)" << std::endl;
  
  //the objects of pddl's problem
  myfile << "	(:objects" << std::endl;
  myfile << "	    youbot" << std::endl;
  int n_slot=3;
  for(int i=1; i <= n_slot; ++i){
     myfile << "	    slot" << std::to_string(i) << std::endl;
  }
  myfile << std::endl;
  for(int i=0; i < objLocations.size(); ++i){
     myfile << "	    " << objLocations.at(i) << std::endl;
  }
  myfile << std::endl;
  myfile.close(); 
   /*
  
  fprintf(fd, "%s\n", ;
  fprintf(fd, "%s\n", );
  //the objects of pddl's problem
  fprintf(fd, "%s\n", "	  (:objects");
  fprintf(fd, "%s\n", "	  	youbot");

  std::string str;
  std::string str2;
 
  // save the graph on file
  /*for(int i=0; i < nodeList.size(); i++ ){
	  fprintf(fd, "%d %s %f %f %d ", nodeList.at(i).id,nodeList.at(i).label.c_str(),nodeList.at(i).pos_x,nodeList.at(i).pos_y,nodeList.at(i).list_id.size());
	  for(int j=0; j < nodeList.at(i).list_id.size(); j++ ){
		  fprintf(fd, "%d ", nodeList.at(i).list_id.at(j));
	  }
	  for(int j=0; j < nodeList.at(i).list_id.size(); j++ ){
		  fprintf(fd, "%f ", nodeList.at(i).distances.at(j));
	  }
	  fprintf(fd, "\n");
	  // close the file
  }*/
 // fclose(fd);
}
