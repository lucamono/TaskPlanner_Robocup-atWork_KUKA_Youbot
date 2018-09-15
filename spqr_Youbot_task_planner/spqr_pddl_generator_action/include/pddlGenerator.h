#ifndef PDDL_GENERATOR
#define PDDL_GENERATOR
#include <fstream>
#include <iostream>
#include <vector>
#include <string.h>
#include <stdio.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <pddl_msgs/PDDLPlannerAction.h>
#include <pddl_msgs/PDDLDomain.h>
#include <pddl_msgs/PDDLProblem.h>
#include <pddl_msgs/PDDLAction.h>
#include <pddl_msgs/PDDLActionArray.h>
#include <pddl_msgs/PDDLObject.h>
#include <pddl_msgs/PDDLStep.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

class PddlProblemGenerator
{
public:
    std::vector<std::string> createProblemPddl(std::vector <std::string> pddlLocations, std::vector <std::string> pddlObjects, std::vector <std::string> edges,  std::vector< std::pair<std::string,std::string> > initObjLocation, std::vector< std::pair<std::string,std::string> > goalObjLocation);
};

#endif
