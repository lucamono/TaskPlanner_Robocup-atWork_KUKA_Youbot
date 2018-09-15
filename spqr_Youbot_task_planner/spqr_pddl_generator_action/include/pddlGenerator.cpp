#include "pddlGenerator.h"
#include <dirent.h>

std::string package_path2 = ros::package::getPath("spqr_pddl_generator_action");  
std::string path_pddl2 = package_path2 + "/config/pddl/problem.pddl";
std::string package_path3 = ros::package::getPath("pddl_planner");  
std::string path_pddl3 = package_path3 + "/Robocup_task/problem.pddl";
std::string path_finalPlan = package_path3 + "/Robocup_task/";

std::vector<std::string> PddlProblemGenerator::createProblemPddl(std::vector <std::string> objLocations, std::vector <std::string> objObjects, std::vector <std::string> edges,  std::vector<std::pair<std::string,std::string> > initObjLocation,  std::vector<std::pair<std::string,std::string> > goalObjLocation){
    
      //giulio
    actionlib::SimpleActionClient<pddl_msgs::PDDLPlannerAction> ac("pddl_planner", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    pddl_msgs::PDDLPlannerGoal goal;
    //INIZIO PROBLEM MSG
    goal.problem.name = "spqr-planning";
    goal.problem.domain = "spqr-task-planning";
    std::vector<pddl_msgs::PDDLObject> objectsProblem;
    pddl_msgs::PDDLObject robot;
    robot.name = "youbot";
    robot.type = "object";
    objectsProblem.push_back(robot);
    
    
    //write mode file
    std::fstream myfile;
    myfile.open (path_pddl3,std::fstream::out);
    myfile << "(define (problem spqr-planning)" << std::endl;
    myfile << "   (:domain spqr-task-planning)" << std::endl;
    
    //the objects of pddl's problem
    myfile << "	(:objects" << std::endl;
    myfile << "	    youbot" << std::endl;
    int n_slot=3;
    for(int i=1; i <= n_slot; ++i)
    {
      myfile << "	    slot" << std::to_string(i) << std::endl;
      //
      pddl_msgs::PDDLObject slot;
      slot.name = "slot" + std::to_string(i);
      slot.type = "object";
      objectsProblem.push_back(slot);
    }
    myfile << std::endl;
    for(int i=0; i < objLocations.size(); ++i)
    {
      myfile << "	    " << objLocations.at(i) << std::endl;
      //
      pddl_msgs::PDDLObject objLoc_msg;
      objLoc_msg.name = objLocations.at(i);
      objLoc_msg.type = "object";
      objectsProblem.push_back(objLoc_msg);
    }
    myfile << std::endl;
    for(int i=0; i < objObjects.size(); ++i)
    {
      myfile << "	    " << objObjects.at(i) << std::endl;
      //
      pddl_msgs::PDDLObject objObj_msg;
      objObj_msg.name = objObjects.at(i);
      objObj_msg.type = "object";
      objectsProblem.push_back(objObj_msg);
      //
      std::cout << "gli elementi sono " << objObjects.at(i) << std::endl;
    }
    myfile << std::endl;
    myfile << "	)";
    myfile << std::endl;

    //
    goal.problem.objects = objectsProblem;
    
    //the init part of pddl's problem
    myfile << "	(:init" << std::endl;
    myfile << "	    (ROBOT youbot)" << std::endl;

    //
    std::vector<std::string> initial_condition;
    std::string init_robot = "(ROBOT youbot)";
    initial_condition.push_back(init_robot);

    for(int i=1; i <= n_slot; ++i)
    {
      myfile << "	    (SLOT slot" << std::to_string(i) << ")" << std::endl;
      //
      std::string init_slot = "(SLOT slot" + std::to_string(i) + ")";
      initial_condition.push_back(init_slot);
    }
    myfile << std::endl;
    for(int i=0; i < objObjects.size(); ++i)
    {
      myfile << "	    (OBJ " << objObjects.at(i) << ")" << std::endl;
      //
      std::string init_obj = "(OBJ " + objObjects.at(i) + ")";
      initial_condition.push_back(init_obj);
    }
    myfile << std::endl;
    for(int i=0; i < objLocations.size(); ++i)
    {
      myfile << "	    (location " << objLocations.at(i) << ")" << std::endl;
      //
      std::string init_loc = "(location " + objLocations.at(i) + ")";
      initial_condition.push_back(init_loc);
    }
    myfile << std::endl;
    for(int i=0; i < edges.size(); ++i)
    {
      myfile << "	    " << edges.at(i) << std::endl;
      //
      std::string init_edg = edges.at(i);
      initial_condition.push_back(init_edg);
    }
    myfile << std::endl;
    for(int i=1; i <= n_slot; ++i)
    {
      myfile << "	    (emptySLOT slot" << std::to_string(i) << ")" << std::endl;
      //
      std::string init_slot = "(emptySLOT slot" + std::to_string(i) + ")";
      initial_condition.push_back(init_slot);
    }
    myfile << std::endl;
    myfile << "	    (robot-at-location youbot WPS)" << std::endl;
    myfile << std::endl;

    
    //
    std::string init_robot_loc = "(robot-at-location youbot WPS)";
    initial_condition.push_back(init_robot_loc);


    for(int i=0; i < initObjLocation.size(); ++i)
    {
      myfile << "	    (obj-at-location " << initObjLocation.at(i).first << " " << initObjLocation.at(i).second << ")" << std::endl;
      //
      std::string init_obj = "(obj-at-location " + initObjLocation.at(i).first + " " + initObjLocation.at(i).second + ")";
      initial_condition.push_back(init_obj);
    }
    myfile << std::endl;
    myfile << "	)";
    myfile << std::endl;

    //
    goal.problem.initial = initial_condition;

    //the goal part of pddl's problem
    myfile << "	(:goal" << std::endl;
    myfile << "               (and " << std::endl;

    //
    std::string goal_obj = "(and";

    for(int i=0; i < goalObjLocation.size(); ++i)
    {
      myfile << "          	    (obj-at-location " << goalObjLocation.at(i).first << " " << goalObjLocation.at(i).second << ")" << std::endl;
      //
      goal_obj = goal_obj + "(obj-at-location " + goalObjLocation.at(i).first + " " + goalObjLocation.at(i).second + ")";
    }
    myfile << "               )" << std::endl;
    myfile << "	)" << std::endl;
    myfile << ")" ;
    myfile.close(); 

    
    goal_obj = goal_obj + ")";
    goal.problem.goal = goal_obj;
    //goal.problem.metric = "ipc seq-sat-lama-2011";
    //FINE PROBLEM MSG

    //INIZIO DOMAIN MSG
    goal.domain.name = "spqr-task-planning";
    goal.domain.requirements = ":typing :action-costs";


    std::vector<std::string> domain_predicates;
    std::string simple_predicate = "(location ?loc)";
    domain_predicates.push_back(simple_predicate);
    simple_predicate = "(OBJ ?o)";
    domain_predicates.push_back(simple_predicate);
    simple_predicate = "(ROBOT ?r)";
    domain_predicates.push_back(simple_predicate);
    simple_predicate = "(SLOT ?sl)";
    domain_predicates.push_back(simple_predicate);
    simple_predicate = "(emptySLOT ?sl)";
    domain_predicates.push_back(simple_predicate);
    simple_predicate = "(obj-at-location ?o ?loc)";
    domain_predicates.push_back(simple_predicate);
    simple_predicate = "(onSLOT ?o ?sl)";
    domain_predicates.push_back(simple_predicate);
    simple_predicate = "(robot-at-location ?r ?loc)";
    domain_predicates.push_back(simple_predicate);
    simple_predicate = "(edge ?loc1 ?loc2)";
    domain_predicates.push_back(simple_predicate);
  
    goal.domain.predicates = domain_predicates;

    std::vector<pddl_msgs::PDDLAction> actionsDomain;
    pddl_msgs::PDDLAction action;
    action.name = "TAKE-OBJ-FROM-LOCATION";
    action.parameters = "(?r ?o ?loc ?sl)";
    action.precondition = "(and (ROBOT ?r)(location ?loc)(OBJ ?o)(SLOT ?sl)(obj-at-location ?o ?loc)(robot-at-location ?r ?loc)(emptySLOT ?sl))";
    action.effect = """(and (not(obj-at-location ?o ?loc))(not(emptySLOT ?sl))(onSLOT ?o ?sl))""";
    actionsDomain.push_back(action);
    action.name = "DROP-OBJECT";
    action.parameters = "(?r ?o ?loc ?sl)";
    action.precondition = "(and (ROBOT ?r)(location ?loc)(OBJ ?o)(SLOT ?sl)(onSLOT ?o ?sl)(robot-at-location ?r ?loc)(not(emptySLOT ?sl)))";
    action.effect = """(and (obj-at-location ?o ?loc)(emptySLOT ?sl)(not(onSLOT ?o ?sl)))""";
    actionsDomain.push_back(action);
    action.name = "MOVE";
    action.parameters = "(?r ?pFrom ?pTo)";
    action.precondition = "(and (ROBOT ?r)(location ?pFrom)(location ?pTo)(edge ?pFrom ?pTo)(robot-at-location ?r ?pFrom)(not(robot-at-location ?r ?pTo)))";
    action.effect = """(and (not(robot-at-location ?r ?pFrom))(robot-at-location ?r ?pTo))""";
    actionsDomain.push_back(action);

    goal.domain.actions = actionsDomain;
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(5.0));
 

    //becco l'ultimo plan
    DIR *dp;
    struct dirent *dirp;
    std::string plan_file;

    std::string name_plan = "plan";
    std::string number_plan = "5";
    bool finished_search = false;
    double ros_wait = ros::Time::now().toSec();
    double actual_time = 0.0;
    int plan_iterator = 0;
    while(!finished_search && (actual_time < 10.0)) {
      actual_time += ros::Time::now().toSec() - ros_wait;
      std::cout << "sono a " << actual_time << std::endl;
      ros::Duration(0.5).sleep();
      if((dp  = opendir(path_finalPlan.c_str())) == NULL) {
        std::cout << "Error" << std::endl;
      }
      while ((dirp = readdir(dp)) != NULL) {
        std::string temp = std::string(dirp->d_name);
        if (temp.find(name_plan) != std::string::npos) {
          //plan_file = temp;
          std::cout << "ok " << temp << std::endl;
          std::vector<std::string> strs_plan;
          boost::split(strs_plan,temp,boost::is_any_of("."));
          int actual = std::atoi(strs_plan[1].c_str());
          std::cout << "per ora vince " << plan_iterator << std::endl;
          if(actual > plan_iterator) {
            plan_iterator = actual;
            plan_file = temp;
          }
          if (temp.find(number_plan) != std::string::npos) {
            finished_search = true;
            std::cout << "trovato " << temp << std::endl;
          }  
        }    
      }
      closedir(dp); 
    }

    std::cout << "cancello e ottengo " << plan_file << std::endl;
    ac.cancelAllGoals();
    int result_roskill = system("rosnode kill pddl_planner");
    int result_pkill = system("pkill downward");
    //


    //ottimizzo il path
    std::string move_action = "move";
    std::string drop_action = "drop";
    std::string take_action = "take";
    std::string ws_eliminate = "ws";
    std::string slot_eliminate = "slot";

    std::vector<std::string> final_action_temp;
    std::ifstream filefinal_plan(path_finalPlan + plan_file);

    for( std::string line; getline( filefinal_plan, line );){
      boost::erase_all(line, "(");
      boost::erase_all(line, ")");
      boost::erase_all(line, "youbot");
      if (line.find(move_action) != std::string::npos) {
            std::vector<std::string> strs;
            boost::split(strs,line,boost::is_any_of(" "));
            final_action_temp.push_back(strs[0] + " " + strs[3]);
            std::cout << line << std::endl;
      }
      else if (line.find(drop_action) != std::string::npos) {
            std::vector<std::string> strs;
            boost::split(strs,line,boost::is_any_of(" "));
            final_action_temp.push_back("drop " + strs[2] + " " + strs[3]);
            std::cout << "drop " + strs[2] + " " + strs[3] << std::endl;
      } 
      else {
            std::vector<std::string> strs;
            boost::split(strs,line,boost::is_any_of(" "));
            final_action_temp.push_back("take " + strs[2] + " " + strs[3]);
            std::cout << "take " + strs[2] + " " + strs[3] << std::endl;
      } 
    }
    filefinal_plan.close();

    std::cout << "  " << std::endl;
    std::cout << "  POI " << std::endl;
    std::cout << "  " << std::endl;
    std::vector<std::string> final_action;
    std::string temp;

    for(int i = 0; i < final_action_temp.size();i++) {
      //se è move
      if (final_action_temp[i].find(move_action) != std::string::npos) {
        temp = final_action_temp[i];
        int j = (i+1);
        if(j >= final_action_temp.size()) {
            final_action.push_back(temp);
            std::cout << temp << std::endl;
        }
        //itero per le restanti locazioni
        for(j = (i+1); j < final_action_temp.size();j++) {
          //se becco un altra move
          if ((final_action_temp[j].find(move_action) != std::string::npos)) {
            temp = final_action_temp[j];
            i = j;
          }
          //se non la becco
          else {
            final_action.push_back(temp);
            std::cout << temp << std::endl;
            break;
          }
        }
      }
      //se è drop
      else if (final_action_temp[i].find(drop_action) != std::string::npos){
        std::vector<std::string> strs_initial;
        std::vector<std::string> strs;
        boost::split(strs_initial,final_action_temp[i],boost::is_any_of(" "));
        temp = strs_initial[0] + " " + strs_initial[1];
        int j = (i+1);
        if(j >= final_action_temp.size()) {
            temp += " " + strs_initial[2];
            final_action.push_back(temp);
            std::cout << temp << std::endl;
        }
        //itero per le restanti locazioni
        for(j = (i+1); j < final_action_temp.size();j++) {
          //se becco un altra drop
          if ((final_action_temp[j].find(drop_action) != std::string::npos)) {
            boost::split(strs,final_action_temp[j],boost::is_any_of(" "));
            temp += " " + strs[1];
            i = j;
          }
          //se non la becco
          else {
            temp += " " + strs_initial[2];
            final_action.push_back(temp);
            std::cout << temp << std::endl;
            break;
          }
        }
      }
      else {
        std::vector<std::string> strs_initial;
        std::vector<std::string> strs;
        boost::split(strs_initial,final_action_temp[i],boost::is_any_of(" "));
        temp = strs_initial[0] + " " + strs_initial[1];
        int j = (i+1);
        if(j >= final_action_temp.size()) {
            temp += " " + strs_initial[2];
            final_action.push_back(temp);
            std::cout << temp << std::endl;
        }
        //itero per le restanti locazioni
        for(j = (i+1); j < final_action_temp.size();j++) {
          //se becco un altra drop
          if ((final_action_temp[j].find(take_action) != std::string::npos)) {
            boost::split(strs,final_action_temp[j],boost::is_any_of(" "));
            temp += " " + strs[1];
            i = j;
          }
          //se non la becco
          else {
            temp += " " + strs_initial[2];
            final_action.push_back(temp);
            std::cout << temp << std::endl;
            break;
          }
        }
      }
    }
    //

    std::cout << "PDDL PROBLEM FILE WRITTEN SUCCESFULLY" << std::endl;
    return final_action;
    
}
