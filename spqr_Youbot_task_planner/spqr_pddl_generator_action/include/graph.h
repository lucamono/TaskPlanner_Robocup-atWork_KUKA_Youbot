#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <list>
#include <fstream>
#include <string>
#include <ros/package.h>


typedef std::vector <float> weights;
typedef std::vector <int> id_adjacency;

struct graphNode{
	int id;
	std::string label;
	float pos_x;
	float pos_y;
	id_adjacency list_id;
	weights distances;
};

typedef struct std::vector<graphNode> pGraph;

class Graph {

    private: 
	pGraph nodeList;

    public:
	Graph(){};
	pGraph getNodeList() { return nodeList; }
	void addNode(graphNode& newNode, std::vector<int> listId, int flag, std::string label="");
	graphNode getNode(int id) {return nodeList.at(id); }
	void removeNode(int id);
	void addArc(int curr,int succ);
	void removeArc(graphNode curr_node, graphNode succ_node);
	void exportGraph();
	pGraph importGraph();
};

#endif