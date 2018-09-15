#ifndef DISTANCE_MATRIX_H
#define DISTANCE_MATRIX_H
#include <iostream>
#include <stdlib.h>
#include <graph.h>
#include <string>
#include <fstream>
#include <sstream>
#include <climits>

using namespace std;

class DistanceMatrix
{
private:
    //n = num nodes
    //e = num edges
    int n,e;
    //intialization of the distance matrix
    float cost[1000][1000];
    // final distance matrix
    float DM[1000][1000];
    Graph graph;
public:
    DistanceMatrix();
    void readedges();
    void AllShortestPaths();
    void showDistanceMatrix();
    void DMllDistanceMatrixPaths();
    float getDistance(string name_node1, string name_node2);
    int name2id(string name);
    Graph getGraphImported();
};

#endif
