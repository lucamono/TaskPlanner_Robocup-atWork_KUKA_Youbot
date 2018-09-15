#include "DistanceMatrix.h"

DistanceMatrix::DistanceMatrix()
{
    cout<<"Loading the graph." << endl;
    graph.importGraph();
    cout<<"Graph imported succesfully." << endl;
    readedges();
    DMllDistanceMatrixPaths();


}

Graph DistanceMatrix::getGraphImported(){
	return this->graph;
}

void DistanceMatrix::showDistanceMatrix(){
    int i,j;
    cout<<"new distance matrix\n";
    for(i=0;i <= n;i++)
    {
        for(j=0;j <= n;j++)
        {
            cout<<DM[i][j]<<"\t";
        }
        cout<<"\n\n";
    }
}


void DistanceMatrix::readedges(){
    int i,j,v1,v2;
    n=graph.getNodeList().size();
    for(i=0;i < n;i++){
        for(j=0;j < n;j++)
            cost[i][j] = INT_MAX;
        cost[i][i]=0;
    }

    e=0;
    for(i=0; i < graph.getNodeList().size(); i++){
        e+=graph.getNodeList().at(i).list_id.size();
    }

    for(i=0;i < graph.getNodeList().size(); i++){
        v1=graph.getNodeList().at(i).id;
        for(j=0;j < graph.getNodeList().at(i).list_id.size(); j++){           
	    v2=graph.getNodeList().at(i).list_id.at(j);
            cost[v1][v2]=graph.getNodeList().at(i).distances.at(j);

        }
    }
}

void DistanceMatrix::DMllDistanceMatrixPaths(){
    int i,j,k;
    for(i=0;i < n;i++){
	//if i nodo che ce serve       
	 for(j=0;j < n;j++)
            DM[i][j]=cost[i][j];
    }
    for(k=0;k < n;k++){
        for(i=0;i < n;i++){
	//if i nodo che ce serve
            for(j=0;j < n;j++){
                if(DM[i][j] > (DM[i][k]+DM[k][j])){
		                    
			DM[i][j]=DM[i][k]+DM[k][j];
                }
            }
        }
    }
   
}

float DistanceMatrix::getDistance(string name_node1, string name_node2)
{
    int id1 = name2id(name_node1);
    int id2 = name2id(name_node2);
    return DM[id1][id2];
}

int DistanceMatrix::name2id(string name)
{
    int numNodes = graph.getNodeList().size();
    for(int i=0; i< numNodes; i++ )
    {
        if(graph.getNodeList().at(i).label == name){
            return graph.getNodeList().at(i).id;

        }
    }
    return -1;
}

