#include <vector>
#include <limits>

using namespace std;

/*** VERTEX ***/
class Vertex
{
public:
    Vertex(int x,int y);
    int _x;
    int _y;
    vector< vector <int> > _neighborsIdx;
    bool checkNeighbors(int idx);
};

Vertex::Vertex(int x,int y): _x(x), _y(y){}

bool Vertex::checkNeighbors(int idx)
{
    bool found = false;
    for(int i=0;i<_neighborsIdx.size();i++)
    {
        if(_neighborsIdx[i][0] == idx)
            found = true;
    }
    return found;
}



/*** Node ***/
class Node
{
public:
    Node(int);
    int _idx;
    int _weight;
    bool _alreadySeen;
    int _antecedent;
};

Node::Node(int idx): _idx(idx)
{
    _weight = -1;
    _alreadySeen = false;
    _antecedent = -1;
}



/*** DIJKSTRA ***/
class Dijkstra
{
public:
    Dijkstra();
    vector<Vertex> _vertices;
    vector<Node> _nodes;
    void initNodes(vector<Vertex>);
    void initPath(int,int);
    int _start,_end,_lastIdx;
    int findSmallestWeight();
    int loop();
    void getPath(vector<Vertex>*);
};

Dijkstra::Dijkstra(){

}

void Dijkstra::initNodes(vector<Vertex> vertices)
{
    for(int i=0;i<vertices.size();i++)
        _vertices.push_back(vertices[i]);

    for(int i=0; i < _vertices.size(); i++)
        _nodes.push_back(Node(i));
}

void Dijkstra::initPath(int startIdx, int endIdx)
{
    _start = startIdx;
    _end = endIdx;
    _lastIdx = 0;
    _nodes[_start]._weight = 0;
}

int Dijkstra::findSmallestWeight()
{
    int weight = numeric_limits<int>::max();
    int idx = -1;
    for(int i=0;i<_nodes.size();i++)
    {
        if(_nodes[i]._alreadySeen == false && _nodes[i]._weight < weight && _nodes[i]._weight > -1 )
        {
            weight = _nodes[i]._weight;
            idx = i;
        }
    }
    return idx;
}

int Dijkstra::loop()
{
    int stopCondition = 0;
    // On recherche le noeud non parcouru ayant le poids le plus faible
    int idx = findSmallestWeight();

    if(idx == -1)
    {
        cout << "No solution" << endl;
        stopCondition = -1;
        return stopCondition;
    }

    if(idx == _end){
        stopCondition = 1;
    }
    _lastIdx = idx;
    _nodes[idx]._alreadySeen = true;
    Vertex v = _vertices[idx];

    // Calcul le nouveau poids des fils du noeud courant
    for(int i=0;i<v._neighborsIdx.size(); i++)
    {
        int childIdx = v._neighborsIdx[i][0];
        int childDistance = v._neighborsIdx[i][1];
        if(_nodes[childIdx]._alreadySeen == false)
        {
            if(_nodes[childIdx]._weight == -1 ||(_nodes[idx]._weight + childDistance) < _nodes[childIdx]._weight)
            {
                _nodes[childIdx]._weight = _nodes[idx]._weight + childDistance;
                _nodes[childIdx]._antecedent = idx;
            }
        }
    }
    return stopCondition;
}

void Dijkstra::getPath(vector<Vertex> *path)
{
    int idx = _end;
    path->push_back(_vertices[idx]);
    while(idx != _start)
    {
        idx = _nodes[idx]._antecedent;
        path->push_back(_vertices[idx]);
    }
}


