#pragma once
#include "Misc/PathfindingDetails.hpp"

#include <deque>
#include <stack>

struct Node {
    float costSoFar = 99999;
    float heuristicCost = 99999;
    bool isInOpenList = true;
    bool isInClosedList = true;
    int row, col, parentRow, parentCol;
};

class AStarPather
{
public:
    /* 
        The class should be default constructible, so you may need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */
    inline void considerNeighbor(float addedCost, Heuristic h, float weight);

    Node* findCheapestInOpenList();
    //void removeFromClosedList(int row, int col); 
    inline void insertChildToOpenList();
    void removeFromOpenList(int row, int col, float cost);

    inline float computeHeuristic(int currRow, int currCol, int goalRow, int goalCol);
    inline float computeOctile(int currRow, int currCol, int goalRow, int goalCol);
    inline float computeManhattan(int currRow, int currCol, int goalRow, int goalCol);
    inline float computeChebyshev(int currRow, int currCol, int goalRow, int goalCol);
    inline float computeEuclidean(int currRow, int currCol, int goalRow, int goalCol);

    void rubberBand();
    void smooth();

    Vec3 getWorldPosParent(Vec3 child);

   // std::deque<Node*> openList;
    //std::deque<Node*> closedList;
    const static int openListBuckets = 1500;
    int elementsInOpenList = 0;
    int smallestBucketIndex = 5; // pointer to the first non-empty bucket

    int boardHeight, boardWidth;

    std::vector<Node*>openList[openListBuckets];
    Node grid[40][40];

    
    //GridPos parent[40][40];

    std::deque<Vec3> smoothedParents;
    Node start, goal;

    float mWeight;
    float mNeighborCost;
    Heuristic mHeuristic;
    Node* mChild;
    Node* currNode = new Node();
    bool up = false, right = false, down = false, left = false; // if true, then it is a valid pos aka not wall
    bool debugColoring;
    bool singleStep;
    bool smoothing;
    bool rubberBanding;

    std::stack<int> mPathStackROW;
    std::stack<int> mPathStackCOL;
};