#include <pch.h>
#include <math.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion


//find cheapest and pop
Node* AStarPather::findCheapestInOpenList() {

    int index = 0;
    Node* ret = new Node();

    for (int i = smallestBucketIndex - 5; i != openListBuckets; ++i) {
        if (i < 0)
            i = 0;
        std::vector<Node*>::iterator it;
        if (!openList[i].empty()) {
            for (it = openList[i].begin(); it != openList[i].end(); ) {
                if (ret->costSoFar + ret->heuristicCost > (*it)->costSoFar + (*it)->heuristicCost) {
                    ret = (*it);
                }
                else
                    ++it;
            }
            smallestBucketIndex = i;
            break;
        }
    }
    return ret;
}


/*
void AStarPather::removeFromClosedList(int row, int col) {

    for (int i = 0; i < closedList.size(); ++i) {
        if (closedList[i]->row == row && closedList[i]->col == col) {
            terrain->set_color(closedList[i]->row, closedList[i]->col, Colors::White);
            closedList.erase(closedList.begin() + i);
        }
    }
}*/
inline void AStarPather::insertChildToOpenList() {
    mChild->isInOpenList = true;
    openList[(int)((mChild->costSoFar + mChild->heuristicCost) * 10)].push_back(mChild);
    elementsInOpenList++;
}

void AStarPather::removeFromOpenList(int row, int col, float cost) {
    int i = (int)((cost) * 10);
    if (!openList[i].empty()) {
        std::vector<Node*>::iterator it;
        for (it = openList[i].begin(); it != openList[i].end(); ) {
            if ((*it)->row == row && (*it)->col == col) {
                if (debugColoring)
                    terrain->set_color((*it)->row, (*it)->col, Colors::White);
                openList[i].erase(it);
                elementsInOpenList--;
                return;
            }
            else
                ++it;
        }
    }
}



inline float AStarPather::computeHeuristic(int currRow, int currCol, int goalRow, int goalCol) {
    if (mHeuristic == Heuristic::OCTILE)
        return computeOctile(currRow, currCol, goalRow, goalCol);

    if (mHeuristic == Heuristic::MANHATTAN)
        return computeManhattan(currRow, currCol, goalRow, goalCol);

    if (mHeuristic == Heuristic::CHEBYSHEV)
        return computeChebyshev(currRow, currCol, goalRow, goalCol);

    if (mHeuristic == Heuristic::EUCLIDEAN)
        return computeEuclidean(currRow, currCol, goalRow, goalCol);

    return -1.0f;
}

inline float AStarPather::computeOctile(int currRow, int currCol, int goalRow, int goalCol) {
    float diffx = std::abs((float)currCol - (float)goalCol);
    float diffy = std::abs((float)currRow - (float)goalRow);
    return 1.001f * (std::min(diffx, diffy) * 1.41421356237f + std::max(diffx, diffy) - std::min(diffx, diffy));
}


inline float AStarPather::computeManhattan(int currRow, int currCol, int goalRow, int goalCol) {
    return (float)(abs(currCol - goalCol) + abs(currRow - goalRow));
}


inline float AStarPather::computeChebyshev(int currRow, int currCol, int goalRow, int goalCol) {

    return (float)std::max(abs(currCol - goalCol), abs(currRow - goalRow));
}


inline float AStarPather::computeEuclidean(int currRow, int currCol, int goalRow, int goalCol) {

    return (float)sqrt(abs(currCol - goalCol) * abs(currCol - goalCol) + abs(currRow - goalRow) * abs(currRow - goalRow));
}


void AStarPather::rubberBand() {
    GridPos third = GridPos{ goal.row, goal.col };
    GridPos second = GridPos{ grid[third.row][third.col].parentRow, grid[third.row][third.col].parentCol };
    GridPos first = GridPos{ grid[second.row][second.col].parentRow, grid[second.row][second.col].parentCol };

    if (first.col == -10 || second.col == -10)
        return;

    int rStart, rEnd, cStart, cEnd;
    bool noWalls = true;
    bool lastTimeWasNoWalls = false;

    while (first.row != start.row || first.col != start.col) {
        rStart = std::min(first.row, third.row);
        rEnd = std::max(first.row, third.row);
        cStart = std::min(first.col, third.col);
        cEnd = std::max(first.col, third.col);
        noWalls = true;

        if (rStart <= second.row && second.row <= rEnd
            && cStart <= second.col && second.col <= cEnd) {
            for (int r = rStart; r <= rEnd; ++r)
                for (int c = cStart; c <= cEnd; ++c) {
                    if (terrain->is_wall(GridPos{ r, c }))
                        noWalls = false;
                }
        }

        if (noWalls && (!lastTimeWasNoWalls || !smoothing)) {
            lastTimeWasNoWalls = true;
            second = first;
            grid[third.row][third.col].parentRow = first.row;
            grid[third.row][third.col].parentCol = first.col;
            first = GridPos{ grid[first.row][first.col].parentRow, grid[first.row][first.col].parentCol };
        }
        else {
            lastTimeWasNoWalls = false;
            third = GridPos{ grid[third.row][third.col].parentRow, grid[third.row][third.col].parentCol };
            second = GridPos{ grid[third.row][third.col].parentRow, grid[third.row][third.col].parentCol };
            first = GridPos{ grid[second.row][second.col].parentRow, grid[second.row][second.col].parentCol };
        }
    }
}


void AStarPather::smooth() {

    Vec3 fourth = terrain->get_world_position(goal.row, goal.col);
    Vec3 third = fourth;
    Vec3 second = getWorldPosParent(third);
    Vec3 first = getWorldPosParent(second);
    Vec3 newPoint;
    smoothedParents.push_back(fourth);

    //DO FIRST THREE WHERE FIRST AND SECOND ARE SAME
    for (float s = 0.75f; s > 0.0f; s -= 0.25f)
    {
        newPoint = Vec3::CatmullRom(first, second, third, fourth, s);
        smoothedParents.push_back(newPoint);
    }
    third = getWorldPosParent(third);
    second = getWorldPosParent(second);
    first = getWorldPosParent(first);

    //DO ALL THE THREES IN THE MIDDLE
    while (first != terrain->get_world_position(start.row, start.col)) {
        smoothedParents.push_back(third);

        for (float s = 0.75f; s > 0.0f; s -= 0.25f) {
            newPoint = Vec3::CatmullRom(first, second, third, fourth, s);
            smoothedParents.push_back(newPoint);
        }
        fourth = getWorldPosParent(fourth);
        third = getWorldPosParent(third);
        second = getWorldPosParent(second);
        first = getWorldPosParent(first);
    }

    //DO ONE BEFORE LAST THREE
    smoothedParents.push_back(third);
    for (float s = 0.75f; s > 0.0f; s -= 0.25f) {
        newPoint = Vec3::CatmullRom(first, first, third, fourth, s);
        smoothedParents.push_back(newPoint);
    }
    smoothedParents.push_back(first);
}


Vec3 AStarPather::getWorldPosParent(Vec3 child) {
    if (grid[terrain->get_grid_position(child).row][terrain->get_grid_position(child).col].parentRow != -10)
        return terrain->get_world_position(grid[terrain->get_grid_position(child).row][terrain->get_grid_position(child).col].parentRow,
            grid[terrain->get_grid_position(child).row][terrain->get_grid_position(child).col].parentCol);
    return child;
}

bool AStarPather::initialize()
{

    for (int i = 0; i != 40; ++i) // row
    {
        for (int j = 0; j != 40; ++j) // column
        {
            if (grid[i][j].isInClosedList || grid[i][j].isInOpenList) {
                grid[i][j].heuristicCost = 99999;
                grid[i][j].costSoFar = 99999;

                grid[i][j].isInOpenList = false;
                grid[i][j].isInClosedList = false;

                if (grid[i][j].row != i)
                    grid[i][j].row = i;

                if (grid[i][j].col != j)
                    grid[i][j].col = j;

                grid[i][j].parentRow = -10;
                grid[i][j].parentCol = -10;
            }
        }
    }

    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */

    for (int i = 0; i < openListBuckets; ++i) {
        openList[i].clear();
    }

    smoothedParents.clear();

}

PathResult AStarPather::compute_path(PathRequest& request)
{

    /*
        This is where you handle pathing requests, each request has several fields:

        start/goal - start and goal world positions
        path - where you will build the path upon completion, path should be
            start to goal, not goal to start
        heuristic - which heuristic calculation to use
        weight - the heuristic weight to be applied
        newRequest - whether this is the first request for this path, should generally
            be true, unless single step is on

        smoothing - whether to apply smoothing to the path
        rubberBanding - whether to apply rubber banding
        singleStep - whether to perform only a single A* step
        debugColoring - whether to color the grid based on the A* state:
            closed list nodes - yellow
            open list nodes - blue

            use terrain->set_color(row, col, Colors::YourColor);
            also it can be helpful to temporarily use other colors for specific states
            when you are testing your algorithms

        method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
            will be A* generally, unless you implement extra credit features

        The return values are:
            PROCESSING - a path hasn't been found yet, should only be returned in
                single step mode until a path is found
            COMPLETE - a path to the goal was found and has been built in request.path
            IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
    */

    if (request.newRequest) {
        for (int i = 0; i < openListBuckets; ++i) {
            openList[i].clear();
        }
        elementsInOpenList = 0;
        start.costSoFar = 0;
        start.row = terrain->get_grid_position(request.start).row;
        start.col = terrain->get_grid_position(request.start).col;
        goal.row = terrain->get_grid_position(request.goal).row;
        goal.col = terrain->get_grid_position(request.goal).col;


        if (start.row == goal.row && start.col == goal.col) { // if bot start and goal are the same
            request.path.push_back(request.start);
            request.path.push_back(request.goal);
            return PathResult::COMPLETE;
        }

        mWeight = request.settings.weight;
        mHeuristic = request.settings.heuristic;
        debugColoring = request.settings.debugColoring;
        singleStep = request.settings.singleStep;
        smoothing = request.settings.smoothing;
        rubberBanding = request.settings.rubberBanding;

        if (debugColoring) {
            terrain->set_color(start.row, start.col, Colors::Orange);
            terrain->set_color(goal.row, goal.col, Colors::Orange);
        }

        boardHeight = terrain->get_map_height();
        boardWidth = terrain->get_map_width();

        int i = 0 , j;
        for (i ; i != boardHeight; ++i) // row
        {
            for (j = 0; j != boardWidth; ++j) // column
            {
                if (grid[i][j].isInClosedList || grid[i][j].isInOpenList) {
                    grid[i][j].heuristicCost = grid[i][j].costSoFar = 99999;
                    grid[i][j].isInClosedList = grid[i][j].isInOpenList = false;
                    grid[i][j].parentCol = grid[i][j].parentRow = -10;
                }
            }
        }
        smallestBucketIndex = 5;

        mChild = &start;
        mChild->parentCol = -10;
        mChild->parentRow = -10;
        mChild->heuristicCost = computeHeuristic(mChild->row, mChild->col, goal.row, goal.col) * mWeight;
        insertChildToOpenList();
    }


    while (elementsInOpenList != 0) {
        currNode = findCheapestInOpenList(); // current node considered
        removeFromOpenList(currNode->row, currNode->col, currNode->costSoFar + currNode->heuristicCost);

        if (currNode->row == goal.row && currNode->col == goal.col) { // We re done

            if (start.row == goal.row && start.col == goal.col) { // if both start and goal are the same
                request.path.push_back(request.start);
                request.path.push_back(request.goal);
                return PathResult::COMPLETE;
            }


            if (rubberBanding) // if rubberband
                rubberBand();

            Node pos = grid[goal.row][goal.col];

            if (smoothing) { // if smooth
                smooth(); 
                while (!smoothedParents.empty())
                {
                    request.path.push_back(smoothedParents.back());
                    smoothedParents.pop_back();
                }
                return PathResult::COMPLETE;
            }

            while (pos.row != start.row || pos.col != start.col) { // if no smooth
                mPathStackROW.push(pos.row);
                mPathStackCOL.push(pos.col);
                pos = grid[pos.parentRow][pos.parentCol];
            }
           // mPathStackROW.push(pos.row); // push the start
           // mPathStackCOL.push(pos.col ); // push the start

            request.path.push_back(request.start);

            while (!mPathStackROW.empty())
            {
                request.path.push_back(terrain->get_world_position(mPathStackROW.top(), mPathStackCOL.top()));
                mPathStackROW.pop();
                mPathStackCOL.pop();
            }
            return PathResult::COMPLETE;
        }

        up =  right = down = left = false; // if true, then it is a valid pos aka not wall

        //TRY ALL CHILDREN START
        // try up
        if (currNode->row != boardHeight-1 && currNode->parentRow-1 != currNode->row) { // isnt at the end of the map and isnt going back same direction
            mChild = &grid[currNode->row+1][currNode->col];
            if (!terrain->is_wall(mChild->row, mChild->col)) {
                considerNeighbor( 1.0f, request.settings.heuristic, mWeight);
                up = true;
            }
        }

        //try right
        if (currNode->col != boardWidth - 1 && currNode->parentCol - 1 != currNode->col) { // isnt at the end of the map
            mChild = &grid[currNode->row][currNode->col+1];
            if (!terrain->is_wall(mChild->row, mChild->col)) {
                considerNeighbor( 1.0f, mHeuristic, mWeight);
                right = true;
            }
        }

        //try down
        if (currNode->row != 0 && currNode->parentRow + 1 != currNode->row) { // isnt at the end of the map
            mChild = &grid[currNode->row - 1][currNode->col];
            if (!terrain->is_wall(mChild->row, mChild->col)) {
                considerNeighbor( 1.0f, mHeuristic, mWeight);
                down = true;
            }
        }

        //try left
        if (currNode->col != 0 && currNode->parentCol + 1 != currNode->col) { // isnt at the end of the map
            mChild = &grid[currNode->row][currNode->col-1];
            if (!terrain->is_wall(mChild->row, mChild->col)) {
                considerNeighbor( 1.0f, mHeuristic, mWeight);
                left = true;
            }
        }

        //try top-right
        if (up && right) {
            mChild = &grid[currNode->row + 1][currNode->col+1];
            if (!terrain->is_wall(mChild->row, mChild->col)) {
                considerNeighbor( 1.4142857f, mHeuristic, mWeight);
            }
        }

        //try top-left
        if (up && left) {
            mChild = &grid[currNode->row + 1][currNode->col-1];
            if (!terrain->is_wall(mChild->row, mChild->col)) {
                considerNeighbor( 1.4142857f, mHeuristic, mWeight);
            }
        }

        //try bot-right
        if (down && right) {
            mChild = &grid[currNode->row - 1][currNode->col+1];
            if (!terrain->is_wall(mChild->row, mChild->col)) {
                considerNeighbor( 1.4142857f, mHeuristic, mWeight);
            }
        }

        //try bot-left
        if (down && left) {
            mChild = &grid[currNode->row - 1][currNode->col-1];
            if (!terrain->is_wall(mChild->row, mChild->col)) {
                considerNeighbor( 1.4142857f, mHeuristic, mWeight);
            }
        }
        //TRY ALL CHILDREN END
        
        currNode->isInClosedList = true;

        if (debugColoring)
            terrain->set_color(currNode->row, currNode->col, Colors::Yellow);

        if(singleStep)
            return PathResult::PROCESSING;

    }
    return PathResult::IMPOSSIBLE;

}

void AStarPather::considerNeighbor(float addedCost, Heuristic h, float weight) {
    mChild->heuristicCost = computeHeuristic(mChild->row, mChild->col, goal.row, goal.col) * mWeight;
    mNeighborCost = currNode->costSoFar + addedCost + mChild->heuristicCost;

    if (!mChild->isInOpenList && !mChild->isInClosedList) {
        mChild->costSoFar = currNode->costSoFar + addedCost;
        insertChildToOpenList();
        mChild->parentRow = currNode->row;
        mChild->parentCol = currNode->col;

        if (debugColoring)
            terrain->set_color(mChild->row, mChild->col, Colors::Blue);
    }
    else if (mNeighborCost < mChild->costSoFar + mChild->heuristicCost) {
        if (mChild->isInOpenList)
            removeFromOpenList(mChild->row, mChild->col, mChild->costSoFar + mChild->heuristicCost);

        mChild->costSoFar = currNode->costSoFar + addedCost;
        mChild->parentRow = currNode->row;
        mChild->parentCol = currNode->col;

        mChild->isInClosedList = false;

        insertChildToOpenList();
        mChild->isInClosedList = false;

        if (debugColoring)
            terrain->set_color(mChild->row, mChild->col, Colors::Blue);
    }
    //printf("%i \n", elementsInOpenList);
}
