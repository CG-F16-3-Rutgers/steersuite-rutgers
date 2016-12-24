//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#define HEURISTIC_WEIGHT 1
#define DIAGONAL_COST 1

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		std::cout<<"\nIn A*"<<'\n';
		gSpatialDatabase = _gSpatialDatabase;

		//Initialize f and g map scores
		std::map<int, AStarPlannerNode*> nodeMap;
		for (int i = 0; i < gSpatialDatabase->getNumCellsX(); ++i) {
			for (int j = 0; j < gSpatialDatabase->getNumCellsZ(); ++j) {
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				nodeMap[index] = new AStarPlannerNode(getPointFromGridIndex(index), (double)INFINITY, (double)INFINITY, nullptr);
			}
		}
		int startID = gSpatialDatabase->getCellIndexFromLocation(start);
		int goalID = gSpatialDatabase->getCellIndexFromLocation(goal);
		(*nodeMap[startID]).g = 0;
		(*nodeMap[startID]).f = (*nodeMap[startID]).g + HEURISTIC_WEIGHT*heuristic(startID, goalID);

		//std::set<int> openSet;
		//openSet.insert(startID);
		add(startID, nodeMap);
		while (size > 0) {
			//Find node in openset with smallest f value
			//int currentNode = getCurrentNode(openSet, nodeMap);
			int currentNode = getmin(nodeMap);
			//Add to closedset, remove from openset
			closedSet.insert(currentNode);
			//openSet.erase(openSet.find(currentNode));

			//Check if we reached the goal
			if (currentNode == goalID) {
				return reconstruct_path(agent_path, currentNode, nodeMap);
			}

			//Search through neighbors, calculate g,f scores, add to openset
			expand(currentNode, goalID, closedSet, nodeMap);
		}


		//std::cout<<"\nIn A*";

		return false;
	}

	double AStarPlanner::heuristic(int startIndex, int endIndex) {
		//If method is true, use Euclidean, else use Manhattan
		bool method = true;
		unsigned int startx, startz, endx, endz;
		gSpatialDatabase->getGridCoordinatesFromIndex(startIndex, startx, startz);
		gSpatialDatabase->getGridCoordinatesFromIndex(endIndex, endx, endz);
		if (method) {
			return ((double)sqrt((startx - endx)*(startx - endx) + (startz - endz)*(startz - endz)));
		}
		else {
			return ((abs((double)startx - endx) + abs((double)startz - endz)));
		}
	}

	bool AStarPlanner::reconstruct_path(std::vector<Util::Point>& agent_path, int currentNode, std::map<int, AStarPlannerNode*> nodeMap) {
		AStarPlannerNode* temp = nodeMap[currentNode];
		while ((*temp).parent) {
			temp = (*temp).parent;
			agent_path.insert(agent_path.begin(), (*temp).point);
		}
		agent_path.insert(agent_path.begin(), (*temp).point);
		//std::cout<<"\nPath length: "<<reverse.size()<<'\n';
		return true;
	}

	int AStarPlanner::getCurrentNode(std::set<int> openset, std::map<int, AStarPlannerNode*> nodeMap) {
		std::set<int>::iterator it;
		double temp = INFINITY;
		//If bigger is true, larger g scores have precedence, else smaller scores have precedence
		for (std::set<int>::iterator i = openset.begin(); i != openset.end(); ++i) {
			if ((*nodeMap[(*i)]).f < temp) {
				temp = (*nodeMap[(*i)]).f;
				it = i;
			}
			else if ((*nodeMap[(*i)]).f == temp) {
				if ((*nodeMap[(*it)]).g < (*nodeMap[(*i)]).g) {
					it = i;

				}
			}
				
		}
		return (*it);
	}

	void AStarPlanner::expand(int currentNode, int goalIndex,std::set<int> closedset, std::map<int, AStarPlannerNode*>& nodeMap) {
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(currentNode, x, z);
		for (int i = MAX(x - 1, 0); i<MIN(x + 2, gSpatialDatabase->getNumCellsX()); i += GRID_STEP) {
			for (int j = MAX(z - 1, 0); j<MIN(z + 2, gSpatialDatabase->getNumCellsZ()); j += GRID_STEP) {
				int neighbor = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				if (canBeTraversed(neighbor) && closedset.count(neighbor) == 0) {
					double tempg;
					if ((i == x) || (j == z)) {
						tempg = (*nodeMap[currentNode]).g + (DIAGONAL_COST*gSpatialDatabase->getTraversalCost(neighbor));
					}
					else {
						tempg = (*nodeMap[currentNode]).g + gSpatialDatabase->getTraversalCost(neighbor);
					}
					if (tempg < (*nodeMap[neighbor]).g) {
						(*nodeMap[neighbor]).g = tempg;
						(*nodeMap[neighbor]).f = (*nodeMap[neighbor]).g + HEURISTIC_WEIGHT*heuristic(neighbor, goalIndex);
						if (find(neighbor) != -1) {
							erase(find(neighbor),nodeMap);
						}
						add(neighbor, nodeMap);
						(*nodeMap[neighbor]).parent = nodeMap[currentNode];
					}
				}
			}
		}
	}
	int AStarPlanner::find(int i) {
		for (int i = 1; i <= size; i++) {
			if (heap[i] == i) {
				return i;
			}
			else {
				return -1;
			}
		}
	};
	void AStarPlanner::erase(int i, std::map<int, AStarPlannerNode*>& nodeMap) {
		heap[i] = heap[size];
		sinkdown(i, nodeMap);
		size--;
	};

	void AStarPlanner::add(int i, std::map<int, AStarPlannerNode*>& nodeMap) {
		heap[size + 1] = i;
		bubbleup(i, nodeMap);
		size++;
	}

	int AStarPlanner::getmin(std::map<int, AStarPlannerNode*>& nodeMap) {
		int temp = heap[1];
		heap[1] = heap[size];
		std::cout << "\n getmin excution: " << '\n';
		sinkdown(1, nodeMap);
		size--;
		return temp;
	};

	void AStarPlanner::bubbleup(int i, std::map<int, AStarPlannerNode*>& nodeMap) {
		while (i > 1) {
			if ((*nodeMap[i]).f > (*nodeMap[i / 2]).f) {
				break;
			}
			else if ((*nodeMap[i]).f == (*nodeMap[i / 2]).f) {
				if ((*nodeMap[i]).g < (*nodeMap[i / 2]).g) {
					int temp = heap[i / 2];
					heap[i / 2] = heap[i];
					heap[i] = temp;
					i = i / 2;
				}
				else {
					break;
				}

			}
			else {
				int temp = heap[i / 2];
				heap[i / 2] = heap[i];
				heap[i] = temp;
				i = i / 2;
			}

		}
	};
	void AStarPlanner::sinkdown(int i, std::map<int, AStarPlannerNode*>& nodeMap) {
		while (i * 2 <= size) {
			if (i * 2 == size) {
				if ((*nodeMap[i]).f > (*nodeMap[i * 2]).f) {
					int temp = heap[i * 2];
					heap[i * 2] = heap[i];
					heap[i] = temp;
					i = i * 2;
				}
				else if ((*nodeMap[i]).f = (*nodeMap[i * 2]).f && (*nodeMap[i]).g > (*nodeMap[i * 2]).g) {
					int temp = heap[i * 2];
					heap[i * 2] = heap[i];
					heap[i] = temp;
					i = i * 2;
				}
				else {
					break;
				}

			}
			else {
				int smallerone;
				if ((*nodeMap[i * 2]).f == (*nodeMap[i * 2 + 1]).f) {
					if ((*nodeMap[i * 2]).g <= (*nodeMap[i * 2 + 1]).f) {
						smallerone = i * 2;
					}
					else {
						smallerone = i * 2 + 1;
					}
					if ((*nodeMap[i * 2]).f < (*nodeMap[i * 2 + 1]).f) {
						smallerone = i * 2;
					}
					else {
						smallerone = i * 2 + 1;
					}
					//compare i with smaller one;
					if ((*nodeMap[i]).f < (*nodeMap[smallerone]).f) {
						break;
					}
					else if ((*nodeMap[i]).f == (*nodeMap[smallerone]).f) {
						if ((*nodeMap[i]).g >(*nodeMap[smallerone]).g) {
							int temp = heap[smallerone];
							heap[smallerone] = heap[i];
							heap[i] = temp;
							i = smallerone;
						}
					}
					else {
						int temp = heap[smallerone];
						heap[smallerone] = heap[i];
						heap[i] = temp;
						i = smallerone;
					}
				}
			}
		};

	}
}