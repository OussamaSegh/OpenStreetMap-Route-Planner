#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel& model, float start_x, float start_y, float end_x, float end_y): m_model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
  	/* Find our closest start and end node for the coordinates */
	start_node = &m_model.FindClosestNode(start_x, start_y);
	end_node = &m_model.FindClosestNode(end_x, end_y);
}


// the CalculateHValue method.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
return node->distance(*end_node);
}


// the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	/* Populate current node's neighbors vector */
	current_node->FindNeighbors();
	/* Go through each neighbor, calc its attributes and then add it to the open list */
	for(auto neighbor : current_node->neighbors)
	{
		neighbor->parent = current_node;
		neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
		neighbor->h_value = CalculateHValue(neighbor);
		open_list.emplace_back(neighbor);
		neighbor->visited = true;
	}
}


// NextNode method to sort the open list and return the next node.

RouteModel::Node *RoutePlanner::NextNode() {
	/* Sort our list of open nodes by their sum of h and g value */
	std::sort(open_list.begin(), open_list.end(), [](const auto & _1st, const auto & _2nd)
	{
		return (_1st->h_value + _1st->g_value) < (_2nd->h_value + _2nd->g_value);
	});
	/* Get the node with lowest value and remove it from the list */
	RouteModel::Node* lowest_node = open_list.front();
	open_list.erase(open_list.begin());
	return lowest_node;
}

// the ConstructFinalPath method to return the final path found from the A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) 
{
    // Create path_found vector
	distance = 0.0f;
	std::vector<RouteModel::Node> path_found;
	/* Repeat as long as we are not at the starting point */
	while(current_node->parent != nullptr)
	{
		path_found.emplace_back(*current_node);
		const RouteModel::Node parent = *(current_node->parent);
		distance += current_node->distance(parent);
		current_node = current_node->parent;
	}

	/* Add start node and multiply with metric scale to convert to meters */ 
    path_found.emplace_back(*current_node);
	distance *= m_model.MetricScale();
	return path_found;
}


// the A* Search algorithm here.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
  	/* Start with starting node */
	start_node->visited = true;
	open_list.emplace_back(start_node);
	/* Our algorithm is running here */
	while(!open_list.empty())
	{
		/* Find best next node for exploration */
		RouteModel::Node* current_node = NextNode();
		/* Check if we have reached our goal already */
		if(current_node->distance(*end_node) == 0)
		{
			m_model.path = ConstructFinalPath(end_node);
			return;
		}
		/* Otherwise, add our current node to the neighbors */
		AddNeighbors(current_node);
	}
}