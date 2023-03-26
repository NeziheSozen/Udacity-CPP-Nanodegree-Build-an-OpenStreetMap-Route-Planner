#include "route_planner.h"
#include <algorithm>

bool isNodesDifferent(RouteModel::Node *a, RouteModel::Node *b)
{
   return a->h_value + a->g_value > b->h_value + b->g_value;
}

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node   = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node){
 current_node->FindNeighbors();
 for (auto& neighbor_node : current_node-> neighbors)
 {
        neighbor_node->parent = current_node;
        neighbor_node->g_value = current_node->g_value + neighbor_node->distance(*current_node);
        neighbor_node->h_value = this->CalculateHValue(neighbor_node);
        if (neighbor_node->visited != true)
        {
           neighbor_node->visited = true;
           this->open_list.push_back(neighbor_node);
        }
  }
}

RouteModel::Node *RoutePlanner::NextNode() {
sort(open_list.begin(), open_list.end(), [] (const RouteModel::Node* a, const RouteModel::Node* b) 
     {return a->g_value + a->h_value > b->g_value + b->h_value;});
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    float current_x = current_node->x;
    float current_y = current_node->y;
    path_found.push_back(*current_node);

    while (current_node != this->start_node)
    {
        distance = distance + current_node->distance(*current_node->parent);
        current_node = current_node->parent;
        current_x = current_node->x;
        current_y = current_node->y;
        path_found.push_back(*current_node);
    }

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    current_node->visited = true;

    while(current_node != end_node){
        AddNeighbors(current_node);
        current_node = NextNode();

    }
    m_Model.path = ConstructFinalPath(current_node);
}
