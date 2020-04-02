#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);

}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return (node->distance(*end_node));     
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    auto neighbors_vector = current_node->neighbors;
    for(auto neighbor : neighbors_vector){
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;
        open_list.emplace_back(neighbor);
    }

}

bool Compare(const RouteModel::Node* node_a, const RouteModel::Node* node_b){
    float f1 = node_a->g_value + node_a->h_value;
    float f2 = node_b->g_value + node_b->h_value;
    return (f1 > f2); //high->low
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node* next_node = open_list.back();
    // open_list.erase(open_list.begin());
    open_list.pop_back();
    return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node->parent!=nullptr){
        path_found.insert(path_found.begin(), *current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;  
    }
    path_found.insert(path_found.begin(), *current_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    start_node->visited = true;
    AddNeighbors(start_node);
    RouteModel::Node *current_node = nullptr;

    while(open_list.size()>0){
        current_node = NextNode();
        if(current_node->distance(*end_node)==0){
            m_Model.path = ConstructFinalPath(current_node);      
        }
        else{
            AddNeighbors(current_node);
        }

    }

}