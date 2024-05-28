#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <limits>
#include <unordered_map>

#define TIME_STEP 64
#define lBL_Strength_ON 750
#define lBL_Strength_OFF 0

using namespace webots;

// Structure to represent a node in the graph
struct Node {
  int x;
  int y;
  std::vector<Node*> neighbors;
};

// Function to calculate the Euclidean distance between two nodes
double distance(const Node* node1, const Node* node2) {
  return std::sqrt(std::pow(node2->x - node1->x, 2) + std::pow(node2->y - node1->y, 2));
}

// Dijkstra algorithm implementation
std::vector<Node*> dijkstra(const Node* start, const Node* end) {
  std::priority_queue<std::pair<double, Node*>, std::vector<std::pair<double, Node*>>, std::greater<std::pair<double, Node*>>> pq;
  std::unordered_map<Node*, double> distances;
  std::unordered_map<Node*, Node*> previous;

  for (const auto& neighbor : start->neighbors) {
    double dist = distance(start, neighbor);
    pq.push({dist, neighbor});
    distances[neighbor] = dist;
    previous[neighbor] = start;
  }

  while (!pq.empty()) {
    Node* current = pq.top().second;
    pq.pop();

    if (current == end)
      break;

    for (const auto& neighbor : current->neighbors) {
      double dist = distances[current] + distance(current, neighbor);
      if (!distances.count(neighbor) || dist < distances[neighbor]) {
        distances[neighbor] = dist;
        previous[neighbor] = current;
        pq.push({dist, neighbor});
      }
    }
  }

  std::vector<Node*> path;
  Node* current = end;
  while (current != start) {
    path.push_back(current);
    current = previous[current];
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());

  return path;
}

int main(int argc, char** argv) {
  Supervisor* supervisor = new Supervisor();
  Node* Pickup_node = supervisor->getSelf();
  
  // Define other nodes and their connections to represent the graph
  Node node0 {0, 0};
  Node node1 {1, 0};
  Node node2 {2, 0};
  Node node3 {3, 0};
  Node node4 {4, 0};
  Node node5 {5, 0};
  Node node6 {6, 0};
  Node node7 {7, 0};
  Node node8 {8, 0};
  Node node9 {9, 0};
  Node node10 {9, 1};
  Node node11 {9, 2};
  Node node12 {9, 3};
  Node node13 {9, 4};
  Node node14 {9, 5};
  
  node0.neighbors = {&node1};
  node1.neighbors = {&node0, &node2};
  node2.neighbors = {&node1, &node3};
  node3.neighbors = {&node2, &node4};
  node4.neighbors = {&node3, &node5};
  node5.neighbors = {&node4, &node6};
  node6.neighbors = {&node5, &node7};
  node7.neighbors = {&node6, &node8};
  node8.neighbors = {&node7, &node9};
  node9.neighbors = {&node8, &node10};
  node10.neighbors = {&node9, &node11};
  node11.neighbors = {&node10, &node12};
  node12.neighbors = {&node11, &node13};
  node13.neighbors = {&node12, &node14};
  node14.neighbors = {&node13};
  
  const Node* startNode = &node0;
  const Node* targetNode = &node14;

  std::vector<Node*> path = dijkstra(startNode, targetNode);

  // Display the path
  std::cout << "Path: ";
  for (const auto& node : path) {
    std::cout << "(" << node->x << ", " << node->y << ") ";
  }
  std::cout << std::endl;

  delete supervisor;
  return 0;
}
