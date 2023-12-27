#ifndef __POSE_GRAPH_H__
#define __POSE_GRAPH_H__

#include <map>

#include "manif/manif.h"

enum class NodeType { POSE, TAG };
enum class EdgeType { ODOM, DETECTION };

struct Node {
  int id;
  NodeType type;
  manif::SE3d transform;
};

struct Edge {
  int i, j;  // Node IDs
  EdgeType type;
  manif::SE3d transform;
};

struct PoseGraph {
  std::map<int, Node> nodes;
  std::map<std::pair<int, int>, Edge> edges;

  void add(const Node& node) { nodes[node.id] = node; }
  void add(const Edge& edge) { edges[{edge.i, edge.j}] = edge; }

  void remove(const int i) { nodes.erase(i); }
  void remove(const int i, const int j) { edges.erase({i, j}); }

  bool contains(const int i) { return nodes.count(i) > 0; }
  bool contains(const int i, const int j) { return edges.count({i, j}) > 0; }
};

#endif  // __POSE_GRAPH_H__
