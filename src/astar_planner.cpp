#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <octomap_astar_ros/astar_planner.hpp>
#include <memory>
#include <set>
#include <octomap/OcTree.h>
#include <unordered_set>
#include <cmath>
#include <tuple>

namespace navigation
{

bool Node::operator==(const Node &other) const {
  return key == other.key;
}

bool Node::operator!=(const Node &other) const {
  return key != other.key;
}

bool Node::operator<(const Node &other) const {

  if (total_cost == other.total_cost) {
    return goal_dist < other.goal_dist;
  }

  return total_cost < other.total_cost;
}

bool Node::operator<=(const Node &other) const {

  if (total_cost == other.total_cost) {
    return goal_dist <= other.goal_dist;
  }

  return total_cost <= other.total_cost;
}

bool CostComparator::operator()(const Node &n1, const Node &n2) const {

  if (n1.total_cost == n2.total_cost) {
    return n1.goal_dist > n2.goal_dist;
  }

  return n1.total_cost > n2.total_cost;
}

bool HashFunction::operator()(const Node &n) const {
  using std::hash;
  return ((hash<int>()(n.key.k[0]) ^ (hash<int>()(n.key.k[1]) << 1)) >> 1) ^ (hash<int>()(n.key.k[2]) << 1);
}

bool LeafComparator::operator()(const std::pair<octomap::OcTree::iterator, double> &l1, const std::pair<octomap::OcTree::iterator, double> &l2) const {
  return l1.second < l2.second;
}

struct OcTreeKeyComparator {
    bool operator()(const octomap::OcTreeKey& lhs, const octomap::OcTreeKey& rhs) const {
        if (lhs.k[0] != rhs.k[0]) return lhs.k[0] < rhs.k[0];
        if (lhs.k[1] != rhs.k[1]) return lhs.k[1] < rhs.k[1];
        return lhs.k[2] < rhs.k[2];
    }
};

std::vector<octomap::OcTreeKey> collision_nodes;
/* AstarPlanner constructor //{ */
AstarPlanner::AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty,
                           double greedy_penalty, double min_altitude, double max_altitude, double timeout_threshold, double max_waypoint_distance,
                           bool unknown_is_occupied) {

  this->safe_obstacle_distance    = safe_obstacle_distance;
  this->euclidean_distance_cutoff = euclidean_distance_cutoff;
  this->planning_tree_resolution  = planning_tree_resolution;
  this->distance_penalty          = distance_penalty;
  this->greedy_penalty            = greedy_penalty;
  this->min_altitude              = min_altitude;
  this->max_altitude              = max_altitude;
  this->timeout_threshold         = timeout_threshold;
  this->max_waypoint_distance     = max_waypoint_distance;
  this->unknown_is_occupied       = unknown_is_occupied;
}
//}

/* findPath //{ */

bool AstarPlanner::isPartOfWall(octomap::OcTree& tree, const octomap::OcTreeKey& key) {
    int count = 0;
    double resolution = tree.getResolution();
    for (int dz = -5; dz <= 5; ++dz) {
        octomap::OcTreeKey neighborKey = key;
        neighborKey.k[2] += dz * resolution;
        if (tree.search(neighborKey) && tree.isNodeOccupied(tree.search(neighborKey))) {
            count++;
            if (count >= 10) {
                return true;
            }
        } else {
            count = 0;
        }
    }
    return false;
}

void AstarPlanner::fillWalls(octomap::OcTree& tree) {
    double resolution = tree.getResolution();
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);

    for (double x = min_x; x <= max_x; x += resolution) {
        for (double y = min_y; y <= max_y; y += resolution) {
            int occupiedCount = 0;
            int freeCount = 0;
            std::vector<octomap::OcTreeKey> keys;

            for (double z = min_z; z <= max_z; z += resolution) {
                octomap::OcTreeKey key;
                if (tree.coordToKeyChecked(octomap::point3d(x, y, z), key)) {
                    keys.push_back(key);
                    auto node = tree.search(key);
                    if (node && tree.isNodeOccupied(node)) {
                        occupiedCount++;
                    } else {
                        freeCount++;
                    }
                }
            }

            if (occupiedCount > freeCount && occupiedCount >= 10) {
                for (const auto& key : keys) {
                    tree.updateNode(key, true);
                }
            }
        }
    }
}
void AstarPlanner::inflateWalls(octomap::OcTree& tree, double inflation_radius) {
    std::set<octomap::OcTreeKey, OcTreeKeyComparator> wallNodes;

    // Identify wall nodes
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
        if (tree.isNodeOccupied(*it)) {
            octomap::OcTreeKey key = it.getKey();
            if (isPartOfWall(tree, key)) {
              wallNodes.insert(key);
            }
        }
    }

    // Inflate only wall nodes
    for (const auto& key : wallNodes) {
        octomap::point3d coord = tree.keyToCoord(key);
        for (double dx = -inflation_radius; dx <= inflation_radius; dx += tree.getResolution()) {
            for (double dy = -inflation_radius; dy <= inflation_radius; dy += tree.getResolution()) {
                for (double dz = -inflation_radius; dz <= inflation_radius; dz += tree.getResolution()) {
                    octomap::point3d inflated_coord = coord + octomap::point3d(dx, dy, dz);
                    tree.updateNode(inflated_coord, true); // Mark as occupied
                }
            }
        }
    }
}

void AstarPlanner::close_holes(octomap::OcTree& tree) {
    std::vector<octomap::OcTreeKey> freeNodes;

    // Identify free nodes
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
        if (!tree.isNodeOccupied(*it)) {
            freeNodes.push_back(it.getKey());
        }
    }

    // Check the next 5 nodes on top for each free node
    for (const auto& key : freeNodes) {
        bool shouldMarkOccupied = false;
        for (int i = 1; i <= 10; ++i) {
            octomap::OcTreeKey topKey = key;
            topKey.k[2] += i; // Move up in the z-direction

            octomap::OcTreeNode* node = tree.search(topKey);
            if (node && tree.isNodeOccupied(node)) {
                shouldMarkOccupied = true;
                break;
            }
        }

        if (shouldMarkOccupied) {
            octomap::point3d coord = tree.keyToCoord(key);
            tree.updateNode(coord, true); // Mark as occupied
        }
    }
}

void AstarPlanner::inflateFloor(octomap::OcTree& tree, double inflation_radius_floor) {
    std::set<octomap::OcTreeKey, OcTreeKeyComparator> freeNodes;

    // Identify free nodes
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
        if (!tree.isNodeOccupied(*it)) {
            freeNodes.insert(it.getKey());
        }
    }

    double resolution = tree.getResolution();

    // Check neighbors and set them to free if the condition is met
    for (const auto& key : freeNodes) {
        std::vector<octomap::OcTreeKey> directions = {
          {key.k[0] + resolution, key.k[1], key.k[2]}, // right
          {key.k[0] - resolution, key.k[1], key.k[2]}, // left
          {key.k[0], key.k[1] + resolution, key.k[2]}, // front
          {key.k[0], key.k[1] - resolution, key.k[2]}  // back
        };

        for (const auto& direction : directions) {
            if (tree.search(direction) == nullptr) { // Check if node is neither free nor occupied
                octomap::OcTreeKey currentKey = direction;
                int steps = 0;
                while (tree.search(currentKey) == nullptr && steps < 5) { // Traverse until an occupied or free node is found or limit is reached
                    tree.updateNode(tree.keyToCoord(currentKey), false); // Mark as free
                    currentKey.k[0] += (direction.k[0] - key.k[0]);
                    currentKey.k[1] += (direction.k[1] - key.k[1]);
                    steps++;
                }
            }
        }
    }
}

std::pair<std::vector<octomap::point3d>, PlanningResult> AstarPlanner::findPath(
    const octomap::point3d &start_coord, const octomap::point3d &goal_coord, const octomap::point3d &pos_cmd, std::shared_ptr<octomap::OcTree> mapping_tree,
    double timeout, std::function<void(const octomap::OcTree &, std::vector<octomap::OcTreeKey>&)> visualizeTree,
    std::function<void(const std::unordered_set<Node, HashFunction> &, const std::unordered_set<Node, HashFunction> &, const octomap::OcTree &)>
        visualizeExpansions) {

  printf("[Astar]: Astar: start [%.2f, %.2f, %.2f]\n", start_coord.x(), start_coord.y(), start_coord.z());
  printf("[Astar]: Astar: goal [%.2f, %.2f, %.2f]\n", goal_coord.x(), goal_coord.y(), goal_coord.z());

  auto time_start = std::chrono::high_resolution_clock::now();
  collision_nodes.clear();
  this->timeout_threshold = timeout;

  //inflateFloor(*mapping_tree, 0.025);

  double occupancy_threshold = mapping_tree->getOccupancyThres();

  //inflateWalls(*mapping_tree, 0.025);

  auto time_start_planning_tree = std::chrono::high_resolution_clock::now();
  auto tree_with_tunnel         = createPlanningTree(mapping_tree, start_coord, planning_tree_resolution);
  // printf("[Astar]: the planning tree took %.2f s to create\n",
  //        std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start_planning_tree).count());


  RCLCPP_INFO(rclcpp::get_logger("Astar"), "The planning tree took %.2f s to create, mapping tree size: %zu",
              std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start_planning_tree).count(),
              mapping_tree->size());

  if (!tree_with_tunnel) {
    RCLCPP_ERROR(rclcpp::get_logger("Astar"), "Could not create a planning tree");
    return {std::vector<octomap::point3d>(), FAILURE};
  }

  auto time_start_visualize = std::chrono::high_resolution_clock::now();

  //visualizeTree((*tree_with_tunnel).first, collision_nodes);

  RCLCPP_INFO(rclcpp::get_logger("Astar"), "The visualization took %.2f s",
            std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start_visualize).count());

  auto tree   = tree_with_tunnel.value().first;
  auto tunnel = tree_with_tunnel.value().second;

  auto map_goal      = goal_coord;
  auto map_query     = tree_with_tunnel->first.search(goal_coord);
  bool original_goal = true;

  if (map_query == NULL) {
    printf("[Astar]: Goal is outside of map\n");
    auto temp_goal = generateTemporaryGoal(start_coord, goal_coord, pos_cmd, tree);
    printf("[Astar]: Generated a temporary goal: [%.2f, %.2f, %.2f]\n", temp_goal.first.x(), temp_goal.first.y(), temp_goal.first.z());
    if (temp_goal.second) {
      std::vector<octomap::point3d> vertical_path;
      vertical_path.push_back(start_coord);
      vertical_path.push_back(temp_goal.first);
      //visualizeExpansions(open, closed, tree);
      visualizeTree((*tree_with_tunnel).first, collision_nodes);
      return {vertical_path, INCOMPLETE};
    } else {
      map_goal      = temp_goal.first;
      original_goal = false;
    }
  } else if (map_query->getValue() == TreeValue::OCCUPIED) {
    printf("[Astar]: Goal is inside an inflated obstacle\n");
    if (distEuclidean(map_goal, start_coord) <= 1 * safe_obstacle_distance) {
      printf("[Astar]: Path special case, we cannot get closer\n");
      return {std::vector<octomap::point3d>(), GOAL_REACHED};
    }
  }

  std::priority_queue<Node, std::vector<Node>, CostComparator> open_heap;
  std::unordered_set<Node, HashFunction>                       open;
  std::unordered_set<Node, HashFunction>                       closed;
  std::unordered_map<Node, Node, HashFunction>                 parent_map;  // first = child, second = parent

  octomap::OcTreeKey start;
  if (tunnel.empty()) {
    start = tree.coordToKey(start_coord);
  } else {
    start = tree.coordToKey(tunnel.back());
  }

  auto planning_start = tree.keyToCoord(start);
  auto goal           = tree.coordToKey(map_goal);

  if (distEuclidean(planning_start, map_goal) <= 2 * planning_tree_resolution) {

    printf("[Astar]: Path special case, we are there\n");

    //visualizeExpansions(open, closed, tree);
    visualizeTree((*tree_with_tunnel).first, collision_nodes);
    return {std::vector<octomap::point3d>(), GOAL_REACHED};
  }

  std::cout << "[Astar]: Planning from: " << planning_start.x() << ", " << planning_start.y() << ", " << planning_start.z() << "\n";
  std::cout << "[Astar]: Planning to: " << map_goal.x() << ", " << map_goal.y() << ", " << map_goal.z() << "\n";

  Node first;
  first.key        = start;
  first.cum_dist   = 0;
  first.goal_dist  = distEuclidean(start, goal, tree);
  first.total_cost = first.cum_dist + first.goal_dist;
  open_heap.push(first);
  open.insert(first);

  Node best_node        = first;
  Node best_node_greedy = first;

  Node last_closed;

  while (!open.empty() && rclcpp::ok()) {

    Node current = open_heap.top();
    open_heap.pop();
    open.erase(current);
    closed.insert(current);

    last_closed = current;

    auto time_now = std::chrono::high_resolution_clock::now();

    if (std::chrono::duration<double>(time_now - time_start).count() > timeout_threshold) {

      printf("[Astar]: Planning timeout! Using current best node as goal.\n");
      auto path_keys = backtrackPathKeys(best_node == first ? best_node_greedy : best_node, first, parent_map);
      printf("[Astar]: Path found. Length: %ld\n", path_keys.size());

      //visualizeExpansions(open, closed, tree);
      visualizeTree((*tree_with_tunnel).first, collision_nodes);
      return {prepareOutputPath(path_keys, tree), INCOMPLETE};
    }

    auto current_coord = tree.keyToCoord(current.key);

    if (distEuclidean(current_coord, map_goal) <= 2 * planning_tree_resolution) {

      auto path_keys = backtrackPathKeys(current, first, parent_map);
      path_keys.push_back(tree.coordToKey(map_goal));
      printf("[Astar]: Path found. Length: %ld\n", path_keys.size());

      //visualizeExpansions(open, closed, tree);
      visualizeTree((*tree_with_tunnel).first, collision_nodes);
      if (original_goal) {
        return {prepareOutputPath(path_keys, tree), COMPLETE};
      }
      return {prepareOutputPath(path_keys, tree), INCOMPLETE};
    }

    // Expand
    //printf("[Astar]: Expanding node at key (%d, %d, %d)\n", current.key[0], current.key[1], current.key[2]);
    //printf("[Astar]: Current coordinate: (%f, %f, %f)\n", current_coord.x(), current_coord.y(), current_coord.z());
    //collision_nodes.clear();
    RobotDimensions robot_dims = {1.1, .5, 1.1}; //{.3, .3, .3};// Example dimensions
    auto neighbors = getNeighbors(current.key, tree, robot_dims);
    //printf("[Astar]: Number of neighbors found: %ld\n", neighbors.size());

    for (auto &nkey : neighbors) {

      Node n;
      n.key = nkey;

      auto closed_query = closed.find(n);
      auto open_query   = open.find(n);

      // in open map
      n.goal_dist  = distEuclidean(nkey, goal, tree);
      n.cum_dist   = current.cum_dist + distEuclidean(current.key, nkey, tree);
      n.total_cost = greedy_penalty * n.goal_dist + distance_penalty * n.cum_dist;

      if (closed_query == closed.end() && open_query == open.end()) {

        if (n <= best_node) {
          best_node = n;
        }

        if (n.goal_dist <= best_node_greedy.goal_dist) {
          best_node_greedy = n;
        }

        open_heap.push(n);
        open.insert(n);
        parent_map[n] = current;
      }
    }
  }

  //visualizeExpansions(open, closed, tree);

  if (best_node != first) {

    auto path_keys = backtrackPathKeys(best_node, first, parent_map);
    visualizeTree((*tree_with_tunnel).first, collision_nodes);
    printf("[Astar]: direct path does not exist, going to the 'best_node'\n");

    return {prepareOutputPath(path_keys, tree), INCOMPLETE};
  }

  if (best_node_greedy != first) {

    auto path_keys = backtrackPathKeys(best_node_greedy, first, parent_map);
    visualizeTree((*tree_with_tunnel).first, collision_nodes);
    printf("[Astar]: direct path does not exist, going to the best_node_greedy'\n");

    return {prepareOutputPath(path_keys, tree), INCOMPLETE};
  }

  if (!tunnel.empty()) {
    std::vector<octomap::point3d> path_to_safety;
    path_to_safety.push_back(start_coord);
    path_to_safety.push_back(tunnel.back());
    printf("[Astar]: path does not exist, escaping no-go zone'\n");
    //visualizeExpansions(open, closed, tree);
    visualizeTree((*tree_with_tunnel).first, collision_nodes);
    return {path_to_safety, INCOMPLETE};
  }
  visualizeTree((*tree_with_tunnel).first, collision_nodes);
  printf("[Astar]: PATH DOES NOT EXIST!\n");

  return {std::vector<octomap::point3d>(), FAILURE};
}
//}

/* getNeighborhood() //{ */

std::vector<octomap::OcTreeKey> AstarPlanner::getNeighborhood(const octomap::OcTreeKey &key, octomap::OcTree &tree) {

  std::vector<octomap::OcTreeKey> neighbors;

  for (auto &d : EXPANSION_DIRECTIONS) {

    auto newkey    = expand(key, d);
    auto tree_node = tree.search(newkey);

    if (tree_node != NULL) {
      // free cell?
      if (tree_node->getValue() == TreeValue::FREE && tree.keyToCoord(newkey).z() >= min_altitude && tree.keyToCoord(newkey).z() <= max_altitude) {
        neighbors.push_back(newkey);
      }
    }
  }

  return neighbors;
}

std::vector<octomap::OcTreeKey> AstarPlanner::getNeighbors(const octomap::OcTreeKey &key, const octomap::OcTree &tree, const RobotDimensions &robot_dims) {
  std::vector<octomap::OcTreeKey> neighbors;

  auto max_expansion = 24;
  int collision_free_count = 0;

  for (auto &d : EXPANSION_DIRECTIONS) {
    auto newkey = expand(key, d);
    auto tree_node = tree.search(newkey);
    // && tree.keyToCoord(newkey).z() >= min_altitude && tree.keyToCoord(newkey).z() <= max_altitude
    if (tree_node != NULL) {
        if (tree_node->getValue() == TreeValue::FREE && tree.keyToCoord(newkey).z() >= min_altitude && tree.keyToCoord(newkey).z() <= max_altitude) {
          //collision_nodes.push_back(newkey);
        //RCLCPP_INFO(rclcpp::get_logger("Astar"), "Condition met: key = (%d, %d, %d), ", key.k[0], key.k[1], key.k[2]);
            if (isCollisionFree(tree, newkey, robot_dims)) {
              neighbors.push_back(newkey);
              collision_free_count++;
                //if (collision_free_count >= max_expansion) {
                    //return neighbors;
                //}
            }
            //continue;
        }
    }
  }
  if (neighbors.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("Astar"), "No neighbors found for key: (%d, %d, %d)", key.k[0], key.k[1], key.k[2]);
  }
  return neighbors;
}


bool AstarPlanner::isCollisionFree(const octomap::OcTree &tree, const octomap::OcTreeKey &key, const RobotDimensions &robot_dims) {
    double resolution = tree.getResolution();
    auto first_node = tree.search(key);
    octomap::point3d key_coord = tree.keyToCoord(key);
            // RCLCPP_INFO(rclcpp::get_logger("Astar"), "Check collision for = (%f, %f, %f), occupancy = %f",
            //     key_coord.x(), key_coord.y(), key_coord.z(), first_node->getOccupancy());

    double half_width = robot_dims.width / 2.0;
    double half_height = robot_dims.height / 2.0;
    double half_length = robot_dims.length / 2.0;

    std::set<octomap::OcTreeKey, OcTreeKeyComparator> expanded_bounding_nodes;
    std::vector<octomap::OcTreeKey> upward_nodes;

    int num_nodes_width = static_cast<int>(std::ceil(half_width / resolution));
    int num_nodes_length = static_cast<int>(std::ceil(half_length / resolution));
    int num_nodes_height = static_cast<int>(std::ceil(half_height / resolution));

    int num_nodes_full_height = static_cast<int>(std::ceil(robot_dims.height / resolution));

    for (int l = 1; l <= num_nodes_length; ++l) {
      for (int h = 0; h <= num_nodes_full_height; ++h) {
          // Check the minimum width boundary
          std::vector<int> min_width_direction = {-num_nodes_width, l, h};
          octomap::OcTreeKey min_width_expanded_key = expand(key, min_width_direction);
          expanded_bounding_nodes.insert(min_width_expanded_key);

          // Check the maximum width boundary
          std::vector<int> max_width_direction = {num_nodes_width, l, h};
          octomap::OcTreeKey max_width_expanded_key = expand(key, max_width_direction);
          expanded_bounding_nodes.insert(max_width_expanded_key);
      }
    }

    for (auto &d : OCTREE_NEIGHBORS_NO_NEG_Z) {
      auto neigbor_key = expand(key, d);
      auto tree_node = tree.search(neigbor_key);
      if (tree_node == NULL) {
        continue;
      }
      if(tree_node->getValue() == TreeValue::OCCUPIED) {
        collision_nodes.push_back(neigbor_key);
        return false;
      }
    }
    // Check each boundary point
    for (const auto& bounding_point_key : expanded_bounding_nodes) {

        auto node = tree.search(bounding_point_key);
        if (node == NULL) {
            for (auto &d : OCTREE_NEIGHBORS_NO_NEG_Z) {
              auto neigbour_bounding_point_key = expand(bounding_point_key, d);
              auto tree_node = tree.search(neigbour_bounding_point_key);
              if (tree_node == NULL) {
                continue;
              }
              if (tree_node->getValue() == TreeValue::OCCUPIED) {
                collision_nodes.push_back(neigbour_bounding_point_key);
                collision_nodes.push_back(bounding_point_key);
                return false;
              }
            }
          continue;
        }
        //octomap::point3d coord = tree.keyToCoord(node_key);


        if (node->getValue() == TreeValue::OCCUPIED) {
            octomap::point3d coord = tree.keyToCoord(bounding_point_key);
            collision_nodes.push_back(bounding_point_key);
            return false;
        }
    }
    return true;
}

/* expand() //{ */

octomap::OcTreeKey AstarPlanner::expand(const octomap::OcTreeKey &key, const std::vector<int> &direction) {

  octomap::OcTreeKey k;

  k.k[0] = key.k[0] + direction[0];
  k.k[1] = key.k[1] + direction[1];
  k.k[2] = key.k[2] + direction[2];

  //RCLCPP_INFO(rclcpp::get_logger("Astar"), "expand = (%d, %d, %d), ", k.k[0], k.k[1], k.k[2]);

  return k;
}

//}

/* distEuclidean() //{ */

double AstarPlanner::distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2) {

  return (p1 - p2).norm();
}

double AstarPlanner::distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree) {

  double voxel_dist = sqrt(pow(k1.k[0] - k2.k[0], 2) + pow(k1.k[1] - k2.k[1], 2) + pow(k1.k[2] - k2.k[2], 2));

  return voxel_dist * tree.getResolution();
}

//}

/* freeStraightPath() //{ */

bool AstarPlanner::freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::OcTree &tree) {

  //auto edf = euclideanDistanceTransform(tree);
  auto edf = euclideanDistanceTransform(std::make_shared<octomap::OcTree>(tree));
  octomap::KeyRay ray;
  tree.computeRayKeys(p1, p2, ray);
  double inflation_radius = std::sqrt(std::pow(0.25 / 2.0, 2) + std::pow(0.25 / 2.0, 2) + std::pow(0.5 / 2.0, 2));

  //RCLCPP_INFO(rclcpp::get_logger("astar_planner"), "Checking path from p1: (%f, %f, %f) to p2: (%f, %f, %f)", p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z());

  for (auto &k : ray) {

    auto tree_node = tree.search(k);

    if (tree_node == NULL) {
      // Path may exist, but goes through unknown cells
      return false;
    }

    if (tree_node->getValue() == TreeValue::OCCUPIED) {
      // Path goes through occupied cells
      return false;
    }
  }

  return true;
}

//}

/* backtrackPathKeys() //{ */

std::vector<octomap::OcTreeKey> AstarPlanner::backtrackPathKeys(const Node &from, const Node &to, std::unordered_map<Node, Node, HashFunction> &parent_map) {

  std::vector<octomap::OcTreeKey> keys;

  Node current = from;
  keys.push_back(current.key);

  while (current.key != to.key) {
    current = parent_map.find(current)->second;
    keys.push_back(current.key);
  };

  keys.push_back(to.key);

  // reverse order
  std::reverse(keys.begin(), keys.end());
  return keys;
}

//}

/* keysToCoords() //{ */

std::vector<octomap::point3d> AstarPlanner::keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::OcTree &tree) {

  std::vector<octomap::point3d> coords;

  for (auto &k : keys) {
    coords.push_back(tree.keyToCoord(k));
  }

  return coords;
}

//}

/* euclideanDistanceTransform() //{ */

DynamicEDTOctomap AstarPlanner::euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree) {

  double x, y, z;

  tree->getMetricMin(x, y, z);
  octomap::point3d metric_min(x, y, z);

  tree->getMetricMax(x, y, z);
  octomap::point3d metric_max(x, y, z);

  DynamicEDTOctomap edf(euclidean_distance_cutoff, tree.get(), metric_min, metric_max, unknown_is_occupied);
  edf.update();

  return edf;
}
//}

/* createPlanningTree() //{ */

std::optional<std::pair<octomap::OcTree, std::vector<octomap::point3d>>> AstarPlanner::createPlanningTree(std::shared_ptr<octomap::OcTree> tree,
                                                                                                          const octomap::point3d &start, double resolution) {

  auto            edf         = euclideanDistanceTransform(tree);
  octomap::OcTree binary_tree = octomap::OcTree(resolution);

  tree->expand();

  for (auto it = tree->begin(); it != tree->end(); it++) {
    if (edf.getDistance(it.getCoordinate()) <= safe_obstacle_distance) {
      binary_tree.setNodeValue(it.getCoordinate(), TreeValue::OCCUPIED);  // obstacle or close to obstacle
    } else {
      binary_tree.setNodeValue(it.getCoordinate(), TreeValue::FREE);  // free and safe
    }
  }

  std::vector<octomap::point3d> tunnel;

  octomap::point3d current_coords    = start;
  auto             binary_tree_query = binary_tree.search(current_coords);

  if (binary_tree_query != NULL && binary_tree_query->getValue() != TreeValue::FREE) {

    printf("[Astar]: start is inside of an inflated obstacle, tunneling out\n");

    // tunnel out of expanded walls

    int iter1 = 0;

    while (rclcpp::ok() && binary_tree_query != NULL && iter1++ <= 100) {

      if (iter1++ > 100) {
        return {};
      }

      tunnel.push_back(current_coords);
      binary_tree.setNodeValue(current_coords, TreeValue::FREE);

      float            obstacle_dist;
      octomap::point3d closest_obstacle;

      edf.getDistanceAndClosestObstacle(current_coords, obstacle_dist, closest_obstacle);
      octomap::point3d dir_away_from_obstacle = current_coords - closest_obstacle;

      if (obstacle_dist >= safe_obstacle_distance) {
        printf("[Astar]: tunnel created with %d\n", int(tunnel.size()));
        break;
      }

      current_coords += dir_away_from_obstacle.normalized() * float(binary_tree.getResolution());

      int iter2 = 0;

      while (binary_tree.search(current_coords) == binary_tree_query) {

        if (iter2++ > 100) {
          return {};
        }

        current_coords += dir_away_from_obstacle.normalized() * float(binary_tree.getResolution());
      }

      binary_tree_query = binary_tree.search(current_coords);
    }
  }

  std::pair<octomap::OcTree, std::vector<octomap::point3d>> result = {binary_tree, tunnel};

  return result;
}

//}

/* filterPath() //{ */

std::vector<octomap::point3d> AstarPlanner::filterPath(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree) {

  if (waypoints.size() < 3) {
    printf("[Astar]: Not enough points for filtering!\n");
    return waypoints;
  }

  /* removing obsolete points //{ */

  std::vector<octomap::point3d> filtered;

  filtered.push_back(waypoints.front());

  size_t k = 2;

  while (k < waypoints.size()) {

    if (!freeStraightPath(filtered.back(), waypoints[k], tree)) {
      filtered.push_back(waypoints[k - 1]);
    }

    k++;
  }

  filtered.push_back(waypoints.back());
  //}

  return filtered;
}
//}

/* prepareOutputPath() //{ */

std::vector<octomap::point3d> AstarPlanner::prepareOutputPath(const std::vector<octomap::OcTreeKey> &keys, octomap::OcTree &tree) {
  auto waypoints = keysToCoords(keys, tree);
  auto processed = filterPath(waypoints, tree);

  return processed;
}  // namespace navigation
//}

/* generateTemporaryGoal() //{ */

std::pair<octomap::point3d, bool> AstarPlanner::generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal,
                                                                      const octomap::point3d &pos_cmd,  octomap::OcTree &tree) {

  bool             vertical_priority = false;
  octomap::point3d temp_goal;

  // check if it is necessary to change altitude and acquire more of map
  if (std::abs(goal.z() - start.z()) > planning_tree_resolution) {
    vertical_priority = true;
    printf("[Astar]: give priority to vertical motion\n");
    temp_goal.x() = start.x();
    temp_goal.y() = start.y();
    // temp_goal.z() = goal.z();  // scan new layers of octomap if needed

    double extra_motion = goal.z() - start.z();
    extra_motion        = (extra_motion / std::abs(extra_motion)) * planning_tree_resolution;

    // double alt_diff = pos_cmd.z() - start.z();  // odometry and desired altitude may differ

    temp_goal.z()   = goal.z() + extra_motion;      // scan new layers of octomap in that case

    if (temp_goal.z() > max_altitude) {
      printf("[Astar]: capping at max altitude\n");
      temp_goal.z() = max_altitude;
    }
    if (temp_goal.z() < min_altitude) {
      printf("[Astar]: capping at min altitude\n");
      temp_goal.z() = min_altitude;
    }
    return {temp_goal, vertical_priority};
  }

  // try to explore unknown cells
  std::set<std::pair<octomap::OcTree::iterator, double>, LeafComparator> leafs;

  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); it++) {

    if (it->getValue() == TreeValue::OCCUPIED) {
      continue;
    }

    auto k = it.getKey();
    k.k[2] += 1;

    if (tree.search(k) == NULL) {
      continue;
    }

    k.k[2] -= 2;

    if (tree.search(k) == NULL) {
      continue;
    }

    leafs.insert({it, distEuclidean(it.getCoordinate(), goal)});
  }

  // sort free nodes on the map edge by their distance from goal
  if (!leafs.empty()) {
    // select the closest point
    return {leafs.begin()->first.getCoordinate(), vertical_priority};
  }

  // solution that is only good for smaller obstacles
  octomap::KeyRay ray;
  tree.computeRayKeys(start, goal, ray);

  for (auto &k : ray) {
    auto coords = tree.keyToCoord(k);
    if (tree.search(coords) != NULL && tree.search(coords)->getValue() == TreeValue::FREE) {
      temp_goal = coords;
    }
  }

  return {temp_goal, vertical_priority};
}
//}

}  // namespace navigation
