#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <octomap/octomap.h>
#include <queue>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <optional>

namespace navigation
{

enum TreeValue
{
  OCCUPIED = 0,
  FREE     = 1
};

enum PlanningResult
{
  COMPLETE = 0,
  GOAL_REACHED,
  INCOMPLETE,
  GOAL_IN_OBSTACLE,
  FAILURE
};


struct Node
{
  octomap::OcTreeKey key;
  double             total_cost;
  double             cum_dist;
  double             goal_dist;
  bool               has_collision;

  bool operator==(const Node &other) const;
  bool operator!=(const Node &other) const;
  bool operator<(const Node &other) const;
  bool operator<=(const Node &other) const;
};

struct CostComparator
{
  bool operator()(const Node &n1, const Node &n2) const;
};

struct LeafComparator
{
  bool operator()(const std::pair<octomap::OcTree::iterator, double> &l1, const std::pair<octomap::OcTree::iterator, double> &l2) const;
};

struct HashFunction
{
  bool operator()(const Node &n) const;
};

struct RobotDimensions {
  double length;
  double width;
  double height;
};

class AstarPlanner {

public:
  AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty, double greedy_penalty,
               double min_altitude, double max_altitude, double timeout_threshold, double max_waypoint_distance, bool unknown_is_occupied);

private:
  double safe_obstacle_distance;
  double euclidean_distance_cutoff;
  double planning_tree_resolution;
  double distance_penalty;
  double greedy_penalty;
  double timeout_threshold;
  double max_waypoint_distance;
  double min_altitude;
  double max_altitude;
  bool   unknown_is_occupied;

public:
  std::pair<std::vector<octomap::point3d>, PlanningResult> findPath(
      const octomap::point3d &start_coord, const octomap::point3d &goal_coord, const octomap::point3d &pos_cmd, std::shared_ptr<octomap::OcTree> mapping_tree,
      double timeout, std::function<void(const octomap::OcTree &, std::vector<octomap::OcTreeKey>&)> visualizeTree,
      std::function<void(const std::unordered_set<Node, HashFunction> &, const std::unordered_set<Node, HashFunction> &, const octomap::OcTree &)>
          visualizeExpansions);

  std::optional<std::pair<octomap::OcTree, std::vector<octomap::point3d>>> createPlanningTree(std::shared_ptr<octomap::OcTree> tree,
                                                                                              const octomap::point3d &start, double resolution);
  void fillWalls(octomap::OcTree& tree);
  void inflateWalls(octomap::OcTree& tree, double inflation_radius);
  void inflateFloor(octomap::OcTree &tree, double inflation_radius_floor);
  void close_holes(octomap::OcTree &tree);
  bool isPartOfWall(octomap::OcTree &tree, const octomap::OcTreeKey &key);
  bool isCollisionFree(const octomap::OcTree &tree, const octomap::OcTreeKey &key, const RobotDimensions &robot_dims);

  static octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const std::vector<int> &direction);

  const std::vector<std::vector<int>> OCTREE_NEIGHBORS = {
      {-1, -1, -1}, {-1, -1, 1}, {-1, 1, -1}, {-1, 1, 1},
      {1, -1, -1},  {1, -1, 1},  {1, 1, -1},  {1, 1, 1}
    };

  private:
  const std::vector<std::vector<int>> EXPANSION_DIRECTIONS = {
    {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
    {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
    {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
    {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}
  };

  const std::vector<std::vector<int>> OCTREE_NEIGHBORS_NO_NEG_Z = {
    {-1, -1, 0}, {-1, -1, 1}, {-1, 1, 0}, {-1, 1, 1},
    {1, -1, 0},  {1, -1, 1},  {1, 1, 0},  {1, 1, 1}
  };



  double                              getNodeDepth(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> getNeighborhood(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> getNeighbors(const octomap::OcTreeKey &key, const octomap::OcTree &tree, const RobotDimensions &robot_dims);
  double calculateDistanceToOccupied(const octomap::OcTree &tree, const octomap::OcTreeKey &key);

  double distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2);

  double distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree);

  bool freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> backtrackPathKeys(const Node &start, const Node &end, std::unordered_map<Node, Node, HashFunction> &parent_map);

  std::vector<octomap::point3d> keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::OcTree &tree);

  DynamicEDTOctomap euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree);

  std::pair<octomap::point3d, bool> generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal, const octomap::point3d &pos_cmd, octomap::OcTree &tree);

  std::vector<octomap::point3d> filterPath(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree);

  std::vector<octomap::point3d> prepareOutputPath(const std::vector<octomap::OcTreeKey> &keys, octomap::OcTree &tree);

  /* geometry_msgs::msg::Quaternion yawToQuaternionMsg(const double &yaw); */
};

}  // namespace navigation
#endif
