#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/msg/point.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <tf2/exceptions.h>

#include <iostream>
#include <functional>
#include "octomap_astar_ros/astar_planner.hpp"
// Output of path in ROS2 format
#include <nav_msgs/msg/path.hpp>
// tf2 Helper to calculate orientation from point list
#include <tf2/LinearMath/Quaternion.h>
#include <octomap_utils/octomap_calculations.hpp>
#include <octomap_utils/octomap_conversions.hpp>
#include <octomap_utils/octomap_mockups.hpp>
#include <octomap_utils/octomap_visualization.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <queue>
#include <set>

using namespace std::chrono_literals;


struct Point3DComparator
{
    bool operator()(const octomap::point3d& lhs, const octomap::point3d& rhs) const
    {
        if (lhs.x() != rhs.x())
            return lhs.x() < rhs.x();
        if (lhs.y() != rhs.y())
            return lhs.y() < rhs.y();
        return lhs.z() < rhs.z();
    }
};

class AstarPlannerNode : public rclcpp::Node {


rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr octomap_viz_pub_;
// Publisher für die Ziel-Marker
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;

// Subscriber für Start und Ziel-Position
rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _sub_pose3d_start_;
std::string _topic_pose3d_start = "astar_planner_node/in/pose3d/start";
octomap::point3d point3d_start;
octomap::point3d point3d_start_old;


rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _sub_pose3d_goal_;
std::string _topic_pose3d_goal = "astar_planner_node/in/pose3d/goal";
octomap::point3d point3d_goal;
octomap::point3d point3d_goal_old;


rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr _sub_octomap_;
std::string _topicIn_octomap = "astar_planner/in/octomap";
std::shared_ptr<octomap::OcTree> octree_;

rclcpp::TimerBase::SharedPtr _walltimer_plan_path_on_posChange_;


rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pub_localPath;
std::string _topic_localPath = "astar_planner_node/out/localPath";
nav_msgs::msg::Path localPath;

rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_path_service_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr explore_service_;

rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr update_pose_goal_pub_;
rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr update_pose_start_pub_;

std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;



    // Declare the parameters
    double safe_obstacle_distance;
    double euclidean_distance_cutoff;
    double planning_tree_resolution;
    double distance_penalty;
    double greedy_penalty;
    double min_altitude;
    double max_altitude;
    double planning_timeout;
    double max_waypoint_distance;
    bool unknown_is_occupied;

    // Add the planner as a member variable
    //navigation::AstarPlanner planner;

public:
    AstarPlannerNode() : Node("astar_planner_node") {
    //   safe_obstacle_distance(0.5),
    //   euclidean_distance_cutoff(1.0),
    //   planning_tree_resolution(0.1),
    //   distance_penalty(1.0),
    //   greedy_penalty(1.0),
    //   min_altitude(0.0),
    //   max_altitude(2.0),
    //   planning_timeout(5.0),
    //   max_waypoint_distance(1.0),
    //   unknown_is_occupied(true),
    //   planner(safe_obstacle_distance, euclidean_distance_cutoff, planning_tree_resolution, distance_penalty, greedy_penalty,
    //           min_altitude, max_altitude, planning_timeout, max_waypoint_distance, unknown_is_occupied) {

        void countFreeNodesInOctree();

        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("astar/example/octomap", 10);
        path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("astar/example/path", 10);
        octomap_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("astar/example/octomapViz", 10);

        goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("astar/example/goal_marker", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->_pub_localPath = this->create_publisher<nav_msgs::msg::Path>(this->_topic_localPath, 10);

        this->_sub_pose3d_start_ = this->create_subscription<geometry_msgs::msg::Pose>(_topic_pose3d_start, 10, std::bind(&AstarPlannerNode::_subCallback_update_startPoint, this, std::placeholders::_1));

        this->_sub_pose3d_goal_ = this->create_subscription<geometry_msgs::msg::Pose>(_topic_pose3d_goal, 10, std::bind(&AstarPlannerNode::_subCallback_update_goalPoint, this, std::placeholders::_1));

        // _walltimer_plan_path_on_posChange_ = this->create_wall_timer(
        //     500ms, std::bind(&AstarPlannerNode::_timerCallback_plan_path_on_posChange, this));

        this->_sub_octomap_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            this->_topicIn_octomap, 10,
            std::bind(&AstarPlannerNode::octomap_callback, this, std::placeholders::_1));

        plan_path_service_ = this->create_service<std_srvs::srv::Trigger>(
            "plan_path",
            std::bind(&AstarPlannerNode::plan_path_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        explore_service_ = this->create_service<std_srvs::srv::Trigger>(
            "exploration",
            std::bind(&AstarPlannerNode::exploration_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&AstarPlannerNode::pose_callback, this, std::placeholders::_1));

        update_pose_goal_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/update_marker_pose/goal", 10);
        update_pose_start_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/update_marker_pose/start", 10);


        // // Initialize the parameters
        // this->declare_parameter("safe_obstacle_distance", 0.5);
        // this->declare_parameter("euclidean_distance_cutoff", 1.0);
        // this->declare_parameter("planning_tree_resolution", 0.1);
        // this->declare_parameter("distance_penalty", 1.0);
        // this->declare_parameter("greedy_penalty", 1.0);
        // this->declare_parameter("min_altitude", 0.0);
        // this->declare_parameter("max_altitude", 2.0);
        // this->declare_parameter("planning_timeout", 5.0);
        // this->declare_parameter("max_waypoint_distance", 1.0);
        // this->declare_parameter("unknown_is_occupied", true);

        // // Get the parameters
        // this->get_parameter("safe_obstacle_distance", safe_obstacle_distance);
        // this->get_parameter("euclidean_distance_cutoff", euclidean_distance_cutoff);
        // this->get_parameter("planning_tree_resolution", planning_tree_resolution);
        // this->get_parameter("distance_penalty", distance_penalty);
        // this->get_parameter("greedy_penalty", greedy_penalty);
        // this->get_parameter("min_altitude", min_altitude);
        // this->get_parameter("max_altitude", max_altitude);
        // this->get_parameter("planning_timeout", planning_timeout);
        // this->get_parameter("max_waypoint_distance", max_waypoint_distance);
        // this->get_parameter("unknown_is_occupied", unknown_is_occupied);

        // Initialize the planner
        //planner = navigation::AstarPlanner(safe_obstacle_distance, euclidean_distance_cutoff, planning_tree_resolution, distance_penalty, greedy_penalty,
         //                              min_altitude, max_altitude, planning_timeout, max_waypoint_distance, unknown_is_occupied);

    }



    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {

        // Konvertiere ROS-Nachricht in Octomap
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (!tree)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to deserialize Octomap message.");
            return;
        }

        // Stelle sicher, dass die Octomap ein OcTree ist
        octomap::OcTree* octree_ptr = dynamic_cast<octomap::OcTree*>(tree);
        if (!octree_ptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Octomap is not an OcTree.");
            delete tree;
            return;
        }

        this->octree_ = std::make_shared<octomap::OcTree>(*octree_ptr);

        delete octree_ptr;

    }


int countFreeNodes(octomap::OcTree* octree) {
    int free_node_count = 0;

    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
        if (octree->isNodeOccupied(*it)) {
            continue;
        }
        free_node_count++;
    }

    return free_node_count;
}

// Usage example
void countFreeNodesInOctree() {
    int free_nodes = countFreeNodes(this->octree_.get());
    RCLCPP_INFO(this->get_logger(), "Total free nodes: %d", free_nodes);
}

geometry_msgs::msg::Pose find_closet_node (octomap::point3d received_pose){


    // Convert received_pose to OcTreeKey
   //this->countFreeNodesInOctree();

    // Round the received_pose coordinates to 2 decimal places
    received_pose.x() = std::round(received_pose.x() * 100.0) / 100.0;
    received_pose.y() = std::round(received_pose.y() * 100.0) / 100.0;
    received_pose.z() = std::round(received_pose.z() * 100.0) / 100.0;
    octomap::OcTreeKey received_tree_key = this->octree_->coordToKey(received_pose);

    octomap::point3d closest_free_point;
    bool found_free_point = false;
    double min_distance = std::numeric_limits<double>::max();

    double min_z = this->octree_->getBBXMin().z();
    double resolution = this->octree_->getResolution();


    // Define the search range in x, y, and z directions
    double search_radius = .7; // Define the radius for searching in x and y directions


    //octomap::OcTreeKey neighbor_key = navigation::AstarPlanner::expand(received_key, neighbor_);

    std::vector<octomap::point3d> neighbor_offsets = {
        {resolution, 0, 0}, {-resolution, 0, 0},
        {0, resolution, 0}, {0, -resolution, 0}
    };


    // auto received_tree_node = this->octree_->search(received_tree_key);
    // if (received_tree_node->getValue() == navigation::TreeValue::FREE) {
    //                 octomap::point3d neighbor_point = this->octree_->keyToCoord(received_tree_key);
    //                 closest_free_point = neighbor_point;
    //                 found_free_point = true;
    // }
    const std::vector<std::vector<int>> base_neighbours_ = {
      {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
    {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
    {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
    {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}
    };
    std::vector<std::vector<int>> neighbours_ = base_neighbours_;
    int search_radius_it = 0;

    for (octomap::OcTree::leaf_iterator it = this->octree_->begin_leafs(), end = this->octree_->end_leafs(); it != end; ++it) {
        if (this->octree_->isNodeOccupied(*it)) {
            continue;
        }
        octomap::point3d node_center = it.getCoordinate();
        double distance = received_pose.distance(node_center);
        if (distance < min_distance) {
            min_distance = distance;
            closest_free_point = node_center;
            found_free_point = true;
        }
    }

    if (found_free_point)
    {
        RCLCPP_INFO(this->get_logger(), "Closest free point found: (%f, %f, %f)",
                    closest_free_point.x(), closest_free_point.y(), closest_free_point.z());

        // Publish the closest free point as a Pose message
        geometry_msgs::msg::Pose closest_pose_msg;
        closest_pose_msg.position.x = closest_free_point.x();
        closest_pose_msg.position.y = closest_free_point.y();
        closest_pose_msg.position.z = closest_free_point.z();
        closest_pose_msg.orientation.w = 1.0; // Assuming no orientation information

        return closest_pose_msg;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No free point found.");
        return geometry_msgs::msg::Pose();
    }
}

void fill_nodes_at_pose(geometry_msgs::msg::Pose pose)
{
    if (!this->octree_)
    {
        RCLCPP_ERROR(this->get_logger(), "Octree is not initialized.");
        return;
    }

    navigation::RobotDimensions robot_dims = {1.2, .6, 1.05};

    // Define the bounds of the robot in the x, y, and z directions
    double half_length = robot_dims.length / 2.0;
    double half_width = robot_dims.width / 2.0;
    double height = robot_dims.height;

    // Iterate over the volume defined by these bounds
    double z_base_footprint = pose.position.z + 0.1; // The base of the robot is 0.15 m above the ground to match with the scan
    for (double dx = -half_length; dx <= half_length; dx += this->octree_->getResolution())
    {
        for (double dy = -half_width; dy <= half_width; dy += this->octree_->getResolution())
        {
            for (double dz = z_base_footprint; dz <= height; dz += this->octree_->getResolution())
            {
                // Calculate the position of the current point
                double x = pose.position.x + dx;
                double y = pose.position.y + dy;
                double z = dz;

                octomap::point3d point(x, y, z);
                //this->octree_->updateNode(point, false);
                if (z <= z_base_footprint){
                    // Mark the corresponding node in the octree as free
                    octomap::point3d point(x, y, z_base_footprint);
                    this->octree_->updateNode(point, false); // false means free
                } else {
                    // Mark the corresponding node in the octree as occupied
                    octomap::point3d point_clear(x, y, z);
                    this->octree_->deleteNode(point_clear); // true means occupied
                }

            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Filled nodes at pose with free nodes.");
}

geometry_msgs::msg::Pose get_start_pose()
{
    auto end_time = this->get_clock()->now() + rclcpp::Duration::from_seconds(10.0);
    geometry_msgs::msg::TransformStamped transform_stamped;

    while (this->get_clock()->now() < end_time)
    {
        try
        {
            transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
            break;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for transform map to base_footprint: %s", ex.what());
        }
    }

    if (transform_stamped.header.stamp == rclcpp::Time(0))
    {
        RCLCPP_INFO(this->get_logger(), "Transform is None");
        return geometry_msgs::msg::Pose();
    }

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = transform_stamped.transform.translation.x;
    start_pose.position.y = transform_stamped.transform.translation.y;
    start_pose.position.z = transform_stamped.transform.translation.z;
    start_pose.orientation.x = transform_stamped.transform.rotation.x;
    start_pose.orientation.y = transform_stamped.transform.rotation.y;
    start_pose.orientation.z = transform_stamped.transform.rotation.z;
    start_pose.orientation.w = transform_stamped.transform.rotation.w;

    fill_nodes_at_pose(start_pose);

    //start_pose.position.z = start_pose.position.z + 0.15; // The base of the robot is 0.15 m above the ground to match with the scan

    return start_pose;
}

void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received initial pose");

    if (!this->octree_)
    {
        RCLCPP_ERROR(this->get_logger(), "Octree is not initialized.");
        return;
    }

    // Convert the received pose to an octomap::point3d
    // find closest goal point
    octomap::point3d received_pose(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    auto closest_pose_msg = find_closet_node(received_pose);
    update_pose_goal_pub_->publish(closest_pose_msg);
    RCLCPP_INFO(this->get_logger(), "Received pose: (%f, %f, %f)", received_pose.x(), received_pose.y(), received_pose.z());

    this->point3d_goal = octomap::point3d(closest_pose_msg.position.x, closest_pose_msg.position.y, closest_pose_msg.position.z);

    // Find closest Start Point
    auto start_pose = get_start_pose();
    octomap::point3d start_point3d(start_pose.position.x, start_pose.position.y, start_pose.position.z);
    auto closest_start_pose = find_closet_node(start_point3d);
    update_pose_start_pub_->publish(closest_start_pose);

    // Print the closest start point
    RCLCPP_INFO(this->get_logger(), "Closest start pose: (%f, %f, %f)", closest_start_pose.position.x, closest_start_pose.position.y, closest_start_pose.position.z);

    navigation::PlanningResult result;
    bool path_found = false;
    octomap::point3d closest_start_point3d(closest_start_pose.position.x, closest_start_pose.position.y, closest_start_pose.position.z);

    this->point3d_start = closest_start_point3d;

    RCLCPP_INFO(this->get_logger(), "Closest start pose: (%f, %f, %f)", closest_start_pose.position.x, closest_start_pose.position.y, closest_start_pose.position.z);

    std::future<navigation::PlanningResult> future = std::async(std::launch::async, [this]() { return plan_path(); });

    future.wait();  // Wait for the plan_path function to finish
    result = future.get(); // Get the result of the plan_path function
    RCLCPP_INFO_STREAM(this->get_logger(), "Path planning completed.");

    // Check the result
    if (result == navigation::PlanningResult::COMPLETE)
    {
        RCLCPP_INFO(this->get_logger(), "Path found successfully.");
        path_found = true;
    }
    else if (result == navigation::PlanningResult::INCOMPLETE)
    {
        RCLCPP_INFO(this->get_logger(), "Path found but incomplete.");
        path_found = true;
    }
    else if (result == navigation::PlanningResult::FAILURE)
    {
        RCLCPP_INFO(this->get_logger(), "Failed to find a path. Adjusting start point and retrying...");
    }

}
    void _subCallback_update_startPoint(geometry_msgs::msg::Pose::SharedPtr _msg_pose3d_start_) {

        RCLCPP_INFO_STREAM(this->get_logger(), "[Begin] _subCallback_update_startPoint");

        // TODO: Declare conversion operator
        this->point3d_start = octomap::point3d(_msg_pose3d_start_->position.x, _msg_pose3d_start_->position.y, _msg_pose3d_start_->position.z);

    }



    void _subCallback_update_goalPoint(geometry_msgs::msg::Pose::SharedPtr _msg_pose3d_goal_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "[Begin] _subCallback_update_goalPoint");

        // TODO: Declare conversion operator
        this->point3d_goal = octomap::point3d(_msg_pose3d_goal_->position.x, _msg_pose3d_goal_->position.y, _msg_pose3d_goal_->position.z);
    }



    bool are_poses_equal(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2) const {
        return pose1.position.x == pose2.position.x &&
            pose1.position.y == pose2.position.y &&
            pose1.position.z == pose2.position.z &&
            pose1.orientation.x == pose2.orientation.x &&
            pose1.orientation.y == pose2.orientation.y &&
            pose1.orientation.z == pose2.orientation.z &&
            pose1.orientation.w == pose2.orientation.w;
    }

    void _timerCallback_plan_path_on_posChange() {
        //)
        if (this->point3d_goal == this->point3d_goal_old && this->point3d_start == this->point3d_start_old) {
            //RCLCPP_INFO_STREAM(this->get_logger(), "[_timerCallback_plan_path_on_posChange] NO positional changes detected");
            return;
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(), "[_timerCallback_plan_path_on_posChange] Positional changes detected. Replanning...");
        }

        auto future = std::async(std::launch::async, [this]() { plan_path(); });

        future.wait();  // Wait for the plan_path function to finish

        RCLCPP_INFO_STREAM(this->get_logger(), "[_timerCallback_plan_path_on_posChange] Path planning completed.");


        this->point3d_goal_old = this->point3d_goal;
        this->point3d_start_old = this->point3d_start;
    }


    // Methode zum Veröffentlichen des Ziels als Marker
    void publishGoalMarker(const octomap::point3d& point, const std::string& frame_id, int marker_id, uint32_t markerType = visualization_msgs::msg::Marker::SPHERE) {
        auto goal_marker = octomap_utils::point3dToMarker(point, frame_id, marker_id, markerType); // Erstelle den Marker
        goal_marker_pub_->publish(goal_marker); // Veröffentliche den Marker
    }


    void visualizeTree(const octomap::OcTree& tree, std::vector<octomap::OcTreeKey>& collision_nodes) {
        octomap_msgs::msg::Octomap octomap_msg;
        if (octomap_msgs::fullMapToMsg(tree, octomap_msg)) {
            octomap_msg.header.frame_id = "map";
            octomap_msg.header.stamp = this->now();
            this->octomap_pub_->publish(octomap_msg);
        }

        auto marker = octomap_utils::octomapToMarkerArray(tree, collision_nodes);

        this->octomap_viz_pub_->publish(marker);
    }


    // TODO: Move to conversion header
    geometry_msgs::msg::Point convertNodeToPoint(const navigation::Node& node, const octomap::OcTree& octree) {
        geometry_msgs::msg::Point p;
        octomap::point3d point = octree.keyToCoord(node.key);
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        return p;
    }



    // TODO: Move to Visualization Header
    void visualizeExpansions(
        const std::unordered_set<navigation::Node, navigation::HashFunction> & set_nodes_to_hash_1,
        const std::unordered_set<navigation::Node, navigation::HashFunction> & set_node_to_hash_2,
        const octomap::OcTree& octree) {

        visualization_msgs::msg::MarkerArray marker_array;

        size_t i = 0;
        for (auto it_map = set_nodes_to_hash_1.begin(); it_map != set_nodes_to_hash_1.end(); it_map++) {

            ++i;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "nodes";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point point = convertNodeToPoint(*it_map, octree);
            marker.pose.position = point;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }

        this->path_pub_->publish(marker_array);

    }

private:
        // Service callback for path planning
    void plan_path_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Received request to plan path");
        try {
            plan_path();
            response->success = true;
            response->message = "Path planning completed successfully.";
        } catch (const std::exception &e) {
            response->success = false;
            response->message = e.what();
        }
    }

    void exploration_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Received request to plan path");
        try {
            plan_path();
            response->success = true;
            response->message = "Path planning completed successfully.";
        } catch (const std::exception &e) {
            response->success = false;
            response->message = e.what();
        }
    }

    // Execute planing
    navigation::PlanningResult plan_path() {

        if (!this->octree_) {
            RCLCPP_INFO_STREAM(this->get_logger(), "[plan_path] OcTree was not init.");
            return navigation::PlanningResult::FAILURE;
        }

        // Start- und Zielkoordinaten festlegen
        octomap::point3d start_coord = this->point3d_start;
        octomap::point3d goal_coord = this->point3d_goal;


        octomap::point3d pos_cmd(0.0, 0.0, 0.0);

        // Planer initialisieren
        double safe_obstacle_distance = .0;
        double euclidean_distance_cutoff = 2.0;
        // double planning_tree_resolution = 0.1;
        double planning_tree_resolution = this->octree_->getResolution();
        double distance_penalty = 1.0;
        double greedy_penalty = 1.;
        double min_altitude = -100.0;
        double max_altitude = 100.0;
        double planning_timeout = 50000.0;  // 1 Sekunde Timeout
        double max_waypoint_distance = .5;
        bool unknown_is_occupied = false;
        //this->octree_->setOccupancyThres(0.8);

        // NOTE: Helper for x and y already declared
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;

        // TODO: Remove Mockup when complete
        // auto tree =  octomap_utils::create_ocTreeMockup();

        // // Optional: Stellen Sie sicher, dass alle Nodes auf der kleinsten Auflösungsebene sind
        // tree->updateInnerOccupancy();

        auto tree = this->octree_;

        // Optional: Stellen Sie sicher, dass alle Nodes auf der kleinsten Auflösungsebene sind
        tree->updateInnerOccupancy();

        tree->getMetricMin(min_x, min_y, min_z);
        tree->getMetricMax(max_x, max_y, max_z);


        // RCLCPP_INFO_STREAM(this->get_logger(), "OctoMap-Dimension-Metric; Min: " << min_x << " " << min_y << " " << min_z << " ; Max: " << max_x << " " << max_y << " " << max_z);


        navigation::AstarPlanner planner(safe_obstacle_distance, euclidean_distance_cutoff, planning_tree_resolution, distance_penalty, greedy_penalty,
                                        min_altitude, max_altitude, planning_timeout, max_waypoint_distance, unknown_is_occupied);

        auto start_find_path = std::chrono::high_resolution_clock::now();
        // Pfad finden
        auto [path, result] = planner.findPath(start_coord, goal_coord, pos_cmd, tree, planning_timeout,
                                            std::bind(&AstarPlannerNode::visualizeTree, this, std::placeholders::_1, std::placeholders::_2),
                                            std::bind(&AstarPlannerNode::visualizeExpansions, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        RCLCPP_INFO(rclcpp::get_logger("Astar"), "findPath took %.2f s",
                    std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_find_path).count());


        // TODO: Generate random id by using the time
        this->publishGoalMarker(start_coord, "map", 987654, visualization_msgs::msg::Marker::SPHERE);
        this->publishGoalMarker(goal_coord, "map", 987655, visualization_msgs::msg::Marker::CUBE);

        //path = create_optimizedLineStringInterpolation_keep(path, 0.05);
        auto _msg_localPath = octomap_utils::create_pathMsg_from_points3d(path);
        this->_pub_localPath->publish(_msg_localPath);


        // Ergebnis überprüfen
        if (result == navigation::PlanningResult::COMPLETE) {
            auto _msg_pathMarker = octomap_utils::visualizePath(path);
            this->_pub_localPath->publish(_msg_localPath);
            return navigation::PlanningResult::COMPLETE;
        } else if (result == navigation::PlanningResult::INCOMPLETE) {
            return navigation::PlanningResult::INCOMPLETE;
        } else {
            return navigation::PlanningResult::FAILURE;
        }

    } // [Member] plan_path


    std::vector<octomap::point3d> create_optimizedLineStringInterpolation_keep(const std::vector<octomap::point3d>& points3d_for_optimation, const double& dist_to_linestring) {
        if (points3d_for_optimation.size() <= 2) {
            return points3d_for_optimation;  // If there are 2 or fewer points, return them as-is.
        }

        std::vector<octomap::point3d> result;
        // Füge den ersten und den letzten Punkt hinzu.
        result.push_back(points3d_for_optimation[0]);
        result.push_back(points3d_for_optimation[1]);

        std::vector<octomap::point3d> currSegment(2);

        for (auto idx_currPoint3d = 2ul; idx_currPoint3d < points3d_for_optimation.size(); idx_currPoint3d++) {
            // Create current Segment
            currSegment[0] = points3d_for_optimation[idx_currPoint3d - 2];
            currSegment[1] = points3d_for_optimation[idx_currPoint3d];

            // Check if point before now lays on the same line with offset 0
            if (std::abs(signedDistance(currSegment, points3d_for_optimation[idx_currPoint3d - 1])) <= std::abs(dist_to_linestring)) {
                // Remove Point before and add new Point
                result.pop_back();
            }

            result.push_back(points3d_for_optimation[idx_currPoint3d]);
        }

        return result;
    }

    // Berechnet den signierten Abstand zwischen einem Punkt und einer Linie, die durch zwei Punkte definiert ist.
    double signedDistance(const std::vector<octomath::Vector3>& segment, const octomath::Vector3& point) {
        octomap::point3d a = segment[0];
        octomap::point3d b = segment[1];
        octomap::point3d ab = b - a;
        octomap::point3d ap = point - a;

        double ab_length = ab.norm();
        octomap::point3d ab_normalized = ab * (1.0 / ab_length);
        double projection_length = ap.dot(ab_normalized);
        octomap::point3d projection = a + ab_normalized * projection_length;

        octomap::point3d distance_vector = point - projection;
        return distance_vector.norm();
    }

};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AstarPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}