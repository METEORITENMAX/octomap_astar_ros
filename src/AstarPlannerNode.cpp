#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/msg/point.hpp>
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

using namespace std::chrono_literals;

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

public:
    AstarPlannerNode() : Node("astar_planner_node") {

        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("astar/example/octomap", 10);
        path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("astar/example/path", 10);
        octomap_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("astar/example/octomapViz", 10);

        goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("astar/example/goal_marker", 10);

        this->_pub_localPath = this->create_publisher<nav_msgs::msg::Path>(this->_topic_localPath, 10);

        this->_sub_pose3d_start_ = this->create_subscription<geometry_msgs::msg::Pose>(_topic_pose3d_start, 10, std::bind(&AstarPlannerNode::_subCallback_update_startPoint, this, std::placeholders::_1));

        this->_sub_pose3d_goal_ = this->create_subscription<geometry_msgs::msg::Pose>(_topic_pose3d_goal, 10, std::bind(&AstarPlannerNode::_subCallback_update_goalPoint, this, std::placeholders::_1));

        _walltimer_plan_path_on_posChange_ = this->create_wall_timer(
            500ms, std::bind(&AstarPlannerNode::_timerCallback_plan_path_on_posChange, this));

        this->_sub_octomap_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            this->_topicIn_octomap, 10,
            std::bind(&AstarPlannerNode::octomap_callback, this, std::placeholders::_1));

        plan_path_service_ = this->create_service<std_srvs::srv::Trigger>(
            "plan_path",
            std::bind(&AstarPlannerNode::plan_path_service_callback, this, std::placeholders::_1, std::placeholders::_2));

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


    // bool plan_path_test() {
    //     // Simulate path planning logic here
    //     std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate some computation time

    //     return true; // Return true if planning was successful
    // }



    // Methode zum Veröffentlichen des Ziels als Marker
    void publishGoalMarker(const octomap::point3d& point, const std::string& frame_id, int marker_id, uint32_t markerType = visualization_msgs::msg::Marker::SPHERE) {
        auto goal_marker = octomap_utils::point3dToMarker(point, frame_id, marker_id, markerType); // Erstelle den Marker
        goal_marker_pub_->publish(goal_marker); // Veröffentliche den Marker
    }


    void visualizeTree(const octomap::OcTree& tree) {
        octomap_msgs::msg::Octomap octomap_msg;
        if (octomap_msgs::fullMapToMsg(tree, octomap_msg)) {
            octomap_msg.header.frame_id = "map";
            octomap_msg.header.stamp = this->now();
            this->octomap_pub_->publish(octomap_msg);
        }

        auto marker = octomap_utils::octomapToMarkerArray(tree);

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

    // Execute planing
    void plan_path() {

        if (!this->octree_) {
            RCLCPP_INFO_STREAM(this->get_logger(), "[plan_path] OcTree was not init.");
            return;
        }

        // Start- und Zielkoordinaten festlegen
        octomap::point3d start_coord = this->point3d_start;
        octomap::point3d goal_coord = this->point3d_goal;


        octomap::point3d pos_cmd(0.0, 0.0, 0.0);

        // Planer initialisieren
        double safe_obstacle_distance = .0;
        double euclidean_distance_cutoff = 1.0;
        // double planning_tree_resolution = 0.1;
        double planning_tree_resolution = this->octree_->getResolution();
        double distance_penalty = 1.0;
        double greedy_penalty = 1.0;
        double min_altitude = -100.0;
        double max_altitude = 100.0;
        double planning_timeout = 1000.0;  // 1 Sekunde Timeout
        double max_waypoint_distance = 1.0;
        bool unknown_is_occupied = false;



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
                                            std::bind(&AstarPlannerNode::visualizeTree, this, std::placeholders::_1),
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
            std::cout << "Path found successfully.\n";
            auto _msg_pathMarker = octomap_utils::visualizePath(path);
            this->_pub_localPath->publish(_msg_localPath);
        } else if (result == navigation::PlanningResult::INCOMPLETE) {
            std::cout << "Path found but incomplete.\n";
        } else {
            std::cout << "Failed to find a path.\n";
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