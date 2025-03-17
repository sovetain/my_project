#include <log4cxx/basicconfigurator.h>
#include <log4cxx/logger.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <chrono>
#include <vector>

#include "common.hpp"
#include "timer.hpp"
#include "prm.hpp"
#include "topological_prm.hpp"
#include "topological_prm_clustering.hpp"
#include "pmm_trajectory3d.hpp"
#include "pmm_mg_trajectory3d.hpp"
#include "yaml-cpp/yaml.h"

using namespace log4cxx;
using namespace pmm;

std::string planner_config_file = "./config_files/1-2-1.yaml";
std::string output_folder = "./prints/";

Vector<3> start;
Vector<3> goal;
YAML::Node planner_config;

// converting prm paths for pmm 
std::vector<Vector<3>> convert_path_to_pmm_format(const path_with_length<Vector<3>>& path) {
    std::vector<Vector<3>> converted_path;
    for (const auto& node : path.plan) {
        converted_path.push_back(node->data);
    }
    return converted_path;
}

// combining prm and pmm
void execute_combined_planning() {
    planner_config = YAML::LoadFile(planner_config_file);

    map_type = loadParam<std::string>(planner_config, "map_type");
    map_file = loadParam<std::string>(planner_config, "map");

    // loading of start and end positions
    std::vector<double> start_pos, end_pos;
    parseArrayParam(planner_config["start"], "position", start_pos);
    parseArrayParam(planner_config["end"], "position", end_pos);
    start = {start_pos[0], start_pos[1], start_pos[2]};
    goal = {end_pos[0], end_pos[1], end_pos[2]};
    
    std::vector<Vector<3>> gates_with_start_end_poses = {start, goal};

    std::shared_ptr<BaseMap> map;
    map->load(map_file);
    
    // generating paths with prm
    std::vector<std::vector<path_with_length<Vector<3>>>> paths_prm =
        TopologicalPRMClustering<Vector<3>>::find_geometrical_paths(planner_config, map, gates_with_start_end_poses, output_folder);
    
    if (paths_prm.empty() || paths_prm[0].empty()) {
        std::cerr << "No valid paths found using Topological PRM" << std::endl;
        return;
    }

    std::vector<path_with_length<Vector<3>>> selected_paths;
    if (paths_prm[0].size() > 2) {
        std::sort(paths_prm[0].begin(), paths_prm[0].end(), [](const path_with_length<Vector<3>>& a, const path_with_length<Vector<3>>& b) {
            return a.length < b.length;
        });
        selected_paths.push_back(paths_prm[0][0]);
        selected_paths.push_back(paths_prm[0][1]);
    } else {
        selected_paths = paths_prm[0];
    }
    
    // ompitimzating paths with pmm
    for (size_t i = 0; i < selected_paths[0].size(); i++) {
        std::vector<Vector<3>> pmm_path = convert_path_to_pmm_format(selected_paths[0][i]);
        
        PMM_MG_Trajectory3D trajectory(pmm_path, {0, 0, 0}, {0, 0, 0}, 9.81, 10.0, 0.01, 100, 0.1, 0.9, 0.001, 100, 0.001, true, 100, 0.1, 0.9, 0.001, true, true);
        
        std::cout << "Optimized trajectory duration: " << trajectory.duration() << " s." << std::endl;
        trajectory.sample_and_export_trajectory(0.1, output_folder + "optimized_path_" + std::to_string(i) + ".csv");
    }
}

int main(int argc, char** argv) {
    startLogger("main");
    execute_combined_planning();
    return 0;
}







/*
#include <iostream>
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <filesystem>
#include "yaml-cpp/yaml.h"

// CTopPRM
#include "common.hpp"
#include "topological_prm_clustering.hpp"

// PMM_UAV
#include "pmm_mg_trajectory3d.hpp"

using namespace std;

// Function to generate a path using CTopPRM
std::vector<Vector<3>> generatePath(const std::string& planner_config_file, const std::string& output_folder) {
    // Load configuration for CTopPRM
    YAML::Node planner_config = YAML::LoadFile(planner_config_file);

    // Extract start and goal positions
    Vector<3> start, goal;
    if (planner_config["start"] && planner_config["start"]["position"]) {
        std::vector<double> start_pos;
        parseArrayParam(planner_config["start"], "position", start_pos);
        start = Vector<3>(start_pos[0], start_pos[1], start_pos[2]);
    } else {
        throw std::runtime_error("Start position is not specified in CTopPRM config.");
    }
    if (planner_config["end"] && planner_config["end"]["position"]) {
        std::vector<double> end_pos;
        parseArrayParam(planner_config["end"], "position", end_pos);
        goal = Vector<3>(end_pos[0], end_pos[1], end_pos[2]);
    } else {
        throw std::runtime_error("Goal position is not specified in CTopPRM config.");
    }

    // Load the map
    std::shared_ptr<BaseMap> map;
    std::string map_type = loadParam<std::string>(planner_config, "map_type");
    std::string map_file = loadParam<std::string>(planner_config, "map");
    if (map_type == "ESDF") {
        map = std::make_shared<ESDFMap>();
    } else {
        throw std::runtime_error("Unsupported map type: " + map_type);
    }
    map->load(map_file);

    // Define waypoints for path planning (start and goal)
    std::vector<Vector<3>> gates_with_start_end = {start, goal};

    // Run CTopPRM path generation
    auto begin = std::chrono::high_resolution_clock::now();
    auto paths_between_gates = TopologicalPRMClustering<Vector<3>>::find_geometrical_paths(planner_config, map, gates_with_start_end, output_folder);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "CTopPRM computation time: " 
              << std::chrono::duration_cast<std::chrono::seconds>(end_time - begin).count() << " sec." << std::endl;

    // Validate the output paths
    if (paths_between_gates.empty() || paths_between_gates[0].empty()) {
        throw std::runtime_error("CTopPRM did not generate any valid paths.");
    }

    // Select the first available path
    auto chosen_path = paths_between_gates[0][0];

    // Convert the path from HeapNode<Vector<3>>* to std::vector<Vector<3>>
    std::vector<Vector<3>> waypoints;
    for (auto* node : chosen_path.plan) {
        if (node) {
            waypoints.push_back(node->data);
        }
    }

    return waypoints;
}

// Function to plan a trajectory using PMM with the given waypoints
int planTrajectoryFromWaypoints(const std::vector<Vector<3>>& waypoints, const YAML::Node& planner_config) {
    if (waypoints.empty()) {
        throw std::runtime_error("No waypoints available for trajectory planning.");
    }

    // Initialize velocity vector (assuming zero velocity at all points)
    std::vector<Vector<3>> velocities(waypoints.size(), Vector<3>::Zero());

    // Extract UAV parameters from the config
    Vector<3> a_max, v_max;
    planner_config["uav"]["maximum_acceleration"] >> a_max;
    planner_config["uav"]["maximum_velocity"] >> v_max;

    // Generate the PMM trajectory
    PMM_MG_Trajectory3D trajectory(waypoints, velocities, a_max, v_max);

    std::cout << "Optimized trajectory duration: " << trajectory.duration() << " sec." << std::endl;
    std::cout << "Trajectory planning completed." << std::endl;

    return 0;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <CTopPRM_config.yaml> <PMM_planner_config.yaml>" << std::endl;
        return -1;
    }

    try {
        std::string ctopprm_config_file = argv[1];
        std::string pmm_planner_config_file = argv[2];

        // Generate waypoints using CTopPRM
        std::vector<Vector<3>> waypoints = generatePath(ctopprm_config_file, output_folder);
        std::cout << "CTopPRM generated " << waypoints.size() << " waypoints." << std::endl;

        // Load PMM configuration
        YAML::Node pmm_planner_config = YAML::LoadFile(pmm_planner_config_file);

        // Plan the trajectory using PMM
        return planTrajectoryFromWaypoints(pmm_planner_config, waypoints);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
}
*/
