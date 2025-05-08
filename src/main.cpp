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

std::string config_file = "../config_files/small_poles.yaml";
std::string planner_config_file = "../config_files/planner_config.yaml";
std::string output_folder = "./prints/";

std::string map_file;
std::string map_type;
YAML::Node config;
YAML::Node planner_config;
Vector<3> start;
Vector<3> goal;
Scalar min_clearance_;

std::vector<pmm::Vector<3>> convert_path_to_pmm_format(const path_with_length<Vector<3>>& path) {
    std::vector<pmm::Vector<3>> result;
    result.reserve(path.plan.size());

    for (const auto& node : path.plan) {
        const Vector<3>& p = node->data;
        pmm::Vector<3> pmm_point;
        for (int i = 0; i < 3; ++i) {
            pmm_point[i] = p[i];
        }
        result.push_back(pmm_point);
    }

    return result;
}

int count_sharp_turns(const path_with_length<Vector<3>>& path) {
    int sharp_turns = 0;
    const double threshold_angle_deg = 45.0;
    const double threshold_cos = std::cos(threshold_angle_deg * M_PI / 180.0); // cos(45°)

    for (size_t i = 1; i + 1 < path.plan.size(); ++i) {
        const Vector<3>& a = path.plan[i - 1]->data;
        const Vector<3>& b = path.plan[i]->data;
        const Vector<3>& c = path.plan[i + 1]->data;

        Vector<3> ab = b - a;
        Vector<3> bc = c - b;

        ab = ab.normalized();
        bc = bc.normalized();

        double cos_angle = ab.dot(bc);
        if (cos_angle < threshold_cos) {
            ++sharp_turns;
        }
    }

    return sharp_turns;
}

bool is_path_collision_free(const std::vector<pmm::Vector<3>>& points, std::shared_ptr<BaseMap>& map, Scalar min_clearance_) {
    bool collision_found = false;
    for (const auto& point : points) {
        double distance = map->getClearence(point);
        if (distance < min_clearance_) {
            //std::cout << "Collision detected at: (" 
                //    << point(0) << ", " << point(1) << ", " << point(2) 
                //    << "), clearance: " << distance << std::endl;
            collision_found = true;
        }
    }
    return !collision_found;
}

void adjust_path_with_gradient(std::vector<pmm::Vector<3>>& points, const std::shared_ptr<BaseMap>& map,
                                    double step_size, Scalar min_clearance_) {
    for (int iter = 0; iter < 5; ++iter) {
        bool adjusted = false;

        for (auto& point : points) {
            double clearance = map->getClearence(point);
            if (clearance < min_clearance_) {
                double dx = (map->getClearence(point + pmm::Vector<3>(step_size, 0.0, 0.0)) - clearance) / step_size;
                double dy = (map->getClearence(point + pmm::Vector<3>(0.0, step_size, 0.0)) - clearance) / step_size;
                double dz = (map->getClearence(point + pmm::Vector<3>(0.0, 0.0, step_size)) - clearance) / step_size;

                pmm::Vector<3> grad(dx, dy, dz);

                if (grad.norm() > 1e-6) {
                    grad = grad.normalized();
                    point += grad * (min_clearance_ - clearance + 0.01); 
                    adjusted = true;
                }
            }
        }

        if (!adjusted) {
            break;
        }
    }
}

std::vector<path_with_length<Vector<3>>> select_paths_by_score(
    const std::vector<path_with_length<Vector<3>>>& all_paths, double alpha, double beta) {
    if (all_paths.size() <= 2) {
        return all_paths;
    }

    std::vector<double> lengths, turns;
    for (const auto& path : all_paths) {
        lengths.push_back(path.length);
        turns.push_back(count_sharp_turns(path));
    }

    double min_len = *std::min_element(lengths.begin(), lengths.end());
    double max_len = *std::max_element(lengths.begin(), lengths.end());
    double min_turn = *std::min_element(turns.begin(), turns.end());
    double max_turn = *std::max_element(turns.begin(), turns.end());

    std::vector<std::pair<path_with_length<Vector<3>>, double>> scored_paths;

    for (size_t i = 0; i < all_paths.size(); ++i) {
        double norm_len = (max_len - min_len > 1e-6) ? (lengths[i] - min_len) / (max_len - min_len) : 0.0;
        double norm_turn = (max_turn - min_turn > 1e-6) ? (turns[i] - min_turn) / (max_turn - min_turn) : 0.0;
        double score = alpha * norm_len + beta * norm_turn;
        scored_paths.emplace_back(all_paths[i], score);
    }

    std::sort(scored_paths.begin(), scored_paths.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });

    return {scored_paths[0].first, scored_paths[1].first};
}

// combining prm and pmm
void execute_combined_planning() {
    config = YAML::LoadFile(config_file);

    map_type = loadParam<std::string>(config, "map_type");
    map_file = loadParam<std::string>(config, "map");
    min_clearance_ = loadParam<double>(config, "min_clearance");

    // loading start and end positions
    if (config["start"] && config["end"]) {
        if (config["start"]["position"]) {
            std::vector<double> start_pos;
            parseArrayParam(config["start"], "position", start_pos);
            start(0) = start_pos[0];
            start(1) = start_pos[1];
            start(2) = start_pos[2];
        } else {
            INFO_RED("you must specify start position");
            exit(1);
        }
        if (config["end"]["position"]) {
            std::vector<double> end_pos;
            parseArrayParam(config["end"], "position", end_pos);
            goal(0) = end_pos[0];
            goal(1) = end_pos[1];
            goal(2) = end_pos[2];
        } else {
            INFO_RED("you must specify end position");
            exit(1);
        }
    } else {
        INFO_RED("you must specify start and end position");
        exit(1);
    }

    std::vector<Vector<3>> gates_with_start_end_poses;
    gates_with_start_end_poses.push_back(start);
    gates_with_start_end_poses.push_back(goal);

    // loading map
    std::shared_ptr<BaseMap> map;
    if (map_type == "ESDF") {
        map = std::make_shared<ESDFMap>();
    } else {
        ERROR("map type " << map_type << " not recognized")
        exit(1);
    }
    map->load(map_file);

    // generating paths with CTopPRM
    std::vector<std::vector<path_with_length<Vector<3>>>> paths_prm =
        TopologicalPRMClustering<Vector<3>>::find_geometrical_paths(
            config, map, gates_with_start_end_poses, output_folder);
    
    if (paths_prm.empty() || paths_prm[0].empty()) {
        std::cerr << "No valid paths found using CTopPRM" << std::endl;
        return;
    } else {
        std::cout << "Number of found paths using CTopPRM: " << paths_prm[0].size() << std::endl;
    }

    // paths sorting
    std::vector<path_with_length<pmm::Vector<3>>> selected_paths;
    std::string selection_method = "shortest"; // could be "least_turns" or "shortest" or "comb"

    // DORABOTAT VYBOR
    if (paths_prm[0].size() > 2) {
        if (selection_method == "shortest") {
            std::sort(paths_prm[0].begin(), paths_prm[0].end(), [](const path_with_length<Vector<3>>& a, const path_with_length<Vector<3>>& b) {
            return a.length < b.length;                          
            });
            selected_paths.push_back(paths_prm[0][0]);
            // selected_paths.push_back(paths_prm[0][1]);
        } else if (selection_method == "least_turns") {
            std::sort(paths_prm[0].begin(), paths_prm[0].end(), [](const path_with_length<Vector<3>>& a, const path_with_length<Vector<3>>& b) {
            return count_sharp_turns(a) < count_sharp_turns(b);
            });
            selected_paths.push_back(paths_prm[0][0]);
            selected_paths.push_back(paths_prm[0][1]);
        } else if (selection_method == "comb") {
            double alpha = 0.7;
            double beta = 0.3;
            selected_paths = select_paths_by_score(paths_prm[0], alpha, beta);
        }
    } else {
        selected_paths = paths_prm[0];
    }
    
    // optimizing paths with PMM
    for (size_t i = 0; i < selected_paths.size(); i++) {
        planner_config = YAML::LoadFile(planner_config_file);

        std::vector<Vector<3>> pmm_path = convert_path_to_pmm_format(selected_paths[i]);

        // drone parameters 
        double max_acc_norm = planner_config["uav"]["maximum_acceleration_norm"].as<double>();
        double max_vel_norm = planner_config["uav"]["maximum_velocity"].as<double>();

        // Velocity Optimization parameters ------------------------------------------------------------/
        // Thrust decomposition
        bool drag = planner_config["thrust_decomposition"]["drag"].as<bool>();
        double TD_acc_precision = planner_config["thrust_decomposition"]["precision"].as<double>();
        int TD_max_iter = planner_config["thrust_decomposition"]["max_iter"].as<int>();

        // Gradient Method optimization parameters
        // First round
        double alpha = planner_config["velocity_optimization"]["first_run"]["alpha"].as<double>();
        double alpha_reduction_factor = planner_config["velocity_optimization"]["first_run"]["alpha_reduction_factor"].as<double>();
        double alpha_min_threshold = planner_config["velocity_optimization"]["first_run"]["alpha_min_threshold"].as<double>();
        int max_iter = planner_config["velocity_optimization"]["first_run"]["max_iter"].as<int>();

        // Second round
        double alpha2 = planner_config["velocity_optimization"]["second_run"]["alpha"].as<double>();
        double alpha_reduction_factor2 = planner_config["velocity_optimization"]["second_run"]["alpha_reduction_factor"].as<double>();
        double alpha_min_threshold2 = planner_config["velocity_optimization"]["second_run"]["alpha_min_threshold"].as<double>();
        int max_iter2 = planner_config["velocity_optimization"]["second_run"]["max_iter"].as<int>();

        double dT_precision = planner_config["velocity_optimization"]["dT_precision"].as<double>();
        bool second_round_opt = true;

        bool debug = planner_config["debug"].as<bool>();
        bool export_trajectory = planner_config["export"]["sampled_trajectory"].as<bool>();
        double sampling_step = planner_config["export"]["sampling_step"].as<double>();
        std::string sampled_trajectory_file = planner_config["export"]["sampled_trajectory_file"].as<std::string>();

        Vector<3> start_velocity = Vector<3>::Zero();
        Vector<3> end_velocity = Vector<3>::Zero();

        // computing trajectory
        pmm::PMM_MG_Trajectory3D tr(pmm_path, start_velocity, end_velocity, max_acc_norm, max_vel_norm, dT_precision, max_iter, alpha,
                            alpha_reduction_factor, alpha_min_threshold, TD_max_iter, TD_acc_precision, second_round_opt, 
                            max_iter2, alpha2, alpha_reduction_factor2, alpha_min_threshold2, drag, debug);

        std::cout << "Optimized original trajectory duration: " << tr.duration() << " s." << std::endl;

        std::vector<Scalar> t_s;
        std::vector<Vector<3>> p_s;
        std::vector<Vector<3>> v_s;
        std::vector<Vector<3>> a_s;

        std::tie(t_s, p_s, v_s, a_s) = tr.get_sampled_trajectory(sampling_step);

        // checking for collision and adjusting path
        if (!is_path_collision_free(p_s, map, min_clearance_)) {
            adjust_path_with_gradient(p_s, map, sampling_step, min_clearance_);
            if (is_path_collision_free(p_s, map, min_clearance_)) {      
                std::cout << "Path successfully adjusted and is collision-free!" << std::endl;

                Vector<3> max_per_axis_acc_vec;
                Vector<3> min_per_axis_acc_vec;
                std::tie(min_per_axis_acc_vec, max_per_axis_acc_vec) = pmm::compute_per_axis_acc_limits_from_limited_thrust(max_acc_norm);
                Scalar max_vel_per_axis = max_vel_norm / sqrt(3);
                Vector<3> max_per_axis_vel_vec = Vector<3>::Constant(max_vel_per_axis);

                std::vector<Vector<3>> init_vel = pmm::compute_initial_velocities(p_s, start_velocity, end_velocity, 
                                                max_per_axis_acc_vec[0], max_vel_per_axis);
                
                // compute and optimize trajectory
                pmm::PMM_MG_Trajectory3D tr_new(p_s, init_vel, max_per_axis_acc_vec, max_per_axis_vel_vec);
                tr_new.optimize_velocities_at_positions(alpha, alpha_reduction_factor, alpha_min_threshold, max_iter, dT_precision, drag);
                
                // recompute trajectory using limited thrust decomposition
                // std::vector<Vector<3>> v = tr_new.get_current_velocities();
                // pmm::PMM_MG_Trajectory3D tr_new_o(p_s, v, max_acc_norm, max_vel_norm, TD_max_iter, TD_acc_precision, drag, debug);

                // second optimization
                // tr_new_o.optimize_velocities_at_positions(alpha2, alpha_reduction_factor2, alpha_min_threshold2, max_iter2, dT_precision, drag);
                
                std::cout << "Optimized new trajectory duration: " << tr_new.duration() << " s." << std::endl;
                tr_new.sample_and_export_trajectory(sampling_step, "../scripts/trajectory_data/" + sampled_trajectory_file);
            } else {
                std::cout << "Still some points in collision." << std::endl;
            }
        }
    }
}

int main(int argc, char** argv) {
    startLogger("main");
    execute_combined_planning();
    return 0;
}

