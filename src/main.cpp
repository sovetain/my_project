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

struct TrajectoryRecord {
    path_with_length<Vector<3>> original_path;
    pmm::PMM_MG_Trajectory3D trajectory;
    double duration;
};

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
    const double threshold_angle_deg = 30.0;
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

std::tuple<bool, Vector<3>> is_path_collision_free(const std::vector<pmm::Vector<3>>& points, std::shared_ptr<BaseMap>& map, Scalar min_clearance_) {
    bool collision_found = false;
    for (const auto& point : points) {
        double distance = map->getClearence(point);
        if (distance < min_clearance_) {
            //std::cout << "Collision detected at: (" 
                //    << point(0) << ", " << point(1) << ", " << point(2) 
                //    << "), clearance: " << distance << std::endl;
            return std::make_tuple(false, point);
        }
    }
    return std::make_tuple(true, Vector<3>::Zero());
}

std::vector<path_with_length<Vector<3>>> select_paths_by_score(
    const std::vector<path_with_length<Vector<3>>>& all_paths, double alpha, double beta, int num_to_select) {
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

    std::vector<path_with_length<Vector<3>>> selected;
    for (int i = 0; i < std::min(num_to_select, static_cast<int>(scored_paths.size())); ++i) {
        selected.push_back(scored_paths[i].first);
    }

    return selected;
}

std::tuple<int, pmm::Vector<3>> get_closest_point(
    const std::vector<pmm::Vector<3>>& path,
    const pmm::Vector<3>& query_point)
{
    int closest_seg_idx = -1;
    pmm::Vector<3> closest_point = pmm::Vector<3>::Zero();
    double min_dist = std::numeric_limits<double>::max();

    if (path.size() < 2) {
        std::cerr << "[ERROR] Path has fewer than 2 points; cannot compute closest segment." << std::endl;
        return {closest_seg_idx, closest_point};
    }

    for (size_t i = 0; i < path.size() - 1; ++i) {
        const auto& p0 = path[i];
        const auto& p1 = path[i + 1];
        pmm::Vector<3> seg = p1 - p0;
        double seg_len_sq = seg.squaredNorm();

        if (seg_len_sq < 1e-8) {
            continue;  // пропускаем очень короткие сегменты
        }

        double t = ((query_point - p0).dot(seg)) / seg_len_sq;
        t = std::clamp(t, 0.0, 1.0);

        pmm::Vector<3> projection = p0 + t * seg;
        double dist = (query_point - projection).norm();

        if (dist < min_dist) {
            min_dist = dist;
            closest_seg_idx = i;
            closest_point = projection;
        }
    }

    if (closest_seg_idx == -1) {
        std::cerr << "[WARNING] No valid closest segment found; returning first point." << std::endl;
        return {0, path[0]};
    }

    return {closest_seg_idx, closest_point};

    return {closest_seg_idx, closest_point};
}

void save_each_path_to_csv(const std::vector<path_with_length<Vector<3>>>& paths, const std::string& folder_path) {
    std::filesystem::create_directories(folder_path);

    for (size_t i = 0; i < paths.size(); ++i) {
        std::string filename = folder_path + "/path_" + std::to_string(i) + ".csv";
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "ERROR: Could not open file: " << filename << std::endl;
            continue;
        }

        file << "x,y,z\n";
        for (const auto& node : paths[i].plan) {
            const Vector<3>& p = node->data;
            file << p[0] << "," << p[1] << "," << p[2] << "\n";
        }

        file.close();
    }
}

void save_single_path_to_csv(const std::vector<pmm::Vector<3>>& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open file: " << filename << std::endl;
        return;
    }

    file << "x,y,z\n";
    for (const auto& point : path) {
        file << point(0) << "," << point(1) << "," << point(2) << "\n";
    }

    file.close();
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
    int num_paths_to_select = std::min(3, static_cast<int>(paths_prm[0].size()));
    
    if (selection_method == "shortest") {
        std::sort(paths_prm[0].begin(), paths_prm[0].end(), [](const path_with_length<Vector<3>>& a, const path_with_length<Vector<3>>& b) {
        return a.length < b.length;                          
        });
        for (int i = 0; i < num_paths_to_select; ++i) {
            selected_paths.push_back(paths_prm[0][i]);
        }
    } else if (selection_method == "least_turns") {
        std::sort(paths_prm[0].begin(), paths_prm[0].end(), [](const path_with_length<Vector<3>>& a, const path_with_length<Vector<3>>& b) {
        return count_sharp_turns(a) < count_sharp_turns(b);
        });
        for (int i = 0; i < num_paths_to_select; ++i) {
            selected_paths.push_back(paths_prm[0][i]);
        }
    } else if (selection_method == "comb") {
        double alpha = 0.5;
        double beta = 0.5;
        selected_paths = select_paths_by_score(paths_prm[0], alpha, beta, num_paths_to_select);
    }

    save_each_path_to_csv(selected_paths, "../scripts/trajectory_data/paths_csv");
    
    double best_duration = std::numeric_limits<double>::max();
    int best_trajectory_index = -1;
    std::vector<TrajectoryRecord> all_trajectories;

    // optimizing paths with PMM
    planner_config = YAML::LoadFile(planner_config_file);
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

    for (size_t i = 0; i < selected_paths.size(); i++) {
        std::vector<Vector<3>> pmm_path = convert_path_to_pmm_format(selected_paths[i]);

        Vector<3> start_velocity = Vector<3>::Zero();
        Vector<3> end_velocity = Vector<3>::Zero();
        
        bool final_collision_free = false;
        int retry_count = 0;
        const int max_retries = 10;
        double first_duration = -1.0;

        while (!final_collision_free && retry_count < max_retries) {
            pmm::PMM_MG_Trajectory3D tr_candidate(pmm_path, start_velocity, end_velocity,
                max_acc_norm, max_vel_norm, dT_precision, max_iter, alpha,
                alpha_reduction_factor, alpha_min_threshold, TD_max_iter,
                TD_acc_precision, second_round_opt, max_iter2, alpha2,
                alpha_reduction_factor2, alpha_min_threshold2, drag, debug);

            std::vector<Scalar> t_s;
            std::vector<Vector<3>> p_s;
            std::vector<Vector<3>> v_s;
            std::vector<Vector<3>> a_s;
            std::tie(t_s, p_s, v_s, a_s) = tr_candidate.get_sampled_trajectory(sampling_step);

            auto [collision_free, collision_point] = is_path_collision_free(p_s, map, min_clearance_);

            if (first_duration < 0.0) {
                first_duration = tr_candidate.duration(); 
            }

            if (collision_free) {
                final_collision_free = true;

                TrajectoryRecord record;
                record.original_path = selected_paths[i];
                record.trajectory = tr_candidate;
                record.duration = tr_candidate.duration();
                all_trajectories.push_back(record);

                if (retry_count > 0) {
                    std::cout << "[INFO] Trajectory " << i
                            << " was corrected. Original duration: " << first_duration
                            << " s, new duration: " << record.duration << " s." << std::endl;
                } else {
                    std::cout << "[INFO] Trajectory " << i
                            << " valid without correction. Duration: " << record.duration << " s." << std::endl;
                }

                if (record.duration < best_duration) {
                    best_duration = record.duration;
                    best_trajectory_index = static_cast<int>(all_trajectories.size() - 1);
                }

                std::string filename = "../scripts/trajectory_data/trajectories/traj_" + std::to_string(i) + ".csv";
                tr_candidate.sample_and_export_trajectory(sampling_step, filename);

            } else {
                int closest_seg_idx;
                pmm::Vector<3> closest_point;
                std::tie(closest_seg_idx, closest_point) = get_closest_point(pmm_path, collision_point);

                pmm_path.insert(pmm_path.begin() + closest_seg_idx + 1, closest_point);
                retry_count++;
            }
        }

        if (!final_collision_free) {
            std::cerr << "[WARNING] Could not generate collision-free trajectory after "
                    << max_retries << " attempts." << std::endl;
        }
    }

    if (best_trajectory_index >= 0) {
    auto& best = all_trajectories[best_trajectory_index];
    best.trajectory.sample_and_export_trajectory(sampling_step, "../scripts/trajectory_data/" + sampled_trajectory_file);

    std::cout << "Best trajectory corresponds to original path with length: " << best.original_path.length << std::endl;
    std::cout << "And has duration: " << best.duration << " s." << std::endl;
    }
}

int main(int argc, char** argv) {
    startLogger("main");
    execute_combined_planning();
    return 0;
}
