#pragma once

#include <cmath>
#include <cstddef>
#include <nav_msgs/msg/occupancy_grid.hpp>


namespace planner_utils {

    constexpr double PI = 3.14159265358979323846;


    inline std::tuple<int, int, int> poseToIndex(
        const geometry_msgs::msg::Pose& pose,
        const nav_msgs::msg::OccupancyGrid& grid,
        int num_angle_bins)
    {
        const double x   = pose.position.x;
        const double y   = pose.position.y;
        const double yaw = tf2::getYaw(pose.orientation);

        const double ox = grid.info.origin.position.x;
        const double oy = grid.info.origin.position.y;
        const double res = grid.info.resolution;

        // orientation of the map
        const double origin_yaw = tf2::getYaw(grid.info.origin.orientation);
        const double cos_ori = std::cos(origin_yaw);
        const double sin_ori = std::sin(origin_yaw);

        // convert world to local grid
        double dx = x - ox;
        double dy = y - oy;
        double gx = (cos_ori * dx + sin_ori * dy) / res;
        double gy = (-sin_ori * dx + cos_ori * dy) / res;

        int i = static_cast<int>(std::floor(gx));
        int j = static_cast<int>(std::floor(gy));

        // normalize yaw and compute discrete bin
        double yaw_local = normalizeAngle(yaw - origin_yaw);
        int bin = getAngleBinIndex(yaw_local, num_angle_bins);

        return std::make_tuple(i, j, bin);
    }


    inline double normalizeAngle(double angle) {
        double a = std::fmod(std::fmod(angle, 2.0 * PI) + 2.0 * PI, 2.0 * PI);
        return a > PI ? a - 2.0 * PI : a;
    }

    inline size_t getIndex(int x, int y, int theta, int width, int num_angle_bins) {
        return static_cast<size_t>(theta)
             + num_angle_bins * (static_cast<size_t>(x) + static_cast<size_t>(width) * static_cast<size_t>(y));
    }

    inline void decodeIndex(size_t index, int width, int num_angle_bins,
                            int &x, int &y, int &theta) {
        theta = index % num_angle_bins;
        size_t tmp = index / num_angle_bins;
        x = tmp % width;
        y = tmp / width;
    }

    inline int getAngleBinIndex(double yaw, int num_bins) {
        double angle_bin_size = 2.0 * PI / num_bins;
        int idx = static_cast<int>(std::floor(yaw / angle_bin_size + 0.5));
        return (idx >= num_bins) ? idx - num_bins : idx;
    }

    inline bool isOccupied(int8_t value, int threshold, bool allow_unknown) {
        return (value < 0) || (value >= threshold);
    }

    inline bool isOccupiedIndex(const nav_msgs::msg::OccupancyGrid &grid, int index,
                                bool allow_unknown, int threshold) {
        return isOccupied(grid.data[index], threshold, allow_unknown);
    }

    inline void worldToGrid(double world_x, double world_y,
                            double origin_x, double origin_y,
                            double cos_ori, double sin_ori, double resolution,
                            int &grid_x, int &grid_y) {
        double dx = world_x - origin_x;
        double dy = world_y - origin_y;
        double gx = (cos_ori * dx + sin_ori * dy) / resolution;
        double gy = (-sin_ori * dx + cos_ori * dy) / resolution;
        grid_x = static_cast<int>(std::floor(gx));
        grid_y = static_cast<int>(std::floor(gy));
    }

    inline void gridToWorld(int grid_x, int grid_y,
                            double origin_x, double origin_y,
                            double cos_ori, double sin_ori, double resolution,
                            double &world_x, double &world_y) {
        double x = (grid_x + 0.5) * resolution;
        double y = (grid_y + 0.5) * resolution;
        world_x = origin_x + cos_ori * x - sin_ori * y;
        world_y = origin_y + sin_ori * x + cos_ori * y;
    }

    inline bool isPathCollisionFree(
        const std::vector<geometry_msgs::msg::PoseStamped>& path_segment,
        const nav_msgs::msg::OccupancyGrid& grid,
        int threshold,
        bool allow_unknown)
    {
        int width = grid.info.width;
        int height = grid.info.height;
        double res = grid.info.resolution;
        double origin_x = grid.info.origin.position.x;
        double origin_y = grid.info.origin.position.y;

        for (const auto& pose : path_segment) {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;

            int i = static_cast<int>((x - origin_x) / res);
            int j = static_cast<int>((y - origin_y) / res);

            if (i < 0 || i >= width || j < 0 || j >= height)
                return false;

            int index = j * width + i;
            if (isOccupiedIndex(grid, index, allow_unknown, threshold))
                return false;
        }

        return true;
    }


}  // namespace planner_utils
