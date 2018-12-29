/*
 *
 * Follow the Corner main class
 *
 * Author: Anders Solberg Pedersen
 * Copyright (C) 2018 Czech Technical University in Prague
 *
 * This file is part of rericha_racing.
 *
 * rericha_racing is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rericha_racing is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with rericha_racing.  If not, see <https://www.gnu.org/licenses/>.
 *
 */



#include "obstacle.h"
#include "gap.h"
#include "corner.h"
#include <vector>
#include <cmath>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"

float last_final_heading_angle = 0;

namespace FollowTheGap {
    namespace AngleFilter {
        size_t left_index;
        size_t right_index;
    };

    // Parameteres
    static float kMaxRange = 0.9;
    static float g_fovAngleMax = M_PI/2+M_PI/16;
    static float g_goal_angle;
    // Constants
    static constexpr float kDistanceToCorner = 0.22;
    static constexpr int unsigned kPublishMessageBufferSize = 10;
    static constexpr int unsigned kSubscribeMessageBufferSize = 1;
    static constexpr float kTurnRadius = 0.3;
    static constexpr float kCarRadius = 0.4;
    static constexpr float kGapWeightCoefficient = 100;
    static constexpr float kCornerWeightCoefficient = 100;
    // static constexpr float kGapWeightCoefficient = 10;
    // static constexpr float kCornerWeightCoefficient = 10;
    static constexpr float kFovAngle = M_PI/4;
    // Max range for obstacles to be taken into account in metres
    // In metres
    /*
    static constexpr float kTrackMaxWidth = 1;
    */
    static constexpr float kTrackMinWidth = 0.35;

    // Static objects
    static Obstacle obstacle_nhol_left(kTurnRadius, M_PI/2, kTurnRadius);
    static Obstacle obstacle_nhol_right(kTurnRadius, -M_PI/2, kTurnRadius);
    static ros::NodeHandle * node_handle_pointer;
    static ros::Publisher publisher_final_heading_angle;
    // For visualizing and debugging
    static ros::Publisher publisher_visualize_largest_gap;
    static ros::Publisher publisher_visualize_final_heading_angle;
    static ros::Publisher publisher_visualize_obstacles;

    // Function declarations
    float FollowTheGapMethod(sensor_msgs::LaserScan::ConstPtr const & lidar_data);
    float FtgWeightedAverageMethod(sensor_msgs::LaserScan::ConstPtr const & lidar_data);
    float FollowTheCornerMethod(sensor_msgs::LaserScan::ConstPtr const & lidar_data);
    float FollowLargestVerticalGapMethod(sensor_msgs::LaserScan::ConstPtr const & lidar_data);
    float FtgWithAngleFilter(sensor_msgs::LaserScan::ConstPtr const & lidar_data);
    std::vector<Gap> FindGapsAngle(std::vector<Obstacle> & obstacles);
    std::vector<Gap> FindGapsVerticalDistance(std::vector<Obstacle> & obstacles);
    float FindVerticalGapSafeAngle(Gap const & gap);
    float CalculateGapCenterAngle(Gap const &);
    float CalculateGapCenterAngleBasic(Gap const &);
    float CalculateFinalHeadingAngle(float const goal_angle, float const gap_center_angle, float const d_min, float const alpha);
    void Callback(sensor_msgs::LaserScan::ConstPtr const & lidar_data);
    void AngleFilterLeftCallback(std_msgs::Int32::ConstPtr const & message );
    void AngleFilterRightCallback(std_msgs::Int32::ConstPtr const & message );
    void GoalAngleCallback(std_msgs::Float64::ConstPtr const & message);
    std::vector<Obstacle> CreateObstacles(sensor_msgs::LaserScan::ConstPtr const & lidar_data);
    std::vector<Obstacle> CreateObstaclesWithAngleFilter(sensor_msgs::LaserScan::ConstPtr const & lidar_data);
    void FilterObstacles(std::vector<Obstacle> & obstacles);
    Corner FindCorner(std::vector<Obstacle> & obstacles);
    float FindSafeCornerAngle(Corner & corner);
    void PublishFinalHeadingAngle(float final_heading_angle);
    // For visualizing and debugging
    void PublishVisualizeFinalHeadingAngle(float final_heading_angle);
    void PublishVisualizeLargestGap(Gap const & largest_gap);
    void PublishVisualizeObstacles(std::vector<Obstacle> const & obstacles);

    // Exceptions
    class InvalidAngleException : public std::logic_error {
        public:
        InvalidAngleException(std::string const & msg)
            : std::logic_error(msg) {
        };
    };
    class CenterOutsideGapException : public std::logic_error {
        public:
        CenterOutsideGapException(std::string const & msg) 
            : std::logic_error(msg) {
        };
    };
    class NoGapFoundException : public std::runtime_error {
        public:
        NoGapFoundException(std::string const & msg)
            : std::runtime_error(msg) {
        };
    };
};

using namespace FollowTheGap;

int main(int argc, char * * argv) {
    ros::init(argc, argv, "follow_the_gap");
    FollowTheGap::node_handle_pointer = new ros::NodeHandle;

    ros::Subscriber lidar_data_subscriber = FollowTheGap::node_handle_pointer
        ->subscribe("/scan", FollowTheGap::kSubscribeMessageBufferSize, FollowTheGap::Callback);
    ros::Subscriber angle_filter_subscriber_right = FollowTheGap::node_handle_pointer
        ->subscribe("/right_constraint_index", FollowTheGap::kSubscribeMessageBufferSize,
                FollowTheGap::AngleFilterRightCallback
    );
    ros::Subscriber angle_filter_subscriber_left = FollowTheGap::node_handle_pointer
        ->subscribe("/left_constraint_index", FollowTheGap::kSubscribeMessageBufferSize,
                FollowTheGap::AngleFilterLeftCallback
    );

    ros::Subscriber goal_angle_subscriber = FollowTheGap::node_handle_pointer
        ->subscribe("/lsr/angle", FollowTheGap::kSubscribeMessageBufferSize,
                FollowTheGap::GoalAngleCallback
    );


    publisher_final_heading_angle = FollowTheGap::node_handle_pointer
        ->advertise<std_msgs::Float32>("/final_heading_angle", kPublishMessageBufferSize);
    publisher_visualize_largest_gap = FollowTheGap::node_handle_pointer
        ->advertise<geometry_msgs::PointStamped>("/visualize_largest_gap", kPublishMessageBufferSize);
    publisher_visualize_final_heading_angle = FollowTheGap::node_handle_pointer
        ->advertise<geometry_msgs::PoseStamped>("/visualize_final_heading_angle", kPublishMessageBufferSize);
    publisher_visualize_obstacles = FollowTheGap::node_handle_pointer
        ->advertise<visualization_msgs::Marker>("/visualize_obstacles", kPublishMessageBufferSize);

    while(ros::ok()) {
        ros::spin();
    }
    return 0;
}

static Obstacle const & FindNearestObstacle(std::vector<Obstacle> const & obstacles) {
    return *std::min_element(obstacles.cbegin(), obstacles.cend(), [](Obstacle const & a, Obstacle const & b) {
        return a.distance < b.distance;
    });
}

float FollowTheGap::FollowTheGapMethod(sensor_msgs::LaserScan::ConstPtr const & lidar_data) {
    std::vector<Obstacle> obstacles = FollowTheGap::CreateObstacles(lidar_data);
    FollowTheGap::FilterObstacles(obstacles);
    std::vector<Gap>::const_iterator largest_gap;
    std::vector<Gap> gaps;
    float final_heading_angle;
    try {
        gaps = FindGapsAngle(obstacles);
        largest_gap = std::max_element(gaps.cbegin(), gaps.cend(),[](Gap const & a, Gap const & b){
            return a.gap_size < b.gap_size;
        });
    } catch (InvalidAngleException const & e) {
        std::cerr << "ERROR: Invalid angle encountered when creating gap array: " << std::endl;
        std::cerr << e.what() << std::endl;
        throw NoGapFoundException("Found invalid angle.");
    }

    if ( largest_gap == gaps.cend() ) {
        throw NoGapFoundException("No gap found");
    }
    else {
        float gap_center_angle;
        try {
            gap_center_angle = CalculateGapCenterAngle(*largest_gap);
        } catch (InvalidAngleException const & e) {
            std::cerr << "ERROR: Exception occurred when calculating gap centre angle" << std::endl;
            std::cerr << e.what() << std::endl;
            throw NoGapFoundException("Found invalid gap center angle");
        } catch (CenterOutsideGapException) {
            // Fall back to the CalculateGapCenterAngleBasic
            std::cerr << "Centre angle was outside gap. Falling back to CalculateGapCenterAngleBasic" << std::endl;
            gap_center_angle = CalculateGapCenterAngleBasic(*largest_gap);
        }
        Obstacle const & nearest_obstacle = FindNearestObstacle(obstacles);
        // TODO: Set goal angle by some other, reasonable means
        // klapajar: TODO: ^^ Probably set this to make it perpendicular to the line connecting edges of largest gap
        //                 -- Another option is to set this with respect to its current position from car location. (Somehow.)
        final_heading_angle = FollowTheGap::CalculateFinalHeadingAngle(g_goal_angle, gap_center_angle, nearest_obstacle.distance, kGapWeightCoefficient);
    }
    if ( std::isnan(final_heading_angle) ) {
        throw NoGapFoundException("Final heading angle was nan");
    }
    PublishVisualizeLargestGap(*largest_gap);
    PublishVisualizeObstacles(obstacles);
    return final_heading_angle;
}

float FollowTheGap::FtgWithAngleFilter(sensor_msgs::LaserScan::ConstPtr const & lidar_data) {
    // std::vector<Obstacle> obstacles = FollowTheGap::CreateObstaclesWithAngleFilter(lidar_data);
    std::vector<Obstacle> obstacles = FollowTheGap::CreateObstacles(lidar_data);
    if ( obstacles.empty() ) {
        std::cerr << "No obstacles found" << std::endl;
        throw NoGapFoundException("No gap found");
    }
    FollowTheGap::FilterObstacles(obstacles);
    std::vector<Gap>::const_iterator largest_gap;
    std::vector<Gap> gaps;
    float final_heading_angle;
    float gap_angle;
    float const filter_angle_left = lidar_data->angle_min + AngleFilter::left_index*lidar_data->angle_increment;
    float const filter_angle_right = lidar_data->angle_min + AngleFilter::right_index*lidar_data->angle_increment;
    try {
        gaps = FindGapsAngle(obstacles);
        bool ok = false;
        while (!ok && !gaps.empty() ) {
            largest_gap = std::max_element(gaps.cbegin(), gaps.cend(),[](Gap const & a, Gap const & b){
                return a.gap_size < b.gap_size;
            });
            gap_angle = CalculateGapCenterAngle(*largest_gap);
            if ( (gap_angle > filter_angle_right) && (gap_angle < filter_angle_left) ) {
                ok = true;
            } else {
                gaps.erase(largest_gap);
            }
        }
    } catch (InvalidAngleException const & e) {
        std::cerr << "ERROR: Invalid angle encountered when creating gap array: " << std::endl;
        std::cerr << e.what() << std::endl;
        throw NoGapFoundException("Found invalid angle.");
    }

    /*
    if ( ( largest_gap == gaps.cend() ) || gaps.empty() ) {
        // No gap found
        gaps = FindGapsVerticalDistance(obstacles);
        bool ok = false;
        while (!ok && !gaps.empty() ) {
            largest_gap = std::max_element(gaps.cbegin(), gaps.cend(),[](Gap const & a, Gap const & b){
                return a.gap_distance < b.gap_distance;
            });
            gap_angle = FindVerticalGapSafeAngle(*largest_gap);
            if ( (gap_angle > filter_angle_right) && (gap_angle < filter_angle_left) ) {
                ok = true;
            } else {
                gaps.erase(largest_gap);
            }
        }
    }
    */
    if ( (largest_gap == gaps.cend()) || gaps.empty() ) {
        throw NoGapFoundException("No gap found");
    }
    else {
        // TODO: Set goal angle by some other, reasonable means
        // klapajar: TODO: ^^ Probably set this to make it perpendicular to the line connecting edges of largest gap
        //                 -- Another option is to set this with respect to its current position from car location. (Somehow.)
        Obstacle const & nearest_obstacle = FindNearestObstacle(obstacles);
        final_heading_angle = FollowTheGap::CalculateFinalHeadingAngle(g_goal_angle, gap_angle, nearest_obstacle.distance, kGapWeightCoefficient);
    }
    if ( std::isnan(final_heading_angle) ) {
        throw NoGapFoundException("Final heading angle was nan");
    }
    PublishVisualizeLargestGap(*largest_gap);
    PublishVisualizeObstacles(obstacles);
    return final_heading_angle;
}


float FollowTheGap::FtgWeightedAverageMethod(sensor_msgs::LaserScan::ConstPtr const & lidar_data) {
    float const max_range = lidar_data->range_max;
    float const min_range = std::max(lidar_data->range_min, 2.0f);
    float const range_increment = 2;
    int unsigned num_its =(max_range-min_range)/range_increment;
    int unsigned s = 0;
    float final_heading_angle = 0;
    // We add angles together weighted by i, such that the angles from lower kMaxRange get highter weights.
    // The final result is then divided by s, which is the sum of all the weights
    for ( int unsigned i = 1; i <= num_its; ++i ) {
        // kMaxRange = min_range + i*range_increment;
        kMaxRange = max_range - i*range_increment;
        float angle;
        try {
            angle = FollowTheGapMethod(lidar_data);
            final_heading_angle += angle * (static_cast<float>(i)/num_its);
            s += i;
        } catch ( NoGapFoundException & e ) {
            // Do nothing
        }
    }
    if ( s == 0 ) {
        throw NoGapFoundException("");
    }
    final_heading_angle /= s;
    return final_heading_angle;
}

float FollowTheGap::FollowTheCornerMethod(sensor_msgs::LaserScan::ConstPtr const & lidar_data) {
    std::vector<Obstacle> obstacles = FollowTheGap::CreateObstacles(lidar_data);
    FollowTheGap::FilterObstacles(obstacles);
    Corner corner = FindCorner(obstacles);

    // float final_heading_angle = FindSafeCornerAngle(corner);
    // Try to weight the corner angle with the goal angle
    float const safe_corner_angle = FindSafeCornerAngle(corner);
//    std::cerr << "safe_angle " << safe_corner_angle << "\n";
    Obstacle const & nearest_obstacle = FindNearestObstacle(obstacles);
    float const final_heading_angle = CalculateFinalHeadingAngle(g_goal_angle,
            safe_corner_angle, nearest_obstacle.distance, kCornerWeightCoefficient);
//    std::cerr << "final_angle " << final_heading_angle << "\n";
    PublishVisualizeLargestGap(corner);
    PublishVisualizeObstacles(obstacles);
    return final_heading_angle;
}

float FollowTheGap::FollowLargestVerticalGapMethod(sensor_msgs::LaserScan::ConstPtr const & lidar_data) {
    float final_heading_angle;
    std::vector<Obstacle> obstacles = FollowTheGap::CreateObstacles(lidar_data);
    FollowTheGap::FilterObstacles(obstacles);
    std::vector<Gap> gaps = FindGapsVerticalDistance(obstacles);
    auto largest_gap = std::max_element(gaps.cbegin(), gaps.cend(),[](Gap const & a, Gap const & b){
        return a.gap_distance < b.gap_distance;
    });
    if ( largest_gap == gaps.cend() ) {
        throw NoGapFoundException("");
    }
    final_heading_angle = FindVerticalGapSafeAngle(*largest_gap);
    PublishVisualizeLargestGap(*largest_gap);
    PublishVisualizeObstacles(obstacles);
    return final_heading_angle;
}

void FollowTheGap::AngleFilterLeftCallback(std_msgs::Int32::ConstPtr const & message ) {
    AngleFilter::left_index = message->data;
}
void FollowTheGap::AngleFilterRightCallback(std_msgs::Int32::ConstPtr const & message ) {
    AngleFilter::right_index = message->data;
}

void FollowTheGap::GoalAngleCallback(std_msgs::Float64::ConstPtr const & message) {
    g_goal_angle = message->data;
}

void FollowTheGap::Callback(sensor_msgs::LaserScan::ConstPtr const & lidar_data ) {
    float final_heading_angle;

    bool ok = false;
    /*
    // Try ftg with weighted average
    if ( !ok ) {
        try {
            kMaxRange = 10;
            final_heading_angle = FtgWeightedAverageMethod(lidar_data);
            ok = true;
        } catch (NoGapFoundException & e) {
            // std::cerr << "No gap found with follow the gap method: ";
            // std::cerr << e.what() << std::endl;
        }
    }
    */
    // Try normal follow the gap method at max range
/*
    if ( !ok ) {
        try {
            kMaxRange = 10;
            final_heading_angle = FollowTheGapMethod(lidar_data);
            ok = true;
        } catch (NoGapFoundException & e) {
            // std::cerr << "No gap found with follow the gap method: ";
            // std::cerr << e.what() << std::endl;
        }
    }
*/
    // Try to find gaps based on distance
/*
    if ( !ok ) {
        try {
            kMaxRange = 10;
            final_heading_angle = FollowLargestVerticalGapMethod(lidar_data);
            ok = true;
        } catch (NoGapFoundException & e) {
            // std::cerr << "No gap found with follow the gap method: ";
            // std::cerr << e.what() << std::endl;
        }
    }
*/
    // Try ftg at lower ranges
    // If that does not work, increase fov
    g_fovAngleMax = M_PI/2;//+M_PI/16;
    // while ( (!ok) && (g_fovAngleMax < lidar_data->angle_max) ) {
        kMaxRange = 4;
        while ( (!ok) && (kMaxRange>=2) ) {
            try {
                final_heading_angle = FollowTheGapMethod(lidar_data);
                ok = true;
//                std::cerr << "gap found with follow the gap method: ";
            } catch (NoGapFoundException & e) {
                kMaxRange = kMaxRange - 0.5;
                // std::cerr << "No gap found with follow the gap method: ";
                // std::cerr << e.what() << std::endl;
            }
        }
        g_fovAngleMax += M_PI/16;
    // }
    // Try corner method

    if ( !ok ) {
        try {
            kMaxRange = 5;
            g_fovAngleMax = M_PI/2-M_PI/8;
            final_heading_angle = FollowTheCornerMethod(lidar_data);
            ok = true;
//            std::cerr << "gap found with follow the corner method: ";
        } catch (NoGapFoundException & e) {
            // std::cerr << "No gap found with follow the corner method: ";
            // std::cerr << e.what() << std::endl;
        }
    }

    // Retry ftg for close ranges
    if ( !ok ) {
        kMaxRange = 2;
        g_fovAngleMax = M_PI/2+M_PI/8;
        while ( (!ok) && (kMaxRange>=1.5) ) {
            try {
                final_heading_angle = FollowTheGapMethod(lidar_data);
                ok = true;
//                std::cerr << "gap found with closer follow the gap method: ";
            } catch (NoGapFoundException & e) {
                kMaxRange = kMaxRange - 0.5;
                // std::cerr << "No gap found with follow the gap method: ";
                // std::cerr << e.what() << std::endl;
            }
        }
    }
    // Retry follow the corner for larger angles
    if (!ok) {
        g_fovAngleMax = M_PI/2-M_PI/16;
        while ( (!ok) && (g_fovAngleMax < M_PI) ) {
            kMaxRange = 3;
            while ( (!ok) && (kMaxRange>=0.5) ) {
                try {
                    final_heading_angle = FollowTheCornerMethod(lidar_data);
                    ok = true;
//                    std::cerr << "gap found with angler follow the corner method: ";
                } catch (NoGapFoundException & e) {
                    kMaxRange = kMaxRange - 0.5;
                    // std::cerr << "No gap found with follow the corner method: ";
                    // std::cerr << e.what() << std::endl;
                }
            }
        g_fovAngleMax += M_PI/32;
        }
    }
    /*
    // Try Ftg with angle filter (Davids Method)
        kMaxRange = 4;
        while ( (!ok) && (kMaxRange>=1) ) {
            try {
                final_heading_angle = FtgWithAngleFilter(lidar_data);
                ok = true;
            } catch (NoGapFoundException & e) {
                kMaxRange = kMaxRange - 0.5;
                // std::cerr << "No gap found with follow the gap method: ";
                // std::cerr << e.what() << std::endl;
            }
        }
        */
   
    /*
    if ( !ok ) {
        g_fovAngleMax += M_PI/16;
        try {
            final_heading_angle = FollowLargestVerticalGapMethod(lidar_data);
            ok = true;
        } catch (NoGapFoundException & e) {
            // std::cerr << "No gap found with follow the gap method: ";
            // std::cerr << e.what() << std::endl;
        }
    }
    */
    if ( ok ) {
        kMaxRange = 3;
        PublishFinalHeadingAngle(final_heading_angle);
        PublishVisualizeFinalHeadingAngle(final_heading_angle);

        last_final_heading_angle = final_heading_angle;
    } else {
        std::cerr << "No gaps found" << std::endl;
/*
        // klapajar TODO: Publish last angle if gap not found. Create conditions to avoid crashing!!!
        // !!!!!!!!! UNSAFE !!!!!!!!!
        std::cerr << "Publishing last value instead." << std::endl;
        PublishFinalHeadingAngle(last_final_heading_angle);
        PublishVisualizeFinalHeadingAngle(last_final_heading_angle);
*/
        kMaxRange = lidar_data->range_max;
        g_fovAngleMax = lidar_data->angle_max;
        std::vector<Obstacle> obstacles = CreateObstacles(lidar_data);
        PublishVisualizeObstacles(obstacles);
        // throw std::exception();
    }
}

void FollowTheGap::PublishFinalHeadingAngle(float final_heading_angle) {
    std_msgs::Float32 final_heading_angle_message;
    final_heading_angle_message.data = final_heading_angle;
    publisher_final_heading_angle.publish(final_heading_angle_message);
}

std::vector<Obstacle> FollowTheGap::CreateObstacles(sensor_msgs::LaserScan::ConstPtr const & lidar_data) {
    size_t const data_size = lidar_data->scan_time/lidar_data->time_increment;
    float const & angle_increment = lidar_data->angle_increment;
    float const & range_max = lidar_data->range_max;
    float const & range_min = lidar_data->range_min;
    float angle = lidar_data->angle_min;
    std::vector<Obstacle> obstacles;
    for ( size_t i = 0; i < data_size; ++i, angle += angle_increment ) {
        float const & range = lidar_data->ranges[i];
        // Throw away all range values that are NAN
        /* Sometimes LIDARs return NAN / inf values */
        if (std::isnan(range)) {
            continue;
        }
        // Filter data which is outside the lidar fov
        if ( (angle < lidar_data->angle_min ) || (angle > lidar_data->angle_max) ) {
            continue;
        }
        // Filter data which is outside desired fov
        if ( std::abs(angle) > g_fovAngleMax ) {
            continue;
        }
        // Filter data which is too far away to be interesting
        if ( range > kMaxRange ) {
            continue;
        }
        // Filter data which is outside the max range or inside min range of lidar
        if ( (range < range_min) || (range > range_max) ) {
            continue;
        }
        // Throw away all outliers (point without neighbours)
        // klapajar: TODO: It is possible to use "is someone around me?" but it leads to losing information about space
        //                 It seems to be better using vectors. but TODO.
        float obstacle_radius = kCarRadius;
        obstacles.emplace_back(range, angle, obstacle_radius);
    }
    // We reverse the array so that the first obstacles are on the left(largest angle, in accordance with the paper)
    std::reverse(obstacles.begin(), obstacles.end());
    return obstacles;
}

std::vector<Obstacle> FollowTheGap::CreateObstaclesWithAngleFilter(
        sensor_msgs::LaserScan::ConstPtr const & lidar_data) {
    size_t const data_size = lidar_data->scan_time/lidar_data->time_increment;
    float const & angle_increment = lidar_data->angle_increment;
    float const & range_max = lidar_data->range_max;
    float const & range_min = lidar_data->range_min;
    // We start at the angle corresponding to the right index
    float angle = lidar_data->angle_min + AngleFilter::right_index*angle_increment;
    std::vector<Obstacle> obstacles;
    std::cerr << "right_index: " << AngleFilter::right_index << std::endl;
    std::cerr << "left_index: " << AngleFilter::left_index << std::endl;
    for ( size_t i = AngleFilter::right_index; i <= AngleFilter::left_index; ++i, angle += angle_increment ) {
        float const & range = lidar_data->ranges[i];
        // Throw away all range values that are NAN
        /* Sometimes LIDARs return NAN / inf values */
        if (std::isnan(range)) {
            continue;
        }
        // Filter data which is outside the lidar fov
        if ( (angle < lidar_data->angle_min ) || (angle > lidar_data->angle_max) ) {
            continue;
        }
        // Filter data which is outside desired fov
        if ( std::abs(angle) > g_fovAngleMax ) {
            continue;
        }
        // Filter data which is too far away to be interesting
        if ( range > kMaxRange ) {
            continue;
        }
        // Filter data which is outside the max range or inside min range of lidar
        if ( (range < range_min) || (range > range_max) ) {
            continue;
        }
        // Throw away all outliers (point without neighbours)
        // klapajar: TODO: It is possible to use "is someone around me?" but it leads to losing information about space
        //                 It seems to be better using vectors. but TODO.
        float obstacle_radius = kCarRadius;
        obstacles.emplace_back(range, angle, obstacle_radius);
    }
    // We reverse the array so that the first obstacles are on the left(largest angle, in accordance with the paper)
    std::reverse(obstacles.begin(), obstacles.end());
    return obstacles;
}

static float FindDistanceBetweenObstacleAndFov(float fov_angle, Obstacle const & obstacle) {
    float gap_size = std::abs(obstacle.angle - fov_angle);
    return obstacle.distance_to_center * std::sin(gap_size) - obstacle.radius;
}

static float FindDistanceBetweenObstacleAndNhol(Obstacle const & obstacle) {
    return obstacle_nhol_left.DistanceBetweenObstacleEdges(obstacle);
}

static float FindDistanceBetweenObstacleAndNholRight(Obstacle const & obstacle) {
    return obstacle_nhol_right.DistanceBetweenObstacleEdges(obstacle);
}

static float FindThetaNhol(Obstacle const & obstacle) {
    // Derivation of this expression was done on paper. See report from pvt spring 2018
    // TODO: Simplify
    // klapajar: TODO: ^^ It would be fine to put these equations to some report (to make them readable and possible to check).
    double const r = kTurnRadius;
    double const r_squared = r*r;
    double const d_o = obstacle.distance_to_center;
    double const d_o_squared = d_o*d_o;
    double const theta_o = std::abs(obstacle.angle);
    double const d_ro_squared = r_squared + d_o_squared - 2*r*d_o*std::cos(M_PI/2 - theta_o);
    double const d_ro = std::sqrt(d_ro_squared);
    double const theta_r = std::acos((r_squared + d_ro_squared - d_o_squared)/(2*r*d_ro));
    double const d_nhol_to_o = d_ro - r;
    double const d_nhol_to_o_squared = d_nhol_to_o*d_nhol_to_o;
    double const d_r_squared = 2*r_squared*(1-std::cos(theta_r));
    double const d_r = std::sqrt(d_r_squared);
    double const theta = std::acos((d_r_squared + d_o_squared - d_nhol_to_o_squared)/
            (2*d_r*d_o));
    double const theta_nhol = theta + theta_o;
    return (float)theta_nhol;
}

std::vector<Gap> FollowTheGap::FindGapsAngle(std::vector<Obstacle> & obstacles) {
    std::vector<Gap> gaps;
    // First find gap between left border and first obstacle
/*
    float const d_fov_l = FindDistanceBetweenObstacleAndFov(-kFovAngle, obstacles.front());
    float const d_nhol_l = FindDistanceBetweenObstacleAndNhol(obstacles.front());
    float theta_lim_l;
    if ( d_nhol_l >= d_fov_l ) {
        theta_lim_l = -kFovAngle;
    } else {
        float theta_nhol_l = FindThetaNhol(obstacles.front());
        theta_lim_l = theta_nhol_l;
    }
    theta_lim_l = -kFovAngle;
    if ( obstacles.front().angle_left > theta_lim_l ) {
        // There is no gap
    } else {
        // To get the correct distance for calculation of gap center angle, we
        // model the leftmost obstacle as a point at distance d_r
        // with angle theta_lim_l
        if ( std::isnan(theta_lim_l) ) {
            throw InvalidAngleException("theta_lim_l was nan");
        }
        float d_r = std::cos(theta_lim_l)*obstacles.front().distance_to_center;
        obstacle_nhol_left = Obstacle(d_r, theta_lim_l, 0);
        // gaps.emplace_back(obstacle_nhol_left, obstacles.front());
    }
*/
    // Then find gap between obstacles
    for ( size_t i = 1; i < obstacles.size(); ++i ) {
        if ( obstacles[i-1].Overlaps(obstacles[i]) ) {
            // No gap
            continue;
        }
        else if ( obstacles[i-1].angle_right < obstacles[i].angle_left ) {
            // No gap
            continue;
        }
        gaps.emplace_back(obstacles[i-1], obstacles[i]);
    }
/*
    // Finally find gap between last obstacle and right border
    float const d_fov_r = FindDistanceBetweenObstacleAndFov(kFovAngle, obstacles.back());
    float const d_nhol_r = FindDistanceBetweenObstacleAndNhol(obstacles.back());
    float theta_lim_r;
    if ( d_nhol_r >= d_fov_r ) {
        theta_lim_r = kFovAngle;
    } else {
        float const theta_nhol_r = FindThetaNhol(obstacles.back());
        theta_lim_r = theta_nhol_r;
    }
    if ( obstacles.back().angle_right < theta_lim_r ) {
        // There is no gap
    } else {
        // To get the correct distance for calculation of gap center angle, we
        // model the rightmost obstacle as a point at distance d_r
        // with angle theta_lim_r
        if ( std::isnan(theta_lim_r) ) {
            throw InvalidAngleException("theta_lim_r was nan");
        }
        float d_r = std::cos(theta_lim_r)*obstacles.back().distance_to_center;
        obstacle_nhol_right = Obstacle(d_r, theta_lim_r, 0);
        // gaps.emplace_back(obstacles.back(), obstacle_nhol_right);
    }
*/
    return gaps;
}

Corner FollowTheGap::FindCorner(std::vector<Obstacle> & obstacles) {
    std::vector<Corner> corners;
    for ( size_t i = 1; i < obstacles.size(); ++i ) {
        Obstacle & obstacle_left = obstacles[i-1];
        Obstacle & obstacle_right = obstacles[i];
        float obstacles_distance = obstacle_left.DistanceBetweenObstacleCentres(obstacle_right);
        if ( obstacle_left.x > obstacle_right.x 
          && (obstacles_distance > kTrackMinWidth) ) {
            // std::cerr << "Right corner found" << std::endl;
            // Found right corner
            corners.emplace_back(obstacle_left, obstacle_right, Corner::CornerTypes::kRight);
        } 
        if ( (obstacle_right.x > obstacle_left.x) 
          && (obstacles_distance > kTrackMinWidth) ) {
            // std::cerr << "Left corner found" << std::endl;
            // Found left corner
            corners.emplace_back(obstacle_left, obstacle_right, Corner::CornerTypes::kLeft);
        }
    }
    if (corners.size() == 1) {
        // We can only accept an unamiguous situation where only one corner was found
        auto largest_corner = std::max_element(corners.cbegin(), corners.cend(), [](Corner const & a, Corner const & b){
                return a.gap_size < b.gap_size;
        });
        return *largest_corner;
    } else {
        throw NoGapFoundException("No corner found");
    }
}

float FollowTheGap::FindSafeCornerAngle(Corner & corner) {
    float angle;
    float theta_d;

    if ( corner.CornerType() == Corner::CornerTypes::kRight ) {
        /* vajnamar: FIXME:  It can happen that we are closer to the corner
         *                   (distance_to_center), than we want the perpendicular
         *                   distance (kDistanceToCorner) to be, then we want
         *                   to compute asin() of values > 1 which leads to a
         *                   NaN result. Temporary fixed by comparing the values
         *                   and forcing theta_d = M_PI/2 for the mentioned case.
         */
        if ( kDistanceToCorner < corner.obstacle_right->distance_to_center )
            theta_d = std::asin(kDistanceToCorner/corner.obstacle_right->distance_to_center);
        else
            theta_d = M_PI/2;
        angle = corner.obstacle_right->angle + theta_d;
        // return corner.obstacle_right->angle_left;
    } else if ( corner.CornerType() == Corner::CornerTypes::kLeft ) {
        if ( kDistanceToCorner < corner.obstacle_left->distance_to_center )
            theta_d = std::asin(kDistanceToCorner/corner.obstacle_left->distance_to_center);
        else
            theta_d = M_PI/2;
        angle = corner.obstacle_left->angle - theta_d;
        // return corner.obstacle_left->angle_right;
    } else {
        // This should not happen
        throw std::runtime_error("FollowTheGap::FindSafeCorner else case not implemented");
    }
    return angle;
}

float FollowTheGap::FindVerticalGapSafeAngle(Gap const & gap) {
    float angle;
    Obstacle const * obstacle_left = gap.obstacle_left;
    Obstacle const * obstacle_right = gap.obstacle_right;
    if ( obstacle_left->distance_to_center > obstacle_right->distance_to_center ) {
        // right obstacle is closer. Implies a right turn, so we must avoid the left side of the right obstacle
        angle = obstacle_right->angle_left;
    } else {
        angle = obstacle_left->angle_right;
    }
    return angle;
}

std::vector<Gap> FollowTheGap::FindGapsVerticalDistance(std::vector<Obstacle> & obstacles) {
    std::vector<Gap> gaps;
    // First find gap between left border and first obstacle
/*
    float const d_fov_l = FindDistanceBetweenObstacleAndFov(-kFovAngle, obstacles.front());
    float const d_nhol_l = FindDistanceBetweenObstacleAndNhol(obstacles.front());
    float theta_lim_l;
    if ( d_nhol_l >= d_fov_l ) {
        theta_lim_l = -kFovAngle;
    } else {
        float theta_nhol_l = FindThetaNhol(obstacles.front());
        theta_lim_l = theta_nhol_l;
    }
    if ( obstacles.front().angle_left > theta_lim_l ) {
        // There is no gap
    } else {
        // To get the correct distance for calculation of gap center angle, we
        // model the leftmost obstacle as a point at distance d_r
        // with angle theta_lim_l
        if ( std::isnan(theta_lim_l) ) {
            throw InvalidAngleException("theta_lim_l was nan");
        }
        float d_r = std::cos(theta_lim_l)*obstacles.front().distance_to_center;
        obstacle_nhol_left = Obstacle(d_r, theta_lim_l, 0);
        // gaps.emplace_back(obstacle_nhol_left, obstacles.front());
    }
*/
    // Then find gap between obstacles
    for ( size_t i = 1; i < obstacles.size(); ++i ) {
        if ( obstacles[i-1].DistanceBetweenObstacleCentres(obstacles[i]) < kCarRadius ) {
            // Gap too small
            continue;
        }
        gaps.emplace_back(obstacles[i-1], obstacles[i]);
    }
/*
    // Finally find gap between last obstacle and right border
    float const d_fov_r = FindDistanceBetweenObstacleAndFov(kFovAngle, obstacles.back());
    float const d_nhol_r = FindDistanceBetweenObstacleAndNhol(obstacles.back());
    float theta_lim_r;
    if ( d_nhol_r >= d_fov_r ) {
        theta_lim_r = kFovAngle;
    } else {
        float const theta_nhol_r = FindThetaNhol(obstacles.back());
        theta_lim_r = theta_nhol_r;
    }
    if ( obstacles.back().angle_right < theta_lim_r ) {
        // There is no gap
    } else {
        // To get the correct distance for calculation of gap center angle, we
        // model the rightmost obstacle as a point at distance d_r
        // with angle theta_lim_r
        if ( std::isnan(theta_lim_r) ) {
            throw InvalidAngleException("theta_lim_r was nan");
        }
        float d_r = std::cos(theta_lim_r)*obstacles.back().distance_to_center;
        obstacle_nhol_right = Obstacle(d_r, theta_lim_r, 0);
        // gaps.emplace_back(obstacles.back(), obstacle_nhol_right);
    }
*/
    return gaps;
}

void FilterLoneObstacleGroups(std::vector<Obstacle> & obstacles) {
    float const max_distance_center = 0.15; // Max distance
    for ( auto it = obstacles.begin(); it != obstacles.end();) {
        //Filter first point
        if ( obstacles.begin() == it) {
            if (obstacles.size() > 1)
            {
                auto nx_it = std::next(it, 1);
                if( it->DistanceBetweenObstacleCentres(*nx_it) > max_distance_center ) {
                    it = obstacles.erase(it);
                } else{
                    it++;
                }
            }
            else{
                it++;
            }
        //Filter endpoint
        } else if ( obstacles.end() == it){
            if (obstacles.size() > 1)
            {
                auto pv_it = std::prev(it, 1);
                if( it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center ) {
                    it = obstacles.erase(it);
                } else{
                    it++;
                }
            }
            else{
                it++;
            }
        //Filter the points that are in the middle
        } else{
            if (obstacles.size() >= 4){
                auto pv_it = std::prev(it, 1);
                auto nx1_it = std::next(it, 1);
                if ( obstacles.end() == nx1_it){
                    if((it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center) && (it->DistanceBetweenObstacleCentres(*nx1_it) > max_distance_center)){
                        it = obstacles.erase(it);
                    }
                    else{
                        it++;
                    }
                } else{
                   auto nx2_it = std::next(it, 2);
                    if((it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center) && (it->DistanceBetweenObstacleCentres(*nx1_it) < max_distance_center) && (nx1_it->DistanceBetweenObstacleCentres(*nx2_it) > max_distance_center)) {
                        it = obstacles.erase(it, nx1_it+1);
                    } else if((it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center) && (it->DistanceBetweenObstacleCentres(*nx1_it) > max_distance_center)){
                        it = obstacles.erase(it);
                    } else{
                        it++;
                    }
                }
            }
            else{
                it++;
            }

        /*} else{
            if (obstacles.size() >= 3){
                auto nx_it = std::next(it, 1);
                auto pv_it = std::prev(it, 1);
                if((it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center) && (it->DistanceBetweenObstacleCentres(*nx_it) > max_distance_center)) {
                    it = obstacles.erase(it);
                } else{
                    it++;
                }
            }
            else{
                it++;
            }*/
        }
    }
}

void FollowTheGap::FilterObstacles(std::vector<Obstacle> & obstacles) {
    FilterLoneObstacleGroups(obstacles);
    // Discard obstacles which overlap with the nonholonomic constraints circles
    for ( auto it = obstacles.begin(); it != obstacles.end();) {
        // if ( it->Overlaps(obstacle_nhol_left) ) {
            // it = obstacles.erase(it);
        // } else if ( it->Overlaps(obstacle_nhol_right) ) {
            // it = obstacles.erase(it);
        ++it;
    }
}

float FollowTheGap::CalculateGapCenterAngle(Gap const & gap) {
    double const d_1 = gap.obstacle_right->distance;
    double const d_2 = gap.obstacle_left->distance;;
    double const theta_1 = std::abs(gap.angle_right);
    double const theta_2 = std::abs(gap.angle_left);
    double theta_gap_c;

    // The original paper covers only the case where the obstacles are on
    // different sides of the x-axis. Here we describe also the cases
    // where obstacles are on the same side
    if ( (gap.angle_left >= 0) && (gap.angle_right <= 0) ) {
        theta_gap_c = std::acos((d_1 + d_2*std::cos(theta_1+theta_2))/
                    (std::sqrt((d_1*d_1) + (d_2*d_2) + 2*d_1*d_2*std::cos(theta_1+theta_2))))
                - theta_1;
        if ( std::isnan(theta_gap_c) ) {
            throw InvalidAngleException("Gap centre angle was nan for case gap.angle_right <= 0 && gap.angle_left >= 0");
        }
    } else if ( gap.angle_right >= 0 ) {
        // TODO: Simplify
        double const l_squared = (d_1*d_1 + d_2*d_2 - 2*d_1*d_2*std::cos(theta_2 - theta_1))/4;
        double const h_squared = (d_1*d_1 + d_2*d_2 - 2*l_squared) / 2;
        double const h = std::sqrt(h_squared);
        double const theta_x = std::acos((h_squared + d_1*d_1 - l_squared)/(2*h*d_1));
        theta_gap_c = theta_1 + theta_x;
        if ( std::isnan(theta_gap_c) ) {
            throw InvalidAngleException("Gap centre angle was nan for case gap.angle_right >= 0");
        }
    } else {
        // gap.angle_left <= 0
        // TODO: Simplify
        double const l_squared = (d_1*d_1 + d_2*d_2 - 2*d_1*d_2*std::cos(theta_1 - theta_2))/4;
        double const h_squared = (d_1*d_1 + d_2*d_2 - 2*l_squared) / 2;
        double const h = std::sqrt(h_squared);
        double const theta_x = std::acos((h_squared + d_2*d_2 - l_squared)/(2*h*d_2));
        theta_gap_c = -(theta_2 + theta_x);
        if ( std::isnan(theta_gap_c) ) {
            std::stringstream error_msg;
            error_msg << "Gap centre angle was nan for case gap.angle_left <= 0" << std::endl;
            throw InvalidAngleException(error_msg.str());
        }
    }

    if ( (theta_gap_c > gap.angle_left) || (theta_gap_c < gap.angle_right) ) {
        // The centre angle is outside the gap, which should never be the case
        throw CenterOutsideGapException("The calculated centre of gap was outside the gap");
    }

    if ( std::isnan(theta_gap_c) ) {
        throw InvalidAngleException("Gap centre angle was nan unknown case");
    }

    return theta_gap_c;
}

float FollowTheGap::CalculateGapCenterAngleBasic(Gap const & gap) {
    float const theta_1 = gap.angle_right;
    float const theta_2 = gap.angle_left;
    return (theta_1 + theta_2) / 2;
}

float FollowTheGap::CalculateFinalHeadingAngle(float const theta_goal, float const theta_c, float const d_min, float const alpha) {
    // float const & alpha = kGapWeightCoefficient;
    float theta_final = ((alpha/d_min) * theta_c + theta_goal) /
        ((alpha/d_min) + 1);
    return theta_final;
}

void FollowTheGap::PublishVisualizeFinalHeadingAngle(float final_heading_angle) {
    geometry_msgs::PoseStamped angle_message;
    angle_message.header.frame_id = "laser";

    tf::Quaternion angle_quaternion;
    angle_quaternion.setEuler(0, 0, final_heading_angle);
    angle_quaternion.normalize();
    tf::quaternionTFToMsg(angle_quaternion, angle_message.pose.orientation);

    publisher_visualize_final_heading_angle.publish(angle_message);
}

void FollowTheGap::PublishVisualizeLargestGap(Gap const & largest_gap) {

    geometry_msgs::PointStamped p0, p1, robot_point;
    robot_point.header.frame_id = "laser";
    robot_point.point.x = 0;
    robot_point.point.y = 0;
    p0.point.x = largest_gap.obstacle_left->distance*std::cos(largest_gap.obstacle_left->angle_right);
    p0.point.y = largest_gap.obstacle_left->distance*std::sin(largest_gap.obstacle_left->angle_right);
    p0.header.frame_id = "laser";
    p1.point.x = largest_gap.obstacle_right->distance*std::cos(largest_gap.obstacle_right->angle_left);
    p1.point.y = largest_gap.obstacle_right->distance*std::sin(largest_gap.obstacle_right->angle_left);
    p1.header.frame_id = "laser";

    publisher_visualize_largest_gap.publish(robot_point);
    publisher_visualize_largest_gap.publish(p0);
    publisher_visualize_largest_gap.publish(p1);

}

void FollowTheGap::PublishVisualizeObstacles(std::vector<Obstacle> const & obstacles) {
    visualization_msgs::Marker obstacle_points;
    obstacle_points.header.frame_id = "laser";
    obstacle_points.header.stamp = ros::Time::now();
    obstacle_points.type = visualization_msgs::Marker::POINTS;

    // obstacle_points.scale.x = kCarRadius;
    // obstacle_points.scale.y = kCarRadius;
    obstacle_points.scale.x = 0.05;
    obstacle_points.scale.y = 0.05;
    obstacle_points.color.r = 0;
    obstacle_points.color.g = 1;
    obstacle_points.color.b = 0;
    obstacle_points.color.a = 1;

    for ( auto const & o : obstacles ) {
        geometry_msgs::Point p;
        p.x = o.x;
        p.y = o.y;
        obstacle_points.points.push_back(p);
    }

    publisher_visualize_obstacles.publish(obstacle_points);
}


