/*
 *
 * Obstacle description class
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
#include <cmath>

namespace FollowTheGap{
    Obstacle::Obstacle(float const distance, float const angle, float const radius)
        : distance_to_center(distance), angle(angle),
        x(distance*std::cos(this->angle)), y(distance*std::sin(this->angle)) {
            if ( radius > (distance-0.01) ) {
                // We make this adjustment to avoid nan during calculation of theta_d
                this->radius = distance-0.01;
            } else {
                this->radius = radius;
            }
            float const theta_d = std::asin(this->radius/distance);
            angle_left = angle+theta_d;
            angle_right = angle-theta_d;
            this->distance = std::sqrt(this->distance_to_center*this->distance_to_center-this->radius*this->radius);
        };
    float Obstacle::DistanceBetweenObstacleEdges(Obstacle const & o) const {
        return DistanceBetweenObstacleCentres(o) - radius - o.radius;
    };
    float Obstacle::DistanceBetweenObstacleCentres(Obstacle const & o) const {
        return std::hypot((x-o.x),(y-o.y));
    };
    bool Obstacle::Overlaps(Obstacle const & o) const {
        return DistanceBetweenObstacleEdges(o) < 0.0;
    };
    std::ostream & operator<<(std::ostream & os, Obstacle const & o) {
        os << "angle: " << o.angle << " ";
        os << "distance: " << o.distance << " ";
        os << "x: " << o.x << " ";
        os << "y: " << o.y << " ";
        os << "radius: " << o.radius;
        return os;
    }
};
