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

#ifndef _OBSTACLE_H_
#define _OBSTACLE_H_

#include <iostream>

namespace FollowTheGap{
    struct Obstacle {
        Obstacle(float const distance, float const angle, float const radius);
        float DistanceBetweenObstacleEdges(Obstacle const & o) const;
        float DistanceBetweenObstacleCentres(Obstacle const & o) const;
        bool Overlaps(Obstacle const & o) const;
        float distance_to_center;
        float angle;
        float angle_left;
        float angle_right;
        float radius;
        float x;
        float y;
        float distance;
    };
    std::ostream & operator<<(std::ostream & os, Obstacle const & o);
};


#endif // OBSTACLE_H_
