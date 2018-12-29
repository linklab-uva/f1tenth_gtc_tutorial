/*
 *
 * Gap description class
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

#ifndef _GAP_H_
#define _GAP_H_

#include "obstacle.h"
#include <iostream>

namespace FollowTheGap {
    class Gap {
        public:
        Gap(Obstacle const & o1, Obstacle const & o2);
        float angle_left;
        float angle_right;
        float gap_size;
        // gap_distance is the distance between the obstacle centres
        float gap_distance;
        Obstacle const * obstacle_left;
        Obstacle const * obstacle_right;
    };
    std::ostream & operator<<(std::ostream & os, Gap const & g);
};


#endif // _GAP_H_
