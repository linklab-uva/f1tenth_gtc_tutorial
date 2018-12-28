/*
 *
 * Corner class
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

#ifndef _CORNER_H_
#define _CORNER_H_

#include "gap.h"

namespace FollowTheGap {
    class Corner : public Gap {
        public:
        enum class CornerTypes {
            kLeft,
            kRight
        };
        Corner() = delete;
        Corner(Obstacle const & o1, Obstacle const & o2, CornerTypes corner_type)
            : corner_type_(corner_type), Gap(o1, o2) {
            };
        CornerTypes CornerType() const {
            return corner_type_;
        };
        private:
        CornerTypes corner_type_;
    };
}

#endif // CORNER_H_

