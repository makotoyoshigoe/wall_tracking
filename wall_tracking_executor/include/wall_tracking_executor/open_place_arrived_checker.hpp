// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "wall_tracking_executor/open_place_checker.hpp"

namespace WallTracking
{
class OpenPlaceArrivedChecker : public OpenPlaceChecker{
    public:
    OpenPlaceArrivedChecker(
        float detect_angle_start,
        float detect_angle_end,
        int detect_angle_division_num, 
        float range_th, 
        float ratio_th);
    ~OpenPlaceArrivedChecker();
    bool open_place_arrived(void);

    private:
};
} // namespace lsa_nav_controller