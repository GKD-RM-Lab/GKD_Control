/*******************************************************************************
* BSD 3-Clause License
*
* Copyright (c) 2021, Qiayuan Liao
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

//
// Created by qiayuan on 8/14/20.
//

#pragma once

#include "types.hpp"

namespace Control
{
   struct BulletSolverConfig
    {
       double resistance_coff_qd_10, resistance_coff_qd_15, resistance_coff_qd_16, resistance_coff_qd_18,
           resistance_coff_qd_30, g;
   };

   class BulletSolver
   {
      public:
       explicit BulletSolver();

       bool solve(Vec3d pos, Vec3d vel, double bullet_speed, double yaw, double v_yaw,
                  double r1, double r2, double dz, int armors_num);
       double getResistanceCoefficient(double bullet_speed) const;
       double getYaw() const {
           return output_yaw_;
       }
       double getPitch() const {
           return -output_pitch_;
       }
       ~BulletSolver() = default;
       Vec3d target_pos_{};

      private:
       BulletSolverConfig config_{};
       double max_track_target_vel_;
       double output_yaw_{}, output_pitch_{};
       double bullet_speed_{}, resistance_coff_{};
       int selected_armor_;
       bool track_target_;

       double fly_time_;
   };
}  // namespace rm_gimbal_controllers
