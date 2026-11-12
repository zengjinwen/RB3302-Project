/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "differential_motion_model.hpp"
#include <cstdio>

namespace nav2_amcl
{

void
RB3302DifferentialMotionModel::initialize(
  double alpha1, double alpha2, double alpha3, double alpha4,
  double alpha5)
{
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
  alpha5_ = alpha5;
  std::fprintf(stderr, "[rb3302_amcl_motion] signed-vector-v2: mean-one translation noise, reverse supported\n");
}

void
RB3302DifferentialMotionModel::odometryUpdate(
  pf_t * pf, const pf_vector_t & pose,
  const pf_vector_t & delta)
{
  pf_sample_set_t * set = pf->sets + pf->current_set;
  const double old_yaw = pose.v[2] - delta.v[2];
  const double distance = hypot(delta.v[0], delta.v[1]);
  const double turn = delta.v[2];

  // No odometry change means no particle motion or injected motion noise.
  if (distance == 0.0 && turn == 0.0) {
    return;
  }

  // Signed displacement in the previous robot frame. No minimum step size,
  // no forward default, and no conversion of reverse motion to forward travel.
  const double forward = cos(old_yaw) * delta.v[0] + sin(old_yaw) * delta.v[1];
  const double sideways = -sin(old_yaw) * delta.v[0] + cos(old_yaw) * delta.v[1];
  // Mean-one positive multiplier: reverse sign is preserved without clipping bias.
  // Absolute translation uncertainty tends to zero continuously with distance.
  const double log_variance = log1p(alpha3_ + alpha4_ * turn * turn);
  const double rotation_sigma = sqrt(alpha1_ * turn * turn + alpha2_ * distance * distance);

  for (int i = 0; i < set->sample_count; ++i) {
    pf_sample_t & sample = set->samples[i];
    if (distance > 0.0) {
      // Noise changes travel magnitude, never its sign. A pure turn cannot
      // generate translation. Preserve lateral odometry displacement as well.
      const double scale = exp(pf_ran_gaussian(sqrt(log_variance)) - 0.5 * log_variance);
      const double yaw = sample.pose.v[2];
      sample.pose.v[0] += scale * (cos(yaw) * forward - sin(yaw) * sideways);
      sample.pose.v[1] += scale * (sin(yaw) * forward + cos(yaw) * sideways);
    }
    sample.pose.v[2] = angleutils::angle_diff(
      sample.pose.v[2] + turn + pf_ran_gaussian(rotation_sigma), 0.0);
  }
}

}  // namespace nav2_amcl

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_amcl::RB3302DifferentialMotionModel, nav2_amcl::MotionModel)
