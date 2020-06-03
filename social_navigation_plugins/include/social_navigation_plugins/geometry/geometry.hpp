// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SOCIAL_NAV_PLUGINS__GEOMETRY__GEOMETRY_HPP_
#define SOCIAL_NAV_PLUGINS__GEOMETRY__GEOMETRY_HPP_

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
using namespace nav2_costmap_2d;

namespace social_geometry
{

void polygonFillCells(
  const std::vector<MapLocation> & polygon,
  std::vector<MapLocation> & polygon_cells);
void getPolygon(
  nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<geometry_msgs::msg::Point> & polygon,
  std::vector<MapLocation> & polygon_cells);
std::vector<geometry_msgs::msg::Point>
makeProxemicShapeFromAngle(float r, float alpha, float orientation = 0.0);
double gaussian(
  double x, double y, double x0, double y0,
  double A, double varx, double vary, double skew);
double asymmetricGaussian(
  double x, double y, double x0, double y0,
  double A, double angle, double var_h, double var_s, double var_r);
}
// namespace social_geometry
#endif  // SOCIAL_NAV_PLUGINS__GEOMETRY__GEOMETRY_HPP_
