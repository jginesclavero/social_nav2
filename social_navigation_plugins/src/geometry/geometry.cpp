// Copyright 2020 Intelligent Robotics Lab
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


#include "social_navigation_plugins/geometry/geometry.hpp"

using namespace nav2_costmap_2d;

namespace social_geometry
{

void polygonFillCells(
  const std::vector<MapLocation> & polygon,
  std::vector<MapLocation> & polygon_cells)
{

  polygon_type b_poly;
  unsigned int min_x = 0, max_x = 0, min_y = 0, max_y = 0;
  for (auto p : polygon) {
    if (min_x == 0 && max_x == 0 &&
      min_y == 0 && max_y == 0)
    {
      min_x = p.x;
      max_x = p.x;
      min_y = p.y;
      max_y = p.y;
    }
    if (p.x < min_x) {min_x = p.x;}
    if (p.x > max_x) {max_x = p.x;}
    if (p.y < min_y) {min_y = p.y;}
    if (p.y > max_y) {max_y = p.y;}

    boost::geometry::append(b_poly.outer(), point_type(p.x, p.y));
  }
  for (unsigned int i = min_x; i < max_x; ++i) {
    for (unsigned int j = min_y; j < max_y; ++j) {
      point_type p(i, j);
      if (boost::geometry::within(p, b_poly)) {
        MapLocation c;
        c.x = i;
        c.y = j;
        polygon_cells.push_back(c);
      }
    }
  }
}

void getPolygon(
  nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<geometry_msgs::msg::Point> & polygon,
  std::vector<MapLocation> & polygon_cells)
{
  // we assume the polygon is given in the global_frame...
  // we need to transform it to map coordinates
  std::vector<MapLocation> map_polygon;
  for (unsigned int i = 0; i < polygon.size(); ++i) {
    MapLocation loc;
    if (!costmap->worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y)) {continue;}
    // Only push a cell if is inside the map
    map_polygon.push_back(loc);
  }
  // get the cells that fill the polygon
  social_geometry::polygonFillCells(map_polygon, polygon_cells);
  //convexFillCells(map_polygon, polygon_cells);
}

std::vector<geometry_msgs::msg::Point>
makeProxemicShapeFromAngle(float r, float alpha, float orientation)
{
  std::vector<geometry_msgs::msg::Point> points;
  // Loop over 32 angles around a circle making a point each time
  int N = 32;
  int it = (int) round((N * alpha) / (2 * M_PI));
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < it; ++i) {
    double angle = i * 2 * M_PI / N + orientation;
    pt.x = cos(angle) * r;
    pt.y = sin(angle) * r;
    points.push_back(pt);
  }
  if (alpha < 2 * M_PI) {
    pt.x = 0.0;
    pt.y = 0.0;
    points.push_back(pt);
  }
  pt.x = points[0].x;
  pt.y = points[0].y;
  points.push_back(pt);
  return points;
}

double gaussian(
  double x, double y, double x0, double y0,
  double A, double varx, double vary, double skew)
{
  double dx = x - x0, dy = y - y0;
  double h = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);
  double mx = cos(angle - skew) * h;
  double my = sin(angle - skew) * h;
  double f1 = pow(mx, 2.0) / (2.0 * varx),
    f2 = pow(my, 2.0) / (2.0 * vary);
  return A * exp(-(f1 + f2));
}

double asymmetricGaussian(
  double x, double y, double x0, double y0,
  double A, double angle, double var_h, double var_s, double var_r)
{
  double sigma;
  double dx = x - x0, dy = y - y0;
  double alpha = atan2(dy, dx) - angle + M_PI_2;
  double alpha_n = tf2NormalizeAngle(alpha);
  if (alpha_n <= 0.0) {sigma = var_r;} else {sigma = var_h;}
  double a = (pow(cos(angle), 2) / (2 * pow(sigma, 2))) +
    (pow(sin(angle), 2) / (2 * pow(var_s, 2)));
  double b = ((2 * sin(angle) * cos(angle)) / (4 * pow(sigma, 2))) -
    ((2 * sin(angle) * cos(angle)) / (4 * pow(var_s, 2)));
  double c = (pow(sin(angle), 2) / (2 * pow(sigma, 2))) +
    (pow(cos(angle), 2) / (2 * pow(var_s, 2)));
  double f1 = a * (pow(x, 2) + pow(x0, 2) - 2 * x * x0);
  double f2 = 2 * b * dx * dy;
  double f3 = c * (pow(y, 2) + pow(y0, 2) - 2 * y * y0);
  return A * exp(-(f1 + f2 + f3));
}

}  // namespace social_geometry
