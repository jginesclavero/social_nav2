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
        min_y == 0 && max_y == 0) {
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
 
  //RCLCPP_INFO(rclcpp::get_logger(""), "min_x [%i], max_x [%i], min_y [%i], max_y [%i]",
  //min_x, max_x, min_y, max_y);

  for (unsigned int i = min_x ; i < max_x; ++i) {
    for (unsigned int j = min_y ; j < max_y; ++j) {
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
  nav2_costmap_2d::Costmap2D* costmap,
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

}  // namespace social_geometry
