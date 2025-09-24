#include "planner_core.hpp"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger & logger)
: logger_(logger) {}

bool PlannerCore::inBounds(int x, int y, int width, int height) const {
  return x >= 0 && y >= 0 && x < width && y < height;
}

double PlannerCore::heuristic_octile(int x1, int y1, int x2, int y2) const {
  int dx = std::abs(x1 - x2);
  int dy = std::abs(y1 - y2);
  return (dx + dy) + (std::sqrt(2.0) - 2) * std::min(dx, dy);
}

void PlannerCore::buildDistanceField(const nav_msgs::msg::OccupancyGrid &map,
                                     std::vector<int> &dist_field,
                                     int inflation_cells) const
{
  int width = map.info.width;
  int height = map.info.height;
  dist_field.assign(width * height, -1);

  std::queue<std::pair<int,int>> q;

  // mark obstacles with distance 0
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int i = idx(x,y,width);
      int8_t v = map.data[i];
      if (v > 50) { // occupied
        dist_field[i] = 0;
        q.push({x,y});
      }
    }
  }

  const int dx4[4] = {1,-1,0,0};
  const int dy4[4] = {0,0,1,-1};

  while (!q.empty()) {
    auto [cx, cy] = q.front(); q.pop();
    int cd = dist_field[idx(cx,cy,width)];
    if (cd >= inflation_cells) continue;

    for (int k=0;k<4;++k) {
      int nx = cx + dx4[k];
      int ny = cy + dy4[k];
      if (!inBounds(nx, ny, width, height)) continue;
      int ni = idx(nx, ny, width);
      if (dist_field[ni] == -1) {
        dist_field[ni] = cd + 1;
        q.push({nx, ny});
      }
    }
  }
}

bool PlannerCore::lineOfSight(const std::vector<int> &dist_field, int x0, int y0, int x1, int y1,
                              int width, int height, const nav_msgs::msg::OccupancyGrid &map) const
{
  int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    int i = idx(x0, y0, width);
    if (dist_field[i] == 0) return false; // lethal obstacle
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 < dx) { err += dx; y0 += sy; }
  }
  return true;
}

nav_msgs::msg::Path PlannerCore::plan(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Point & goal,
  double inflation_radius_m,
  double inflation_weight)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map.header.frame_id;
  path.header.stamp = rclcpp::Clock().now();

  int width = map.info.width;
  int height = map.info.height;
  double res = map.info.resolution;

  // --- Convert to grid indices ---
  auto toGrid = [&](double wx, double wy) {
    int gx = static_cast<int>((wx - map.info.origin.position.x) / res);
    int gy = static_cast<int>((wy - map.info.origin.position.y) / res);
    return std::make_pair(gx, gy);
  };

  auto [sx, sy] = toGrid(start.position.x, start.position.y);
  auto [gx, gy] = toGrid(goal.x, goal.y);

  if (!inBounds(sx, sy, width, height) || !inBounds(gx, gy, width, height)) {
    RCLCPP_WARN(logger_, "Start or goal outside map bounds");
    return path;
  }

  // --- Build distance field ---
  int inflation_cells = static_cast<int>(std::ceil(inflation_radius_m / res));
  std::vector<int> dist_field;
  buildDistanceField(map, dist_field, inflation_cells);

  // --- A* search ---
  struct Node {
    int x, y;
    double g, f;
    bool operator>(const Node &other) const { return f > other.f; }
  };

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  std::unordered_map<int, double> g_score;
  std::unordered_map<int, std::pair<int,int>> came_from;

  int start_i = idx(sx, sy, width);
  int goal_i = idx(gx, gy, width);

  open.push({sx, sy, 0.0, heuristic_octile(sx, sy, gx, gy)});
  g_score[start_i] = 0.0;

  const int dx8[8] = {1,-1,0,0, 1,1,-1,-1};
  const int dy8[8] = {0,0,1,-1, 1,-1,1,-1};
  const double move_cost[8] = {1,1,1,1, std::sqrt(2),std::sqrt(2),std::sqrt(2),std::sqrt(2)};

  bool found = false;

  while (!open.empty()) {
    Node cur = open.top(); open.pop();
    int ci = idx(cur.x, cur.y, width);

    if (ci == goal_i) {
      found = true;
      break;
    }

    for (int k=0;k<8;++k) {
      int nx = cur.x + dx8[k];
      int ny = cur.y + dy8[k];
      if (!inBounds(nx, ny, width, height)) continue;
      int ni = idx(nx, ny, width);

      // skip lethal obstacle
      if (dist_field[ni] == 0) continue;

      // base cost
      double tentative_g = g_score[ci] + move_cost[k];

      // add penalty for being near obstacles
      if (dist_field[ni] > 0) {
        double penalty = (inflation_cells - dist_field[ni] + 1) * inflation_weight;
        tentative_g += penalty;
      }

      if (!g_score.count(ni) || tentative_g < g_score[ni]) {
        g_score[ni] = tentative_g;
        came_from[ni] = {cur.x, cur.y};
        double f = tentative_g + heuristic_octile(nx, ny, gx, gy);
        open.push({nx, ny, tentative_g, f});
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "Failed to find path");
    return path;
  }

  // --- Reconstruct path ---
  std::vector<std::pair<int,int>> rev_cells;
  int cx = gx, cy = gy;
  while (!(cx == sx && cy == sy)) {
    rev_cells.push_back({cx, cy});
    int ci = idx(cx, cy, width);
    auto [px, py] = came_from[ci];
    cx = px; cy = py;
  }
  rev_cells.push_back({sx, sy});
  std::reverse(rev_cells.begin(), rev_cells.end());

  for (auto [x,y] : rev_cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = map.info.origin.position.x + (x + 0.5) * res;
    pose.pose.position.y = map.info.origin.position.y + (y + 0.5) * res;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

} // namespace robot
