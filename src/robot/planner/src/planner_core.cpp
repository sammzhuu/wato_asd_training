#include "planner_core.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger & logger)
: logger_(logger)
{
}

// Octile heuristic for 8-connected grid
inline double PlannerCore::heuristic_octile(int x1, int y1, int x2, int y2) const
{
  double dx = std::abs(x1 - x2);
  double dy = std::abs(y1 - y2);
  const double F = std::sqrt(2.0) - 1.0;
  return (dx < dy) ? (F * dx + dy) : (F * dy + dx);
}

inline bool PlannerCore::inBounds(int x, int y, int width, int height) const
{
  return x >= 0 && y >= 0 && x < width && y < height;
}

inline bool PlannerCore::isCellFree(const std::vector<int8_t> &inflated, int x, int y, int width, int height) const
{
  if (!inBounds(x, y, width, height)) return false;
  return inflated[idx(x, y, width)] == 0;
}

// Build inflated occupancy grid (0 = free, 1 = occupied/inflated)
void PlannerCore::buildInflatedMap(const nav_msgs::msg::OccupancyGrid &map,
                                   std::vector<int8_t> &inflated,
                                   int inflation_cells) const
{
  int width = map.info.width;
  int height = map.info.height;
  inflated.assign(width * height, 0);

  // mark original obstacles
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int i = idx(x, y, width);
      int8_t v = map.data[i];
      if (v < 0) {
        // unknown treated as free here; could also be treated as occupied
        continue;
      }
      if (v > 50) { // occupied threshold
        inflated[i] = 1;
      }
    }
  }

  if (inflation_cells <= 0) return;

  // BFS expansion from obstacles to inflate them
  std::queue<std::pair<int,int>> q;
  std::vector<int> dist(width * height, -1);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (inflated[idx(x,y,width)] == 1) {
        q.push({x,y});
        dist[idx(x,y,width)] = 0;
      }
    }
  }

  const int dx4[4] = {1,-1,0,0};
  const int dy4[4] = {0,0,1,-1};

  while (!q.empty()) {
    auto p = q.front(); q.pop();
    int cx = p.first, cy = p.second;
    int cd = dist[idx(cx,cy,width)];
    if (cd >= inflation_cells) continue;

    for (int k=0;k<4;++k) {
      int nx = cx + dx4[k];
      int ny = cy + dy4[k];
      if (!inBounds(nx, ny, width, height)) continue;
      int ni = idx(nx, ny, width);
      if (dist[ni] == -1) {
        dist[ni] = cd + 1;
        inflated[ni] = 1; // mark inflated
        q.push({nx, ny});
      }
    }
  }
}

// line-of-sight check (Bresenham sampling) against inflated occupancy
bool PlannerCore::lineOfSight(const std::vector<int8_t> &inflated, int x0, int y0, int x1, int y1,
                              int width, int height, const nav_msgs::msg::OccupancyGrid & /*map*/) const
{
  // Bresenham integer line
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;
  while (true) {
    if (!inBounds(x, y, width, height)) return false;
    if (inflated[idx(x,y,width)] != 0) return false; // blocked
    if (x == x1 && y == y1) break;
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 < dx)  { err += dx;  y += sy; }
  }
  return true;
}

nav_msgs::msg::Path PlannerCore::plan(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Point & goal,
  double inflation_radius_m)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map.header.frame_id;
  path.header.stamp = rclcpp::Clock().now();

  if (map.data.empty()) {
    RCLCPP_WARN(logger_, "PlannerCore::plan called with empty map");
    return path;
  }

  int width = static_cast<int>(map.info.width);
  int height = static_cast<int>(map.info.height);
  double resolution = map.info.resolution;

  // Convert world to map coordinates
  auto worldToMap = [&](double wx, double wy, int &mx, int &my) {
    mx = static_cast<int>(std::floor((wx - map.info.origin.position.x) / resolution));
    my = static_cast<int>(std::floor((wy - map.info.origin.position.y) / resolution));
  };

  int sx, sy, gx, gy;
  worldToMap(start.position.x, start.position.y, sx, sy);
  worldToMap(goal.x, goal.y, gx, gy);

  if (!inBounds(sx, sy, width, height)) {
    RCLCPP_WARN(logger_, "Start is out of map bounds: (%d,%d)", sx, sy);
    return path;
  }
  if (!inBounds(gx, gy, width, height)) {
    RCLCPP_WARN(logger_, "Goal is out of map bounds: (%d,%d)", gx, gy);
    return path;
  }

  // Prepare inflated occupancy grid
  int inflation_cells = static_cast<int>(std::ceil(inflation_radius_m / resolution));
  std::vector<int8_t> inflated;
  buildInflatedMap(map, inflated, inflation_cells);

  // Reject if start/goal are inside inflated obstacles
  if (!isCellFree(inflated, sx, sy, width, height)) {
    RCLCPP_WARN(logger_, "Start lies in an inflated obstacle cell");
    return path;
  }
  if (!isCellFree(inflated, gx, gy, width, height)) {
    RCLCPP_WARN(logger_, "Goal lies in an inflated obstacle cell");
    return path;
  }

  // A* structures
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> g_score(width * height, INF);
  std::vector<int> parent(width * height, -1);
  std::vector<char> closed(width * height, 0);

  // Priority queue: (f, x, y)
  struct PQ {
    double f;
    int x, y;
    bool operator>(const PQ &o) const { return f > o.f; }
  };
  std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> open;

  auto push_open = [&](int x, int y, double g) {
    double h = heuristic_octile(x, y, gx, gy);
    double f = g + h;
    open.push({f, x, y});
    g_score[idx(x,y,width)] = g;
  };

  // init
  push_open(sx, sy, 0.0);
  parent[idx(sx,sy,width)] = -1;

  const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  const double move_cost[8] = {1.0, std::sqrt(2.0), 1.0, std::sqrt(2.0),
                               1.0, std::sqrt(2.0), 1.0, std::sqrt(2.0)};
  bool found = false;

  while (!open.empty()) {
    PQ cur = open.top();
    open.pop();

    int cx = cur.x;
    int cy = cur.y;
    int cindex = idx(cx, cy, width);

    if (closed[cindex]) continue;
    closed[cindex] = 1;

    if (cx == gx && cy == gy) {
      found = true;
      break;
    }

    double cg = g_score[cindex];

    for (int k = 0; k < 8; ++k) {
      int nx = cx + dx8[k];
      int ny = cy + dy8[k];
      if (!inBounds(nx, ny, width, height)) continue;
      if (!isCellFree(inflated, nx, ny, width, height)) continue;

      double tentative_g = cg + move_cost[k];

      int nidx = idx(nx, ny, width);
      if (tentative_g < g_score[nidx]) {
        g_score[nidx] = tentative_g;
        parent[nidx] = cindex;
        double h = heuristic_octile(nx, ny, gx, gy);
        double f = tentative_g + h;
        open.push({f, nx, ny});
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "PlannerCore::plan - no path found");
    return path;
  }

  // reconstruct raw grid path
  std::vector<int> grid_path;
  int cur = idx(gx, gy, width);
  while (cur != -1) {
    grid_path.push_back(cur);
    cur = parent[cur];
  }
  std::reverse(grid_path.begin(), grid_path.end());

  // convert grid indices to world poses
  std::vector<std::pair<int,int>> cells;
  cells.reserve(grid_path.size());
  for (int index_val : grid_path) {
    int x = index_val % width;
    int y = index_val / width;
    cells.emplace_back(x, y);
  }

  // smoothing: try to shortcut using line-of-sight
  std::vector<std::pair<int,int>> smooth;
  size_t i = 0;
  while (i < cells.size()) {
    size_t j = cells.size() - 1;
    // greedily find farthest j >= i such that LOS(i,j) is clear
    for (; j > i; --j) {
      if (lineOfSight(inflated, cells[i].first, cells[i].second,
                      cells[j].first, cells[j].second, width, height, map)) {
        break;
      }
    }
    smooth.push_back(cells[i]);
    i = j;
    if (i == cells.size() - 1) {
      smooth.push_back(cells.back());
      break;
    }
  }

  // convert to PoseStamped and push into path (world coordinates)
  for (const auto &c : smooth) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = map.header.frame_id;
    ps.header.stamp = rclcpp::Clock().now();
    ps.pose.position.x = (c.first + 0.5) * resolution + map.info.origin.position.x;
    ps.pose.position.y = (c.second + 0.5) * resolution + map.info.origin.position.y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }

  return path;
}

} // namespace robot
