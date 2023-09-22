#include "delaunay.hpp"

#include <iostream>
#include <chrono>
#include <random>

using std::cout;

int main() {
  int n = 1000;  // number of points
  std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<> dis(0.0, 1000.0);

  std::vector<delaunay::Point> points(n);
  for (int i = 0; i < n; ++i) {
    points[i] = delaunay::Point(dis(rng), dis(rng));
    // cout << points[i] << '\n';
  }

  const auto triangulation = delaunay::triangulate(points);
  for (auto const& e : triangulation.edges) {
    cout << e.p0.x << " " << e.p0.y << " " << e.p1.x << " " << e.p1.y << '\n';
  }
}