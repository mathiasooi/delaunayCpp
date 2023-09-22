#pragma once

#include <algorithm>
#include <iostream>
#include <vector>

namespace delaunay {

constexpr double eps = 1e-4;

struct Point {
  double x, y;

  Point() : x{0}, y{0} {}
  Point(double _x, double _y) : x{_x}, y{_y} {}

  friend std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << "x=" << p.x << "  y=" << p.y;
    return os;
  }

  bool operator==(const Point& other) const { return (other.x == x && other.y == y); }

  bool operator!=(const Point& other) const { return !operator==(other); }
};

struct Edge {
  Point p0, p1;

  Edge(Point const& _p0, Point const& _p1) : p0{_p0}, p1{_p1} {}

  friend std::ostream& operator<<(std::ostream& os, const Edge& e) {
    os << "p0: [" << e.p0 << " ] p1: [" << e.p1 << "]";
    return os;
  }

  bool operator==(const Edge& other) const {
    return ((other.p0 == p0 && other.p1 == p1) ||
            (other.p0 == p1 && other.p1 == p0));
  }
};

struct Circle {
  double x, y, radius;
  Circle() = default;
};

struct Triangle {
  Point p0, p1, p2;
  Edge e0, e1, e2;
  Circle circle;

  Triangle(const Point& _p0, const Point& _p1, const Point& _p2)
      : p0{_p0},
        p1{_p1},
        p2{_p2},
        e0{_p0, _p1},
        e1{_p1, _p2},
        e2{_p0, _p2},
        circle{}
  {
    const double ax = p1.x - p0.x;
    const double ay = p1.y - p0.y;
    const double bx = p2.x - p0.x;
    const double by = p2.y - p0.y;

    const double m = p1.x * p1.x - p0.x * p0.x + p1.y * p1.y - p0.y * p0.y;
    const double u = p2.x * p2.x - p0.x * p0.x + p2.y * p2.y - p0.y * p0.y;
    const double s = 1. / (2. * (ax * by - ay * bx));

    circle.x = ((p2.y - p0.y) * m + (p0.y - p1.y) * u) * s;
    circle.y = ((p0.x - p2.x) * m + (p1.x - p0.x) * u) * s;

    const double dx = p0.x - circle.x;
    const double dy = p0.y - circle.y;
    circle.radius = dx * dx + dy * dy;
  }
};

struct Delaunay {
  std::vector<Triangle> triangles;
  std::vector<Edge> edges;
};

Delaunay triangulate(const std::vector<Point>& points)
{
  using Node = Point;
  if (points.size() < 3) {
    return Delaunay{};
  }
  double xmin = points[0].x;
  double xmax = xmin;
  double ymin = points[0].y;
  double ymax = ymin;
  for (Point const& pt : points) {
    xmin = std::min(xmin, pt.x);
    xmax = std::max(xmax, pt.x);
    ymin = std::min(ymin, pt.y);
    ymax = std::max(ymax, pt.y);
  }

  const double dx = xmax - xmin;
  const double dy = ymax - ymin;
  const double dmax = std::max(dx, dy);
  const double midx = (xmin + xmax) / 2.0;
  const double midy = (ymin + ymax) / 2.0;

  /* Init Delaunay triangulation. */
  Delaunay d = Delaunay{};

  const Point p0 = Node{midx - 20 * dmax, midy - dmax};
  const Point p1 = Node{midx, midy + 20 * dmax};
  const Point p2 = Node{midx + 20 * dmax, midy - dmax};
  d.triangles.emplace_back(Triangle{p0, p1, p2});

  for (auto const& pt : points) {
    std::vector<Edge> edges;
    std::vector<Triangle> tmps;
    for (auto const& tri : d.triangles) {
      /* Check if the point is inside the triangle circumcircle. */
      const double dist = (tri.circle.x - pt.x) * (tri.circle.x - pt.x) +
                        (tri.circle.y - pt.y) * (tri.circle.y - pt.y);
      if ((dist - tri.circle.radius) <= eps) {
        edges.push_back(tri.e0);
        edges.push_back(tri.e1);
        edges.push_back(tri.e2);
      }
      else {
        tmps.push_back(tri);
      }
    }

    /* Delete duplicate edges. */
    std::vector<bool> remove(edges.size(), false);
    for (auto it1 = edges.begin(); it1 != edges.end(); ++it1) {
      for (auto it2 = edges.begin(); it2 != edges.end(); ++it2) {
        if (it1 == it2) {
          continue;
        }
        if (*it1 == *it2) {
          remove[std::distance(edges.begin(), it1)] = true;
          remove[std::distance(edges.begin(), it2)] = true;
        }
      }
    }

    edges.erase(
        std::remove_if(edges.begin(), edges.end(),
                       [&](auto const& e) { return remove[&e - &edges[0]]; }),
        edges.end());

    /* Update triangulation. */
    for (Edge const& e : edges) {
      tmps.push_back({e.p0, e.p1, {pt.x, pt.y}});
    }
    d.triangles = tmps;
  }

  /* Remove original super triangle. */
  d.triangles.erase(
      std::remove_if(d.triangles.begin(), d.triangles.end(),
                     [&](auto const& tri) {
                       return ((tri.p0 == p0 || tri.p1 == p0 || tri.p2 == p0) ||
                               (tri.p0 == p1 || tri.p1 == p1 || tri.p2 == p1) ||
                               (tri.p0 == p2 || tri.p1 == p2 || tri.p2 == p2));
                     }),
      d.triangles.end());

  /* Add edges. */
  for (Triangle const& tri : d.triangles) {
    d.edges.push_back(tri.e0);
    d.edges.push_back(tri.e1);
    d.edges.push_back(tri.e2);
  }
  return d;
}

} /* namespace delaunay */