#include "AABB.h"
#include "CoreLayer/Math/Geometry.h"

Point3f minP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::min(p1[0], p2[0]), std::min(p1[1], p2[1]),
                 std::min(p1[2], p2[2])};
}

Point3f maxP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::max(p1[0], p2[0]), std::max(p1[1], p2[1]),
                 std::max(p1[2], p2[2])};
}

AABB AABB::Union(const AABB &other) const {
  Point3f min = minP(other.pMin, pMin), max = maxP(other.pMax, pMax);
  return AABB{min, max};
}

void AABB::Expand(const AABB &other) {
  pMin = minP(pMin, other.pMin);
  pMax = maxP(pMax, other.pMax);
}

AABB AABB::Union(const Point3f &other) const {
  Point3f min = minP(other, pMin), max = maxP(other, pMax);
  return AABB{min, max};
}

void AABB::Expand(const Point3f &other) {
  pMin = minP(pMin, other);
  pMax = maxP(pMax, other);
}

bool AABB::Overlap(const AABB &other) const {
  for (int dim = 0; dim < 3; ++dim) {
    if (pMin[dim] > other.pMax[dim] || pMax[dim] < other.pMin[dim]) {
      return false;
    }
  }
  return true;
}

bool AABB::RayIntersect(const Ray &ray, float *tMin, float *tMax) const {
  //* todo 实现AABB与光线求交
  for (int dim = 0; dim < 3; ++dim) {
    if (ray.direction[dim] == 0.0f) {
      if (ray.origin[dim] < pMin[dim] || ray.origin[dim] > pMax[dim]) {
        return false;
      }
      continue;
    }

    float invDir = 1.f / ray.direction[dim];
    float tNear = (pMin[dim] - ray.origin[dim]) * invDir;
    float tFar = (pMax[dim] - ray.origin[dim]) * invDir;
    if (invDir < 0.f) {
      std::swap(tNear, tFar);
    }

    *tMin = std::max(*tMin, tNear);
    *tMax = std::min(*tMax, tFar);

    if (*tMin > *tMax) return false;
  }

  return true;
}

Point3f AABB::Center() const {
  return Point3f{(pMin[0] + pMax[0]) * .5f, (pMin[1] + pMax[1]) * .5f,
                 (pMin[2] + pMax[2]) * .5f};
}