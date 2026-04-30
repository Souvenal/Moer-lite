#include "ThinLens.h"
#include "CoreLayer/Math/Geometry.h"
#include "CoreLayer/Math/Transform.h"
#include "FastMath/FastMath.h"

ThinLensCamera::ThinLensCamera(const Json& json) : PerspectiveCamera(json) {
    lensRadius    = json["lensRadius"].get<float>();
    focalDistance = json["focalDistance"].get<float>();
}

Ray ThinLensCamera::sampleRay(const CameraSample& sample, Vector2f NDC) const {
    // TODO
    float x = (NDC[0] - 0.5f) * film->size[0] + sample.xy[0],
        y = (0.5f - NDC[1]) * film->size[1] + sample.xy[1];

    float tanHalfFov = fm::tan(verticalFov * 0.5f);
    float z = -film->size[1] * 0.5f / tanHalfFov;
    
    float r = fm::sqrt(sample.lens[0]);
    float theta = 2 * PI * sample.lens[1];
    Point3f origin = transform.toWorld(Point3f(r * fm::cos(theta), r * fm::sin(theta), 0.f));
    
    float f = focalDistance / -z;
    Point3f pFoucus = transform.toWorld(Point3f(x * f, y * f, z * f));
    Vector3f direction = pFoucus - origin;
    direction = normalize(direction);
    
    return Ray(origin, direction, tNear, tFar, timeStart);
}

Ray ThinLensCamera::sampleRayDifferentials(const CameraSample& sample, Vector2f NDC) const {
    float x = (NDC[0] - 0.5f) * film->size[0] + sample.xy[0],
        y = (0.5f - NDC[1]) * film->size[1] + sample.xy[1];

    float tanHalfFov = fm::tan(verticalFov * 0.5f);
    float z = -film->size[1] * 0.5f / tanHalfFov;

    float r = fm::sqrt(sample.lens[0]);
    float theta = 2 * PI * sample.lens[1];
    Point3f origin = transform.toWorld(Point3f(r * fm::cos(theta), r * fm::sin(theta), 0.f));

    float f = focalDistance / -z;

    Vector3f direction = normalize(
        transform.toWorld(Point3f(x * f, y * f, z * f)) - origin);
    Vector3f directionX = normalize(
        transform.toWorld(Point3f((x + 1.f) * f, y * f, z * f)) - origin);
    Vector3f directionY = normalize(
        transform.toWorld(Point3f(x * f, (y + 1.f) * f, z * f)) - origin);

    Ray ret{origin, direction, tNear, tFar, timeStart};
    ret.hasDifferentials = true;
    ret.directionX = directionX;
    ret.directionY = directionY;
    ret.originX = ret.originY = origin;
    return ret;
}

REGISTER_CLASS(ThinLensCamera, "thinlens")