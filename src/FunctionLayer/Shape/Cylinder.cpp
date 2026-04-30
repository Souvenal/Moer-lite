#include "Cylinder.h"
#include "ResourceLayer/Factory.h"
bool Cylinder::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    
    //* 1.光线变换到局部空间
    Ray localRay = transform.inverseRay(ray);

    //* 2.联立方程求解
    float dirX = localRay.direction[0], dirY = localRay.direction[1];
    float oriX = localRay.origin[0], oriY = localRay.origin[1];
    float A = dirX * dirX + dirY * dirY;
    float B = 2 * (oriX * dirX + oriY * dirY);
    float C = oriX * oriX + oriY * oriY - radius * radius;
    float t0 = 0, t1 = 0;
    if (!Quadratic(A, B, C, &t0, &t1)) {
        return false;
    }

    //* 3.检验交点是否在圆柱范围内
    float t;
    Point3f hitPoint;
    float phi;

    Point3f hitPoint0 = localRay.at(t0);
    float phi0 = std::atan2(hitPoint0[1], hitPoint0[0]);
    if (phi0 < 0) phi0 += 2 * PI;

    Point3f hitPoint1 = localRay.at(t1);
    float phi1 = std::atan2(hitPoint1[1], hitPoint1[0]);
    if (phi1 < 0) phi1 += 2 * PI;

    auto isValid = [&](const float t, const Point3f &hitPoint, float phi) {
        return
            t > localRay.tNear && t < localRay.tFar &&
            hitPoint[2] > 0 && hitPoint[2] < height &&
            phi < phiMax;
    };
    if (isValid(t0, hitPoint0, phi0)) {
        t = t0;
        hitPoint = hitPoint0;
        phi =  phi0;
    } else if (isValid(t1, hitPoint1, phi1)) {
        t = t1;
        hitPoint = hitPoint1;
        phi = phi1;
    } else {
        return false;
    }
    
    //* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
    ray.tFar = t;

    //* Write your code here.
    *u = phi / phiMax;
    *v = hitPoint[2] / height;
    return true;
}

void Cylinder::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆柱相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    //* Write your code here.
    /// ----------------------------------------------------

    float phi = u * phiMax;
    float x = radius * std::cos(phi);
    float y = radius * std::sin(phi);
    float z = v * height;
    Point3f hitPoint(x, y, z);
    intersection->position = transform.toWorld(hitPoint);
    
    Vector3f localNormal(std::cos(phi), std::sin(phi), 0.f);
    intersection->normal = normalize(transform.toWorld(localNormal));

    intersection->shape = this;
    intersection->distance = distance;
    intersection->texCoord = Vector2f{u, v};
    Vector3f tangent{1.f, 0.f, .0f};
    Vector3f bitangent;
    if (std::abs(dot(tangent, intersection->normal)) > .9f) {
        tangent = Vector3f(.0f, 1.f, .0f);
    }
    bitangent = normalize(cross(tangent, intersection->normal));
    tangent = normalize(cross(intersection->normal, bitangent));
    intersection->tangent = tangent;
    intersection->bitangent = bitangent;
}

void Cylinder::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cylinder::Cylinder(const Json &json) : Shape(json) {
    radius = fetchOptional(json,"radius",1.f);
    height = fetchOptional(json,"height",1.f);
    phiMax = fetchOptional(json,"phi_max",2 * PI);
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
}

REGISTER_CLASS(Cylinder,"cylinder")
