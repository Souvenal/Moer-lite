#include "Triangle.h"
#include "CoreLayer/Math/Constant.h"
#include "CoreLayer/Math/Geometry.h"
#include <FunctionLayer/Acceleration/Linear.h>
//--- Triangle ---
Triangle::Triangle(int _primID, int _vtx0Idx, int _vtx1Idx, int _vtx2Idx,
                   const TriangleMesh *_mesh)
    : primID(_primID), vtx0Idx(_vtx0Idx), vtx1Idx(_vtx1Idx), vtx2Idx(_vtx2Idx),
      mesh(_mesh) {
  Point3f vtx0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]),
          vtx1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]),
          vtx2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);
  boundingBox.Expand(vtx0);
  boundingBox.Expand(vtx1);
  boundingBox.Expand(vtx2);
  this->geometryID = mesh->geometryID;
}

bool Triangle::rayIntersectShape(Ray &ray, int *primID, float *u,
                                 float *v) const {
  //* todo 实现三角形与光线求交
  Point3f O = ray.origin;
  Vector3f D = ray.direction;
  auto& vertexBuffer = mesh->meshData->vertexBuffer;
  Point3f v0 = mesh->transform.toWorld(vertexBuffer[vtx0Idx]);
  Point3f v1 = mesh->transform.toWorld(vertexBuffer[vtx1Idx]);
  Point3f v2 = mesh->transform.toWorld(vertexBuffer[vtx2Idx]);

  Vector3f e0 = v1 - v0, e1 = v2 - v0;
  Vector3f N = cross(e0, e1);
  float dot_N_D = dot(N, D);
  if (std::abs(dot_N_D) < EPSILON) {
    // Ray is parallel to the plane 
    return false;
  }
  
  float t = dot(N, v0 - O) / dot_N_D;
  if (t < ray.tNear || t > ray.tFar) {
    return false;
  }
  
  Point3f P = ray.at(t);
  if (dot(cross(v1 - v0, P - v0), N) < 0.f ||
      dot(cross(v2 - v1, P - v1), N) < 0.f ||
      dot(cross(v0 - v2, P - v2), N) < 0.f) {
    return false;
  }

  float dot_e0_e1 = dot(e0, e1);
  float dot_e0_e0 = dot(e0, e0);
  float dot_e1_e1 = dot(e1, e1);
  float dot_e0_v0P = dot(e0, P - v0);
  float dot_e1_v0P = dot(e1, P - v0);
  float delta = dot_e0_e0 * dot_e1_e1 - dot_e0_e1 * dot_e0_e1;
  *u = (dot_e0_v0P * dot_e1_e1 - dot_e0_e1 * dot_e1_v0P) / delta;
  *v = (dot_e0_e0 * dot_e1_v0P - dot_e0_e1 * dot_e0_v0P) / delta;
  assert(0 <= *u && *u <= 1.f && 0 <= *v && *v <= 1.f && *u + *v <= 1.f);

  ray.tFar = t;
  *primID = this->primID;
  return true;
}

void Triangle::fillIntersection(float distance, int primID, float u, float v,
                                Intersection *intersection) const {
  // 该函数实际上不会被调用
  return;
}

//--- TriangleMesh ---
TriangleMesh::TriangleMesh(const Json &json) : Shape(json) {
  const auto &filepath = fetchRequired<std::string>(json, "file");
  meshData = MeshData::loadFromFile(filepath);
}

RTCGeometry TriangleMesh::getEmbreeGeometry(RTCDevice device) const {
  RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

  float *vertexBuffer = (float *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float),
      meshData->vertexCount);
  for (int i = 0; i < meshData->vertexCount; ++i) {
    Point3f vertex = transform.toWorld(meshData->vertexBuffer[i]);
    vertexBuffer[3 * i] = vertex[0];
    vertexBuffer[3 * i + 1] = vertex[1];
    vertexBuffer[3 * i + 2] = vertex[2];
  }

  unsigned *indexBuffer = (unsigned *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
      3 * sizeof(unsigned), meshData->faceCount);
  for (int i = 0; i < meshData->faceCount; ++i) {
    indexBuffer[i * 3] = meshData->faceBuffer[i][0].vertexIndex;
    indexBuffer[i * 3 + 1] = meshData->faceBuffer[i][1].vertexIndex;
    indexBuffer[i * 3 + 2] = meshData->faceBuffer[i][2].vertexIndex;
  }
  rtcCommitGeometry(geometry);
  return geometry;
}

bool TriangleMesh::rayIntersectShape(Ray &ray, int *primID, float *u,
                                     float *v) const {
  //* 当使用embree加速时，该方法不会被调用
  int geomID = -1;
  return acceleration->rayIntersect(ray, &geomID, primID, u, v);
}

void TriangleMesh::fillIntersection(float distance, int primID, float u,
                                    float v, Intersection *intersection) const {
  //* todo 填充光线与三角网格求交得到的交点信息
  intersection->distance = distance;
  intersection->shape = this;

  //* 1. 在三角形内部用插值计算交点坐标
  int v0Idx = meshData->faceBuffer[primID][0].vertexIndex,
      v1Idx = meshData->faceBuffer[primID][1].vertexIndex,
      v2Idx = meshData->faceBuffer[primID][2].vertexIndex;
  auto& vertexBuffer = meshData->vertexBuffer;
  Point3f v0 = transform.toWorld(vertexBuffer[v0Idx]);
  Point3f v1 = transform.toWorld(vertexBuffer[v1Idx]);
  Point3f v2 = transform.toWorld(vertexBuffer[v2Idx]);
  intersection->position = Point3f(
      v0[0] * (1 - u - v) + v1[0] * u + v2[0] * v,
      v0[1] * (1 - u - v) + v1[1] * u + v2[1] * v,
      v0[2] * (1 - u - v) + v1[2] * u + v2[2] * v);

  //* 2. 在三角形内部用插值计算法线
  auto& normalBuffer = meshData->normalBuffer;
  int n0Idx = meshData->faceBuffer[primID][0].normalIndex,
      n1Idx = meshData->faceBuffer[primID][1].normalIndex,
      n2Idx = meshData->faceBuffer[primID][2].normalIndex;
  Vector3f n0 = transform.toWorld(normalBuffer[n0Idx]);
  Vector3f n1 = transform.toWorld(normalBuffer[n1Idx]);
  Vector3f n2 = transform.toWorld(normalBuffer[n2Idx]);
  Vector3f normal = n0 * (1 - u - v) + n1 * u + n2 * v;
  intersection->normal = normalize(normal);

  //* 3. 在三角形内部用插值计算纹理坐标
  auto& texcodBuffer = meshData->texcodBuffer;
  int t0Idx = meshData->faceBuffer[primID][0].texcodIndex,
      t1Idx = meshData->faceBuffer[primID][1].texcodIndex,
      t2Idx = meshData->faceBuffer[primID][2].texcodIndex;
  Vector2f tex0 = texcodBuffer[t0Idx];
  Vector2f tex1 = texcodBuffer[t1Idx];
  Vector2f tex2 = texcodBuffer[t2Idx];
  Vector2f tex = tex0 * (1 - u - v) + tex1 * u + tex2 * v;
  intersection->texCoord = tex;
  
  //* 4. 在三角形内部用插值计算交点的切线和副切线
  Vector3f e0 = v1 - v0, e1 = v2 - v0;
  Vector2f deltaUV1 = tex1 - tex0;
  Vector2f deltaUV2 = tex2 - tex0;
  float f = 1.0f / (deltaUV1[0] * deltaUV2[1] - deltaUV2[0] * deltaUV1[1]);
  Vector3f tangent = f * (deltaUV2[1] * e0 - deltaUV1[1] * e1);
  Vector3f bitangent = f * (-deltaUV2[0] * e0 + deltaUV1[0] * e1);
  intersection->tangent = normalize(tangent);
  intersection->bitangent = normalize(bitangent);
}

void TriangleMesh::initInternalAcceleration() {
  acceleration = Acceleration::createAcceleration();
  int primCount = meshData->faceCount;
  for (int primID = 0; primID < primCount; ++primID) {
    int vtx0Idx = meshData->faceBuffer[primID][0].vertexIndex,
        vtx1Idx = meshData->faceBuffer[primID][1].vertexIndex,
        vtx2Idx = meshData->faceBuffer[primID][2].vertexIndex;
    std::shared_ptr<Triangle> triangle =
        std::make_shared<Triangle>(primID, vtx0Idx, vtx1Idx, vtx2Idx, this);
    acceleration->attachShape(triangle);
  }
  acceleration->build();
  // TriangleMesh的包围盒就是其内部加速结构的包围盒
  boundingBox = acceleration->boundingBox;
}
REGISTER_CLASS(TriangleMesh, "triangle")