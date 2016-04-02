#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <random>

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/component_wise.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <SDL2/SDL.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

using glm::vec3;
using glm::mat4;

static float TAU = 2.0f * glm::pi<float>();

#define array_count(arr) (sizeof(arr) / sizeof(arr[0]))

#include "queue.h"
#include <base/array.h>

struct RandomSequence {
  int seed;
};

int get_next_random(RandomSequence *random) {
  int seed = (random->seed * 1103515245U + 12345U) & 0x7fffffffU;
  random->seed = seed;
  return seed;
}

float random_float(RandomSequence *random) {
  int value = get_next_random(random);

  return (float)value / 0x7fffffffU;
}

struct AABB {
  vec3 min;
  vec3 max;
};

enum MaterialType {
  DIFF,
  REFR,
  REFL
};

struct Mesh {
  void *data = NULL;

  float *vertices = NULL;
  float *normals = NULL;
  int *indices = NULL;

  u32 vertices_count = 0;
  u32 normals_count = 0;
  u32 indices_count = 0;

  AABB bounds;
};

struct Material {
  vec3 color;
  vec3 emission;
  MaterialType type;
};

struct Model {
  Mesh mesh;
  uint32_t material_index;
};

struct Triangle {
  vec3 positions[3];
  vec3 normal;
  uint32_t material_index;
  AABB bounds;
};

void allocate_mesh(Mesh *mesh, u32 vertices_count, u32 normals_count, u32 indices_count) {
  u32 vertices_size = vertices_count * sizeof(float);
  u32 normals_size = normals_count * sizeof(float);
  u32 indices_size = indices_count * sizeof(int);

  u8 *data = static_cast<u8*>(malloc(vertices_size + normals_size + indices_size));

  float *vertices = (float*)data;
  float *normals = (float*)(data + vertices_size);
  int *indices = (int*)(data + vertices_size + normals_size);

  mesh->data = data;

  mesh->vertices = vertices;
  mesh->vertices_count = vertices_count;

  mesh->normals = normals;
  mesh->normals_count = normals_count;

  mesh->indices = indices;
  mesh->indices_count = indices_count;
}

struct BVHNode {
  bool is_leaf;
  AABB bounds;

  u32 count;
  Triangle *triangles;

  struct BVHNode *left;
  struct BVHNode *right;
};

struct World {
  Array<Model> models;
  Array<Triangle> triangles;
  Array<Material> materials;

  BVHNode *bvh;
};

void merge_aabb(AABB *a, AABB &b) {
  if (b.min.x < a->min.x) { a->min.x = b.min.x; }
  if (b.min.y < a->min.y) { a->min.y = b.min.y; }
  if (b.min.z < a->min.z) { a->min.z = b.min.z; }

  if (b.max.x > a->max.x) { a->max.x = b.max.x; }
  if (b.max.y > a->max.y) { a->max.y = b.max.y; }
  if (b.max.z > a->max.z) { a->max.z = b.max.z; }
}

AABB compute_bounding_volume(Triangle *triangles, u32 count) {
  AABB result;

  for (u32 i=0; i<count; i++) {
    Triangle *it = triangles + i;
    if (i == 0) {
      result = it->bounds;
    } else {
      merge_aabb(&result, it->bounds);
    }
  }

  return result;
}

vec3 aabb_center(AABB bounds) {
  return bounds.min + (bounds.max - bounds.min) / 2.0f;
}

float surface_area(AABB bounds) {
  vec3 extent = bounds.max - bounds.min;
  return 2.0f * (extent.x * extent.z + extent.x * extent.y + extent.y * extent.z);
}

u32 partition_objects(AABB bounds, Triangle *triangles, u32 count) {
  vec3 center;
  for (u32 i=0; i<count; i++) {
    vec3 item_center = aabb_center(triangles[i].bounds);
    if (i == 0) {
      center = item_center;
    } else {
      center = (center + item_center) / 2.0f;
    }
  }

  float best_cost = FLT_MAX;
  u32 best_axis;

  for (u32 axis=0; axis<3; axis++) {
    std::sort(triangles, triangles + count, [axis](Triangle &a, Triangle &b) {
      return aabb_center(a.bounds)[axis] < aabb_center(b.bounds)[axis];
    });

    u32 k = 0;
    for (u32 i=0; i<count; i++) {
      Triangle *item = triangles + i;

      if (aabb_center(item->bounds)[axis] < center[axis]) {
        k += 1;
      } else {
        break;
      }
    }

    float left_cost = surface_area(compute_bounding_volume(triangles, k));
    float right_cost = surface_area(compute_bounding_volume(triangles, count - k));
    float cost = left_cost + right_cost;

    if (cost < best_cost) {
      best_cost = cost;
      best_axis = axis;
    }
  }

  std::sort(triangles, triangles + count, [best_axis](Triangle &a, Triangle &b) {
    return aabb_center(a.bounds)[best_axis] < aabb_center(b.bounds)[best_axis];
  });

  u32 k = 0;
  for (u32 i=0; i<count; i++) {
    Triangle *item = triangles + i;

    if (aabb_center(item->bounds)[best_axis] < center[best_axis]) {
      k += 1;
    } else {
      break;
    }
  }

  return k;
}

void top_down_bvtree(BVHNode **tree, Triangle *triangles, u32 count) {
  BVHNode *node = new BVHNode;

  *tree = node;

  node->bounds = compute_bounding_volume(&triangles[0], count);

  if (count <= 1) {
    node->is_leaf = true;
    node->count = count;
    node->triangles = &triangles[0];
  } else {
    u32 k = partition_objects(node->bounds, &triangles[0], count);

    if (k == count || k == 0) {
      node->is_leaf = true;
      node->count = count;
      node->triangles = &triangles[0];
    } else {
      node->is_leaf = false;
      top_down_bvtree(&node->left, &triangles[0], k);
      top_down_bvtree(&node->right, &triangles[k], count - k);
    }
  }
}

void load_model_work(World *world, const char *path) {
  Assimp::Importer importer;

  const aiScene* scene = importer.ReadFile(path, aiProcess_GenNormals |
      aiProcess_Triangulate |
      aiProcess_JoinIdenticalVertices);

  for (u32 i=0; i<scene->mNumMaterials; i++) {
    aiMaterial *material = scene->mMaterials[i];

    Material model_material;
    model_material.type = DIFF;
    aiColor4D diffuse;
    if (aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, &diffuse) == AI_SUCCESS) {
      model_material.color = vec3(diffuse.r, diffuse.g, diffuse.b);
    }

    aiColor4D emissive;
    if (aiGetMaterialColor(material, AI_MATKEY_COLOR_EMISSIVE, &emissive) == AI_SUCCESS) {
      model_material.emission = vec3(emissive.r, emissive.g, emissive.b);
    }

    array::push_back(world->materials, model_material);
  }

  if (scene->HasMeshes()) {
    for (u32 i=0; i<scene->mNumMeshes; i++) {
      aiMesh *mesh_data = scene->mMeshes[i];

      u32 index_count = 0;

      for (u32 l=0; l<mesh_data->mNumFaces; l++) {
        aiFace face = mesh_data->mFaces[l];

        index_count += face.mNumIndices;
      }

      Model model;
      model.material_index = mesh_data->mMaterialIndex;
      model.mesh.bounds.min = vec3(FLT_MAX);
      model.mesh.bounds.max = vec3(FLT_MIN);

      u32 vertices_count = mesh_data->mNumVertices * 3;
      u32 normals_count = vertices_count;
      u32 indices_count = index_count;

      u32 vertices_index = 0;
      u32 normals_index = 0;
      u32 indices_index = 0;

      allocate_mesh(&model.mesh, vertices_count, normals_count, indices_count);

      for (u32 l=0; l<mesh_data->mNumVertices; l++) {
        model.mesh.vertices[vertices_index++] = mesh_data->mVertices[l].x;
        model.mesh.vertices[vertices_index++] = mesh_data->mVertices[l].y;
        model.mesh.vertices[vertices_index++] = mesh_data->mVertices[l].z;

#define FIND_MIN(a, b) if ((a) < (b)) { (b) = (a); }
#define FIND_MAX(a, b) if ((a) > (b)) { (b) = (a); }

        FIND_MIN(mesh_data->mVertices[l].x, model.mesh.bounds.min.x);
        FIND_MIN(mesh_data->mVertices[l].y, model.mesh.bounds.min.y);
        FIND_MIN(mesh_data->mVertices[l].z, model.mesh.bounds.min.z);
        FIND_MAX(mesh_data->mVertices[l].x, model.mesh.bounds.max.x);
        FIND_MAX(mesh_data->mVertices[l].y, model.mesh.bounds.max.y);
        FIND_MAX(mesh_data->mVertices[l].z, model.mesh.bounds.max.z);

        model.mesh.normals[normals_index++] = mesh_data->mNormals[l].x;
        model.mesh.normals[normals_index++] = mesh_data->mNormals[l].y;
        model.mesh.normals[normals_index++] = mesh_data->mNormals[l].z;
      }

      for (u32 l=0; l<mesh_data->mNumFaces; l++) {
        aiFace face = mesh_data->mFaces[l];

        for (u32 j=0; j<face.mNumIndices; j++) {
          model.mesh.indices[indices_index++] = face.mIndices[j];
        }
      }

      array::push_back(world->models, model);

      for (u32 l=0; l<model.mesh.indices_count; l += 3) {
        int indices_a = model.mesh.indices[l + 0] * 3;
        int indices_b = model.mesh.indices[l + 1] * 3;
        int indices_c = model.mesh.indices[l + 2] * 3;

        vec3 a = *(vec3 *)(model.mesh.vertices + indices_a);
        vec3 b = *(vec3 *)(model.mesh.vertices + indices_b);
        vec3 c = *(vec3 *)(model.mesh.vertices + indices_c);

        vec3 normal_a = *(vec3 *)(model.mesh.normals + indices_a);
        vec3 normal_b = *(vec3 *)(model.mesh.normals + indices_b);
        vec3 normal_c = *(vec3 *)(model.mesh.normals + indices_c);

        Triangle triangle;
        triangle.positions[0] = a;
        triangle.positions[1] = b;
        triangle.positions[2] = c;

        triangle.normal = glm::normalize((normal_a + normal_b + normal_c) / 3.0f);

        triangle.material_index = model.material_index;
        model.mesh.bounds.min = vec3(FLT_MAX);
        model.mesh.bounds.max = vec3(FLT_MIN);

        for (u32 k=0; k<3; k++) {
          FIND_MIN(triangle.positions[k].x, triangle.bounds.min.x);
          FIND_MIN(triangle.positions[k].y, triangle.bounds.min.y);
          FIND_MIN(triangle.positions[k].z, triangle.bounds.min.z);
          FIND_MAX(triangle.positions[k].x, triangle.bounds.max.x);
          FIND_MAX(triangle.positions[k].y, triangle.bounds.max.y);
          FIND_MAX(triangle.positions[k].z, triangle.bounds.max.z);
        }

        array::push_back(world->triangles, triangle);
      }
    }
  }
}

struct Ray {
  vec3 origin;
  vec3 direction;
  vec3 inv;
};

struct HitResult {
  float distance;
  vec3 normal;
  uint32_t material_index;
};

bool aabb_intersection(AABB b, Ray r) {
  vec3 min = (b.min - r.origin) * r.inv;
  vec3 max = (b.max - r.origin) * r.inv;

  float tmin = glm::max(glm::max(glm::min(min.x, max.x), glm::min(min.y, max.y)), glm::min(min.z, max.z));
  float tmax = glm::min(glm::min(glm::max(min.x, max.x), glm::max(min.y, max.y)), glm::max(min.z, max.z));

  if (tmax < 0.0) {
    return false;
  }

  if (tmin > tmax) {
    return false;
  }

  return true;
}

bool intersect_triangle(vec3 v0, vec3 v1, vec3 v2, const Ray &r, float *result_distance) {
  static float Epsilon = std::numeric_limits<float>::epsilon();

  vec3 e1 = v1 - v0;
  vec3 e2 = v2 - v0;

  vec3 p = glm::cross(r.direction, e2);

  float a = glm::dot(e1, p);

  if (a > -Epsilon && a < Epsilon) {
    return false;
  }

  float inv_det = 1.0f / a;

  vec3 s = r.origin - v0;

  float u = glm::dot(s, p) * inv_det;

  if (u < 0.0f || u > 1.0f) {
    return false;
  }

  vec3 q = glm::cross(s, e1);

  float v = glm::dot(r.direction, q) * inv_det;

  if (v < 0.0f || u + v > 1.0f) {
    return false;
  }

  float t = glm::dot(e2, q) * inv_det;

  if (t > Epsilon) {
    *result_distance = t;
    return true;
  }

  return false;
}

void intersect_model(HitResult *result, Model *model, const Ray &r) {
  bool hit = false;

  if (!aabb_intersection(model->mesh.bounds, r)) {
    result->distance = FLT_MAX;
    return;
  }

  float distance = FLT_MAX;

  float result_distance;

  u32 index;
  for (u32 i=0; i<model->mesh.indices_count; i += 3) {
    int indices_a = model->mesh.indices[i + 0] * 3;
    int indices_b = model->mesh.indices[i + 1] * 3;
    int indices_c = model->mesh.indices[i + 2] * 3;

    vec3 a = *(vec3 *)(model->mesh.vertices + indices_a);
    vec3 b = *(vec3 *)(model->mesh.vertices + indices_b);
    vec3 c = *(vec3 *)(model->mesh.vertices + indices_c);

    if (intersect_triangle(a, b, c, r, &result_distance)) {
      if (result_distance < distance) {
        index = i;
        distance = result_distance;
        hit = true;
      }
    }
  }

  if (hit) {
    int indices_a = model->mesh.indices[index + 0] * 3;
    int indices_b = model->mesh.indices[index + 1] * 3;
    int indices_c = model->mesh.indices[index + 2] * 3;

    vec3 normal_a = *(vec3 *)(model->mesh.normals + indices_a);
    vec3 normal_b = *(vec3 *)(model->mesh.normals + indices_b);
    vec3 normal_c = *(vec3 *)(model->mesh.normals + indices_c);

    result->normal = glm::normalize((normal_a + normal_b + normal_c) / 3.0f);
    result->material_index = model->material_index;
    result->distance = (distance - 1e-4f);
    return;
  }

  result->distance = FLT_MAX;
}

void bvh_intersect(BVHNode *root, Ray r, HitResult *result) {
  int stack_ptr = 0;
  BVHNode *stack[32];
  stack[0] = root;

  while (stack_ptr >= 0) {
    BVHNode *node = stack[stack_ptr];
    stack_ptr -= 1;

    if (aabb_intersection(node->bounds, r)) {
      if (node->is_leaf) {
        for (u32 i=0; i<node->count; i++) {
          Triangle *triangle = node->triangles + i;
          if (node->count == 1 || aabb_intersection(triangle->bounds, r)) {
            float intersection_distance;
            if (intersect_triangle(triangle->positions[0], triangle->positions[1], triangle->positions[2], r, &intersection_distance)) {
              if (intersection_distance < result->distance) {
                result->distance = intersection_distance;
                result->normal = triangle->normal;
                result->material_index = triangle->material_index;
              }
            }
          }
        }
      } else {
        stack[++stack_ptr] = node->left;
        stack[++stack_ptr] = node->right;
      }
    }
  }
}

void intersect_all(HitResult *closest, World *world, const Ray &r) {
  HitResult result;
  result.distance = FLT_MAX;

#if 0
  for (auto it = array::begin(world->triangles); it != array::end(world->triangles); it++) {
    if (!aabb_intersection(it->bounds, r)) {
      continue;
    }

    float intersection_distance;
    if (intersect_triangle(it->positions[0], it->positions[1], it->positions[2], r, &intersection_distance)) {
      if (intersection_distance < result.distance) {
        result.distance = intersection_distance;
        result.normal = it->normal;
        result.material_index = it->material_index;
      }
    }
  }
#else
  bvh_intersect(world->bvh, r, &result);
#endif
  *closest = result;
}

vec3 reflect(vec3 value, vec3 normal) {
  return value - normal * 2.0f * glm::dot(normal, value);
}

vec3 cosine_sample_hemisphere(float u1, float u2) {
  float theta = TAU * u2;
  float r = glm::sqrt(u1);

  float x = r * glm::cos(theta);
  float y = r * glm::sin(theta);

  return vec3(x, y, glm::sqrt(glm::max(0.0f, 1.0f - u1)));
}

vec3 radiance(World *world, Ray ray, int max_bounces, RandomSequence *random) {
  int depth_iteration = 0;

  vec3 color(0.0, 0.0, 0.0);
  vec3 reflectance(1.0, 1.0, 1.0);

  HitResult hit;
  while (true) {
    intersect_all(&hit, world, ray);
    if (hit.distance == FLT_MAX) {
      return color + vec3(0.7, 0.7, 0.8)/20.0f * reflectance;
    }

    vec3 n = hit.normal;
    vec3 hit_position = ray.origin + ray.direction * (hit.distance - 1e-4f);
    vec3 normal = glm::dot(n, ray.direction) < 0 ? n : n * -1.0f;

    Material material = world->materials[hit.material_index];

    vec3 f = material.color;

    float p = glm::compMax(f);

    color = color + reflectance * material.emission;

    if (++depth_iteration > max_bounces) {
      if (random_float(random) < p) {
        f = f / p;
      } else {
        return color;
      }
    }

    reflectance = reflectance * f;

    if (material.type == DIFF) {
      // http://www.rorydriscoll.com/2009/01/07/better-sampling/
      vec3 sdir;
      if (glm::abs(normal.x) > 0.1) {
        sdir = glm::normalize(glm::cross(vec3(0.0, 1.0, 0.0), normal));
      } else {
        sdir = glm::normalize(glm::cross(vec3(1.0, 0.0, 0.0), normal));
      }

      vec3 tdir = glm::cross(normal, sdir);

      vec3 sample = cosine_sample_hemisphere(random_float(random), random_float(random));

      vec3 d = (sdir * sample.x + tdir * sample.y + normal * sample.z);

      ray.origin = hit_position;
      ray.direction = glm::normalize(d);
      ray.inv = 1.0f / ray.direction;
      continue;
    } else if (material.type == REFL) {
      ray.origin = hit_position;
      ray.direction = reflect(ray.direction, normal);
      ray.inv = 1.0f / ray.direction;
      continue;
    } else if (material.type == REFR) {
      Ray reflRay;
      reflRay.origin = hit_position;
      reflRay.direction = reflect(ray.direction, n);
      ray.inv = 1.0f / ray.direction;
      bool into = glm::dot(n, normal) > 0.0;
      float nc=1, nt=1.5, nnt=into?nc/nt:nt/nc, ddn=glm::dot(ray.direction, normal), cos2t;
      if ((cos2t=1-nnt*nnt*(1-ddn*ddn))<0){
        ray = reflRay;
        continue;
      }

      vec3 tdir = glm::normalize(ray.direction*nnt - n*((into?1:-1)*(ddn*nnt+glm::sqrt(cos2t))));
      float a=nt-nc;
      float b=nt+nc;
      float R0=a*a/(b*b);
      float c = 1-(into?-ddn:glm::dot(tdir, n));

      float Re=R0+(1-R0)*c*c*c*c*c;
      float Tr=1-Re;
      float P=.25+.5*Re;
      float RP=Re/P;
      float TP=Tr/(1-P);

      if (random_float(random) < P) {
        reflectance = reflectance*RP;
        ray = reflRay;
      } else {
        reflectance = reflectance*TP;
        ray.origin = hit_position;
        ray.direction = tdir;
        ray.inv = 1.0f / ray.direction;
      }
      continue;
    }
  }
}

struct Camera {
  vec3 position;
  vec3 rotation;
  glm::mat4 view_matrix;
  int width;
  int height;
};

Ray get_camera_ray(Camera *camera, float x, float y) {
  vec3 from = glm::unProject(
      glm::vec3(x, y, 0.0),
      glm::mat4(),
      camera->view_matrix,
      glm::vec4(0.0f, 0.0f, camera->width, camera->height));

  vec3 direction = glm::normalize(from - camera->position);

  Ray ray;
  ray.origin = from;
  ray.direction = direction;
  ray.inv = 1.0f / ray.direction;

  return ray;
}

namespace RenderTileState {
  enum RenderTileState {
    RENDERING,
    DONE,
    WAITING
  };
}

struct RenderData {
  int min_x;
  int min_y;
  int max_x;
  int max_y;

  int samps;
  int max_bounces;

  int index_x;
  int index_y;
  RandomSequence random;

  Camera *camera;
  vec3 *colors;
  RenderTileState::RenderTileState state;
  World *world;
};

u64 get_performance_counter() {
  return SDL_GetPerformanceCounter();
}

u64 get_performance_frequency() {
  return SDL_GetPerformanceFrequency();
}

inline int to_int(float x) {
  return int(glm::pow(glm::clamp(x, 0.0f, 1.0f), 1.0f / 2.2f) * 255 + 0.5f);
}

char *mprintf(const char *format, ...) {
  va_list args;
  va_start(args, format);
  char *data;
  vasprintf(&data, format, args);
  va_end(args);

  return data;
}

void export_image(char *name, vec3 *colors, int width, int height) {
  u8 *dst = (u8 *)malloc(4 * width * height);

  for (int i=0; i<width*height; i++) {
    dst[i * 4 + 0] = to_int(colors[i].x);
    dst[i * 4 + 1] = to_int(colors[i].y);
    dst[i * 4 + 2] = to_int(colors[i].z);
    dst[i * 4 + 3] = 255;
  }

  char *path = mprintf("../../../%s.png", name);
  stbi_write_png(path, width, height, 4, dst, width * 4);
  free(path);

  free(dst);
}

void render(void *data) {
  RenderData *work = (RenderData *)data;
  int min_x = work->min_x;
  int min_y = work->min_y;
  int max_x = work->max_x;
  int max_y = work->max_y;
  int samps = work->samps;
  int max_bounces = work->max_bounces;
  Camera *camera = work->camera;
  vec3 *colors = work->colors;
  int width = camera->width;
  int height = camera->height;
  World *world = work->world;
  RandomSequence random = work->random;

  work->state = RenderTileState::RENDERING;

  for (int y=min_y; y<max_y; y++) {
    for (int x=min_x; x<max_x; x++) {
      int i = (height - y - 1) * width + x;
      vec3 pixel_color = vec3(0.0);

      for (int sy=0; sy<2; sy++) {
        for (int sx=0; sx<2; sx++) {
          float dx = ((float)sx + 0.5)/2.0;
          float dy = ((float)sy + 0.5)/2.0;

          Ray ray = get_camera_ray(camera, x - 0.5f + dx, y - 0.5f + dy);

          for (int s=0; s<samps; s++) {
            vec3 ray_color = radiance(world, ray, max_bounces, &random);
            pixel_color = pixel_color + ray_color * (1.0f / samps);
          }
        }
      }

      vec3 final_pixel_color = glm::clamp((pixel_color / 4.0f), vec3(0.0), vec3(1.0));
      colors[i] += final_pixel_color;
    }
  }

  work->state = RenderTileState::DONE;
}

int main(int argc, char **argv) {
  std::srand(std::time(NULL));
  int width = 256;
  int height = width * (9.0f / 16.0f);
  int max_bounces = 4;
  int samps = 130;
  char *model_file = (char *)"box.obj";

  if (argc > 1) {
    samps = atoi(argv[1]);
  }

  if (argc > 2) {
    max_bounces = atoi(argv[2]);
  }

  if (argc > 3) {
    model_file = argv[3];
  }

  float aspect = (float)height / (float)width;

  chdir(SDL_GetBasePath());

  Queue main_queue = {};
  initialize_queue(&main_queue, 128);
  create_workers(&main_queue, SDL_GetCPUCount());

  u8 *pixels = (u8 *)malloc(width * height * 4);

  World world;

  load_model_work(&world, model_file);

  vec3 *colors = new vec3[width * height];

  Camera camera;
  /* camera.position = vec3(2.0, 1.0, 5.7); */
  camera.position = vec3(0.0, 1.0, 3.0);
  camera.rotation = vec3(0.0, 0.0, 0.0);
  camera.view_matrix = glm::perspective(glm::radians(50.0f), (float)width / (float)height, 0.1f, 1000.0f);
  camera.view_matrix = glm::rotate(camera.view_matrix, camera.rotation.x, vec3(1.0, 0.0, 0.0));
  camera.view_matrix = glm::rotate(camera.view_matrix, camera.rotation.y, vec3(0.0, 1.0, 0.0));
  camera.view_matrix = glm::rotate(camera.view_matrix, camera.rotation.z, vec3(0.0, 0.0, 1.0));
  camera.view_matrix = glm::translate(camera.view_matrix, (camera.position * -1.0f));
  camera.width = width;
  camera.height = height;

  top_down_bvtree(&world.bvh, world.triangles.data, world.triangles.size);

  u32 tile_count_x = 8;
  u32 tile_count_y = 8;

  RenderData data[tile_count_x * tile_count_y];

  u64 start = get_performance_counter();

  u32 tile_width = width / tile_count_x;
  u32 tile_height = height / tile_count_y;

  u32 count = 0;
  u32 tile_start_x = 0;
  u32 tile_start_y = 0;
  for (u32 y=tile_start_y; y<tile_count_y; y++) {
    for (u32 x=tile_start_x; x<tile_count_x; x++) {
      RenderData *item = data + count++;

      item->random.seed = count;
      item->index_x = x;
      item->index_y = y;
      item->min_x = x * tile_width;
      item->min_y = y * tile_height;
      item->max_x = item->min_x + tile_width;
      item->max_y = item->min_y + tile_height;
      item->state = RenderTileState::WAITING;

      item->colors = colors;
      item->camera = &camera;
      item->world = &world;
      item->samps = samps;
      item->max_bounces = max_bounces;

      if (x == (tile_count_x - 1)) {
        item->max_x = width;
      }

      if (y == (tile_count_y - 1)) {
        item->max_y = height;
      }
    }
  }

  for (u32 i=0; i<count; i++) {
    add_work(&main_queue, render, data + i);
  }

  int window_width = 720;
  int window_height = window_width * aspect;

  SDL_Init(SDL_INIT_EVERYTHING);
  SDL_Window *window = SDL_CreateWindow("Pathtracer",
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      window_width, window_height,
      SDL_WINDOW_SHOWN);

  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  SDL_Rect screen_rect;
  screen_rect.x = 0;
  screen_rect.y = 0;
  screen_rect.w = width;
  screen_rect.h = height;

  SDL_Texture *screen_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, width, height);

  SDL_Event event;
  bool running = true;
  bool image_exported = false;

  while (running) {
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
        case SDL_QUIT:
          running = false;
          break;
      }
    }

    for (int i=0; i<width*height; i++) {
      pixels[i * 4 + 0] = to_int(colors[i].z);
      pixels[i * 4 + 1] = to_int(colors[i].y);
      pixels[i * 4 + 2] = to_int(colors[i].x);
      pixels[i * 4 + 3] = 255;
    }

    SDL_UpdateTexture(screen_texture, &screen_rect, pixels, width * 4);
    SDL_RenderCopy(renderer, screen_texture, NULL, NULL);

    for (u32 tile_index=0; tile_index<array_count(data); tile_index++) {
      RenderData *item = data + tile_index;

      float scale_x = ((float)window_width / (float)width);
      float scale_y = ((float)window_height / (float)height);

      if (item->state == RenderTileState::RENDERING) {
        SDL_Rect rect;
        rect.x = glm::floor((float)item->min_x * scale_x);
        rect.y = glm::floor((float)item->min_y * scale_y);
        rect.w = glm::ceil((float)(item->max_x - item->min_x) * scale_x);
        rect.h = glm::ceil((float)(item->max_y - item->min_y) * scale_y);

        rect.y = window_height - rect.y - rect.h;

        SDL_SetRenderDrawColor(renderer, 228, 214, 42, 255);
        SDL_RenderDrawRect(renderer, &rect);
      }
    }

    if (!image_exported && main_queue.completion_goal == main_queue.completion_count) {
      u64 end = get_performance_counter();
      float time = (float)((end - start) / (float)get_performance_frequency());

      char *name = mprintf("image_%d_%d_%dx%d %.2fs", samps, max_bounces, width, height, time);
      export_image(name, colors, width, height);
      free(name);
      image_exported = true;
    }

    SDL_RenderPresent(renderer);
    SDL_Delay(50);
  }

  exit(1);
}
