#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <random>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <SDL2/SDL.h>

#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "stb_image_write.h"

typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t s8;
typedef int32_t s32;
typedef int64_t s64;

using glm::dvec3;
using glm::dmat4;
using glm::dvec4;

#define array_count(arr) (sizeof(arr) / sizeof(arr[0]))

struct AABB {
  dvec3 min;
  dvec3 max;
};

struct Mesh {
  void *data = NULL;

  float *vertices = NULL;
  float *normals = NULL;
  float *uv = NULL;
  int *indices = NULL;

  u32 vertices_count = 0;
  u32 normals_count = 0;
  u32 uv_count = 0;
  u32 indices_count = 0;

  AABB bounds;
};

void allocate_mesh(Mesh *mesh, u32 vertices_count, u32 normals_count, u32 indices_count, u32 uv_count) {
  u32 vertices_size = vertices_count * sizeof(float);
  u32 normals_size = normals_count * sizeof(float);
  u32 indices_size = indices_count * sizeof(int);
  u32 uv_size = uv_count * sizeof(float);

  u8 *data = static_cast<u8*>(malloc(vertices_size + normals_size + indices_size + uv_size));

  float *vertices = (float*)data;
  float *normals = (float*)(data + vertices_size);
  int *indices = (int*)(data + vertices_size + normals_size);
  float *uv = (float*)(data + vertices_size + normals_size + indices_size);

  mesh->data = data;
  mesh->vertices = vertices;
  mesh->vertices_count = vertices_count;

  mesh->normals = normals;
  mesh->normals_count = normals_count;

  mesh->indices = indices;
  mesh->indices_count = indices_count;

  mesh->uv = uv;
  mesh->uv_count = uv_count;
}

void load_model_work(std::vector<Mesh> *models, const char *path) {
  Assimp::Importer importer;

  const aiScene* scene = importer.ReadFile(path, aiProcess_GenNormals |
      aiProcess_CalcTangentSpace |
      aiProcess_Triangulate |
      aiProcess_JoinIdenticalVertices |
      aiProcess_OptimizeGraph |
      aiProcess_SortByPType);

  float max_distance = 0.0f;
  AABB bounds;
  bounds.min.x = DBL_MAX;
  bounds.min.y = DBL_MAX;
  bounds.min.z = DBL_MAX;
  bounds.max.x = DBL_MIN;
  bounds.max.y = DBL_MIN;
  bounds.max.z = DBL_MIN;

  if (scene->HasMeshes()) {
    for (u32 i=0; i<scene->mNumMeshes; i++) {
      u32 count = 0;
      u32 index_count = 0;

      aiMesh *mesh_data = scene->mMeshes[i];
      count += mesh_data->mNumVertices;

      for (u32 l=0; l<mesh_data->mNumFaces; l++) {
        aiFace face = mesh_data->mFaces[l];

        index_count += face.mNumIndices;
      }
      u32 vertices_count = count * 3;
      u32 normals_count = vertices_count;
      u32 uv_count = 0;
      u32 indices_count = index_count;

      u32 vertices_index = 0;
      u32 normals_index = 0;
      u32 indices_index = 0;

      Mesh mesh;
      allocate_mesh(&mesh, vertices_count, normals_count, indices_count, uv_count);

      for (u32 l=0; l<mesh_data->mNumVertices; l++) {
        mesh.vertices[vertices_index++] = mesh_data->mVertices[l].x;
        mesh.vertices[vertices_index++] = mesh_data->mVertices[l].y;
        mesh.vertices[vertices_index++] = mesh_data->mVertices[l].z;

#define FIND_MIN(a, b) if ((a) < (b)) { (b) = (a); }
#define FIND_MAX(a, b) if ((a) > (b)) { (b) = (a); }

        FIND_MIN(mesh_data->mVertices[l].x, bounds.min.x);
        FIND_MIN(mesh_data->mVertices[l].y, bounds.min.y);
        FIND_MIN(mesh_data->mVertices[l].z, bounds.min.z);
        FIND_MAX(mesh_data->mVertices[l].x, bounds.max.x);
        FIND_MAX(mesh_data->mVertices[l].y, bounds.max.y);
        FIND_MAX(mesh_data->mVertices[l].z, bounds.max.z);

        float new_distance = glm::length(dvec3(mesh_data->mVertices[l].x, mesh_data->mVertices[l].y, mesh_data->mVertices[l].z));
        if (new_distance > max_distance) {
          max_distance = new_distance;
        }

        mesh.normals[normals_index++] = mesh_data->mNormals[l].x;
        mesh.normals[normals_index++] = mesh_data->mNormals[l].y;
        mesh.normals[normals_index++] = mesh_data->mNormals[l].z;
      }

      for (u32 l=0; l<mesh_data->mNumFaces; l++) {
        aiFace face = mesh_data->mFaces[l];

        for (u32 j=0; j<face.mNumIndices; j++) {
          mesh.indices[indices_index++] = face.mIndices[j];
        }
      }

      mesh.bounds = bounds;
      models->push_back(mesh);
    }
  }
}

typedef void PlatformWorkQueueCallback(void *data);

struct WorkEntry {
  PlatformWorkQueueCallback *callback;
  void *data;
};

struct Queue {
  u32 volatile next_entry_to_write;
  u32 volatile next_entry_to_read;

  u32 volatile completion_count;
  u32 volatile completion_goal;

  WorkEntry entries[2048];

  SDL_sem *semaphore;
};

bool do_queue_work(Queue *queue) {
  bool sleep = false;

  u32 original_next_index = queue->next_entry_to_read;
  int new_next_index = (original_next_index + 1) % array_count(queue->entries);

  if (original_next_index != queue->next_entry_to_write) {
    SDL_bool value = SDL_AtomicCAS((SDL_atomic_t *)&queue->next_entry_to_read, original_next_index, new_next_index);

    if (value) {
      WorkEntry *entry = queue->entries + original_next_index;
      entry->callback(entry->data);

      SDL_AtomicIncRef((SDL_atomic_t *)&queue->completion_count);
    }
  } else {
    sleep = true;
  }

  return sleep;
}

static int thread_function(void *data) {
  Queue *queue = (Queue *)data;

  while (true) {
    if (do_queue_work(queue)) {
      SDL_SemWait(queue->semaphore);
    }
  }
}

void add_work(Queue *queue, PlatformWorkQueueCallback *callback, void *data) {
  u32 new_next_entry_to_write = (queue->next_entry_to_write + 1) % array_count(queue->entries);

  assert(new_next_entry_to_write != queue->next_entry_to_read);

  WorkEntry *entry = queue->entries + queue->next_entry_to_write;

  entry->callback = callback;
  entry->data = data;

  queue->completion_goal += 1;

  SDL_CompilerBarrier();

  queue->next_entry_to_write = new_next_entry_to_write;
  SDL_SemPost(queue->semaphore);
}

void complete_all_work(Queue *queue) {
  while (queue->completion_goal != queue->completion_count) {
    do_queue_work(queue);
    printf("%d/%d\n", queue->completion_count, queue->completion_goal);
  }

  queue->completion_count = 0;
  queue->completion_goal = 0;
}

struct Ray {
  dvec3 origin;
  dvec3 direction;
};

enum Material {
  DIFF,
  REFR,
  REFL
};

inline double random_double() {
  return (double)std::rand() / (double)RAND_MAX;
}

struct HitResult {
  bool hit;
  dvec3 position;
  dvec3 normal;
  dvec3 color;
  dvec3 emission;
  Material material;
  double distance;
};

enum ObjectType {
  SphereType,
  PlaneType,
  ModelType
};

struct Sphere {
  double radius;
  dvec3 position;
  dvec3 emission;
  dvec3 color;
  Material material;
};

struct Plane {
  dvec3 position;
  dvec3 normal;
  dvec3 emission;
  dvec3 color;
  Material material;
};

struct ModelObject {
  Mesh *model;
  dvec3 position;
  dvec3 color;
  dvec3 emission;
  Material material;
};

struct SceneObject {
  ObjectType type;

  Sphere sphere;
  Plane plane;
  ModelObject model;
};


void intersect_plane(HitResult *result, Plane *plane, const Ray &r) {
  float denom = glm::dot(plane->normal, r.direction);
  if (fabs(denom) > 0.0) {
    double t = glm::dot(plane->position - r.origin, plane->normal) / denom;
    if (t >= 0.001) {
      result->hit = true;
      result->position = r.origin + r.direction * t;
      result->normal = plane->normal;
      result->color = plane->color;
      result->emission = plane->emission;
      result->material = plane->material;
      result->distance = t;
      return;
    }
  }

  result->hit = false;
  return;
}

void intersect_sphere(HitResult *result, Sphere *sphere, const Ray &r) {
  dvec3 op = sphere->position - r.origin;
  double t;
  double eps = 1e-4;
  double b = glm::dot(op, r.direction);
  double det = b * b - glm::dot(op, op) + sphere->radius * sphere->radius;

  if (det < 0) {
    result->hit = false;
    return;
  } else {
    det = sqrt(det);
  }

  t = (t = b - det);
  if (t > eps) {
    result->hit = true;
    result->position = r.origin + r.direction * t;
    result->normal = glm::normalize(result->position - sphere->position);
    result->color = sphere->color;
    result->emission = sphere->emission;
    result->material = sphere->material;
    result->distance = t;
    return;
  }

  t = b + det;
  if (t > eps) {
    result->hit = true;
    result->position = r.origin + r.direction * t;
    result->normal = glm::normalize(result->position - sphere->position);
    result->color = sphere->color;
    result->emission = sphere->emission;
    result->material = sphere->material;
    result->distance = t;
    return;
  }

  result->hit = false;
  return;
}

bool aabb_intersection(AABB b, Ray r) {
  dvec3 dirfrac;
  dirfrac.x = 1.0 / r.direction.x;
  dirfrac.y = 1.0 / r.direction.y;
  dirfrac.z = 1.0 / r.direction.z;

  float t1 = (b.min.x - r.origin.x)*dirfrac.x;
  float t2 = (b.max.x - r.origin.x)*dirfrac.x;
  float t3 = (b.min.y - r.origin.y)*dirfrac.y;
  float t4 = (b.max.y - r.origin.y)*dirfrac.y;
  float t5 = (b.min.z - r.origin.z)*dirfrac.z;
  float t6 = (b.max.z - r.origin.z)*dirfrac.z;

  float tmin = glm::max(glm::max(glm::min(t1, t2), glm::min(t3, t4)), glm::min(t5, t6));
  float tmax = glm::min(glm::min(glm::max(t1, t2), glm::max(t3, t4)), glm::max(t5, t6));

  double t;
  if (tmax < 0) {
    t = tmax;
    return false;
  }

  if (tmin > tmax) {
    t = tmax;
    return false;
  }

  t = tmin;
  return true;
}

void intersect_model(HitResult *result, ModelObject *model, const Ray &r) {
  bool hit = false;
  double scale = 12.0;

  dmat4 model_view;
  model_view = glm::translate(model_view, model->position);
  model_view = glm::scale(model_view, dvec3(scale));
  dmat4 res = glm::inverse(model_view);

  AABB transformed_bounds;
  transformed_bounds.min = dvec3(model_view * dvec4(model->model->bounds.min, 1.0));
  transformed_bounds.max = dvec3(model_view * dvec4(model->model->bounds.max, 1.0));

  if (!aabb_intersection(transformed_bounds, r)) {
    result->hit = false;
    return;
  }

  dvec3 start = dvec3(res * dvec4(r.origin, 1.0));
  dvec3 direction = dvec3(res * dvec4(r.direction, 0.0));

  Mesh *mesh = model->model;

  double distance = DBL_MAX;

  dvec3 result_position;

  u32 index;
  for (u32 i=0; i<mesh->indices_count; i += 3) {
    int indices_a = mesh->indices[i + 0] * 3;
    int indices_b = mesh->indices[i + 1] * 3;
    int indices_c = mesh->indices[i + 2] * 3;

    dvec3 a = dvec3(mesh->vertices[indices_a + 0],
                    mesh->vertices[indices_a + 1],
                    mesh->vertices[indices_a + 2]);

    dvec3 b = dvec3(mesh->vertices[indices_b + 0],
                    mesh->vertices[indices_b + 1],
                    mesh->vertices[indices_b + 2]);

    dvec3 c = dvec3(mesh->vertices[indices_c + 0],
                    mesh->vertices[indices_c + 1],
                    mesh->vertices[indices_c + 2]);

    if (glm::intersectRayTriangle(start, direction, a, b, c, result_position)) {
      if (result_position.z < distance) {
        index = i;
        distance = result_position.z;
        hit = true;
      }
    }
  }

  if (hit) {
    int indices_a = mesh->indices[index + 0] * 3;
    int indices_b = mesh->indices[index + 1] * 3;
    int indices_c = mesh->indices[index + 2] * 3;

    dvec3 normal_a = dvec3(mesh->normals[indices_a + 0],
                           mesh->normals[indices_a + 1],
                           mesh->normals[indices_a + 2]);

    dvec3 normal_b = dvec3(mesh->normals[indices_b + 0],
                           mesh->normals[indices_b + 1],
                           mesh->normals[indices_b + 2]);

    dvec3 normal_c = dvec3(mesh->normals[indices_c + 0],
                           mesh->normals[indices_c + 1],
                           mesh->normals[indices_c + 2]);
    result->hit = true;
    result->position = r.origin + r.direction * distance;
    result->normal = glm::normalize((normal_a + normal_b + normal_c) / 3.0);
    result->color = model->color;
    result->emission = model->emission;
    result->material = model->material;
    result->distance = distance;
  }
}

struct World {
  std::vector<Sphere> spheres;
  std::vector<Plane> planes;
  std::vector<ModelObject> models;
};

void intersect_all(HitResult *closest, World *world, const Ray &r) {
  HitResult hit;
  closest->hit = false;
  double distance = DBL_MAX;

  for (auto it = world->spheres.begin(); it != world->spheres.end(); it++) {
    intersect_sphere(&hit, &(*it), r);
    if (hit.hit && hit.distance < distance) {
      *closest = hit;
      distance = hit.distance;
    }
  }

  for (auto it = world->planes.begin(); it != world->planes.end(); it++) {
    intersect_plane(&hit, &(*it), r);
    if (hit.hit && hit.distance < distance) {
      *closest = hit;
      distance = hit.distance;
    }
  }

  for (auto it = world->models.begin(); it != world->models.end(); it++) {
    intersect_model(&hit, &(*it), r);
    if (hit.hit && hit.distance < distance) {
      *closest = hit;
      distance = hit.distance;
    }
  }
}

dvec3 reflect(const dvec3 &value, const dvec3 &normal) {
  return value - normal * 2.0 * glm::dot(normal, value);
}

dvec3 radiance(World *world, Ray ray, int max_bounces) {
  int depth_iteration = 0;

  dvec3 color(0.0, 0.0, 0.0);
  dvec3 reflectance(1.0, 1.0, 1.0);

  HitResult hit;
  while (true) {
    intersect_all(&hit, world, ray);
    if (!hit.hit) {
      return color;
    }

    dvec3 n = hit.normal;
    dvec3 hit_position = hit.position;
    dvec3 normal = glm::dot(n, ray.direction) < 0 ? n : n * -1.0;
    dvec3 f = hit.color;

    double p = f.x > f.y && f.x > f.z ? f.x : f.y > f.z ? f.y : f.z;

    color = color + reflectance * hit.emission;

    if (++depth_iteration > max_bounces) {
      if (random_double() < p) {
        f = f / p;
      } else {
        return color;
      }
    }

    reflectance = reflectance * f;

    if (hit.material == DIFF) {
      // http://www.rorydriscoll.com/2009/01/07/better-sampling/
      double angle = 2.0 * M_PI * random_double();
      double u = random_double();
      double r = sqrt(u);

      dvec3 sdir;
      if (fabs(normal.x) > 0.1) {
        sdir = glm::normalize(glm::cross(dvec3(0.0, 1.0, 0.0), normal));
      } else {
        sdir = glm::normalize(glm::cross(dvec3(1.0, 0.0, 0.0), normal));
      }

      dvec3 tdir = glm::cross(normal, sdir);

      dvec3 d = (sdir * cos(angle) * r +
          tdir * sin(angle) * r +
          normal * glm::max(0.0, sqrt(1.0 - u)));

      ray.origin = hit_position;
      ray.direction = glm::normalize(d);
      continue;
    } else if (hit.material == REFL) {
      ray.origin = hit_position;
      ray.direction = reflect(ray.direction, normal);
      continue;
    } else if (hit.material == REFR) {
      Ray reflRay;
      reflRay.origin = hit_position;
      reflRay.direction = reflect(ray.direction, n);
      bool into = glm::dot(n, normal) > 0.0;
      double nc=1, nt=1.5, nnt=into?nc/nt:nt/nc, ddn=glm::dot(ray.direction, normal), cos2t;
      if ((cos2t=1-nnt*nnt*(1-ddn*ddn))<0){
        ray = reflRay;
        continue;
      }

      dvec3 tdir = glm::normalize(ray.direction*nnt - n*((into?1:-1)*(ddn*nnt+sqrt(cos2t))));
      double a=nt-nc;
      double b=nt+nc;
      double R0=a*a/(b*b);
      double c = 1-(into?-ddn:glm::dot(tdir, n));

      double Re=R0+(1-R0)*c*c*c*c*c;
      double Tr=1-Re;
      double P=.25+.5*Re;
      double RP=Re/P;
      double TP=Tr/(1-P);

      if (random_double() < P) {
        reflectance = reflectance*RP;
        ray = reflRay;
      } else {
        reflectance = reflectance*TP;
        ray.origin = hit_position;
        ray.direction = tdir;
      }
    }
  }
}

struct Camera {
  dvec3 position;
  glm::dmat4 view_matrix;
  int width;
  int height;
};

Ray get_camera_ray(Camera *camera, double x, double y) {
  dvec3 from = glm::unProject(
      glm::dvec3(x, y, 0.0),
      glm::dmat4(),
      camera->view_matrix,
      glm::dvec4(0.0, 0.0, camera->width, camera->height));

  dvec3 direction = glm::normalize(from - camera->position);

  Ray ray;
  ray.origin = from;
  ray.direction = direction;

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
  int minX;
  int minY;
  int maxX;
  int maxY;

  int samps;
  int max_bounces;

  int index_x;
  int index_y;

  Camera *camera;
  dvec3 *colors;
  RenderTileState::RenderTileState state;
  World *world;
};

u64 get_performance_counter() {
  return SDL_GetPerformanceCounter();
}

u64 get_performance_frequency() {
  return SDL_GetPerformanceFrequency();
}

inline int to_int(double x) {
  return int(pow(glm::clamp(x, 0.0, 1.0), 1 / 2.2) * 255 + .5);
}

void export_image(u8 *pixels, dvec3 *colors, int width, int height) {
  for (int i=0; i<width*height; i++) {
    pixels[i * 4 + 0] = to_int(colors[i].x);
    pixels[i * 4 + 1] = to_int(colors[i].y);
    pixels[i * 4 + 2] = to_int(colors[i].z);
    pixels[i * 4 + 3] = 255;
  }

  stbi_write_png("../../../image.png", width, height, 4, pixels, width * 4);
}

void render(void *data) {
  RenderData *work = (RenderData *)data;
  int minX = work->minX;
  int minY = work->minY;
  int maxX = work->maxX;
  int maxY = work->maxY;
  int samps = work->samps;
  int max_bounces = work->max_bounces;
  Camera *camera = work->camera;
  dvec3 *colors = work->colors;
  int width = camera->width;
  int height = camera->height;
  World *world = work->world;

  work->state = RenderTileState::RENDERING;

  for (int y=minY; y<maxY; y++) {
    for (int x=minX; x<maxX; x++) {
      int i = (height - y - 1) * width + x;
      dvec3 pixel_color = dvec3(0.0);
      for (int sy=0; sy<2; sy++) {
        for (int sx=0; sx<2; sx++) {
          double r1=2*random_double(), dx=r1<1 ? sqrt(r1)-1: 1-sqrt(2-r1);
          double r2=2*random_double(), dy=r2<1 ? sqrt(r2)-1: 1-sqrt(2-r2);

          Ray ray = get_camera_ray(camera, (sx + 0.5 + dx) / 2.0 + x - 0.5, (sy + 0.5 + dy) / 2.0 + y - 0.5);

          for (int s=0; s<samps; s++) {
            dvec3 ray_color = radiance(world, ray, max_bounces);
            pixel_color = pixel_color + ray_color * (1.0 / samps);
          }
        }
      }

      dvec3 final_pixel_color = glm::clamp((pixel_color / 4.0), dvec3(0.0), dvec3(1.0));
      colors[i] += final_pixel_color;
    }
  }

  work->state = RenderTileState::DONE;
}

Plane create_plane(dvec3 position, dvec3 normal, dvec3 color, dvec3 emission, Material material) {
  Plane plane;
  plane.position = position;
  plane.normal = normal;
  plane.color = color;
  plane.emission = emission;
  plane.material = material;
  return plane;
}

ModelObject create_model(Mesh *model, dvec3 position, dvec3 color, dvec3 emission, Material material) {
  ModelObject model_object;
  model_object.model = model;
  model_object.position = position;
  model_object.color = color;
  model_object.emission = emission;
  model_object.material = material;
  return model_object;
}

bool render_tile_sort(RenderData &a, RenderData &b) {
  glm::vec2 center = glm::vec2(4, 4);

  return glm::distance(glm::vec2(a.index_x, a.index_y), center) < glm::distance(glm::vec2(b.index_x, b.index_y), center);
}

int main(int argc, char *argv[]) {
  std::srand(std::time(NULL));
#if 0
  int width = 2880/2;
  int height = 1800/2;
  int max_bounces = 3;
  int samps = 300;
#else
  int width = 512;
  int height = width * (3.0 / 4.0);
  int max_bounces = 1;
  int samps = 10;
#endif
  float aspect = (float)height / (float)width;
  u32 tile_start_x = 0;
  u32 tile_start_y = 0;

  chdir(SDL_GetBasePath());

  Queue main_queue = {};
  main_queue.next_entry_to_write = 0;
  main_queue.next_entry_to_read = 0;

  main_queue.completion_count = 0;
  main_queue.completion_goal = 0;
  main_queue.semaphore = SDL_CreateSemaphore(0);

  for (int i=0; i<SDL_GetCPUCount(); i++) {
    SDL_CreateThread(thread_function, "main_worker_thread", &main_queue);
  }

  u8 *pixels = (u8 *)malloc(width * height * 4);

  std::vector<Mesh>models;

  load_model_work(&models, "model.obj");

  dvec3 *colors = new dvec3[width * height];

  World world;

  world.spheres.push_back({ 40, dvec3(50, 115, 60), dvec3(10),dvec3(1,1,1), DIFF });
  world.spheres.push_back({ 16.5, dvec3(73,16.5,55), dvec3(),dvec3(1,1,1)*0.999, REFL });

  world.planes.push_back(create_plane(dvec3(0, 0, 0), dvec3(0.0, 0.0, 1.0), dvec3(0.75), dvec3(0.0), DIFF));
  world.planes.push_back(create_plane(dvec3(5, 0, 0), dvec3(1.0, 0.0, 0.0), dvec3(39.0 / 255.0, 39.0 / 255.0, 214.0 / 255.0), dvec3(0.0), DIFF));
  world.planes.push_back(create_plane(dvec3(95, 0, 0), dvec3(1.0, 0.0, 0.0), dvec3(214.0 / 255.0, 204.0 / 255.0, 39.0 / 214.0), dvec3(0.0), DIFF));
  world.planes.push_back(create_plane(dvec3(0, 80, 0), dvec3(0.0, 1.0, 0.0), dvec3(0.75), dvec3(), DIFF));
  world.planes.push_back(create_plane(dvec3(0, 0, 0), dvec3(0.0, 1.0, 0.0), dvec3(0.75), dvec3(0.0), DIFF));
  world.planes.push_back(create_plane(dvec3(0, 0, 160), dvec3(0.0, 0.0, 1.0), dvec3(0.75), dvec3(0.0), DIFF));

  for (auto it = models.begin(); it != models.end(); it++) {
    world.models.push_back(create_model(&(*it), dvec3(35, 0, 50), dvec3(193.0 / 255, 80.0 / 255.0, 27.0 / 255), dvec3(0.0), DIFF));
  }

  Camera camera;
  camera.position = dvec3(50.0, 42.0, 155.6);
  camera.view_matrix = glm::perspective(glm::radians(60.0), (double)width / (double)height, 1.0, 1000.0);
  camera.view_matrix = glm::translate(camera.view_matrix, (camera.position * -1.0));
  camera.width = width;
  camera.height = height;

  u32 tile_count_x = 9;
  u32 tile_count_y = 9;

  u64 start = get_performance_counter();

  RenderData data[tile_count_x * tile_count_y];
  u32 tile_width = width / tile_count_x;
  u32 tile_height = height / tile_count_y;

  u32 count = 0;
  for (u32 y=tile_start_y; y<tile_count_y; y++) {
    for (u32 x=tile_start_x; x<tile_count_x; x++) {
      RenderData *item = data + count++;

      item->index_x = x;
      item->index_y = y;
      item->minX = x * tile_width;
      item->minY = y * tile_height;
      item->maxX = item->minX + tile_width;
      item->maxY = item->minY + tile_height;
      item->samps = samps;
      item->max_bounces = max_bounces;
      item->camera = &camera;
      item->colors = colors;
      item->state = RenderTileState::WAITING;
      item->world = &world;

      if (x == (tile_count_x - 1)) {
        item->maxX = width;
      }

      if (y == (tile_count_y - 1)) {
        item->maxY = height;
      }
    }
  }

  std::sort(data, data + array_count(data) - 1, render_tile_sort);

  for (u32 i=0; i<array_count(data); i++) {
    add_work(&main_queue, render, data + i);
  }

#if 1
  int window_width = 1080;
  int window_height = window_width * aspect;

  SDL_Init(SDL_INIT_EVERYTHING);
  SDL_Window *window = SDL_CreateWindow("Pathtracer",
      SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED,
      window_width, window_height,
      SDL_WINDOW_SHOWN);

  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  u8 *screen = (u8 *)malloc(width * height * 4);
  memset(screen, 0, width * height * 4);

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
      screen[i * 4 + 0] = to_int(colors[i].z);
      screen[i * 4 + 1] = to_int(colors[i].y);
      screen[i * 4 + 2] = to_int(colors[i].x);
      screen[i * 4 + 3] = 255;
    }

    SDL_UpdateTexture(screen_texture, &screen_rect, screen, width * 4);

    SDL_RenderCopy(renderer, screen_texture, NULL, NULL);

    SDL_SetRenderDrawColor(renderer, 228, 214, 42, 255);

    for (u32 i=0; i<array_count(data); i++) {
      RenderData *item = data + i;

      int tile_width = (item->maxX - item->minX) * ((float)window_width / (float)width);
      int tile_height = (item->maxY - item->minY) * ((float)window_height / (float)height);

      if (item->state == RenderTileState::RENDERING) {
        SDL_Rect rect;
        rect.x = item->index_x * tile_width;
        rect.y = window_height - (item->index_y * tile_height);
        rect.w = tile_width;
        rect.h = -1 * tile_height;
        SDL_RenderDrawRect(renderer, &rect);
      }
    }

    if (!image_exported && main_queue.completion_goal == main_queue.completion_count) {
      export_image(pixels, colors, width, height);
      image_exported = true;
    }

    SDL_RenderPresent(renderer);
    SDL_Delay(20);
  }
#else
  complete_all_work(&main_queue);
  export_image(pixels, colors, width, height);
  float time = (float)(get_performance_counter() - start)/(float)get_performance_frequency() * 1000.0f;
  printf("%.3fms\n", time);
#endif

  return 1;
}
