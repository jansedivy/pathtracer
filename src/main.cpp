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

#include "queue.h"
#include "array.h"

struct AABB {
  dvec3 min;
  dvec3 max;
};

enum Material {
  DIFF,
  REFR,
  REFL
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

struct Model {
  Mesh mesh;
  dvec3 color;
  dvec3 emission;
  Material material;
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

struct World {
  Array<Model> models;
};

void load_model_work(World *world, const char *path) {
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

      Model model;
      model.material = DIFF;

      aiMaterial *material = scene->mMaterials[mesh_data->mMaterialIndex];

      aiColor4D diffuse;
      aiColor4D emissive;

      if (aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, &diffuse) == AI_SUCCESS) {
        model.color = dvec3(diffuse.r, diffuse.g, diffuse.b);
      }

      if (aiGetMaterialColor(material, AI_MATKEY_COLOR_EMISSIVE, &emissive) == AI_SUCCESS) {
        model.emission = dvec3(emissive.r, emissive.g, emissive.b);
      }

      u32 vertices_count = count * 3;
      u32 normals_count = vertices_count;
      u32 uv_count = 0;
      u32 indices_count = index_count;

      u32 vertices_index = 0;
      u32 normals_index = 0;
      u32 indices_index = 0;

      allocate_mesh(&model.mesh, vertices_count, normals_count, indices_count, uv_count);

      for (u32 l=0; l<mesh_data->mNumVertices; l++) {
        model.mesh.vertices[vertices_index++] = mesh_data->mVertices[l].x;
        model.mesh.vertices[vertices_index++] = mesh_data->mVertices[l].y;
        model.mesh.vertices[vertices_index++] = mesh_data->mVertices[l].z;

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

      model.mesh.bounds = bounds;
      push_back(world->models, model);
    }
  }
}

struct Ray {
  dvec3 origin;
  dvec3 direction;
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

void intersect_model(HitResult *result, Model *model, const Ray &r) {
  bool hit = false;

  if (!aabb_intersection(model->mesh.bounds, r)) {
    result->hit = false;
    return;
  }

  dvec3 start = r.origin;
  dvec3 direction = r.direction;

  double distance = DBL_MAX;

  dvec3 result_position;

  u32 index;
  for (u32 i=0; i<model->mesh.indices_count; i += 3) {
    int indices_a = model->mesh.indices[i + 0] * 3;
    int indices_b = model->mesh.indices[i + 1] * 3;
    int indices_c = model->mesh.indices[i + 2] * 3;

    dvec3 a = dvec3(model->mesh.vertices[indices_a + 0],
                    model->mesh.vertices[indices_a + 1],
                    model->mesh.vertices[indices_a + 2]);

    dvec3 b = dvec3(model->mesh.vertices[indices_b + 0],
                    model->mesh.vertices[indices_b + 1],
                    model->mesh.vertices[indices_b + 2]);

    dvec3 c = dvec3(model->mesh.vertices[indices_c + 0],
                    model->mesh.vertices[indices_c + 1],
                    model->mesh.vertices[indices_c + 2]);

    if (glm::intersectRayTriangle(start, direction, a, b, c, result_position)) {
      if (result_position.z < distance) {
        index = i;
        distance = result_position.z;
        hit = true;
      }
    }
  }

  if (hit) {
    int indices_a = model->mesh.indices[index + 0] * 3;
    int indices_b = model->mesh.indices[index + 1] * 3;
    int indices_c = model->mesh.indices[index + 2] * 3;

    dvec3 normal_a = dvec3(model->mesh.normals[indices_a + 0],
                           model->mesh.normals[indices_a + 1],
                           model->mesh.normals[indices_a + 2]);

    dvec3 normal_b = dvec3(model->mesh.normals[indices_b + 0],
                           model->mesh.normals[indices_b + 1],
                           model->mesh.normals[indices_b + 2]);

    dvec3 normal_c = dvec3(model->mesh.normals[indices_c + 0],
                           model->mesh.normals[indices_c + 1],
                           model->mesh.normals[indices_c + 2]);
    result->hit = true;
    result->position = r.origin + r.direction * distance;
    result->normal = glm::normalize((normal_a + normal_b + normal_c) / 3.0);
    result->color = model->color;
    result->emission = model->emission;
    result->material = model->material;
    result->distance = distance;
  }
}

void intersect_all(HitResult *closest, World *world, const Ray &r) {
  HitResult hit;
  closest->hit = false;
  double distance = DBL_MAX;

  for (auto it = begin(world->models); it != end(world->models); it++) {
    intersect_model(&hit, it, r);
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
      double r = glm::sqrt(u);

      dvec3 sdir;
      if (glm::abs(normal.x) > 0.1) {
        sdir = glm::normalize(glm::cross(dvec3(0.0, 1.0, 0.0), normal));
      } else {
        sdir = glm::normalize(glm::cross(dvec3(1.0, 0.0, 0.0), normal));
      }

      dvec3 tdir = glm::cross(normal, sdir);

      dvec3 d = (sdir * glm::cos(angle) * r +
          tdir * glm::sin(angle) * r +
          normal * glm::max(0.0, glm::sqrt(1.0 - u)));

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

      dvec3 tdir = glm::normalize(ray.direction*nnt - n*((into?1:-1)*(ddn*nnt+glm::sqrt(cos2t))));
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

void export_image(dvec3 *colors, int width, int height) {
  u8 *dst = (u8 *)malloc(4 * width * height);

  for (int i=0; i<width*height; i++) {
    dst[i * 4 + 0] = to_int(colors[i].x);
    dst[i * 4 + 1] = to_int(colors[i].y);
    dst[i * 4 + 2] = to_int(colors[i].z);
    dst[i * 4 + 3] = 255;
  }

  stbi_write_png("../../../image.png", width, height, 4, dst, width * 4);

  free(dst);
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
          double r1=2*random_double(), dx=r1<1 ? glm::sqrt(r1)-1: 1-glm::sqrt(2-r1);
          double r2=2*random_double(), dy=r2<1 ? glm::sqrt(r2)-1: 1-glm::sqrt(2-r2);

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

int main(int argc, char *argv[]) {
  std::srand(std::time(NULL));
#if 0
  int width = 2880/2;
  int height = 1800/2;
  int max_bounces = 3;
  int samps = 300;
#else
  int width = 256;
  int height = width * (4.0 / 4.0);
  int max_bounces = 2;
  int samps = 500;
#endif
  float aspect = (float)height / (float)width;

  chdir(SDL_GetBasePath());

  Queue main_queue = {};
  initialize_queue(&main_queue, 64);
  create_workers(&main_queue, SDL_GetCPUCount());

  u8 *pixels = (u8 *)malloc(width * height * 4);

  World world;

  load_model_work(&world, "box.obj");

  dvec3 *colors = new dvec3[width * height];

  Camera camera;
  camera.position = dvec3(0.0, 1.0, 3.1);
  camera.view_matrix = glm::perspective(glm::radians(50.0), (double)width / (double)height, 1.0, 1000.0);
  camera.view_matrix = glm::translate(camera.view_matrix, (camera.position * -1.0));
  camera.width = width;
  camera.height = height;

  u32 tile_count_x = 1;
  u32 tile_count_y = 8;

  RenderData data[tile_count_x * tile_count_y];
  u32 tile_width = width / tile_count_x;
  u32 tile_height = height / tile_count_y;

  u32 count = 0;
  u32 tile_start_x = 0;
  u32 tile_start_y = 0;
  for (u32 y=tile_start_y; y<tile_count_y; y++) {
    for (u32 x=tile_start_x; x<tile_count_x; x++) {
      RenderData *item = data + count++;

      item->index_x = x;
      item->index_y = y;
      item->minX = x * tile_width;
      item->minY = y * tile_height;
      item->maxX = item->minX + tile_width;
      item->maxY = item->minY + tile_height;
      item->state = RenderTileState::WAITING;

      item->colors = colors;
      item->camera = &camera;
      item->world = &world;
      item->samps = samps;
      item->max_bounces = max_bounces;

      if (x == (tile_count_x - 1)) {
        item->maxX = width;
      }

      if (y == (tile_count_y - 1)) {
        item->maxY = height;
      }
    }
  }

  for (u32 i=0; i<array_count(data); i++) {
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

    for (u32 i=0; i<array_count(data); i++) {
      RenderData *item = data + i;

      int tile_width = item->maxX - item->minX;
      int tile_height = item->maxY - item->minY;

      if (item->state == RenderTileState::RENDERING) {
        SDL_Rect rect;
        rect.x = item->index_x * tile_width;
        rect.y = item->index_y * tile_height;
        rect.w = tile_width;
        rect.h = tile_height;

        rect.y = window_height - rect.y - rect.h;

        SDL_SetRenderDrawColor(renderer, 228, 214, 42, 255);
        SDL_RenderDrawRect(renderer, &rect);
      }
    }

    if (!image_exported && main_queue.completion_goal == main_queue.completion_count) {
      export_image(colors, width, height);
      image_exported = true;
    }

    SDL_RenderPresent(renderer);
    SDL_Delay(50);
  }

  return 1;
}
