//
// Implementation for Yocto/RayTrace.
//

//
// LICENSE:
//
// Copyright (c) 2016 -- 2021 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "yocto_raytrace.h"
#include <iostream>
#include <yocto/yocto_cli.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_shading.h>
#include <yocto/yocto_shape.h>

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SCENE EVALUATION
// -----------------------------------------------------------------------------
namespace yocto {

// Generates a ray from a camera for yimg::image plane coordinate uv and
// the lens coordinates luv.
static ray3f eval_camera(const camera_data& camera, const vec2f& uv) {
  auto film = camera.aspect >= 1
                  ? vec2f{camera.film, camera.film / camera.aspect}
                  : vec2f{camera.film * camera.aspect, camera.film};
  auto q    = transform_point(camera.frame,
      {film.x * (0.5f - uv.x), film.y * (uv.y - 0.5f), camera.lens});
  auto e    = transform_point(camera.frame, {0, 0, 0});
  return {e, normalize(e - q)};
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR PATH TRACING
// -----------------------------------------------------------------------------
namespace yocto {
double reflectance(double cosine, double ref_idx) {
  // Use Schlick's approximation for reflectance.
  auto r0 = (1 - ref_idx) / (1 + ref_idx);
  r0      = r0 * r0;
  return r0 + (1 - r0) * pow((1 - cosine), 5);
}
vec3f refract2(const vec3f& uv, const vec3f& n, double etai_over_etat) {
  auto  cos_theta      = fmin(dot(-uv, n), 1.0);
  vec3f r_out_perp     = etai_over_etat * (uv + cos_theta * n);
  vec3f r_out_parallel = -sqrt(fabs(1.0 - length_squared(r_out_perp))) * n;
  return r_out_perp + r_out_parallel;
}
vec3f shade_indirect(const scene_data& scene, ray3f ray, int bounce,
    int max_bounces, const bvh_scene& bvh, rng_state& rng) {  
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) return eval_environment(scene, ray.d);

  const auto& instance = scene.instances[isec.instance];
  const auto& shape    = scene.shapes[instance.shape];
  auto outgoing = -ray.d;

  //position,normal texcoord
  auto&  position = transform_point(instance.frame,eval_position(shape, isec.element, isec.uv));
  auto& normal = transform_direction(instance.frame, eval_normal(shape, isec.element, isec.uv));
  auto& texcoord = eval_texcoord(shape, isec.element, isec.uv);

  //material values
  const auto& material = eval_material(scene,instance,isec.element,isec.uv);
 // scene.materials[instance.material];
  auto& color = material.color;

  // opacity
  if (rand1f(rng) < 1 - material.opacity)
    return shade_indirect(scene, ray3f{position, ray.d}, bounce + 1, max_bounces, bvh, rng);

  //radiance
  auto radiance = material.emission;

  if (bounce >= max_bounces) return radiance;


  if (!shape.points.empty()) {
    normal = -ray.d;
  } else if (!shape.lines.empty()) {
    normal = orthonormalize(-ray.d, normal);
  } else if (!shape.triangles.empty()) {
    if (dot(-ray.d, normal) < 0) {
      normal = -normal;
    }
  }
  switch (material.type) {
    case material_type::matte: {
      auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
      radiance += color * shade_indirect(scene,
                                       ray3f{position, incoming}, bounce + 1,
                                       max_bounces, bvh, rng);
      break;
    }
   
    case material_type::reflective: {
      if (!material.roughness) {//polished metal
        auto incoming = reflect(outgoing, normal);
        radiance += fresnel_schlick(color, normal, outgoing) *
                    shade_indirect(scene, ray3f{position, incoming},
                        bounce + 1, max_bounces, bvh, rng);
      } 
      else {//rough metal
        float exponent = 2 / (material.roughness * material.roughness);
        auto halfway = sample_hemisphere_cospower(exponent,normal, rand2f(rng));
        auto incoming = reflect(outgoing, halfway);
        radiance += color*shade_indirect(scene, ray3f{position, incoming}, bounce + 1, max_bounces, bvh, rng);
      };
      break;
    }
   
    case material_type::glossy: { //rough plastic
      float exponent = 2 / (material.roughness * material.roughness);
      auto  halfway = sample_hemisphere_cospower(exponent, normal, rand2f(rng));
      if (rand1f(rng) < fresnel_schlick(vec3f{0.04}, halfway, outgoing).x) {
        auto& incoming = reflect(outgoing, halfway);
        radiance += shade_indirect(scene, ray3f{position, incoming}, bounce + 1,
            max_bounces, bvh, rng);
      } else {
        auto& incoming = sample_hemisphere_cos(normal, rand2f(rng));
        radiance += color * shade_indirect(scene, ray3f{position, incoming},
                                bounce + 1, max_bounces, bvh, rng);
      }
      break;
    }
    
    case material_type::transparent : { //polished dielectrics
      if (rand1f(rng) <fresnel_schlick(vec3f{0.04}, normal, outgoing).x) {
        auto& incoming = reflect(outgoing, normal);
        radiance += shade_indirect(scene, ray3f{position, incoming}, bounce + 1,max_bounces,  bvh,  rng);
      } else {
        auto& incoming = -outgoing;
        radiance += color * shade_indirect(scene,ray3f{position, incoming}, bounce + 1, max_bounces, bvh, rng);
      }
      break;
    }
    case material_type::refractive: {
      // std::cout << "loooooooooool";refe
      // refrac
        
     // auto attenuation      = vec3f{1.0, 1.0, 1.0};
      auto ior         = material.ior;
      //std::cout << ior << "\n";
      float refraction_ratio;
     
      if (dot(-ray.d, normal) < 0) {//front face
        normal           = -normal;
        refraction_ratio =ior;
      } else {
        refraction_ratio = (1.0 / ior);
       
      }
      vec3f unit_direction = -ray.d;
      //vec3f refracted      = refract(unit_direction, normal, refraction_ratio);

      double cos_theta = fmin(dot(-unit_direction, normal), 1.0);
      double sin_theta = sqrt(1.0 - cos_theta * cos_theta);
      
      bool  cannot_refract = refraction_ratio * sin_theta > 1.0;
      vec3f direction;
     
      if (cannot_refract ||
          rand1f(rng) - 0.1 >
              fresnel_schlick(vec3f{refraction_ratio}, normal, outgoing).x) {
      direction = reflect(unit_direction, normal);
      radiance += color*shade_indirect(scene, ray3f{position,direction},
                              bounce + 1, max_bounces, bvh, rng);
    }
      else {
        direction = refract(unit_direction, normal, refraction_ratio);
        radiance +=  color*shade_indirect(scene, ray3f{position,direction},
                                bounce + 1, max_bounces, bvh, rng);
      }
    
        break;
    }
  }
  return radiance;
}
  // Raytrace renderer.
static vec4f shade_raytrace(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  return rgb_to_rgba(
      shade_indirect(scene, ray, bounce, params.bounces, bvh, rng));
}

// Matte renderer.
static vec4f shade_matte(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // YOUR CODE GOES HERE ----
  return {0, 0, 0, 0};
}

// Eyelight renderer.
static vec4f shade_eyelight(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {

  auto isec = intersect_bvh(bvh,scene, ray);
  if (!isec.hit) return {0, 0, 0};

  auto& instance = scene.instances[isec.instance];
  auto& material = scene.materials[instance.material];
  auto& shape    = scene.shapes[instance.shape];
  auto  normal   = transform_direction(
      instance.frame, eval_normal(shape, isec.element, isec.uv));
  vec4f mat = {material.color.x, material.color.y, material.color.z, 1.0};
  return mat * dot(normal, -ray.d);
}

static vec4f shade_normal(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh,scene, ray);
  if (!isec.hit) return {0, 0, 0};
  auto& instance = scene.instances[isec.instance];
  auto& shape    = scene.shapes[instance.shape];
  auto  normal   = transform_direction(
      instance.frame, eval_normal(shape, isec.element, isec.uv));
  vec4f norm = {normal.x, normal.y, normal.z, 1.0};
  return norm * 0.5 + 0.5;
}

static vec4f shade_texcoord(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) return {0, 0, 0};
  auto& instance = scene.instances[isec.instance];
  auto& shape    = scene.shapes[instance.shape];
  auto  coordinates = eval_texcoord(shape, isec.element, isec.uv);
  return {fmod(coordinates.x, 1), fmod(coordinates.y, 1)};
}

static vec4f shade_color(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto intersection = intersect_bvh(bvh, scene, ray);
  if (!intersection.hit) return {0, 0, 0};
  auto& material     = scene.materials[intersection.instance];
  auto color        = material.color;
  return {color.x, color.y, color.z, 1.0};
}

// Trace a single ray from the camera using the given algorithm.
using raytrace_shader_func = vec4f (*)(const scene_data& scene,
    const bvh_scene& bvh, const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params);
static raytrace_shader_func get_shader(const raytrace_params& params) {
  switch (params.shader) {
    case raytrace_shader_type::raytrace: return shade_raytrace;
    case raytrace_shader_type::matte: return shade_matte;
    case raytrace_shader_type::eyelight: return shade_eyelight;
    case raytrace_shader_type::normal: return shade_normal;
    case raytrace_shader_type::texcoord: return shade_texcoord;
    case raytrace_shader_type::color: return shade_color;
    default: {
      throw std::runtime_error("sampler unknown");
      return nullptr;
    }
  }
}

// Build the bvh acceleration structure.
bvh_scene make_bvh(const scene_data& scene, const raytrace_params& params) {
  return make_bvh(scene, false, false, params.noparallel);
}

// Init a sequence of random number generators.
raytrace_state make_state(
    const scene_data& scene, const raytrace_params& params) {
  auto& camera = scene.cameras[params.camera];
  auto  state  = raytrace_state{};
  if (camera.aspect >= 1) {
    state.width  = params.resolution;
    state.height = (int)round(params.resolution / camera.aspect);
  } else {
    state.height = params.resolution;
    state.width  = (int)round(params.resolution * camera.aspect);
  }
  state.samples = 0;
  state.image.assign(state.width * state.height, {0, 0, 0, 0});
  state.hits.assign(state.width * state.height, 0);
  state.rngs.assign(state.width * state.height, {});
  auto rng_ = make_rng(1301081);
  for (auto& rng : state.rngs) {
    rng = make_rng(961748941ull, rand1i(rng_, 1 << 31) / 2 + 1);
  }
  return state;
}

// Progressively compute an image by calling trace_samples multiple times.
void raytrace_samples(raytrace_state& state, const scene_data& scene,
    const bvh_scene& bvh, const raytrace_params& params) {
  if (state.samples >= params.samples) return;
  auto& camera = scene.cameras[params.camera];
  auto  shader = get_shader(params);
  state.samples += 1;
  if (params.samples == 1) {
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % state.width, j = idx / state.width;
      auto u = (i + 0.5f) / state.width, v = (j + 0.5f) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    }
  } else if (params.noparallel) {
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % state.width, j = idx / state.width;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    }
  } else {
    parallel_for(state.width * state.height, [&](int idx) {
      auto i = idx % state.width, j = idx / state.width;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    });
  }
}

// Check image type
static void check_image(
    const color_image& image, int width, int height, bool linear) {
  if (image.width != width || image.height != height)
    throw std::invalid_argument{"image should have the same size"};
  if (image.linear != linear)
    throw std::invalid_argument{
        linear ? "expected linear image" : "expected srgb image"};
}

// Get resulting render
color_image get_render(const raytrace_state& state) {
  auto image = make_image(state.width, state.height, true);
  get_render(image, state);
  return image;
}
void get_render(color_image& image, const raytrace_state& state) {
  check_image(image, state.width, state.height, true);
  auto scale = 1.0f / (float)state.samples;
  for (auto idx = 0; idx < state.width * state.height; idx++) {
    image.pixels[idx] = state.image[idx] * scale;
  }
}

}  // namespace yocto
