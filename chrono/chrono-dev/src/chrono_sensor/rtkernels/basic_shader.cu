// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// RT kernels for box geometries
//
// =============================================================================
#include <optix.h>
#include <optixu/optixu_math_namespace.h>

#include "chrono_sensor/rtkernels/ray_utils.h"
#include "chrono_sensor/scene/lights.h"

using namespace optix;

rtDeclareVariable(PerRayData_radiance, prd_radiance, rtPayload, );
rtDeclareVariable(PerRayData_shadow, prd_shadow, rtPayload, );
rtDeclareVariable(float3, shading_normal, attribute shading_normal, );
rtDeclareVariable(float3, geometric_normal, attribute geometric_normal, );
rtDeclareVariable(float, t_hit, rtIntersectionDistance, );
rtDeclareVariable(optix::Ray, ray, rtCurrentRay, );

rtDeclareVariable(float3, ambient_light_color, , );
rtDeclareVariable(float3, Ka, , );
rtDeclareVariable(float3, Kd, , );
rtDeclareVariable(float3, Ks, , );
rtDeclareVariable(float, transparency, , );
rtDeclareVariable(float, phong_exp, , );
rtDeclareVariable(float, fresnel_exp, , );
rtDeclareVariable(float, fresnel_min, , );
rtDeclareVariable(float, fresnel_max, , );

rtDeclareVariable(float2, texcoord, attribute texcoord, );
rtDeclareVariable(int, has_texture, , );
rtTextureSampler<float4, 2> Kd_map;

rtDeclareVariable(rtObject, root_node, , );
rtDeclareVariable(float, scene_epsilon, , );
rtDeclareVariable(float, max_scene_distance, , );
rtDeclareVariable(float, importance_cutoff, , );
rtDeclareVariable(int, max_depth, , );

rtBuffer<PointLight> lights;

RT_PROGRAM void normal_shader() {
    // set the result equal to distance to the intersection
    if (prd_radiance.type == DEPTH) {
        prd_radiance.distance = t_hit;
    } else if (prd_radiance.type == LIDAR) {
    } else if (prd_radiance.type == CAMERA) {
        prd_radiance.color = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, shading_normal)) * 0.5f + 0.5f;
    }
}

RT_PROGRAM void reflective_shader() {
    // set the result equal to distance to the intersection
    if (prd_radiance.type == DEPTH) {
        prd_radiance.distance = t_hit;
        prd_radiance.first_distance = t_hit;
    } else if (prd_radiance.type == LIDAR) {
        prd_radiance.first_distance = t_hit;
        float3 geometric_normal_glob = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, geometric_normal));
        float3 shading_normal_glob = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, shading_normal));
        float3 forward_normal = faceforward(shading_normal_glob, -ray.direction, geometric_normal_glob);

        float intensity = dot(forward_normal, -ray.direction);

        prd_radiance.color = make_float3(t_hit, intensity, 0.f);
    } else if (prd_radiance.type == CAMERA) {
        prd_radiance.first_distance = t_hit;
        float3 geometric_normal_glob = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, geometric_normal));
        float3 shading_normal_glob = normalize(rtTransformNormal(RT_OBJECT_TO_WORLD, shading_normal));
        // make sure we have the normal that is facing where the ray came from
        float3 forward_normal = faceforward(shading_normal_glob, -ray.direction, geometric_normal_glob);
        // float3 forward_normal = faceforward(geometric_normal_glob, -ray.direction, geometric_normal_glob);

        float3 resulting_color = make_float3(0.0f);
        //=================
        // ambient color
        //=================

        // get Kd either from color or from texture
        float3 tmp_kd = Kd;
        float3 tmp_ka = Ka;
        // float3 tmp_ka = Kd;
        if (has_texture) {
            tmp_kd = make_float3(tex2D(Kd_map, texcoord.x, texcoord.y));
            tmp_ka = make_float3(tex2D(Kd_map, texcoord.x, texcoord.y));
        }

        resulting_color += tmp_ka * ambient_light_color;  // the color is at least equal to ambient color factor

        float3 hit_point = ray.origin + ray.direction * t_hit;  // + 0.01 * forward_normal;
        for (int i = 0; i < lights.size(); i++) {
            PointLight l = lights[i];
            float dist_to_light = length(l.pos - hit_point);
            float3 dir_to_light = normalize(l.pos - hit_point);

            //
            float light_normal_intensity = dot(forward_normal, dir_to_light);

            // 0 if we already know there is a shadow, 1 if we might be able to see the light
            float3 light_attenuation = make_float3(static_cast<float>(light_normal_intensity > 0.f));

            // if we think we can see the light, let's see if we are correct
            if (light_normal_intensity > 0.0f) {
                // float light_attenuation = clamp(((l.max_range - dist_to_light) / l.max_range), 0.0f, 1.0f);
                light_attenuation = make_float3(light_normal_intensity);
                // light_normal_intensity =
                //     light_normal_intensity * clamp(((l.max_range - dist_to_light) / l.max_range), 0.0f, 1.0f);

                // check shadows
                PerRayData_shadow prd_shadow;
                prd_shadow.attenuation = make_float3(1.0f);
                Ray shadow_ray(hit_point, dir_to_light, SHADOW_RAY_TYPE, scene_epsilon, dist_to_light);
                rtTrace(root_node, shadow_ray, prd_shadow);
                light_attenuation = prd_shadow.attenuation;
            }

            // if any of our channels can see the light, let's calculate the contribution
            if (fmaxf(light_attenuation) > 0.0f) {
                // linear light fall off -> TODO: change to fall with 1/r^2 (surface area expansion)

                float3 light_contrib = l.color * light_normal_intensity * light_attenuation *
                                       clamp(((l.max_range - dist_to_light) / l.max_range), 0.0f, 1.0f);

                // light_contrib = light_contrib * light_attenuation;
                // diffuse component with point light falling off

                //=================
                // diffuse color
                //=================
                // resulting_color += tmp_kd * light_normal_intensity * light_contrib;
                resulting_color += tmp_kd * light_contrib;

                // printf("Dot to light: %f\n", light_normal_intensity);

                // specular component
                float3 halfway_vec = normalize(dir_to_light - ray.direction);
                float dot_fn_halfway = dot(forward_normal, halfway_vec);
                if (dot_fn_halfway > 0.0f) {
                    //=================
                    // specular color
                    //=================
                    resulting_color += Ks * light_contrib * pow(dot_fn_halfway, phong_exp);
                }
            }
        }

        // calculate transparency and send ray through material
        float refract_importance = prd_radiance.importance * (1 - transparency);
        if (refract_importance > importance_cutoff && prd_radiance.depth < max_depth) {
            PerRayData_radiance prd_refraction =
                make_radiance_data(make_float3(0), refract_importance, prd_radiance.depth + 1, 0, 0, CAMERA);

            float3 refract_dir;
            refract(refract_dir, ray.direction, forward_normal, 1.f);
            Ray refraction_ray(hit_point, refract_dir, RADIANCE_RAY_TYPE, scene_epsilon, max_scene_distance);
            rtTrace(root_node, refraction_ray, prd_refraction);

            //==============================================
            // composite color from refraction and diffusion
            //==============================================
            resulting_color = transparency * resulting_color + (1 - transparency) * prd_refraction.color;
        }

        float3 reflect_amount = Ks * fresnel_schlick(dot(forward_normal, -ray.direction), fresnel_exp,
                                                     make_float3(fresnel_min), make_float3(fresnel_max));

        float reflect_importance = prd_radiance.importance * luminance(reflect_amount);

        if (reflect_importance > importance_cutoff && prd_radiance.depth < max_depth) {
            // PerRayData_radiance prd_reflection;
            PerRayData_radiance prd_reflection =
                make_radiance_data(make_float3(0), reflect_importance, prd_radiance.depth + 1, 0, 0, CAMERA);

            float3 reflect_dir = reflect(ray.direction, forward_normal);
            Ray reflection_ray(hit_point, reflect_dir, RADIANCE_RAY_TYPE, scene_epsilon, max_scene_distance);

            rtTrace(root_node, reflection_ray, prd_reflection);

            //==============================================
            // composite color from reflection and diffusion
            //==============================================
            resulting_color = (1 - reflect_amount) * resulting_color + reflect_amount * prd_reflection.color;
        }

        prd_radiance.color = resulting_color;
    }
}

RT_PROGRAM void hit_shadow() {
    // if the shadow ray hits anything before reaching the light, light clearly cannot hit this point
    prd_shadow.attenuation = make_float3(0.0f);
    rtTerminateRay();
}
