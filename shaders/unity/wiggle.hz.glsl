// fish / shark / seaweed: sine wiggle along local z, albedo tinted by a per-material tint row, PBR lit
#include "renderer.glsl"

@vs
#define HZ_RENDERER_LIGHTING_CLUSTERS
out vec3 v_world_pos;
out vec3 v_normal;
out vec2 v_uv;
flat out uint v_material_offset;
out float v_view_depth;
@material
struct MaterialFS {
    vec4 color;
    @texture2D
    uint albedo_tex;
    @texture2D
    uint metallic_gloss;
    @texture2D
    uint tints;
    float smoothness;
    float metallic;
    float wave_1_frequency;
    float wave_1_speed;
    float distance;
    float offset;
};

void main() {
    VertexInstanceData v = unpack_instance_data();
    MaterialFS mat = MaterialFS_unpack(v.material_offset);
    float wave = sin((mat.offset + hz_scene.time) * mat.wave_1_speed + v.pos.z * mat.wave_1_frequency);
    vec3 pos = vec3(wave * mat.distance + v.pos.x, v.pos.y, v.pos.z);
    vec4 world_pos = v.model * vec4(pos, 1.0);
    gl_Position = hz_scene.view_proj * world_pos;
    v_world_pos = world_pos.xyz;
    v_normal = hz_transform_normal(v.model, v.normal);
    v_uv = v.uv;
    v_material_offset = v.material_offset;
    v_view_depth = -(hz_scene.view * world_pos).z;
}
@end

@fs
#define HZ_RENDERER_SHADOWS
#define HZ_RENDERER_IBL
#define HZ_RENDERER_LIGHTING_CLUSTERS
#define HZ_RENDERER_FOG
in vec3 v_world_pos;
in vec3 v_normal;
in vec2 v_uv;
flat in uint v_material_offset;
in float v_view_depth;
out vec4 frag_color;

void main() {
    MaterialFS mat = MaterialFS_unpack(v_material_offset);
    vec4 albedo = texture_sample(albedo_tex, mat.albedo_tex, v_uv);
    vec3 tint = texture_sample(tints, mat.tints, vec2(mat.offset, 0.0)).rgb;
    float gloss = texture_sample(metallic_gloss, mat.metallic_gloss, v_uv).a;
    // albedo.a masks where the tint applies
    vec3 base_color = mix(albedo.rgb, albedo.rgb * tint, albedo.a) * mat.color.rgb;

    // no normal map: the shading normal is the mesh normal
    SurfaceInput surface = SurfaceInput(base_color, mat.metallic, mat.smoothness * gloss);
    LightingInput li = LightingInput(v_world_pos, normalize(v_normal), hz_scene.camera_pos);

    vec3 lit = pbr_directional_lighting(surface, li, hz_scene.lights, calculate_shadow(v_world_pos),
                                        hz_shadow.shadow_color);
    HZ_CLUSTER_LOOP_BEGIN(v_view_depth)
        lit += pbr_point_light(surface, li, light);
    HZ_CLUSTER_LOOP_END
    lit += ibl_ambient(surface, li);
    frag_color = vec4(hz_apply_fog(lit, v_view_depth), 1.0);
}
@end
