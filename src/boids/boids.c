#include "app/app.h"
#include "assets/assets.h"
#include "camera_controllers/camera_controllers.h"
#include "ecs/camera/ecs_camera.h"
#include "ecs/ecs.h"
#include "ecs/ecs_assets/ecs_assets.h"
#include "ecs/ecs_editor/ecs_editor.h"
#include "ecs/input/ecs_input.h"
#include "ecs/renderer/ecs_renderer.h"
#include "ecs/transform/ecs_transform.h"
#include "input.h"
#include "lib/hash.h"
#include "lib/hzmath.h"
#include "lib/hzptr.h"
#include "lib/memory.h"
#include "lib/random.h"
#include "lib/string_builder.h"
#include "lib/thread_context.h"
#include "lib/typedefs.h"
#include "os/os.h"
#include "rendering/renderer.h"
#include "scene/scene.h"
#include "ui/ui.h"

#include "boids/boids_sim.h"
#include "boids/boids_sim.gen.h"
#include "generated/types_app.gen.h"

#define SCENE_FILE "unity/scenes/Boids.hzt"
#define HUD_AVG_FRAMES 100u
#define BOID_START_COUNT 10000u
#define BOID_STEP 5000u
// batch and instance capacity are fixed at renderer init, so the flock has a ceiling
#define BOID_MAX 1000000u
#define BOID_MIN_FPS 30.0
#define BOID_INSTANCE_CAPACITY (BOID_MAX + 1024u)

// 1: the fish skip ecs_renderer and draw through instance batches packed by boid_draw_sys
#ifndef BOIDS_DIRECT_DRAW
#define BOIDS_DIRECT_DRAW 1
#endif

const HzAppConfig hz_app_config = {
    .window_title = "Boids",
    .vsync = HZ_VSYNC_OFF,
};

ECS_COMPONENT_TYPEDEF(BoidPosition);
ECS_COMPONENT_TYPEDEF(BoidHeading);
ECS_COMPONENT_TYPEDEF(BoidAnimPlayer);
ECS_COMPONENT_TYPEDEF(BoidSimConfig);
ARRAY_TYPEDEF_ALL(BoidBucket);
ECS_ARRAY(BoidBucket);

#if BOIDS_DIRECT_DRAW
// one batch per fish submesh; batch slot i is fish query row i
typedef struct BoidDirectDraw {
  RendererState *renderer;
  HzHandleT(RendererInstanceBatch) *batches;
  u32 batch_count;
} BoidDirectDraw;
ECS_COMPONENT_TYPEDEF(BoidDirectDraw);
static_assert(sizeof(WorldTransform) == sizeof(Transform), "batch writes read WorldTransform as Transform");
#endif

typedef struct {
  EcsWorld *world;
  RendererState *renderer;
  AssetSystem *assets;
  UISystem *ui;
  SceneSystem scene_sys;
  SceneHandle scene;
  EcsQuery *camera_query;
  EcsQuery *settings_query;
  EcsQuery *school_query;
  EcsQuery *target_query;
  EcsQuery *obstacle_query;
  EcsQuery *boid_query;
  FlyCameraCtrl fly;
  b32 fly_seeded;
  EcsEditorDefault editor;
  b32 editor_enabled;
  u32 spawned_total;
  b32 flock_spawned;
  i32 pending_delta; // fish to add (+) or remove (-) at the next frame's main-lane block
  Random spawn_rng;
  UIInteractAnim add_anim, remove_anim;
  b32 anim_wired;
  BoidBucket *buckets;
  Array(BoidBucket) buckets_arr;
  BoidSimConfig config;
#if BOIDS_DIRECT_DRAW
  HzHandleT(ModelBlobAsset) fish_model;
  BoidDirectDraw direct;
#endif
  // HUD timings: ring of the last HUD_AVG_FRAMES samples, main lane only
  f64 hud_ecs[HUD_AVG_FRAMES], hud_cpu[HUD_AVG_FRAMES], hud_frame[HUD_AVG_FRAMES];
  u32 hud_idx, hud_filled;
  f64 hud_ecs_avg, hud_cpu_avg, hud_frame_avg;
} AppState;
HZPTR_TYPEDEF(AppState);

hz_internal f32 boid_clip_duration(const BoidAnimClipBlobAsset *clip) {
  return clip->sample_rate * (f32)(clip->frame_count - 1);
}

hz_internal f32 boid_clip_frame(const BoidAnimClipBlobAsset *clip, f32 time, i32 *frame0, i32 *frame1) {
  f32 frame_f = time / clip->sample_rate;
  *frame0 = (i32)frame_f;
  *frame1 = *frame0 + 1 < clip->frame_count ? *frame0 + 1 : clip->frame_count - 1;
  return frame_f - (f32)*frame0;
}

// sim-driven roots write WorldTransform: transform propagation only follows LocalTransform on parented entities
ecs_system(boid_anim_player_sys, EcsInOut(BoidAnimPlayer), EcsInOut(WorldTransform),
           EcsResIn(AssetSystem)) {
  f32 dt = it->delta_time > 0.05f ? 0.05f : it->delta_time;
  const AssetSystem *assets = ecs_sys_get_resource(AssetSystem);
  for (i32 i = 0; i < it->count; i++) {
    BoidAnimPlayer *player = ecs_sys_get_component_mut(BoidAnimPlayer, i);
    WorldTransform *xf = ecs_sys_get_component_mut(WorldTransform, i);
    BoidAnimClipBlobAsset *clip = asset_get(assets, player->clip, BoidAnimClipBlobAsset);
    if (!clip || clip->frame_count < 2) continue;

    f32 duration = boid_clip_duration(clip);
    player->current_time += dt;
    while (player->current_time >= duration) player->current_time -= duration;

    i32 f0, f1;
    f32 t = boid_clip_frame(clip, player->current_time, &f0, &f1);
    Array(vec3) positions = blob_array_get(vec3, &clip->positions);
    Array(vec4) rotations = blob_array_get(vec4, &clip->rotations);
    // clips are baked in Unity space, 180 degrees about Y from engine space (the exporter mirrors X)
    v3 raw = v3_lerp(positions.items[f0], positions.items[f1], t);
    v3 p = {-raw.x, raw.y, -raw.z};
    xf->position = p;
    if (rotations.len == positions.len) {
      vec4 r0 = rotations.items[f0], r1 = rotations.items[f1];
      xf->rotation = quat_slerp((quat){-r0.x, r0.y, -r0.z, r0.w}, (quat){-r1.x, r1.y, -r1.z, r1.w}, t);
    }
    player->dest_position[0] = p.x;
    player->dest_position[1] = p.y;
    player->dest_position[2] = p.z;
  }
}

ecs_system(boid_insert_sys, EcsIn(BoidPosition), EcsIn(BoidHeading), EcsWith(BoidTag),
           EcsArrayInOut(Array(BoidBucket))) {
  Array(BoidBucket) *barr = ecs_sys_get_array_mut(Array(BoidBucket));
  for (i32 i = 0; i < it->count; i++) {
    const BoidPosition *p = ecs_sys_get_component(BoidPosition, i);
    const BoidHeading *h = ecs_sys_get_component(BoidHeading, i);
    u32 hash = spatial_hash_3f(p->x, p->y, p->z, CELL_SIZE) % GRID_SIZE;
    u32 slot = atomic_u32_inc_eval(&barr->items[hash].count) - 1;
    if (slot < MAX_PER_BUCKET) {
      BoidBucketEntry *entry = &barr->items[hash].entries[slot];
      entry->px = p->x; entry->py = p->y; entry->pz = p->z;
      entry->hx = h->x; entry->hy = h->y; entry->hz = h->z;
    }
  }
}

ecs_system_array(boid_merge_cells_sys, EcsArrayInOut(Array(BoidBucket)), EcsResIn(BoidSimConfig)) {
  Array(BoidBucket) *barr = ecs_sys_get_array_mut(Array(BoidBucket));
  const BoidSimConfig *config = ecs_sys_get_resource(BoidSimConfig);
  for (i32 i = 0; i < it->count; i++) {
    BoidBucket *bucket = &barr->items[it->offset + i];
    if (bucket->count == 0) continue;
    u32 count = bucket->count > MAX_PER_BUCKET ? MAX_PER_BUCKET : bucket->count;

    f32 sum_ax = 0, sum_ay = 0, sum_az = 0, sum_sx = 0, sum_sy = 0, sum_sz = 0;
    for (u32 j = 0; j < count; j++) {
      BoidBucketEntry *entry = &bucket->entries[j];
      sum_ax += entry->hx; sum_ay += entry->hy; sum_az += entry->hz;
      sum_sx += entry->px; sum_sy += entry->py; sum_sz += entry->pz;
    }
    bucket->sum_align_x = sum_ax; bucket->sum_align_y = sum_ay; bucket->sum_align_z = sum_az;
    bucket->sum_sep_x = sum_sx; bucket->sum_sep_y = sum_sy; bucket->sum_sep_z = sum_sz;

    f32 fx = bucket->entries[0].px, fy = bucket->entries[0].py, fz = bucket->entries[0].pz;
    f32 best_target = 1e18f;
    i32 target_idx = 0;
    for (i32 t = 0; t < NUM_TARGETS; t++) {
      f32 dx = config->target_positions[t][0] - fx, dy = config->target_positions[t][1] - fy,
          dz = config->target_positions[t][2] - fz;
      f32 d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < best_target) { best_target = d2; target_idx = t; }
    }
    bucket->nearest_target_idx = target_idx;

    f32 best_obstacle = 1e18f;
    i32 obstacle_idx = 0;
    for (i32 o = 0; o < NUM_OBSTACLES; o++) {
      f32 dx = config->obstacle_positions[o][0] - fx, dy = config->obstacle_positions[o][1] - fy,
          dz = config->obstacle_positions[o][2] - fz;
      f32 d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < best_obstacle) { best_obstacle = d2; obstacle_idx = o; }
    }
    bucket->nearest_obstacle_idx = obstacle_idx;
    bucket->nearest_obstacle_dist = m_sqrtf(best_obstacle);
  }
}

ecs_system(boid_steer_sys, EcsInOut(BoidPosition), EcsInOut(BoidHeading), EcsInOut(WorldTransform),
           EcsWith(BoidTag), EcsArrayIn(Array(BoidBucket)), EcsResIn(BoidSimConfig)) {
  const Array(BoidBucket) *barr = ecs_sys_get_array(Array(BoidBucket));
  const BoidSimConfig *config = ecs_sys_get_resource(BoidSimConfig);
  f32 dt = it->delta_time > 0.05f ? 0.05f : it->delta_time;

  for (i32 i = 0; i < it->count; i++) {
    BoidPosition *pos = ecs_sys_get_component_mut(BoidPosition, i);
    BoidHeading *head = ecs_sys_get_component_mut(BoidHeading, i);
    WorldTransform *xf = ecs_sys_get_component_mut(WorldTransform, i);
    f32 px = pos->x, py = pos->y, pz = pos->z;
    f32 fx = head->x, fy = head->y, fz = head->z;

    const BoidBucket *bucket = &barr->items[spatial_hash_3f(px, py, pz, CELL_SIZE) % GRID_SIZE];
    u32 count = bucket->count > MAX_PER_BUCKET ? MAX_PER_BUCKET : bucket->count;
    f32 ax = fx, ay = fy, az = fz, sx = px, sy = py, sz = pz;
    i32 ncount = 1;
    if (count > 0) {
      ncount = (i32)count;
      ax = bucket->sum_align_x; ay = bucket->sum_align_y; az = bucket->sum_align_z;
      sx = bucket->sum_sep_x; sy = bucket->sum_sep_y; sz = bucket->sum_sep_z;
    }
    f32 inv_count = 1.0f / (f32)ncount;

    const f32 *obs = config->obstacle_positions[bucket->nearest_obstacle_idx];
    const f32 *tgt = config->target_positions[bucket->nearest_target_idx];

    f32 a_dx = ax * inv_count - fx, a_dy = ay * inv_count - fy, a_dz = az * inv_count - fz;
    f32 a_len = m_sqrtf(a_dx * a_dx + a_dy * a_dy + a_dz * a_dz);
    f32 ar_x = 0, ar_y = 0, ar_z = 0;
    if (a_len > 0.0001f) {
      f32 inv = BOID_ALIGNMENT_WEIGHT / a_len;
      ar_x = a_dx * inv; ar_y = a_dy * inv; ar_z = a_dz * inv;
    }

    f32 s_dx = px * (f32)ncount - sx, s_dy = py * (f32)ncount - sy, s_dz = pz * (f32)ncount - sz;
    f32 s_len = m_sqrtf(s_dx * s_dx + s_dy * s_dy + s_dz * s_dz);
    f32 sr_x = 0, sr_y = 0, sr_z = 0;
    if (s_len > 0.0001f) {
      f32 inv = BOID_SEPARATION_WEIGHT / s_len;
      sr_x = s_dx * inv; sr_y = s_dy * inv; sr_z = s_dz * inv;
    }

    f32 t_dx = tgt[0] - px, t_dy = tgt[1] - py, t_dz = tgt[2] - pz;
    f32 t_len = m_sqrtf(t_dx * t_dx + t_dy * t_dy + t_dz * t_dz);
    f32 tr_x = 0, tr_y = 0, tr_z = 0;
    if (t_len > 0.0001f) {
      f32 inv = BOID_TARGET_WEIGHT / t_len;
      tr_x = t_dx * inv; tr_y = t_dy * inv; tr_z = t_dz * inv;
    }

    f32 o_dx = px - obs[0], o_dy = py - obs[1], o_dz = pz - obs[2];
    f32 o_len = m_sqrtf(o_dx * o_dx + o_dy * o_dy + o_dz * o_dz);
    f32 av_x = 0, av_y = 0, av_z = 0;
    if (o_len > 0.0001f) {
      f32 inv = 1.0f / o_len;
      av_x = (obs[0] + o_dx * inv * BOID_OBSTACLE_AVERSION_DISTANCE) - px;
      av_y = (obs[1] + o_dy * inv * BOID_OBSTACLE_AVERSION_DISTANCE) - py;
      av_z = (obs[2] + o_dz * inv * BOID_OBSTACLE_AVERSION_DISTANCE) - pz;
    }

    f32 nx = ar_x + sr_x + tr_x, ny = ar_y + sr_y + tr_y, nz = ar_z + sr_z + tr_z;
    f32 n_len = m_sqrtf(nx * nx + ny * ny + nz * nz);
    if (n_len > 0.0001f) {
      nx /= n_len; ny /= n_len; nz /= n_len;
    } else {
      nx = fx; ny = fy; nz = fz;
    }

    b32 avoid = bucket->nearest_obstacle_dist - BOID_OBSTACLE_AVERSION_DISTANCE < 0;
    f32 tfx = avoid ? av_x : nx, tfy = avoid ? av_y : ny, tfz = avoid ? av_z : nz;

    f32 hx = fx + dt * (tfx - fx), hy = fy + dt * (tfy - fy), hz = fz + dt * (tfz - fz);
    f32 h_len = m_sqrtf(hx * hx + hy * hy + hz * hz);
    if (h_len > 0.0001f) {
      hx /= h_len; hy /= h_len; hz /= h_len;
    }

    f32 move = BOID_MOVE_SPEED * dt;
    pos->x = px + hx * move;
    pos->y = py + hy * move;
    pos->z = pz + hz * move;
    head->x = hx; head->y = hy; head->z = hz;
    xf->position = (v3){pos->x, pos->y, pos->z};
    xf->rotation = quat_look((v3){hx, hy, hz}, V3_UP);
  }
}

#if BOIDS_DIRECT_DRAW
// fish rows are dense across the query, so each lane packs its rows straight into the batch slots
ecs_system(boid_draw_sys, EcsIn(WorldTransform), EcsWith(BoidTag), EcsResIn(BoidDirectDraw)) {
  const BoidDirectDraw *dd = ecs_sys_get_resource(BoidDirectDraw);
  const Transform *xf = (const Transform *)ecs_sys_get_component(WorldTransform, 0);
  u32 first = (u32)(it->frame_offset + it->offset);
  for (u32 b = 0; b < dd->batch_count; b++)
    renderer_instance_batch_write(dd->renderer, dd->batches[b], first, xf, (u32)it->count);
}

// main lane: once the fish model is resident, one batch per submesh, drawn as ecs_renderer would draw the fish
hz_internal void boids_direct_draw_create(AppState *state) {
  if (!asset_is_ready(state->assets, state->fish_model)) return;
  RendererState *r = state->renderer;
  const RendererModel *model = renderer_model(r, renderer_upload_model_asset_v2(r, state->assets, state->fish_model));
  u32 count = 0;
  for (u32 m = 0; m < model->meshes.len; m++) count += model->meshes.items[m].submeshes.len;
  state->direct.batches = ALLOC_ARRAY(&state->assets->allocator, HzHandleT(RendererInstanceBatch), count);
  for (u32 m = 0; m < model->meshes.len; m++) {
    const RendererMesh *mesh = &model->meshes.items[m];
    if (mesh->is_skinned) {
      log_error("boids: direct draw needs a static fish model");
      continue;
    }
    for (u32 s = 0; s < mesh->submeshes.len; s++) {
      HzHandleT(RendererInstanceBatch) b = renderer_instance_batch_create(
          r, &(RendererInstanceBatchDesc){.submesh = mesh->submeshes.items[s],
                                          .capacity = BOID_MAX,
                                          .velocity = renderer_gbuffer_active(r)});
      if (handle_is_valid(HANDLE_CAST(HzHandle, b))) state->direct.batches[state->direct.batch_count++] = b;
    }
  }
  state->direct.renderer = r;
  log_info("boids: direct draw through % instance batch(es)", fmt_u32(state->direct.batch_count));
}
#endif

// main lane, outside the frame scope: spawns `total` fish split across the loaded schools
hz_internal void boids_spawn(AppState *state, u32 total) {
  EcsWorld *w = state->world;
  i32 nschools = 0;
  ecs_query_foreach(state->school_query, sit) { nschools += sit.count; }
  if (nschools == 0 || total == 0) return;
  u32 per_school = total / (u32)nschools;

  u32 spawned = 0;
  ecs_query_foreach(state->school_query, sit) {
    const BoidSchool *schools = ecs_iter_get_component(&sit, BoidSchool, 0);
    const WorldTransform *xfs = ecs_iter_get_component(&sit, WorldTransform, 1);
    for (i32 s = 0; s < sit.count; s++) {
      HzHandleT(ModelBlobAsset) fish = asset_load(ModelBlobAsset, state->assets, .path = schools[s].model_path.value);
#if BOIDS_DIRECT_DRAW
      // batch slots follow query rows, which carry no model: every school must share one
      if (!handle_is_valid(HANDLE_CAST(HzHandle, state->fish_model))) state->fish_model = fish;
      if (!handle_equals(HANDLE_CAST(HzHandle, fish), HANDLE_CAST(HzHandle, state->fish_model))) {
        log_error("boids: direct draw needs one fish model; school % differs", fmt_i32(s));
        continue;
      }
#else
      BlobAssetRef(ModelBlobAsset) fish_ref = blob_asset_ref_from_handle(ModelBlobAsset, fish);
#endif
      v3 c = xfs[s].position;
      f32 r = schools[s].initial_radius;
      for (u32 i = 0; i < per_school; i++) {
        v3 dir = {random_f32_range(&state->spawn_rng, -0.5f, 0.5f), random_f32_range(&state->spawn_rng, -0.5f, 0.5f),
                  random_f32_range(&state->spawn_rng, -0.5f, 0.5f)};
        dir = v3_dot(dir, dir) > 1e-8f ? v3_normalize(dir) : V3_UP;
        v3 p = v3_add(c, v3_scale(dir, r));
        EcsEntity e = ecs_entity_create(w);
        ecs_component_set(w, e, BoidPosition, {.x = p.x, .y = p.y, .z = p.z});
        ecs_component_set(w, e, BoidHeading, {.x = dir.x, .y = dir.y, .z = dir.z});
        ecs_component_add(w, e, BoidTag);
        ecs_component_set(w, e, LocalTransform,
                          {.position = p, .rotation = quat_look(dir, V3_UP), .scale = {1.0f, 1.0f, 1.0f}});
#if !BOIDS_DIRECT_DRAW
        ecs_component_set(w, e, EcsModelRef, {.model = fish_ref});
        ecs_component_set(w, e, EcsVisibility, {.visible = true});
#endif
        spawned++;
      }
    }
  }
  state->spawned_total += spawned;
  state->flock_spawned = true;
  log_info("boids: % fish (+% across % school(s))", fmt_u32(state->spawned_total), fmt_u32(spawned),
           fmt_u32((u32)nschools));
}

// main lane, outside the frame scope: destroys up to `count` fish
hz_internal void boids_despawn(AppState *state, u32 count) {
  u32 removed = 0;
  ecs_query_foreach(state->boid_query, bit) {
    for (i32 i = 0; i < bit.count && removed < count; i++, removed++) ecs_entity_destroy(state->world, bit.entities[i]);
  }
  state->spawned_total -= removed;
  log_info("boids: % fish (-%)", fmt_u32(state->spawned_total), fmt_u32(removed));
}

// main lane: attaches a clip player to each target and obstacle, feeding the sim their positions
hz_internal void boids_wire_animation(AppState *state) {
  EcsWorld *w = state->world;
  i32 targets = 0, obstacles = 0;
  ecs_query_foreach(state->target_query, tit) {
    const SampledAnimationClip *clips = ecs_iter_get_component(&tit, SampledAnimationClip, 1);
    for (i32 i = 0; i < tit.count && targets < NUM_TARGETS; i++, targets++) {
      ecs_component_set(w, tit.entities[i], BoidAnimPlayer,
                        {.clip = asset_load(BoidAnimClipBlobAsset, state->assets, .path = clips[i].clip_path.value),
                         .dest_position = state->config.target_positions[targets]});
    }
  }
  ecs_query_foreach(state->obstacle_query, oit) {
    const SampledAnimationClip *clips = ecs_iter_get_component(&oit, SampledAnimationClip, 1);
    for (i32 i = 0; i < oit.count && obstacles < NUM_OBSTACLES; i++, obstacles++) {
      ecs_component_set(w, oit.entities[i], BoidAnimPlayer,
                        {.clip = asset_load(BoidAnimClipBlobAsset, state->assets, .path = clips[i].clip_path.value),
                         .dest_position = state->config.obstacle_positions[obstacles]});
    }
  }
  if (targets > 0 || obstacles > 0) {
    state->anim_wired = true;
    log_info("boids: % target(s), % obstacle(s) animated", fmt_i32(targets), fmt_i32(obstacles));
  }
}

HZ_APP_API void app_init(AppMemory *memory) {
  if (!is_main_thread()) return;
  AppState *state = memory->state;

  state->world = ecs_world_create_ex(&(EcsWorldInitDesc){.arena_reserved = GB(1), .cmd_buffer_arena_reserved = MB(512)});
  EcsWorld *w = state->world;
  ecs_resource_set_ref(w, AppMemory, memory);
  ecs_input_init(w);
  ecs_renderer_init(w, &(RendererInitDesc){
                           .sample_count = 4,
                           .vertex_buffer_size = MB(64),
                           .index_buffer_size = MB(16),
                           .max_batches = 2048,
                           .max_model_instances = BOID_INSTANCE_CAPACITY,
                           .max_submesh_instances = BOID_INSTANCE_CAPACITY,
                           .max_shadow_caster_instances = BOID_INSTANCE_CAPACITY,
#if BOIDS_DIRECT_DRAW
                           .max_batch_instances = BOID_MAX,
#endif
                           .shadows_enabled = true,
                           .shadow = {.cascade_size = 2048, .max_distance = 130.0f, .shadow_filter_radius_texels = 2},
                           .scene_color_copy_enabled = true,
                           .bloom_enabled = true,
                           .dof_enabled = true,
                       });
  ecs_transform_init(w, NULL);
  ecs_assets_init(w, &(AssetSystemCreateDesc){.max_inflight_file_ops = 512});

  state->renderer = ecs_resource_get_mut(w, RendererState);
  state->assets = ecs_resource_get_mut(w, AssetSystem);

  state->buckets = ALLOC_ARRAY(&state->assets->allocator, BoidBucket, GRID_SIZE);
  state->buckets_arr = arr_from_c_array(BoidBucket, state->buckets, GRID_SIZE);
  ecs_resource_set_ref(w, Array(BoidBucket), &state->buckets_arr);
  ecs_resource_set_ref(w, BoidSimConfig, &state->config);
#if BOIDS_DIRECT_DRAW
  ecs_resource_set_ref(w, BoidDirectDraw, &state->direct);
#endif

  scene_system_create(&state->scene_sys, w, state->assets, &state->assets->allocator);
  renderer_register_asset_unloaders(state->renderer, state->assets);
  state->camera_query = ecs_query_cached(w, EcsIn(Camera));
  state->settings_query = ecs_query_cached(w, EcsIn(EcsSceneSettings));
  state->school_query = ecs_query_cached(w, EcsIn(BoidSchool), EcsIn(WorldTransform));
  state->target_query = ecs_query_cached(w, EcsIn(BoidTarget), EcsIn(SampledAnimationClip));
  state->obstacle_query = ecs_query_cached(w, EcsIn(BoidObstacle), EcsIn(SampledAnimationClip));
  state->boid_query = ecs_query_cached(w, EcsIn(BoidPosition), EcsWith(BoidTag));
  state->scene = scene_load_file(&state->scene_sys, SCENE_FILE);

  state->ui = ui_create(&(UIInitDesc){.max_glyphs = 4096}, state->assets);
  ecs_resource_set_ref(w, UISystem, state->ui);
  ecs_editor_default_init(&state->editor, &(EcsEditorDefaultDesc){
                                              .world = w,
                                              .assets = state->assets,
                                              .ui = state->ui,
                                              .allocator = &state->assets->allocator,
                                              .canvas_width = memory->canvas_width,
                                              .canvas_height = memory->canvas_height,
                                          });
  log_info("boids demo initialized: loading %", fmt_cstr(SCENE_FILE));
#ifndef WASM
  os_set_fullscreen(true);
#endif
}

hz_internal HzUIString hud_str(StringBuilder *sb) {
  return (HzUIString){.length = (i32)sb_length(sb), .chars = sb_get(sb)};
}

// main lane; shows the averages of previous frames, then folds in this frame's samples
hz_internal void boids_hud_draw(AppState *state, f64 ecs_ms, f64 cpu_ms, f64 frame_ms) {
  // non-scoped: clay reads the strings at end of frame
  Allocator ta = tctx_temp_allocator(NULL);
  u32 fps = state->hud_frame_avg > 0.001 ? (u32)(1000.0 / state->hud_frame_avg) : 0;
  StringBuilder ecs_sb = sb_create(32, &ta), cpu_sb = sb_create(32, &ta), frame_sb = sb_create(32, &ta),
                fps_sb = sb_create(32, &ta);
  sb_append_format(&ecs_sb, "ECS: % ms", fmt_f32_p((f32)state->hud_ecs_avg, 2));
  sb_append_format(&cpu_sb, "CPU: % ms", fmt_f32_p((f32)state->hud_cpu_avg, 2));
  sb_append_format(&frame_sb, "Frame: % ms", fmt_f32_p((f32)state->hud_frame_avg, 2));
  sb_append_format(&fps_sb, "FPS: %", fmt_u32(fps));
  OsSafeAreaInsets safe = os_get_safe_area();
  ui_element({
      .id = ui_id("BoidsStats"),
      .floating = {.attach_to = UI_ATTACH_TO_ROOT,
                   .attach_points = {.element = UI_ATTACH_POINT_LEFT_TOP, .parent = UI_ATTACH_POINT_LEFT_TOP},
                   .offset = {safe.left + 16, safe.top + 16}},
      .layout = {.sizing = {.width = ui_sizing_fit(0), .height = ui_sizing_fit(0)},
                 .layout_direction = UI_TOP_TO_BOTTOM,
                 .padding = ui_padding_all(14),
                 .child_gap = 4},
      .background_color = {0, 0, 0, 250},
      .corner_radius = ui_corner_radius(6),
      .border = {.color = {255, 255, 255, 40}, .width = ui_border_outside(1)},
  }) {
    ui_text(hud_str(&ecs_sb),
            ui_text_config({.fontSize = 26, .weight = UI_FONT_SEMIBOLD, .textColor = {144, 238, 144, 255}}));
    HzUITextConfig white = ui_text_config({.fontSize = 26, .weight = UI_FONT_SEMIBOLD, .textColor = {255, 255, 255, 255}});
    ui_text(hud_str(&cpu_sb), white);
    ui_text(hud_str(&frame_sb), white);
    ui_text(hud_str(&fps_sb), white);
  }

  u32 k = state->hud_idx;
  state->hud_ecs[k] = ecs_ms;
  state->hud_cpu[k] = cpu_ms;
  state->hud_frame[k] = frame_ms;
  state->hud_idx = (k + 1) % HUD_AVG_FRAMES;
  if (state->hud_filled < HUD_AVG_FRAMES) state->hud_filled++;
  f64 ecs = 0, cpu = 0, frame = 0;
  for (u32 i = 0; i < state->hud_filled; i++) {
    ecs += state->hud_ecs[i];
    cpu += state->hud_cpu[i];
    frame += state->hud_frame[i];
  }
  state->hud_ecs_avg = ecs / (f64)state->hud_filled;
  state->hud_cpu_avg = cpu / (f64)state->hud_filled;
  state->hud_frame_avg = frame / (f64)state->hud_filled;
}

hz_internal b32 boids_button(AppState *state, const InputSystem *input, HzUIElementId id, CString label,
                             f32 width, b32 enabled, UIInteractAnim *anim) {
  HzUIColor base = enabled ? (HzUIColor){40, 40, 40, 250} : (HzUIColor){20, 20, 20, 250};
  HzUIColor bg = ui_color_lerp(ui_color_lerp(base, (HzUIColor){70, 70, 70, 250}, enabled ? anim->hover : 0.0f),
                               (HzUIColor){25, 25, 25, 250}, enabled ? anim->press : 0.0f);
  b32 clicked = false;
  ui_element({
      .id = id,
      .layout = {.sizing = {.width = ui_sizing_fixed(width), .height = ui_sizing_fixed(44)},
                 .child_alignment = {.x = UI_ALIGN_X_CENTER, .y = UI_ALIGN_Y_CENTER}},
      .background_color = bg,
      .corner_radius = ui_corner_radius(6),
      .border = {.color = {255, 255, 255, 40}, .width = ui_border_outside(1)},
  }) {
    *anim = ui_interact_anim(state->ui, id, input);
    clicked = enabled && ui_hovered() && input->buttons[MOUSE_LEFT].released_this_frame;
    ui_text(((HzUIString){.length = (i32)cstr_len(label), .chars = label}),
            ui_text_config({.fontSize = 22,
                            .weight = UI_FONT_SEMIBOLD,
                            .textColor = enabled ? (HzUIColor){255, 255, 255, 255} : (HzUIColor){255, 255, 255, 90}}));
  }
  return clicked;
}

// main lane: fish count controls; the change lands at the next frame's main-lane block
hz_internal void boids_controls_draw(AppState *state, const InputSystem *input) {
  Allocator ta = tctx_temp_allocator(NULL);
  f64 fps = state->hud_frame_avg > 0.001 ? 1000.0 / state->hud_frame_avg : 0.0;
  b32 fps_capped = state->hud_filled > 0 && fps < BOID_MIN_FPS;
  b32 can_add = !fps_capped && state->spawned_total + BOID_STEP <= BOID_MAX;
  b32 can_remove = state->spawned_total >= BOID_STEP;
  StringBuilder count_sb = sb_create(32, &ta);
  sb_append_format(&count_sb, "Fish: %", fmt_u32(state->spawned_total));
  OsSafeAreaInsets safe = os_get_safe_area();
  // narrow screens: the bar spans the width between the margins instead of overflowing it
  f32 avail = (f32)os_get_screen_desc().width - safe.left - safe.right - 2.0f * 16.0f;
  b32 compact = avail < 14.0f * 2 + 84.0f * 2 + 14.0f * 2 + 200.0f;
  f32 button_w = compact ? 72.0f : 84.0f;
  f32 gap = compact ? 10.0f : 14.0f;
  f32 count_w = compact ? m_maxf(avail - 14.0f * 2 - button_w * 2 - gap * 2, 0.0f) : 200.0f;
  ui_element({
      .id = ui_id("BoidsControls"),
      .floating = {.attach_to = UI_ATTACH_TO_ROOT,
                   .attach_points = {.element = UI_ATTACH_POINT_CENTER_BOTTOM, .parent = UI_ATTACH_POINT_CENTER_BOTTOM},
                   .offset = {0, -(safe.bottom + 16)}},
      .layout = {.sizing = {.width = ui_sizing_fit(0), .height = ui_sizing_fit(0)},
                 .layout_direction = UI_TOP_TO_BOTTOM,
                 .padding = ui_padding_all(14),
                 .child_gap = 6,
                 .child_alignment = {.x = UI_ALIGN_X_CENTER}},
      .background_color = {0, 0, 0, 250},
      .corner_radius = ui_corner_radius(6),
      .border = {.color = {255, 255, 255, 40}, .width = ui_border_outside(1)},
  }) {
    ui_element({
        .id = ui_id("BoidsControlsRow"),
        .layout = {.sizing = {.width = ui_sizing_fit(0), .height = ui_sizing_fit(0)},
                   .child_gap = (u16)gap,
                   .child_alignment = {.y = UI_ALIGN_Y_CENTER}},
    }) {
      if (boids_button(state, input, ui_id("FishRemove"), "-5k", button_w, can_remove, &state->remove_anim))
        state->pending_delta = -(i32)BOID_STEP;
      ui_element({.id = ui_id("FishCount"),
                  .layout = {.sizing = {.width = ui_sizing_fixed(count_w), .height = ui_sizing_fit(0)},
                             .child_alignment = {.x = UI_ALIGN_X_CENTER}}}) {
        ui_text(hud_str(&count_sb),
                ui_text_config({.fontSize = 26, .weight = UI_FONT_SEMIBOLD, .textColor = {255, 255, 255, 255}}));
      }
      if (boids_button(state, input, ui_id("FishAdd"), "+5k", button_w, can_add, &state->add_anim))
        state->pending_delta = (i32)BOID_STEP;
    }
    if (fps_capped) {
      ui_text(ui_string("Below 30 FPS: adding is capped"),
              ui_text_config({.fontSize = 16, .textColor = {255, 170, 120, 255}}));
    }
  }
}

HZ_APP_API void app_update_and_render(AppMemory *memory) {
  AppState *state = memory->state;
  EcsWorld *w = state->world;
  // collective: every lane, every frame, before any main-thread-only work
  ui_font_update(state->ui);

  if (is_main_thread()) {
    Camera *cam = ecs_query_get_singleton(Camera, state->camera_query);
    if (cam) {
      if (!state->fly_seeded) {
        flycam_sync_from_camera(&state->fly, cam);
        state->fly.move_speed = 20.0f;
        state->fly_seeded = true;
      } else if (!state->editor_enabled) {
        flycam_update(&state->fly, cam, ecs_resource_get_mut(w, InputSystem), memory->dt);
      }
    }

    // the schools arrive with the scene; respawn is a no-op until they do
    if (!state->flock_spawned) {
      state->spawn_rng = random_create(0x9F6ABC1u);
      boids_spawn(state, BOID_START_COUNT);
    } else if (state->pending_delta > 0) {
      boids_spawn(state, (u32)state->pending_delta);
    } else if (state->pending_delta < 0) {
      boids_despawn(state, (u32)(-state->pending_delta));
    }
    state->pending_delta = 0;
    if (!state->anim_wired) boids_wire_animation(state);
#if BOIDS_DIRECT_DRAW
    if (state->spawned_total > 0 && !state->direct.renderer) boids_direct_draw_create(state);
#endif
  }
  lane_sync();

  b32 simulate = state->spawned_total > 0;
  if (simulate) {
    Range(u32) range = lane_range(GRID_SIZE);
    for (u32 i = range.min; i < range.max; i++) state->buckets[i].count = 0;
  }
  lane_sync();

  u64 t_ecs_start = os_time_now();
  ecs_input_schedule(w);
  ecs_assets_schedule(w);
  if (simulate) {
    ecs_schedule(w, boid_anim_player_sys);
    ecs_schedule_unsafe_resource_access_balanced(w, boid_insert_sys);
    ecs_schedule_array(w, boid_merge_cells_sys);
    ecs_schedule_balanced(w, boid_steer_sys);
#if BOIDS_DIRECT_DRAW
    if (state->direct.batch_count > 0) ecs_schedule(w, boid_draw_sys);
#endif
  }
  // the flycam owns camera placement; transform sync would overwrite it
  ecs_camera_schedule(w, &(EcsCameraScheduleDesc){.transform_sync_off = true});
  ecs_transform_schedule(w);
  ecs_renderer_schedule(w);
  ecs_flush(w, memory->dt);
  u64 t_ecs_end = os_time_now();
#if BOIDS_DIRECT_DRAW
  if (is_main_thread() && simulate) {
    for (u32 b = 0; b < state->direct.batch_count; b++)
      renderer_instance_batch_submit(state->renderer, state->direct.batches[b], state->spawned_total);
  }
  // the submits land before renderer_update's phase 6 on every lane
  lane_sync();
#endif

  Camera *cam = ecs_query_get_singleton(Camera, state->camera_query);
  const EcsSceneSettings *settings = ecs_query_get_singleton(EcsSceneSettings, state->settings_query);
  ColorF32 clear = settings ? settings->clear_color : (ColorF32){0, 0, 0, 1};
  clear.a = 1.0f;
  if (cam) {
    renderer_update(state->renderer,
                    &(RendererBeginFrameDesc){.camera = cam, .clear_color = clear, .time = memory->total_time});
  }
  u64 t_cpu_end = os_time_now();

  if (is_main_thread()) {
    InputSystem *input = ecs_resource_get_mut(w, InputSystem);
    ui_begin_frame(state->ui, input, memory->dt);
    if (input->buttons[KEY_F1].pressed_this_frame) state->editor_enabled = !state->editor_enabled;
    if (cam && state->editor_enabled) {
      ecs_editor_default_update(&state->editor, &(EcsEditorDefaultUpdateDesc){
                                                    .camera = cam,
                                                    .canvas_width = memory->canvas_width,
                                                    .canvas_height = memory->canvas_height,
                                                });
    }
    boids_controls_draw(state, input);
    boids_hud_draw(state, os_ticks_to_ms(os_time_diff(t_ecs_end, t_ecs_start)),
                   os_ticks_to_ms(os_time_diff(t_cpu_end, t_ecs_start)), (f64)memory->dt * 1000.0);
    ui_end_frame(state->ui);
  }
}

HZ_APP_API size_t app_state_size(void) { return sizeof(AppState); }
