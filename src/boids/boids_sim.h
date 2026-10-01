#pragma once
#include "lib/typedefs.h"
#include "lib/reflect.h"
#include "lib/hzmath.h"
#include "lib/blob_asset.h"
#include "lib/string.h"
#include "hz_engine_defines.h"
#include "ecs/ecs.h"

#define MAX_PER_BUCKET 256
#define NUM_BOIDS 1000000
#define NUM_TARGETS 2
#define NUM_OBSTACLES 1
#define GRID_SIZE 8192
#define CELL_SIZE 8.0f

#define BOID_SEPARATION_WEIGHT 1.0f
#define BOID_ALIGNMENT_WEIGHT 1.0f
#define BOID_TARGET_WEIGHT 2.0f
#define BOID_OBSTACLE_AVERSION_DISTANCE 30.0f
#define BOID_MOVE_SPEED 25.0f

typedef struct BoidBucketEntry {
  f32 px, py, pz;
  f32 hx, hy, hz;
} BoidBucketEntry;

typedef struct BoidBucket {
  u32 count;
  f32 sum_align_x, sum_align_y, sum_align_z;
  f32 sum_sep_x, sum_sep_y, sum_sep_z;
  i32 nearest_target_idx;
  i32 nearest_obstacle_idx;
  f32 nearest_obstacle_dist;
  BoidBucketEntry entries[MAX_PER_BUCKET];
} BoidBucket;

HZ_STRUCT_TYPEDEF(BoidAnimClipBlobAsset) {
  BlobAssetHeader header;
  f32 sample_rate;
  i32 frame_count;
  BlobArray(vec3) positions;
  BlobArray(vec4) rotations;
};

// scene-loaded components: field names must match HZExportCommands.AppendBoidComponents
ECS_COMPONENT_TYPEDEF(BoidSchool);
HZ_REFLECT()
typedef struct BoidSchool {
  u32 count;
  f32 initial_radius;
  f32 cell_radius;
  f32 separation_weight;
  f32 alignment_weight;
  f32 target_weight;
  f32 obstacle_aversion_distance;
  f32 move_speed;
  String128Bytes model_path;
} BoidSchool;

// Tag: this entity is a flock target (red fish). Animated via SampledAnimationClip.
ECS_COMPONENT_TYPEDEF(BoidTarget);
HZ_REFLECT()
typedef struct BoidTarget {
  u8 _;
} BoidTarget;

// Tag: this entity is a flock obstacle (shark). Animated via SampledAnimationClip.
ECS_COMPONENT_TYPEDEF(BoidObstacle);
HZ_REFLECT()
typedef struct BoidObstacle {
  u8 _;
} BoidObstacle;

// baked transform clip the demo resolves and plays on the entity
ECS_COMPONENT_TYPEDEF(SampledAnimationClip);
HZ_REFLECT()
typedef struct SampledAnimationClip {
  String128Bytes clip_path;
} SampledAnimationClip;

typedef struct BoidSimConfig {
  f32 target_positions[NUM_TARGETS][3];
  f32 obstacle_positions[NUM_OBSTACLES][3];
} BoidSimConfig;

typedef struct BoidPosition {
  f32 x, y, z;
} BoidPosition;

typedef struct BoidHeading {
  f32 x, y, z;
} BoidHeading;

typedef struct BoidTag {
  u8 _;
} BoidTag;

typedef struct BoidAnimPlayer {
  HzHandleT(BoidAnimClipBlobAsset) clip;
  f32 current_time;
  b32 remapped; // scene-space remap applied once on first-ready
  f32 *dest_position;
} BoidAnimPlayer;
