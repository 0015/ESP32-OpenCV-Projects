/*
 * cube_solve — table-free layer-by-layer solver, sized for ESP32-S3.
 * Produces a simplified move sequence in the same U/R/F/D/L/B frame the cube
 * was scanned in (no whole-cube rotations). Each move is tagged with the
 * beginner-method stage it belongs to, so the UI can show learning context.
 */
#pragma once
#include "cube_model.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CUBE_MAX_MOVES 512

/* Beginner-method stages, in solve order */
enum {
    CUBE_ST_CROSS = 1,     /* bottom cross                     */
    CUBE_ST_CORNERS,       /* bottom corners (first layer)     */
    CUBE_ST_MIDDLE,        /* middle-layer edges               */
    CUBE_ST_TOPCROSS,      /* top cross (edge orientation)     */
    CUBE_ST_TOPTWIST,      /* top corners: orient (2-look OLL) */
    CUBE_ST_TOPCORNER,     /* top corners: position (A-perms)  */
    CUBE_ST_TOPEDGE,       /* top edges: position (U-perms)    */
    CUBE_ST_COUNT = 7,
};

const char *cube_stage_name(int stage);   /* "Bottom cross" ... */

typedef struct {
    uint8_t moves[CUBE_MAX_MOVES];
    uint8_t stage[CUBE_MAX_MOVES];        /* CUBE_ST_* per move */
    int     count;
} cube_solution_t;

/* Returns 0 on success; negative stage number on internal failure
 * (input should be validated with cube_validate() first). */
int cube_solve(const cube_t *start, cube_solution_t *sol);

/* In-place move-sequence simplification (merge same-face turns, including
 * across an interleaved opposite-face move). stage may be NULL; when given,
 * it is kept parallel to mv (merges keep the earlier stage tag).
 * Returns new length. */
int cube_simplify(uint8_t *mv, uint8_t *stage, int n);

#ifdef __cplusplus
}
#endif
