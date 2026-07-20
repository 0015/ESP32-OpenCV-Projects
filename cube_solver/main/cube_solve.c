/*
 * cube_solve — beginner layer-by-layer solver (no lookup tables, no cube
 * rotations, no slice moves). Stages:
 *   1. bottom cross        (budgeted IDDFS per edge, case-logic fallback)
 *   2. bottom corners      (lift + budgeted {U,A,B} IDDFS, sexy-move fallback)
 *   3. middle-layer edges  (standard left/right insertion algorithms)
 *   4. top cross           (F R U R' U' F' with case positioning)
 *   5. top corner twist    (2-look OLL: IDDFS over {U, Sune, AntiSune})
 *   6. top corner position (IDDFS over {U, U^k A-perm U^-k})
 *   7. top edge position   (IDDFS over {U^k U-perm U^-k})
 *
 * Every stage preserves what previous stages solved. The per-piece searches
 * keep the human-followable stage structure but cut ~25-30% of the moves the
 * pure textbook algorithms would emit. Verified on the host with tens of
 * thousands of random scrambles (see test/host).
 */
#include "cube_solve.h"
#include <string.h>

/* ---- move emission context ------------------------------------------------*/
#define RAW_MAX 900

typedef struct {
    cube_t  c;
    uint8_t out[RAW_MAX];
    uint8_t stg[RAW_MAX];
    int     n;
    int     fail;          /* stage number that failed, 0 = ok */
    int     stage;         /* current CUBE_ST_* tag             */
} sctx_t;

static void emit(sctx_t *s, int face, int turns)
{
    turns &= 3;
    if (!turns || s->fail) return;
    uint8_t m = cube_mv(face, turns);
    cube_move(&s->c, m);
    if (s->n < RAW_MAX) {
        s->stg[s->n] = (uint8_t)s->stage;
        s->out[s->n++] = m;
    } else {
        s->fail = 99;
    }
}

const char *cube_stage_name(int stage)
{
    static const char *names[CUBE_ST_COUNT + 1] = {
        "?",
        "Bottom cross",
        "Bottom corners",
        "Middle layer",
        "Top cross",
        "Top corners: twist",
        "Top corners: place",
        "Top edges",
    };
    return (stage >= 1 && stage <= CUBE_ST_COUNT) ? names[stage] : "?";
}

/* ---- side-face ring (order U-CW: F -> L -> B -> R) ------------------------*/
static const uint8_t ring[4] = { CF_F, CF_L, CF_B, CF_R };

static int ringpos(int face)
{
    for (int i = 0; i < 4; i++)
        if (ring[i] == face) return i;
    return -1;
}
static int right_of(int f) { return ring[(ringpos(f) + 3) & 3]; }
static int left_of(int f)  { return ring[(ringpos(f) + 1) & 3]; }

/* slots indexed by ring position of their primary side face */
static const uint8_t uslot_by_ring[4]   = { E_UF, E_UL, E_UB, E_UR };
static const uint8_t dslot_by_ring[4]   = { E_DF, E_DL, E_DB, E_DR };
/* middle slot whose face pair is (P, right(P)), P = ring[pos] */
static const uint8_t midslot_by_ring[4] = { E_FR, E_FL, E_BL, E_BR };
/* U/D corner slot whose side pair is (A, right(A)), A = ring[pos] */
static const uint8_t ucorner_by_ring[4] = { C_URF, C_UFL, C_ULB, C_UBR };
static const uint8_t dcorner_by_ring[4] = { C_DFR, C_DLF, C_DBL, C_DRB };

/* side face touched by a U/D edge slot */
static int slot_side(int slot)
{
    switch (slot) {
    case E_UR: case E_DR: return CF_R;
    case E_UF: case E_DF: return CF_F;
    case E_UL: case E_DL: return CF_L;
    case E_UB: case E_DB: return CF_B;
    }
    return -1;
}

/* ---- piece lookup ----------------------------------------------------------*/
static int find_edge(const cube_t *c, int a, int b, int *ori)
{
    for (int s = 0; s < 12; s++) {
        uint8_t c0 = c->f[cube_ef[s][0]], c1 = c->f[cube_ef[s][1]];
        if (c0 == a && c1 == b) { *ori = 0; return s; }
        if (c0 == b && c1 == a) { *ori = 1; return s; }
    }
    return -1;
}

static int find_corner(const cube_t *c, int a, int b, int cc, int *dpos)
{
    for (int s = 0; s < 8; s++) {
        uint8_t x = c->f[cube_cf[s][0]], y = c->f[cube_cf[s][1]], z = c->f[cube_cf[s][2]];
        int has_a = (x == a) + (y == a) + (z == a);
        int has_b = (x == b) + (y == b) + (z == b);
        int has_c = (x == cc) + (y == cc) + (z == cc);
        if (has_a == 1 && has_b == 1 && has_c == 1) {
            *dpos = (x == a) ? 0 : (y == a) ? 1 : 2;
            return s;
        }
    }
    return -1;
}

static int edge_home_ok(const cube_t *c, int slot)
{
    return c->f[cube_ef[slot][0]] == cube_ef[slot][0] / 9 &&
           c->f[cube_ef[slot][1]] == cube_ef[slot][1] / 9;
}

static int corner_home_ok(const cube_t *c, int slot)
{
    return c->f[cube_cf[slot][0]] == cube_cf[slot][0] / 9 &&
           c->f[cube_cf[slot][1]] == cube_cf[slot][1] / 9 &&
           c->f[cube_cf[slot][2]] == cube_cf[slot][2] / 9;
}

/* ---- budgeted IDDFS over raw moves ------------------------------------------
 * Finds a shortest move sequence (from the allowed set) satisfying pred while
 * staying inside a node budget. Returns length >= 0 and fills path, or -1
 * (no solution within maxd / budget exhausted) — caller falls back to the
 * deterministic textbook logic, so this is purely an optimization.
 */
#define SEARCH_MAX_DEPTH 8

typedef int (*pred_fn)(const cube_t *c, const void *arg);

static long s_nodes;

static int dfs_raw(const cube_t *c, int depth, const uint8_t *moves, int nm,
                   pred_fn pred, const void *arg, int last_face, uint8_t *path,
                   long budget)
{
    if (pred(c, arg)) return 0;
    if (depth == 0) return -1;
    for (int i = 0; i < nm; i++) {
        int f = moves[i] / 3;
        if (f == last_face) continue;
        /* commuting opposite faces: canonical order only */
        if (last_face >= 0 && f % 3 == last_face % 3 && f < last_face) continue;
        if (++s_nodes > budget) return -2;
        cube_t t = *c;
        cube_move(&t, moves[i]);
        int r = dfs_raw(&t, depth - 1, moves, nm, pred, arg, f, path + 1, budget);
        if (r >= 0) { *path = moves[i]; return r + 1; }
        if (r == -2) return -2;
    }
    return -1;
}

static int search_raw(sctx_t *s, const uint8_t *moves, int nm, int maxd,
                      pred_fn pred, const void *arg, long budget)
{
    uint8_t path[SEARCH_MAX_DEPTH];
    if (maxd > SEARCH_MAX_DEPTH) maxd = SEARCH_MAX_DEPTH;
    s_nodes = 0;
    for (int d = 0; d <= maxd; d++) {
        int r = dfs_raw(&s->c, d, moves, nm, pred, arg, -1, path, budget);
        if (r >= 0) {
            for (int i = 0; i < r; i++) emit(s, path[i] / 3, path[i] % 3 + 1);
            return 1;
        }
        if (r == -2) return 0;
    }
    return 0;
}

static const uint8_t all_moves[18] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17
};

/* ---- stage 1: bottom cross ---------------------------------------------------*/
typedef struct { uint8_t n; uint8_t faces[4]; } cross_arg_t;

static int cross_pred(const cube_t *c, const void *va)
{
    const cross_arg_t *a = (const cross_arg_t *)va;
    for (int i = 0; i < a->n; i++)
        if (!edge_home_ok(c, dslot_by_ring[ringpos(a->faces[i])])) return 0;
    return 1;
}

static void solve_cross_edge_cases(sctx_t *s, int X)
{
    int target = dslot_by_ring[ringpos(X)];
    for (int guard = 0; ; guard++) {
        if (guard > 8) { s->fail = 1; return; }
        int ori, slot = find_edge(&s->c, CF_D, X, &ori);
        if (slot < 0) { s->fail = 1; return; }

        if (slot == target && ori == 0) return;              /* solved */

        if (slot <= E_UB) {                                   /* U layer */
            int A = slot_side(slot);
            emit(s, CF_U, (ringpos(X) - ringpos(A)) & 3);     /* align above X */
            if (ori == 0) {                                   /* D color up */
                emit(s, X, 2);
            } else {                                          /* flipped */
                int Z = right_of(X);
                emit(s, X, 1); emit(s, Z, 1); emit(s, CF_U, 3); emit(s, Z, 3);
            }
        } else if (slot <= E_DB) {                            /* D layer, wrong/flipped */
            emit(s, slot_side(slot), 2);
        } else {                                              /* middle layer: lift */
            int lf, lt;                                       /* lift move face/turns */
            switch (slot) {
            case E_FR: lf = CF_R; lt = 1; break;
            case E_FL: lf = CF_L; lt = 3; break;
            case E_BL: lf = CF_L; lt = 1; break;
            default:   lf = CF_R; lt = 3; break;              /* E_BR */
            }
            emit(s, lf, lt); emit(s, CF_U, 1); emit(s, lf, 4 - lt);
        }
    }
}

static void solve_cross_edge(sctx_t *s, int p, cross_arg_t *arg)
{
    arg->faces[arg->n++] = ring[p];                           /* current + earlier */
    if (search_raw(s, all_moves, 18, 5, cross_pred, arg, 200000)) return;
    solve_cross_edge_cases(s, ring[p]);
}

/* ---- stage 2: bottom corners --------------------------------------------------*/
typedef struct { uint8_t n; uint8_t slots[4]; } corner_arg_t;

static int corner_pred(const cube_t *c, const void *va)
{
    const corner_arg_t *a = (const corner_arg_t *)va;
    for (int p = 0; p < 4; p++)                               /* whole cross intact */
        if (!edge_home_ok(c, dslot_by_ring[p])) return 0;
    for (int i = 0; i < a->n; i++)
        if (!corner_home_ok(c, a->slots[i])) return 0;
    return 1;
}

static void solve_d_corner(sctx_t *s, int A, corner_arg_t *arg)
{
    int B = right_of(A);
    int target = dcorner_by_ring[ringpos(A)];
    arg->slots[arg->n++] = (uint8_t)target;

    const uint8_t mvset[9] = {
        cube_mv(CF_U, 1), cube_mv(CF_U, 2), cube_mv(CF_U, 3),
        cube_mv(A, 1),    cube_mv(A, 2),    cube_mv(A, 3),
        cube_mv(B, 1),    cube_mv(B, 2),    cube_mv(B, 3),
    };

    for (int guard = 0; ; guard++) {
        if (guard > 10) { s->fail = 2; return; }
        int dpos, slot = find_corner(&s->c, CF_D, A, B, &dpos);
        if (slot < 0) { s->fail = 2; return; }

        if (slot == target && s->c.f[cube_cf[slot][0]] == CF_D) return;  /* solved */

        if (slot >= C_DFR) {                                  /* D layer: lift to U */
            int p = 0;
            while (dcorner_by_ring[p] != slot) p++;
            int Q = right_of(ring[p]);
            emit(s, Q, 1); emit(s, CF_U, 1); emit(s, Q, 3);
            continue;
        }
        /* in U layer: shortest insertion with {U, A, B} */
        if (search_raw(s, mvset, 9, 7, corner_pred, arg, 200000)) return;

        /* fallback: rotate above target, then repeat sexy move */
        int p = 0;
        while (ucorner_by_ring[p] != slot) p++;
        emit(s, CF_U, (ringpos(A) - p) & 3);
        for (int r = 0; r < 6; r++) {
            slot = find_corner(&s->c, CF_D, A, B, &dpos);
            if (slot == target && s->c.f[cube_cf[slot][0]] == CF_D) return;
            emit(s, B, 1); emit(s, CF_U, 1); emit(s, B, 3); emit(s, CF_U, 3);
        }
    }
}

/* ---- stage 3: middle layer -----------------------------------------------------*/
static void emit_insert_right(sctx_t *s, int P, int Q)  /* U Q U' Q' U' P' U P */
{
    emit(s, CF_U, 1); emit(s, Q, 1); emit(s, CF_U, 3); emit(s, Q, 3);
    emit(s, CF_U, 3); emit(s, P, 3); emit(s, CF_U, 1); emit(s, P, 1);
}
static void emit_insert_left(sctx_t *s, int P, int Q)   /* U' P' U P U Q U' Q' */
{
    emit(s, CF_U, 3); emit(s, P, 3); emit(s, CF_U, 1); emit(s, P, 1);
    emit(s, CF_U, 1); emit(s, Q, 1); emit(s, CF_U, 3); emit(s, Q, 3);
}

static void solve_mid_edge(sctx_t *s, int P)
{
    int Q = right_of(P);
    int target = midslot_by_ring[ringpos(P)];
    for (int guard = 0; ; guard++) {
        if (guard > 6) { s->fail = 3; return; }
        int ori, slot = find_edge(&s->c, P, Q, &ori);
        if (slot < 0) { s->fail = 3; return; }

        if (slot == target && edge_home_ok(&s->c, slot)) return;

        if (slot >= E_FR) {                                   /* stuck in middle: eject */
            int p = 0;
            while (midslot_by_ring[p] != slot) p++;
            emit_insert_right(s, ring[p], right_of(ring[p]));
            continue;
        }
        /* U layer (D layer impossible: cross occupies it) */
        int S = s->c.f[cube_ef[slot][1]];                     /* side sticker color */
        int A = slot_side(slot);
        emit(s, CF_U, (ringpos(S) - ringpos(A)) & 3);
        int ucol = (S == P) ? Q : P;                          /* color on U sticker */
        if (ucol == right_of(S)) emit_insert_right(s, S, right_of(S));
        else                     emit_insert_left(s, left_of(S), S);
    }
}

/* ---- stage 4: top cross (edge orientation) --------------------------------------*/
static void solve_u_cross(sctx_t *s)
{
    for (int guard = 0; ; guard++) {
        if (guard > 6) { s->fail = 4; return; }
        int o[4], cnt = 0;
        for (int p = 0; p < 4; p++) {
            o[p] = (s->c.f[cube_ef[uslot_by_ring[p]][0]] == CF_U);
            cnt += o[p];
        }
        if (cnt == 4) return;
        if (cnt == 2) {
            if (o[0] && o[2]) emit(s, CF_U, 1);               /* line F-B -> L-R */
            else if (!(o[1] && o[3])) {                       /* adjacent pair */
                int a = 0;
                while (!(o[a] && o[(a + 1) & 3])) a++;
                emit(s, CF_U, (1 - a) & 3);                   /* pair to L(1)+B(2) */
            }
        }
        emit(s, CF_F, 1); emit(s, CF_R, 1); emit(s, CF_U, 1);
        emit(s, CF_R, 3); emit(s, CF_U, 3); emit(s, CF_F, 3);
    }
}

/* ---- stages 5-7: tiny IDDFS over macro operators ---------------------------------*/
typedef struct { uint8_t n; uint8_t mv[16]; } macro_t;

#define MV(F, T) ((F) * 3 + (T) - 1)

static const macro_t OP_U   = { 1, { MV(CF_U, 1) } };
static const macro_t OP_U2  = { 1, { MV(CF_U, 2) } };
static const macro_t OP_UP  = { 1, { MV(CF_U, 3) } };
/* Sune / AntiSune: orient U corners, keep U-edge orientation and F2L */
static const macro_t OP_SUNE  = { 7, { MV(CF_R,1), MV(CF_U,1), MV(CF_R,3), MV(CF_U,1),
                                       MV(CF_R,1), MV(CF_U,2), MV(CF_R,3) } };
static const macro_t OP_ASUNE = { 7, { MV(CF_R,1), MV(CF_U,2), MV(CF_R,3), MV(CF_U,3),
                                       MV(CF_R,1), MV(CF_U,3), MV(CF_R,3) } };
/* Aa-perm: R' F R' B2 R F' R' B2 R2 — pure 3-cycle of U corners
 * (keeps every edge and all orientations) */
static const macro_t OP_APERM = { 9, { MV(CF_R,3), MV(CF_F,1), MV(CF_R,3), MV(CF_B,2),
                                       MV(CF_R,1), MV(CF_F,3), MV(CF_R,3), MV(CF_B,2),
                                       MV(CF_R,2) } };
/* Ua/Ub-perm: pure 3-cycles of U edges (keep corners + orientations)
 * Ua: R U' R U R U R U' R' U' R2      Ub = Ua^-1 */
static const macro_t OP_UAPERM = { 11, { MV(CF_R,1), MV(CF_U,3), MV(CF_R,1), MV(CF_U,1),
                                         MV(CF_R,1), MV(CF_U,1), MV(CF_R,1), MV(CF_U,3),
                                         MV(CF_R,3), MV(CF_U,3), MV(CF_R,2) } };
static const macro_t OP_UBPERM = { 11, { MV(CF_R,2), MV(CF_U,1), MV(CF_R,1), MV(CF_U,1),
                                         MV(CF_R,3), MV(CF_U,3), MV(CF_R,3), MV(CF_U,3),
                                         MV(CF_R,3), MV(CF_U,1), MV(CF_R,3) } };

static void apply_macro(cube_t *c, const macro_t *m)
{
    for (int i = 0; i < m->n; i++) cube_move(c, m->mv[i]);
}

/* macro conjugated by U^k: U^k m U^-k */
static void build_conj(macro_t *dst, const macro_t *src, int k)
{
    dst->n = 0;
    if (k) dst->mv[dst->n++] = MV(CF_U, k);
    for (int i = 0; i < src->n; i++) dst->mv[dst->n++] = src->mv[i];
    if (k) dst->mv[dst->n++] = MV(CF_U, 4 - k);
}

static int ll_oriented(const cube_t *c)
{
    for (int i = 0; i < 9; i++)
        if (c->f[i] != CF_U) return 0;                       /* whole U face up */
    return 1;
}

static int ll_corners_solved(const cube_t *c)
{
    return corner_home_ok(c, C_URF) && corner_home_ok(c, C_UFL) &&
           corner_home_ok(c, C_ULB) && corner_home_ok(c, C_UBR);
}

static int cube_solved_pred(const cube_t *c) { return cube_is_solved(c); }

static int macro_dfs(const cube_t *c, int depth, const macro_t *const *ops, int nops,
                     int (*pred)(const cube_t *), int last_u, uint8_t *path)
{
    if (pred(c)) return 0;
    if (depth == 0) return -1;
    for (int i = 0; i < nops; i++) {
        int is_u = (ops[i]->n == 1);
        if (is_u && last_u) continue;                         /* U-ops never adjacent */
        cube_t t = *c;
        apply_macro(&t, ops[i]);
        int r = macro_dfs(&t, depth - 1, ops, nops, pred, is_u, path + 1);
        if (r >= 0) { *path = (uint8_t)i; return r + 1; }
    }
    return -1;
}

static void run_macro_stage(sctx_t *s, const macro_t *const *ops, int nops,
                            int (*pred)(const cube_t *), int maxd, int stage)
{
    uint8_t path[10];
    for (int d = 0; d <= maxd; d++) {
        int len = macro_dfs(&s->c, d, ops, nops, pred, 0, path);
        if (len >= 0) {
            for (int i = 0; i < len; i++) {
                const macro_t *m = ops[path[i]];
                for (int k = 0; k < m->n; k++)
                    emit(s, m->mv[k] / 3, m->mv[k] % 3 + 1);
            }
            return;
        }
    }
    s->fail = stage;
}

/* ---- move simplification -----------------------------------------------------*/
static int same_axis(int f1, int f2) { return f1 % 3 == f2 % 3; }

int cube_simplify(uint8_t *mv, uint8_t *stage, int n)
{
    int len = 0;
    for (int i = 0; i < n; i++) {
        int face = mv[i] / 3, turns = mv[i] % 3 + 1;
        int stg  = stage ? stage[i] : 0;
        for (;;) {
            int j = -1;                                       /* merge candidate */
            if (len > 0 && mv[len - 1] / 3 == face) j = len - 1;
            else if (len > 1 && same_axis(mv[len - 1] / 3, face) && mv[len - 2] / 3 == face)
                j = len - 2;
            if (j < 0) break;
            /* across stage boundaries only U turns may merge: positional stage
             * invariants are defined modulo a final U alignment, and the UI
             * shows per-move stage labels */
            if (stage && stage[j] != stg && face != CF_U) break;
            turns = (turns + mv[j] % 3 + 1) & 3;
            if (stage) stg = stage[j];                        /* keep earlier stage */
            for (int k = j; k < len - 1; k++) {               /* remove element j */
                mv[k] = mv[k + 1];
                if (stage) stage[k] = stage[k + 1];
            }
            len--;
            if (!turns) break;
        }
        if (turns) {
            mv[len] = cube_mv(face, turns);
            if (stage) stage[len] = (uint8_t)stg;
            len++;
        }
    }
    return len;
}

/* ---- top level -----------------------------------------------------------------*/
int cube_solve(const cube_t *start, cube_solution_t *sol)
{
    static sctx_t s;                                          /* keep off task stack */
    memset(&s, 0, sizeof(s));
    s.c = *start;

    s.stage = CUBE_ST_CROSS;
    cross_arg_t cross_arg = { 0, {0, 0, 0, 0} };
    for (int p = 0; p < 4; p++) solve_cross_edge(&s, p, &cross_arg);

    s.stage = CUBE_ST_CORNERS;
    corner_arg_t corner_arg = { 0, {0, 0, 0, 0} };
    for (int p = 0; p < 4; p++) solve_d_corner(&s, ring[p], &corner_arg);

    s.stage = CUBE_ST_MIDDLE;
    for (int p = 0; p < 4; p++) solve_mid_edge(&s, ring[p]);

    s.stage = CUBE_ST_TOPCROSS;
    solve_u_cross(&s);

    s.stage = CUBE_ST_TOPTWIST;
    const macro_t *twist_ops[5] = { &OP_U, &OP_U2, &OP_UP, &OP_SUNE, &OP_ASUNE };
    run_macro_stage(&s, twist_ops, 5, ll_oriented, 6, CUBE_ST_TOPTWIST);

    s.stage = CUBE_ST_TOPCORNER;
    macro_t aconj[4];
    for (int k = 0; k < 4; k++) build_conj(&aconj[k], &OP_APERM, k);
    const macro_t *corner_ops[7] = { &OP_U, &OP_U2, &OP_UP,
                                     &aconj[0], &aconj[1], &aconj[2], &aconj[3] };
    run_macro_stage(&s, corner_ops, 7, ll_corners_solved, 4, CUBE_ST_TOPCORNER);

    s.stage = CUBE_ST_TOPEDGE;
    macro_t uconj[8];
    const macro_t *edge_ops[8];
    for (int k = 0; k < 4; k++) {
        build_conj(&uconj[k],     &OP_UAPERM, k);
        build_conj(&uconj[4 + k], &OP_UBPERM, k);
        edge_ops[k]     = &uconj[k];
        edge_ops[4 + k] = &uconj[4 + k];
    }
    run_macro_stage(&s, edge_ops, 8, cube_solved_pred, 3, CUBE_ST_TOPEDGE);

    if (s.fail) return -s.fail;
    if (!cube_is_solved(&s.c)) return -100;

    int len = cube_simplify(s.out, s.stg, s.n);
    if (len > CUBE_MAX_MOVES) return -101;
    memcpy(sol->moves, s.out, len);
    memcpy(sol->stage, s.stg, len);
    sol->count = len;
    return 0;
}
