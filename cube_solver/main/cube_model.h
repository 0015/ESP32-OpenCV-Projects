/*
 * cube_model — 3x3x3 Rubik's cube facelet model (portable C, no ESP deps).
 *
 * Faces (kociemba order): U=0 R=1 F=2 D=3 L=4 B=5.
 * Facelet index = face*9 + i, i = row*3+col (0..8) viewed looking straight at
 * the face with the cube held U-up / F-front:
 *   U: row0 = B-adjacent, col0 = L-adjacent   (viewed from above, F at bottom)
 *   D: row0 = F-adjacent, col0 = L-adjacent   (viewed from below, F at top)
 *   F,R,B,L: row0 = U-adjacent; F col0 = L, R col0 = F, B col0 = R, L col0 = B
 * Facelet VALUES are face ids (0..5) = the color of that face's center.
 *
 * Move encoding: move = face*3 + (turns-1), turns 1=CW, 2=half, 3=CCW.
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { CF_U = 0, CF_R, CF_F, CF_D, CF_L, CF_B };

typedef struct { uint8_t f[54]; } cube_t;

/* Edge slots (order matches cube_ef / cube_ec tables) */
enum { E_UR = 0, E_UF, E_UL, E_UB, E_DR, E_DF, E_DL, E_DB, E_FR, E_FL, E_BL, E_BR };
/* Corner slots (order matches cube_cf / cube_cc tables) */
enum { C_URF = 0, C_UFL, C_ULB, C_UBR, C_DFR, C_DLF, C_DBL, C_DRB };

extern const uint8_t cube_ef[12][2];   /* edge slot -> 2 facelet indices   */
extern const uint8_t cube_ec[12][2];   /* edge slot -> 2 face ids (solved) */
extern const uint8_t cube_cf[8][3];    /* corner slot -> 3 facelet indices */
extern const uint8_t cube_cc[8][3];    /* corner slot -> 3 face ids        */

static inline uint8_t cube_mv(int face, int turns) { return (uint8_t)(face * 3 + turns - 1); }

void        cube_reset(cube_t *c);                 /* solved state           */
void        cube_move(cube_t *c, uint8_t move);
uint8_t     cube_move_inv(uint8_t move);
const char *cube_move_name(uint8_t move);          /* "U" "U2" "U'" ...      */
int         cube_is_solved(const cube_t *c);

/* Scan-state validation. Returns CUBE_OK or an error code below.
 * On color-count errors *bad_color (if non-NULL) gets the face id whose color
 * has count > 9 (best guess of which face to rescan). */
enum {
    CUBE_OK = 0,
    CUBE_ERR_CENTERS,      /* centers not the identity mapping (internal)   */
    CUBE_ERR_COUNT,        /* some color count != 9                         */
    CUBE_ERR_BAD_PIECE,    /* an edge/corner color combo doesn't exist      */
    CUBE_ERR_TWIST,        /* corner orientation sum != 0 (mod 3)           */
    CUBE_ERR_FLIP,         /* edge orientation sum != 0 (mod 2)             */
    CUBE_ERR_PARITY,       /* corner/edge permutation parity mismatch       */
};
int cube_validate(const cube_t *c, int *bad_color);

#ifdef __cplusplus
}
#endif
