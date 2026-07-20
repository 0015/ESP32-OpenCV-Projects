/*
 * cube_model — facelet-level cube state, moves, and scan validation.
 * Pure portable C so it can also be compiled and tested on the host.
 */
#include "cube_model.h"
#include <string.h>

/* ---- move tables -----------------------------------------------------------
 * Each base (CW) move = five 4-cycles of facelet positions.
 * Cycle {a,b,c,d}: sticker at a moves to b, b->c, c->d, d->a.
 * Facelet numbering: U=0..8 R=9..17 F=18..26 D=27..35 L=36..44 B=45..53.
 * Derived from cube geometry (x=R, y=U, z=F); verified by host tests
 * (move^4 == id, checkerboard pattern, solver round-trips).
 */
static const uint8_t move_cycles[6][5][4] = {
    /* U */ {{0, 2, 8, 6},    {1, 5, 7, 3},    {18, 36, 45, 9}, {19, 37, 46, 10}, {20, 38, 47, 11}},
    /* R */ {{9, 11, 17, 15}, {10, 14, 16, 12},{20, 2, 51, 29}, {23, 5, 48, 32},  {26, 8, 45, 35}},
    /* F */ {{18, 20, 26, 24},{19, 23, 25, 21},{6, 9, 29, 44},  {7, 12, 28, 41},  {8, 15, 27, 38}},
    /* D */ {{27, 29, 35, 33},{28, 32, 34, 30},{24, 15, 51, 42},{25, 16, 52, 43}, {26, 17, 53, 44}},
    /* L */ {{36, 38, 44, 42},{37, 41, 43, 39},{0, 18, 27, 53}, {3, 21, 30, 50},  {6, 24, 33, 47}},
    /* B */ {{45, 47, 53, 51},{46, 50, 52, 48},{0, 42, 35, 11}, {1, 39, 34, 14},  {2, 36, 33, 17}},
};

/* ---- cubie tables (kociemba layout, 0-indexed) ----------------------------*/
const uint8_t cube_cf[8][3] = {
    /* URF */ {8, 9, 20},   /* UFL */ {6, 18, 38}, /* ULB */ {0, 36, 47},
    /* UBR */ {2, 45, 11},  /* DFR */ {29, 26, 15},/* DLF */ {27, 44, 24},
    /* DBL */ {33, 53, 42}, /* DRB */ {35, 17, 51},
};
const uint8_t cube_cc[8][3] = {
    {CF_U, CF_R, CF_F}, {CF_U, CF_F, CF_L}, {CF_U, CF_L, CF_B}, {CF_U, CF_B, CF_R},
    {CF_D, CF_F, CF_R}, {CF_D, CF_L, CF_F}, {CF_D, CF_B, CF_L}, {CF_D, CF_R, CF_B},
};
const uint8_t cube_ef[12][2] = {
    /* UR */ {5, 10},  /* UF */ {7, 19},  /* UL */ {3, 37},  /* UB */ {1, 46},
    /* DR */ {32, 16}, /* DF */ {28, 25}, /* DL */ {30, 43}, /* DB */ {34, 52},
    /* FR */ {23, 12}, /* FL */ {21, 41}, /* BL */ {50, 39}, /* BR */ {48, 14},
};
const uint8_t cube_ec[12][2] = {
    {CF_U, CF_R}, {CF_U, CF_F}, {CF_U, CF_L}, {CF_U, CF_B},
    {CF_D, CF_R}, {CF_D, CF_F}, {CF_D, CF_L}, {CF_D, CF_B},
    {CF_F, CF_R}, {CF_F, CF_L}, {CF_B, CF_L}, {CF_B, CF_R},
};

void cube_reset(cube_t *c)
{
    for (int i = 0; i < 54; i++) c->f[i] = (uint8_t)(i / 9);
}

void cube_move(cube_t *c, uint8_t move)
{
    int face  = move / 3;
    int turns = move % 3 + 1;
    for (int t = 0; t < turns; t++) {
        for (int k = 0; k < 5; k++) {
            const uint8_t *cy = move_cycles[face][k];
            uint8_t tmp   = c->f[cy[3]];
            c->f[cy[3]]   = c->f[cy[2]];
            c->f[cy[2]]   = c->f[cy[1]];
            c->f[cy[1]]   = c->f[cy[0]];
            c->f[cy[0]]   = tmp;
        }
    }
}

uint8_t cube_move_inv(uint8_t move)
{
    return (uint8_t)((move / 3) * 3 + (2 - move % 3));
}

const char *cube_move_name(uint8_t move)
{
    static const char *names[18] = {
        "U", "U2", "U'", "R", "R2", "R'", "F", "F2", "F'",
        "D", "D2", "D'", "L", "L2", "L'", "B", "B2", "B'",
    };
    return (move < 18) ? names[move] : "?";
}

int cube_is_solved(const cube_t *c)
{
    for (int i = 0; i < 54; i++)
        if (c->f[i] != i / 9) return 0;
    return 1;
}

int cube_validate(const cube_t *c, int *bad_color)
{
    /* centers: facelet values are defined via center colors, so center of
     * face f must be f */
    for (int f = 0; f < 6; f++)
        if (c->f[f * 9 + 4] != f) return CUBE_ERR_CENTERS;

    /* exactly 9 stickers of each color */
    int cnt[6] = {0};
    for (int i = 0; i < 54; i++) {
        if (c->f[i] > 5) return CUBE_ERR_COUNT;
        cnt[c->f[i]]++;
    }
    for (int col = 0; col < 6; col++) {
        if (cnt[col] != 9) {
            if (bad_color) {
                /* prefer reporting an over-counted color */
                *bad_color = col;
                for (int k = 0; k < 6; k++)
                    if (cnt[k] > 9) { *bad_color = k; break; }
            }
            return CUBE_ERR_COUNT;
        }
    }

    /* corners: identify piece + orientation in every slot */
    int cperm[8], cori_sum = 0, used_c = 0;
    for (int s = 0; s < 8; s++) {
        int ori = -1;
        for (int o = 0; o < 3; o++) {
            uint8_t col = c->f[cube_cf[s][o]];
            if (col == CF_U || col == CF_D) { ori = o; break; }
        }
        if (ori < 0) return CUBE_ERR_BAD_PIECE;
        uint8_t c0 = c->f[cube_cf[s][ori]];
        uint8_t c1 = c->f[cube_cf[s][(ori + 1) % 3]];
        uint8_t c2 = c->f[cube_cf[s][(ori + 2) % 3]];
        int piece = -1;
        for (int p = 0; p < 8; p++)
            if (cube_cc[p][0] == c0 && cube_cc[p][1] == c1 && cube_cc[p][2] == c2) { piece = p; break; }
        if (piece < 0 || (used_c & (1 << piece))) return CUBE_ERR_BAD_PIECE;
        used_c |= 1 << piece;
        cperm[s] = piece;
        cori_sum += ori;
    }
    if (cori_sum % 3 != 0) return CUBE_ERR_TWIST;

    /* edges */
    int eperm[12], eori_sum = 0, used_e = 0;
    for (int s = 0; s < 12; s++) {
        uint8_t a = c->f[cube_ef[s][0]];
        uint8_t b = c->f[cube_ef[s][1]];
        int piece = -1, ori = 0;
        for (int p = 0; p < 12; p++) {
            if (cube_ec[p][0] == a && cube_ec[p][1] == b) { piece = p; ori = 0; break; }
            if (cube_ec[p][0] == b && cube_ec[p][1] == a) { piece = p; ori = 1; break; }
        }
        if (piece < 0 || (used_e & (1 << piece))) return CUBE_ERR_BAD_PIECE;
        used_e |= 1 << piece;
        eperm[s] = piece;
        eori_sum += ori;
    }
    if (eori_sum % 2 != 0) return CUBE_ERR_FLIP;

    /* permutation parity must match between corners and edges */
    int cpar = 0, epar = 0;
    for (int i = 0; i < 8; i++)
        for (int j = i + 1; j < 8; j++)
            if (cperm[i] > cperm[j]) cpar ^= 1;
    for (int i = 0; i < 12; i++)
        for (int j = i + 1; j < 12; j++)
            if (eperm[i] > eperm[j]) epar ^= 1;
    if (cpar != epar) return CUBE_ERR_PARITY;

    return CUBE_OK;
}
