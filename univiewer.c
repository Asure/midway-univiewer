/*
 * univiewer.c  —  Midway Universe 3D viewer
 * C99 + SDL2  (Linux and Windows)
 *
 * Usage:  univiewer <file.UNI>
 *
 * Reads the compiled .UNI binary, resolves the IMG file references from
 * the same directory, and renders the scene with a simple perspective
 * projection.
 *
 * UNI file layout (empirically derived from GXPORTAL.UNI / UNI00.ASM):
 *   [0 -  1]  constant (270 in all files seen)
 *   [2 -  3]  num_imgs  — number of IMG file references
 *   [4 -  5]  num_objs  — number of object records
 *   [6 -  7]  halfy     — screen Y of the horizon (signed 16-bit)
 *   [8 -  9]  ???
 *   [10- 11]  sky_colr  — BGR555
 *   [12- 15]  ZMIN / zfar (signed 32-bit 16.16 fixed point)
 *   [16- 19]  WORLD_Y        "
 *   [20- 31]  ???
 *   [32- 33]  gnd_colr  — BGR555
 *   [34- 47]  ???
 *   --- IMG table at offset 48, num_imgs × 64 bytes ---
 *   Each entry: first ≤40 bytes = Windows absolute path (null-term),
 *               remainder = padding.  We strip to the basename.
 *   --- Object records at offset 48 + num_imgs*64, num_objs × 48 bytes ---
 *   [0 -  1]  img_file_index  (0-based into IMG table)
 *   [2 - 17]  image_name[16]  (null-terminated, rest padded)
 *   [18- 19]  unk / anipt offset from ASM (+N)
 *   [20- 23]  X   signed 32-bit 16.16 fixed point
 *   [24- 27]  Y   "
 *   [28- 31]  Z   "   (depth; larger = further)
 *   [32- 33]  flags   bit4=HFL  bit5=VFL
 *   [34- 47]  padding
 *
 * IMG file layout (wmpstruc.inc, IMGVER 0x63F):
 *   LIB_HDR[30]: IMGCNT[2] PALCNT[2] OSET[4] VERSION[2] ...
 *   At OSET: IMAGE[50] × IMGCNT, then PALETTE[26] × PALCNT
 *   IMAGE[50]: N_s[16] FLAGS[2] ANIX[2] ANIY[2] W[2] H[2] PALNUM[2]
 *              OSET[4] DATA[4] LIB[2] ANIX2[2] ANIY2[2] ANIZ2[2]
 *              FRM[2] PTTBLNUM[2] OPALS[2]
 *   PALETTE[26]: N_s[10] FLAGS[1] BITSPIX[1] NUMC[2] OSET[4]
 *                DATA[2] LIB[2] COLIND[1] CMAP[1] spare[2]
 *   Pixel data: raw 8-bit palette-indexed, rows padded to even stride
 *   Palette data: NUMC × 2 bytes BGR555 (R=bits14-10, G=9-5, B=4-0)
 */

#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* Constants                                                            */
/* ------------------------------------------------------------------ */
#define MAX_IMG_FILES   32
#define MAX_IMAGES      256   /* per IMG file */
#define MAX_PALS        64    /* per IMG file */
#define MAX_OBJS        4096
#define IMG_SZ          50    /* IMAGE struct bytes (on disk) */
#define PAL_SZ          26    /* PALETTE struct bytes (on disk) */
#define OBJ_STRIDE      48
#define IMG_REC_STRIDE  64    /* bytes per IMG-path slot in UNI header */
#define UNI_HDR_SZ      48

#define SCREEN_W        1280
#define SCREEN_H        720

/* ------------------------------------------------------------------ */
/* Data structures                                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    char        name[16];
    int         w, h;
    int         pal_idx;
    int         anix, aniy;       /* anchor / animation point */
    long        data_offset;      /* byte offset in IMG file  */
    int         row_stride;       /* (w+1)/2*2 bytes          */
    SDL_Texture *tex;             /* created on demand        */
} ImgImage;

typedef struct {
    char        name[10];
    int         numc;
    long        data_offset;      /* byte offset in IMG file  */
    uint32_t    colors[256];      /* ARGB32, 0=transparent    */
} ImgPal;

typedef struct {
    char      path[512];          /* resolved OS path         */
    int       nimgs;
    ImgImage  imgs[MAX_IMAGES];
    int       npals;
    ImgPal    pals[MAX_PALS];
    int       loaded;
} ImgFile;

typedef struct {
    int         img_file_idx;
    char        img_name[16];
    float       x, y, z;         /* world coords (game units)*/
    int         flags;            /* bit4=HFL bit5=VFL        */
    ImgImage   *img;              /* resolved                 */
    SDL_Texture*tex;              /* resolved (may share)     */
} UniObj;

typedef struct {
    /* header fields */
    int         num_imgs;
    int         num_objs;
    int         halfy;
    float       zmin, world_y;
    uint32_t    gnd_col, sky_col; /* ARGB32 */

    /* resolved objects */
    UniObj      objs[MAX_OBJS];

    /* IMG files */
    int         n_img_files;
    ImgFile     img_files[MAX_IMG_FILES];
} UniWorld;

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */
static uint32_t bgr555_to_argb(uint16_t v)
{
    /* Format: XRRRRRGGGGGBBBBB (little-endian word).
       5→8 bit expansion matches shim_setvgapal15: (c5<<3)|(c5>>2) */
    uint8_t r5 = (uint8_t)((v >> 10) & 0x1f);
    uint8_t g5 = (uint8_t)((v >>  5) & 0x1f);
    uint8_t b5 = (uint8_t)( v        & 0x1f);
    uint8_t r  = (uint8_t)((r5 << 3) | (r5 >> 2));
    uint8_t g  = (uint8_t)((g5 << 3) | (g5 >> 2));
    uint8_t b  = (uint8_t)((b5 << 3) | (b5 >> 2));
    return (uint32_t)(0xff000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}

/* Read little-endian values from a byte buffer */
static uint16_t ru16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}
static int16_t rs16(const uint8_t *p) {
    return (int16_t)ru16(p);
}
static uint32_t ru32(const uint8_t *p) {
    return (uint32_t)(p[0] | (p[1]<<8) | (p[2]<<16) | (p[3]<<24));
}
static int32_t rs32(const uint8_t *p) {
    return (int32_t)ru32(p);
}

/* Extract just the filename from a Windows absolute path */
static void basename_win(const char *src, char *dst, int dstlen)
{
    /* find last \ or / */
    const char *last = src;
    const char *p = src;
    while (*p) {
        if (*p == '\\' || *p == '/') last = p + 1;
        p++;
    }
    strncpy(dst, last, dstlen - 1);
    dst[dstlen - 1] = '\0';
    /* uppercase for case-insensitive match on Linux */
    for (int i = 0; dst[i]; i++)
        if (dst[i] >= 'a' && dst[i] <= 'z') dst[i] = (char)(dst[i] - 32);
}

/* Build a full path by combining a directory and a filename.
   Try exact case, then uppercase, then lowercase. */
static int resolve_path(const char *dir, const char *fname, char *out, int outlen)
{
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s/%s", dir, fname);
    if (SDL_RWFromFile(tmp, "rb")) { strncpy(out, tmp, outlen-1); out[outlen-1]=0; return 1; }

    /* try all-uppercase */
    char upper[64];
    strncpy(upper, fname, sizeof(upper)-1); upper[sizeof(upper)-1]=0;
    for (int i=0; upper[i]; i++) if(upper[i]>='a'&&upper[i]<='z') upper[i]=(char)(upper[i]-32);
    snprintf(tmp, sizeof(tmp), "%s/%s", dir, upper);
    if (SDL_RWFromFile(tmp, "rb")) { strncpy(out, tmp, outlen-1); out[outlen-1]=0; return 1; }

    /* try all-lowercase */
    char lower[64];
    strncpy(lower, fname, sizeof(lower)-1); lower[sizeof(lower)-1]=0;
    for (int i=0; lower[i]; i++) if(lower[i]>='A'&&lower[i]<='Z') lower[i]=(char)(lower[i]+32);
    snprintf(tmp, sizeof(tmp), "%s/%s", dir, lower);
    if (SDL_RWFromFile(tmp, "rb")) { strncpy(out, tmp, outlen-1); out[outlen-1]=0; return 1; }

    return 0;
}

/* ------------------------------------------------------------------ */
/* IMG file loading                                                     */
/* ------------------------------------------------------------------ */
static int imgfile_load(ImgFile *f, SDL_Renderer *rend)
{
    (void)rend; /* textures created later on demand */
    FILE *fp = fopen(f->path, "rb");
    if (!fp) { fprintf(stderr, "Cannot open IMG: %s\n", f->path); return 0; }

    /* read entire file */
    fseek(fp, 0, SEEK_END);
    long fsz = ftell(fp);
    rewind(fp);
    uint8_t *buf = (uint8_t*)malloc(fsz);
    if (!buf) { fclose(fp); return 0; }
    fread(buf, 1, fsz, fp);
    fclose(fp);

    /* LIB_HDR: IMGCNT[2] PALCNT[2] OSET[4] VERSION[2] ... (30 bytes)
     * NUMDEFPAL=3: three default palettes are NOT stored in the file.
     * Stored palette count = PALCNT - 3.
     * IMAGE.PALNUM in the file = internal_index + 3, so subtract 3 to get array index. */
    if (fsz < 30) { free(buf); return 0; }
    int imgcnt = ru16(buf + 0);
    int palcnt = ru16(buf + 2);
    long oset  = (long)ru32(buf + 4);

    int stored_pals = palcnt - 3;  /* only these are actually in the file */
    if (stored_pals < 0) stored_pals = 0;

    f->nimgs = (imgcnt > MAX_IMAGES) ? MAX_IMAGES : imgcnt;
    f->npals = (stored_pals > MAX_PALS) ? MAX_PALS : stored_pals;

    /* --- IMAGE records at oset, 50 bytes each --- */
    for (int i = 0; i < f->nimgs; i++) {
        long off = oset + (long)(i * IMG_SZ);
        if (off + IMG_SZ > fsz) break;
        ImgImage *im = &f->imgs[i];

        memcpy(im->name, buf + off, 16);
        im->name[15] = '\0';
        /* uppercase name for later lookup */
        for (int k = 0; im->name[k]; k++)
            if (im->name[k] >= 'a' && im->name[k] <= 'z')
                im->name[k] = (char)(im->name[k] - 32);

        /* FLAGS[2]=off+16  ANIX[2]=off+18  ANIY[2]=off+20
           W[2]=off+22  H[2]=off+24  PALNUM[2]=off+26
           OSET[4]=off+28  DATA[4]=off+32
           Row stride: (W+3)&~3  — rounds up to multiple of 4 (per itimg.asm) */
        im->anix    = (int)(int16_t)ru16(buf + off + 18);
        im->aniy    = (int)(int16_t)ru16(buf + off + 20);
        im->w       = (int)ru16(buf + off + 22);
        im->h       = (int)ru16(buf + off + 24);
        int palnum  = (int)ru16(buf + off + 26);
        im->pal_idx = palnum - 3;  /* subtract NUMDEFPAL to get stored array index */
        if (im->pal_idx < 0) im->pal_idx = 0;
        im->data_offset = (long)ru32(buf + off + 28);
        im->row_stride  = (im->w + 3) & ~3; /* itimg.asm: add ax,3  and al,0FCh */
        im->tex     = NULL;
    }

    /* --- PALETTE records: (PALCNT-3) stored starting right after IMAGE records --- */
    long pal_base = oset + (long)(imgcnt * IMG_SZ);
    for (int i = 0; i < f->npals; i++) {
        long off = pal_base + (long)(i * PAL_SZ);
        if (off + PAL_SZ > fsz) break;
        ImgPal *p = &f->pals[i];

        memcpy(p->name, buf + off, 10);
        p->name[9] = '\0';
        /* BITSPIX[1]=off+11  NUMC[2]=off+12  OSET[4]=off+14 */
        p->numc = (int)ru16(buf + off + 12);
        if (p->numc > 256) p->numc = 256;
        long pdata_off = (long)ru32(buf + off + 14);

        /* colour 0 = always transparent */
        p->colors[0] = 0x00000000u;
        for (int c = 1; c < p->numc; c++) {
            long co = pdata_off + (long)(c * 2);
            if (co + 2 > fsz) break;
            p->colors[c] = bgr555_to_argb(ru16(buf + co));
        }
    }

    f->loaded = 1;
    free(buf);
    return 1;
}

/* ------------------------------------------------------------------ */
/* Build SDL_Texture for one image (loads pixel data from disk)        */
/* ------------------------------------------------------------------ */
static SDL_Texture *make_texture(ImgFile *f, ImgImage *im, SDL_Renderer *rend)
{
    if (im->w <= 0 || im->h <= 0) return NULL;

    /* pick palette — use pal_idx from IMAGE struct */
    ImgPal *pal = NULL;
    if (im->pal_idx >= 0 && im->pal_idx < f->npals)
        pal = &f->pals[im->pal_idx];
    if (!pal && f->npals > 0)
        pal = &f->pals[0];

    FILE *fp = fopen(f->path, "rb");
    if (!fp) return NULL;

    int stride = im->row_stride;
    int pixsz  = stride * im->h;
    uint8_t *pixels = (uint8_t*)malloc(pixsz);
    if (!pixels) { fclose(fp); return NULL; }

    fseek(fp, im->data_offset, SEEK_SET);
    if ((int)fread(pixels, 1, pixsz, fp) < pixsz) {
        free(pixels); fclose(fp); return NULL;
    }
    fclose(fp);

    /* expand to ARGB32 */
    uint32_t *rgba = (uint32_t*)malloc(im->w * im->h * 4);
    if (!rgba) { free(pixels); return NULL; }

    for (int y = 0; y < im->h; y++) {
        const uint8_t *row = pixels + y * stride;
        uint32_t      *dst = rgba    + y * im->w;
        for (int x = 0; x < im->w; x++) {
            uint8_t idx = row[x];
            dst[x] = (pal && (int)idx < pal->numc) ? pal->colors[idx] : 0u;
        }
    }
    free(pixels);

    SDL_Texture *tex = SDL_CreateTexture(rend,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC,
        im->w, im->h);
    if (!tex) { free(rgba); return NULL; }

    SDL_UpdateTexture(tex, NULL, rgba, im->w * 4);
    SDL_SetTextureBlendMode(tex, SDL_BLENDMODE_BLEND);
    free(rgba);
    return tex;
}

/* ------------------------------------------------------------------ */
/* UNI loading                                                          */
/* ------------------------------------------------------------------ */
static int uni_load(const char *uni_path, UniWorld *world, SDL_Renderer *rend)
{
    FILE *fp = fopen(uni_path, "rb");
    if (!fp) { fprintf(stderr, "Cannot open: %s\n", uni_path); return 0; }
    fseek(fp, 0, SEEK_END);
    long fsz = ftell(fp); rewind(fp);
    uint8_t *buf = (uint8_t*)malloc(fsz);
    if (!buf) { fclose(fp); return 0; }
    fread(buf, 1, fsz, fp);
    fclose(fp);

    if (fsz < UNI_HDR_SZ) { free(buf); return 0; }

    /* --- parse header --- */
    world->num_imgs  = (int)ru16(buf + 2);
    world->num_objs  = (int)ru16(buf + 4);
    world->halfy     = (int)(int16_t)rs16(buf + 6);
    world->zmin      = (float)rs32(buf + 12) / 65536.0f;
    world->world_y   = (float)rs32(buf + 16) / 65536.0f;
    world->gnd_col   = bgr555_to_argb(ru16(buf + 32));
    world->sky_col   = bgr555_to_argb(ru16(buf + 10));

    fprintf(stderr, "UNI: num_imgs=%d num_objs=%d halfy=%d zmin=%.4f world_y=%.2f\n",
            world->num_imgs, world->num_objs,
            world->halfy, world->zmin, world->world_y);

    if (world->num_imgs > MAX_IMG_FILES) world->num_imgs = MAX_IMG_FILES;
    if (world->num_objs > MAX_OBJS)      world->num_objs = MAX_OBJS;

    /* --- get directory of the UNI file for relative IMG lookups --- */
    char uni_dir[512] = ".";
    {
        const char *last = uni_path;
        const char *p    = uni_path;
        while (*p) {
            if (*p == '/' || *p == '\\') last = p;
            p++;
        }
        if (last != uni_path) {
            size_t dlen = (size_t)(last - uni_path);
            if (dlen >= sizeof(uni_dir)) dlen = sizeof(uni_dir)-1;
            memcpy(uni_dir, uni_path, dlen);
            uni_dir[dlen] = '\0';
        }
    }

    /* --- IMG table --- */
    world->n_img_files = world->num_imgs;
    for (int i = 0; i < world->num_imgs; i++) {
        long slot_off = UNI_HDR_SZ + (long)(i * IMG_REC_STRIDE);
        if (slot_off + IMG_REC_STRIDE > fsz) break;

        /* extract Windows path and reduce to filename */
        char win_path[64] = {0};
        memcpy(win_path, buf + slot_off, 40);
        win_path[40] = '\0';

        char fname[64];
        basename_win(win_path, fname, sizeof(fname));

        ImgFile *f = &world->img_files[i];
        f->loaded = 0;
        f->nimgs  = 0;
        f->npals  = 0;

        if (!resolve_path(uni_dir, fname, f->path, sizeof(f->path))) {
            fprintf(stderr, "  IMG[%d] not found: %s\n", i, fname);
        } else {
            fprintf(stderr, "  IMG[%d] %s\n", i, f->path);
            imgfile_load(f, rend);
        }
    }

    /* --- object records --- */
    long obj_base = UNI_HDR_SZ + (long)(world->num_imgs * IMG_REC_STRIDE);
    int loaded = 0;
    for (int i = 0; i < world->num_objs; i++) {
        long off = obj_base + (long)(i * OBJ_STRIDE);
        if (off + OBJ_STRIDE > fsz) break;

        UniObj *o = &world->objs[loaded];

        o->img_file_idx = (int)ru16(buf + off);
        memcpy(o->img_name, buf + off + 2, 16);
        o->img_name[15] = '\0';
        /* uppercase name for lookup */
        for (int k = 0; o->img_name[k]; k++)
            if (o->img_name[k] >= 'a' && o->img_name[k] <= 'z')
                o->img_name[k] = (char)(o->img_name[k] - 32);

        o->x     = (float)rs32(buf + off + 20) / 65536.0f;
        o->y     = (float)rs32(buf + off + 24) / 65536.0f;
        o->z     = (float)rs32(buf + off + 28) / 65536.0f;
        o->flags = (int)ru16(buf + off + 32);
        o->img   = NULL;
        o->tex   = NULL;

        /* skip degenerate objects */
        if (o->z <= 0.0001f) continue;

        /* resolve image */
        int fi = o->img_file_idx;
        if (fi >= 0 && fi < world->n_img_files) {
            ImgFile *f = &world->img_files[fi];
            if (f->loaded) {
                for (int j = 0; j < f->nimgs; j++) {
                    if (strncmp(f->imgs[j].name, o->img_name, 15) == 0) {
                        o->img = &f->imgs[j];
                        if (!o->img->tex)
                            o->img->tex = make_texture(f, o->img, rend);
                        o->tex = o->img->tex;
                        break;
                    }
                }
            }
        }

        loaded++;
    }
    world->num_objs = loaded;
    fprintf(stderr, "Resolved %d / %d objects with textures\n",
            loaded, (int)ru16(buf + 4));

    free(buf);
    return 1;
}

/* ------------------------------------------------------------------ */
/* Depth sort (painter's algorithm — largest Z first)                  */
/* ------------------------------------------------------------------ */
static UniObj *g_sort_objs = NULL;
static int obj_cmp_z(const void *a, const void *b)
{
    const UniObj *oa = &g_sort_objs[*(const int*)a];
    const UniObj *ob = &g_sort_objs[*(const int*)b];
    if (ob->z > oa->z) return  1;
    if (ob->z < oa->z) return -1;
    return 0;
}

/* ------------------------------------------------------------------ */
/* 8×8 bitmap font, ASCII 32-127.  MSB = leftmost pixel.              */
/* ------------------------------------------------------------------ */
static const uint8_t g_font8[96][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 32   */
    {0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x00}, /* 33 ! */
    {0x66,0x66,0x66,0x00,0x00,0x00,0x00,0x00}, /* 34 " */
    {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00}, /* 35 # */
    {0x0C,0x3F,0x03,0x1E,0x30,0x1F,0x0C,0x00}, /* 36 $ */
    {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00}, /* 37 % */
    {0x1C,0x36,0x1C,0x6F,0x3B,0x33,0x6E,0x00}, /* 38 & */
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, /* 39 ' */
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, /* 40 ( */
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, /* 41 ) */
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, /* 42 * */
    {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, /* 43 + */
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06}, /* 44 , */
    {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, /* 45 - */
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, /* 46 . */
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, /* 47 / */
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, /* 48 0 */
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, /* 49 1 */
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, /* 50 2 */
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, /* 51 3 */
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, /* 52 4 */
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, /* 53 5 */
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, /* 54 6 */
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, /* 55 7 */
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, /* 56 8 */
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, /* 57 9 */
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00}, /* 58 : */
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x0C}, /* 59 ; */
    {0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x00}, /* 60 < */
    {0x00,0x00,0x3F,0x00,0x3F,0x00,0x00,0x00}, /* 61 = */
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, /* 62 > */
    {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, /* 63 ? */
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, /* 64 @ */
    {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, /* 65 A */
    {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, /* 66 B */
    {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, /* 67 C */
    {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, /* 68 D */
    {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, /* 69 E */
    {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, /* 70 F */
    {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, /* 71 G */
    {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, /* 72 H */
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 73 I */
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, /* 74 J */
    {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, /* 75 K */
    {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, /* 76 L */
    {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, /* 77 M */
    {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, /* 78 N */
    {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, /* 79 O */
    {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, /* 80 P */
    {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, /* 81 Q */
    {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, /* 82 R */
    {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, /* 83 S */
    {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 84 T */
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, /* 85 U */
    {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, /* 86 V */
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, /* 87 W */
    {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, /* 88 X */
    {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, /* 89 Y */
    {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, /* 90 Z */
    {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00}, /* 91 [ */
    {0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00}, /* 92 \ */
    {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00}, /* 93 ] */
    {0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00}, /* 94 ^ */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, /* 95 _ */
    {0x06,0x06,0x0C,0x00,0x00,0x00,0x00,0x00}, /* 96 ` */
    {0x00,0x00,0x1E,0x30,0x3E,0x33,0x6E,0x00}, /* 97 a */
    {0x07,0x06,0x06,0x3E,0x66,0x66,0x3B,0x00}, /* 98 b */
    {0x00,0x00,0x1E,0x33,0x03,0x33,0x1E,0x00}, /* 99 c */
    {0x38,0x30,0x30,0x3E,0x33,0x33,0x6E,0x00}, /* 100 d */
    {0x00,0x00,0x1E,0x33,0x3F,0x03,0x1E,0x00}, /* 101 e */
    {0x1C,0x36,0x06,0x0F,0x06,0x06,0x0F,0x00}, /* 102 f */
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x1F}, /* 103 g */
    {0x07,0x06,0x36,0x6E,0x66,0x66,0x67,0x00}, /* 104 h */
    {0x0C,0x00,0x0E,0x0C,0x0C,0x0C,0x1E,0x00}, /* 105 i */
    {0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1E}, /* 106 j */
    {0x07,0x06,0x66,0x36,0x1E,0x36,0x67,0x00}, /* 107 k */
    {0x0E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 108 l */
    {0x00,0x00,0x33,0x7F,0x7F,0x6B,0x63,0x00}, /* 109 m */
    {0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x00}, /* 110 n */
    {0x00,0x00,0x1E,0x33,0x33,0x33,0x1E,0x00}, /* 111 o */
    {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x0F}, /* 112 p */
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x78}, /* 113 q */
    {0x00,0x00,0x3B,0x6E,0x66,0x06,0x0F,0x00}, /* 114 r */
    {0x00,0x00,0x3E,0x03,0x1E,0x30,0x1F,0x00}, /* 115 s */
    {0x08,0x0C,0x3E,0x0C,0x0C,0x2C,0x18,0x00}, /* 116 t */
    {0x00,0x00,0x33,0x33,0x33,0x33,0x6E,0x00}, /* 117 u */
    {0x00,0x00,0x33,0x33,0x33,0x1E,0x0C,0x00}, /* 118 v */
    {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00}, /* 119 w */
    {0x00,0x00,0x63,0x36,0x1C,0x36,0x63,0x00}, /* 120 x */
    {0x00,0x00,0x33,0x33,0x33,0x3E,0x30,0x1F}, /* 121 y */
    {0x00,0x00,0x3F,0x19,0x0C,0x26,0x3F,0x00}, /* 122 z */
    {0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00}, /* 123 { */
    {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, /* 124 | */
    {0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00}, /* 125 } */
    {0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00}, /* 126 ~ */
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}, /* 127   */
};

static void draw_char(SDL_Renderer *r, int x, int y, char c, uint8_t R, uint8_t G, uint8_t B)
{
    unsigned u = (unsigned)(uint8_t)c;
    if (u < 32 || u > 127) return;
    const uint8_t *glyph = g_font8[u - 32];
    SDL_SetRenderDrawColor(r, R, G, B, 255);
    for (int row = 0; row < 8; row++) {
        uint8_t bits = glyph[row];
        for (int col = 0; col < 8; col++) {
            if (bits & (1u << col)) {
                SDL_RenderDrawPoint(r, x + col, y + row);
            }
        }
    }
}
static void draw_str(SDL_Renderer *r, int x, int y, const char *s, uint8_t R, uint8_t G, uint8_t B)
{
    for (; *s; s++, x += 8) draw_char(r, x, y, *s, R, G, B);
}

/* ------------------------------------------------------------------ */
/* Tooltip helper                                                       */
/* ------------------------------------------------------------------ */
static void draw_tooltip(SDL_Renderer *r, int x, int y, const UniObj *o)
{
    if (!o) return;
    char lines[6][64];
    int n = 0;
    snprintf(lines[n++], 64, "name: %.15s", o->img_name);
    snprintf(lines[n++], 64, "img:  %d", o->img_file_idx);
    snprintf(lines[n++], 64, "X=%.2f Y=%.2f", o->x, o->y);
    snprintf(lines[n++], 64, "Z=%.4f", o->z);
    snprintf(lines[n++], 64, "fl=0x%04X", o->flags);
    if (o->img) snprintf(lines[n++], 64, "size: %dx%d  pal=%d",
                         o->img->w, o->img->h, o->img->pal_idx);

    int w = 160, h = n * 10 + 6;
    int tx = x + 12, ty = y - h / 2;
    if (tx + w > SCREEN_W) tx = x - w - 4;
    if (ty < 2) ty = 2;
    if (ty + h > SCREEN_H) ty = SCREEN_H - h - 2;

    SDL_SetRenderDrawColor(r, 0, 0, 0, 200);
    SDL_Rect bg = {tx-1, ty-1, w+2, h+2};
    SDL_RenderFillRect(r, &bg);
    SDL_SetRenderDrawColor(r, 200, 200, 100, 255);
    SDL_RenderDrawRect(r, &bg);
    for (int i = 0; i < n; i++)
        draw_str(r, tx+2, ty+2+i*10, lines[i], 220, 220, 100);
}

/* ------------------------------------------------------------------ */
/* Main                                                                 */
/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: univiewer <file.UNI>\n");
        return 1;
    }

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window   *win  = SDL_CreateWindow("Midway Universe Viewer",
                             SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                             SCREEN_W, SCREEN_H,
                             SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    SDL_Renderer *rend = SDL_CreateRenderer(win, -1,
                             SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_SetRenderDrawBlendMode(rend, SDL_BLENDMODE_BLEND);

    /* --- load universe --- */
    UniWorld *world = (UniWorld*)calloc(1, sizeof(UniWorld));
    if (!uni_load(argv[1], world, rend)) {
        fprintf(stderr, "Failed to load %s\n", argv[1]);
        return 1;
    }

    /* --- compute initial camera from Z-weighted centroid (filter extreme outliers) ---
     * Coordinates are in screen-pixel units at reference depth.
     * focal ≈ min_Z so that scale ≈ 1 for the nearest objects (1 world unit = 1 pixel). */
    float init_cam_x = 0.0f, init_focal = 1.0f;
    {
        double sw = 0, swx = 0;
        float  minz = 1e9f;
        for (int i = 0; i < world->num_objs; i++) {
            float x = world->objs[i].x, y = world->objs[i].y, z = world->objs[i].z;
            if (z <= 0 || fabsf(x) > 2000 || fabsf(y) > 1000) continue;
            double w = 1.0 / z;
            swx += x * w;  sw += w;
            if (z < minz) minz = z;
        }
        if (sw > 0) init_cam_x = (float)(swx / sw);
        if (minz < 1e9f) init_focal = minz;
    }

    float cam_x   = init_cam_x;
    float cam_y   = world->world_y; /* camera eye is at floor height */
    float cam_z   = 0.0f;           /* looking into +Z */
    float focal   = init_focal;
    int   halfy   = world->halfy;

    /* build sort index */
    int *order = (int*)malloc(world->num_objs * sizeof(int));
    for (int i = 0; i < world->num_objs; i++) order[i] = i;
    g_sort_objs = world->objs;
    qsort(order, world->num_objs, sizeof(int), obj_cmp_z);

    /* --- camera / UI state --- */
    int  show_grid    = 0;
    int  show_borders = 0;
    int  show_sky     = 1;
    int  show_objs    = 1;
    int  show_map     = 0; /* TAB: 2D top-down XZ map */
    int  hover_idx    = -1;
    float move_speed  = 5.0f;   /* world units per keypress */

    int mx = 0, my = 0;
    int drag = 0, drag_sx = 0, drag_sy = 0;
    float drag_cam_x = 0, drag_cam_z = 0;

    int ww = SCREEN_W, wh = SCREEN_H;

    int running = 1;
    SDL_Event ev;

    while (running) {
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
            case SDL_QUIT: running = 0; break;

            case SDL_WINDOWEVENT:
                if (ev.window.event == SDL_WINDOWEVENT_RESIZED) {
                    ww = ev.window.data1;
                    wh = ev.window.data2;
                }
                break;

            case SDL_MOUSEMOTION:
                mx = ev.motion.x; my = ev.motion.y;
                if (drag) {
                    /* pan at a rate of 1 world unit per screen pixel (scale≈1 for near objects) */
                    cam_x = drag_cam_x - (float)(mx - drag_sx);
                    cam_z = drag_cam_z + (float)(my - drag_sy) * init_focal;
                }
                break;

            case SDL_MOUSEBUTTONDOWN:
                if (ev.button.button == SDL_BUTTON_LEFT) {
                    drag = 1;
                    drag_sx = ev.button.x; drag_sy = ev.button.y;
                    drag_cam_x = cam_x; drag_cam_z = cam_z;
                }
                break;

            case SDL_MOUSEBUTTONUP:
                if (ev.button.button == SDL_BUTTON_LEFT) drag = 0;
                break;

            case SDL_MOUSEWHEEL:
                if (!show_map) {
                    focal += (float)ev.wheel.y * focal * 0.1f;
                    if (focal < 0.001f) focal = 0.001f;
                    if (focal > 1000.0f) focal = 1000.0f;
                }
                break;

            case SDL_KEYDOWN: {
                SDL_Keymod mod = SDL_GetModState();
                SDL_Keycode k  = ev.key.keysym.sym;
                int shift = (mod & KMOD_SHIFT) ? 1 : 0;
                int ctrl  = (mod & KMOD_CTRL)  ? 1 : 0;

                if (k == SDLK_ESCAPE) { running = 0; break; }
                if (k == SDLK_TAB || ev.key.keysym.scancode == SDL_SCANCODE_TAB) {
                    show_map ^= 1;
                    fprintf(stderr, "TAB: show_map=%d\n", show_map);
                    break;
                }
                if (k == SDLK_HOME) {
                    cam_x = init_cam_x; cam_y = world->world_y;
                    cam_z = 0.0f; focal = init_focal;
                    break;
                }
                if (k == SDLK_PLUS  || k == SDLK_EQUALS || k == SDLK_KP_PLUS)  { focal *= 1.2f; break; }
                if (k == SDLK_MINUS || k == SDLK_KP_MINUS) { focal /= 1.2f; if(focal<0.001f)focal=0.001f; break; }

                /* Shift+T grid, Shift+B borders, Shift+O objects, Shift+S sky */
                if (shift) {
                    if (k == SDLK_t) { show_grid    ^= 1; break; }
                    if (k == SDLK_b) { show_borders ^= 1; break; }
                    if (k == SDLK_o) { show_objs    ^= 1; break; }
                    if (k == SDLK_s) { show_sky     ^= 1; break; }
                }

                /* Arrow keys: left/right = cam_x, up/down = cam_z (forward/back).
                 * Shift+arrow: same direction, 2× speed.
                 * Ctrl+up/down: cam_y (eye height). */
                if (ctrl) {
                    if (k == SDLK_UP)   { cam_y -= move_speed; break; }
                    if (k == SDLK_DOWN) { cam_y += move_speed; break; }
                } else {
                    float spd  = shift ? move_speed * 2.0f : move_speed;
                    float zspd = init_focal * (shift ? 4.0f : 2.0f);
                    if (k == SDLK_LEFT)  { cam_x -= spd;  break; }
                    if (k == SDLK_RIGHT) { cam_x += spd;  break; }
                    if (k == SDLK_UP)    { cam_z += zspd; break; }
                    if (k == SDLK_DOWN)  { cam_z -= zspd; if(cam_z<0)cam_z=0; break; }
                }
                break;
            }
            } /* switch */
        } /* event loop */

        /* ---- render ---- */
        int cx = ww / 2;

        /* ==================== 2D MAP MODE (TAB) ==================== */
        if (show_map) {
            SDL_SetRenderDrawColor(rend, 15, 15, 25, 255);
            SDL_RenderClear(rend);

            /* compute scene bounds (exclude extreme outliers) */
            float bx0=1e9f,bx1=-1e9f,bz0=1e9f,bz1=-1e9f;
            for (int i = 0; i < world->num_objs; i++) {
                float x=world->objs[i].x, z=world->objs[i].z;
                if (fabsf(x)>2000||fabsf(world->objs[i].y)>1000||z<=0) continue;
                if(x<bx0)bx0=x; if(x>bx1)bx1=x;
                if(z<bz0)bz0=z; if(z>bz1)bz1=z;
            }
            if (bx1 <= bx0) { bx0=0; bx1=640; }
            if (bz1 <= bz0) { bz0=0; bz1=10; }
            /* include camera position in bounds so crosshair is always visible */
            if (cam_x < bx0) bx0=cam_x; if (cam_x > bx1) bx1=cam_x;
            if (cam_z < bz0) bz0=cam_z; if (cam_z > bz1) bz1=cam_z;

            float pad = 0.05f;
            float rx = bx1-bx0, rz = bz1-bz0;
            bx0 -= rx*pad; bx1 += rx*pad;
            bz0 -= rz*pad; bz1 += rz*pad;

            /* Use SEPARATE X and Z scales so both axes fill the screen.
             * X range (scene width) and Z range (scene depth) differ wildly for
             * 2.5D universes, so a uniform scale would crush one axis to nothing.
             * Z is FLIPPED: near (small Z) = bottom of screen, far (large Z) = top. */
            int marg = 30;
            int map_top    = marg;
            int map_bottom = wh - marg - 30;
            float sx_scale = (float)(ww - 2*marg) / (bx1 - bx0);
            float sz_scale = (float)(map_bottom - map_top) / (bz1 - bz0);
            int   map_ox   = marg + (int)(-(bx0) * sx_scale);

#define MAP_SX(wx) (map_ox + (int)((wx) * sx_scale))
/* far Z at top, near Z at bottom */
#define MAP_SZ(wz) (map_bottom - (int)(((wz) - bz0) * sz_scale))

            /* draw camera-relative reference lines */
            SDL_SetRenderDrawColor(rend, 40, 40, 60, 255);
            SDL_RenderDrawLine(rend, MAP_SX(bx0), MAP_SZ(cam_z), MAP_SX(bx1), MAP_SZ(cam_z));
            SDL_SetRenderDrawColor(rend, 30, 50, 30, 255);
            SDL_RenderDrawLine(rend, MAP_SX(cam_x), map_top, MAP_SX(cam_x), map_bottom);

            /* color table per img file */
            static const uint8_t MCOLR[][3] = {
                {255,100,100},{100,255,100},{100,150,255},
                {255,230, 80},{220, 80,220},{ 80,220,220},
                {255,160, 40},{200,200,200}
            };
            int ncols = (int)(sizeof(MCOLR)/sizeof(MCOLR[0]));

            /* hover dot tracking in map mode */
            hover_idx = -1;

            for (int i = 0; i < world->num_objs; i++) {
                float x=world->objs[i].x, z=world->objs[i].z;
                if (fabsf(x)>2000||fabsf(world->objs[i].y)>1000||z<=0) continue;
                int fi = world->objs[i].img_file_idx % ncols;
                int px = MAP_SX(x), pz = MAP_SZ(z);
                SDL_SetRenderDrawColor(rend, MCOLR[fi][0], MCOLR[fi][1], MCOLR[fi][2], 200);
                SDL_Rect dot = {px-2, pz-2, 5, 5};
                SDL_RenderFillRect(rend, &dot);
                if (abs(mx-px)<=4 && abs(my-pz)<=4) hover_idx = i;
            }

            /* camera crosshair */
            int cpx = MAP_SX(cam_x), cpz = MAP_SZ(cam_z);
            SDL_SetRenderDrawColor(rend, 255, 255, 80, 255);
            SDL_RenderDrawLine(rend, cpx-8, cpz,   cpx+8, cpz);
            SDL_RenderDrawLine(rend, cpx,   cpz-8, cpx,   cpz+8);
            SDL_Rect cambox = {cpx-3, cpz-3, 7, 7};
            SDL_RenderDrawRect(rend, &cambox);

            /* legend */
            for (int fi = 0; fi < world->n_img_files && fi < ncols; fi++) {
                char leg[64];
                const char *p = world->img_files[fi].path;
                /* show just the basename */
                const char *bn = p;
                for (const char *q=p; *q; q++) if(*q=='/'||*q=='\\') bn=q+1;
                snprintf(leg, sizeof(leg), "%d %s", fi, bn);
                draw_str(rend, ww-260, 4+fi*10, leg, MCOLR[fi][0], MCOLR[fi][1], MCOLR[fi][2]);
            }

            /* HUD */
            char hud[160];
            snprintf(hud, sizeof(hud),
                "MAP  cam X=%.1f Z=%.3f  TAB=3D view  Home=reset",
                cam_x, cam_z);
            SDL_SetRenderDrawColor(rend,0,0,0,160);
            SDL_Rect hudbar = {0, wh-14, ww, 14};
            SDL_RenderFillRect(rend, &hudbar);
            draw_str(rend, 4, wh-12, hud, 200, 200, 200);

            /* tooltip in map mode */
            if (hover_idx >= 0)
                draw_tooltip(rend, mx, my, &world->objs[hover_idx]);

#undef MAP_SX
#undef MAP_SZ
            SDL_RenderPresent(rend);
            continue;  /* skip 3D render this frame */
        }

        /* ==================== 3D VIEW ==================== */

        /* background: sky above halfy, ground below */
        if (show_sky) {
            uint8_t sr = (world->sky_col >> 16) & 0xff;
            uint8_t sg = (world->sky_col >>  8) & 0xff;
            uint8_t sb = (world->sky_col      ) & 0xff;
            uint8_t gr = (world->gnd_col >> 16) & 0xff;
            uint8_t gg = (world->gnd_col >>  8) & 0xff;
            uint8_t gb = (world->gnd_col      ) & 0xff;

            /* if both are black (common), use a subtle blue/grey instead */
            if (sr == 0 && sg == 0 && sb == 0) { sr=0x20; sg=0x28; sb=0x40; }
            if (gr == 0 && gg == 0 && gb == 0) { gr=0x30; gg=0x28; gb=0x20; }

            SDL_SetRenderDrawColor(rend, sr, sg, sb, 255);
            SDL_Rect sky = {0, 0, ww, halfy};
            SDL_RenderFillRect(rend, &sky);

            SDL_SetRenderDrawColor(rend, gr, gg, gb, 255);
            SDL_Rect gnd = {0, halfy, ww, wh - halfy};
            SDL_RenderFillRect(rend, &gnd);
        } else {
            SDL_SetRenderDrawColor(rend, 20, 20, 30, 255);
            SDL_RenderClear(rend);
        }

        /* horizon line */
        SDL_SetRenderDrawColor(rend, 60, 60, 60, 255);
        SDL_RenderDrawLine(rend, 0, halfy, ww, halfy);

        /* ---- draw objects back-to-front ---- */
        hover_idx = -1;
        if (show_objs) {
            for (int oi = 0; oi < world->num_objs; oi++) {
                const UniObj *o = &world->objs[order[oi]];
                if (!o->tex || !o->img) continue;

                float dz = o->z - cam_z;
                if (dz <= 0.001f) continue;

                float scale = focal / dz;
                int sx = cx  + (int)((o->x - cam_x) * scale);
                int sy = halfy + (int)((o->y - cam_y) * scale);

                /* anchor point offsets */
                int ax = o->img->anix;
                int ay = o->img->aniy;
                int iw = o->img->w;
                int ih = o->img->h;

                int flip = 0;
                if (o->flags & 0x10) flip |= SDL_FLIP_HORIZONTAL;
                if (o->flags & 0x20) flip |= SDL_FLIP_VERTICAL;

                /* scale sprite dimensions and anchor by depth (per gxd.asm NORMAL_SCALE) */
                SDL_Rect dst;
                dst.w = (int)(iw * scale);
                dst.h = (int)(ih * scale);
                dst.x = sx - (int)(ax * scale);
                dst.y = sy - (int)(ay * scale);

                SDL_RenderCopyEx(rend, o->tex, NULL, &dst,
                                 0.0, NULL, (SDL_RendererFlip)flip);

                /* border highlight */
                if (show_borders) {
                    SDL_SetRenderDrawColor(rend, 80, 160, 80, 180);
                    SDL_RenderDrawRect(rend, &dst);
                }

                /* hover detection */
                if (mx >= dst.x && mx < dst.x + dst.w &&
                    my >= dst.y && my < dst.y + dst.h) {
                    hover_idx = order[oi];
                }
            }
        }

        /* grid overlay (world XZ plane at Y=0) */
        if (show_grid) {
            SDL_SetRenderDrawColor(rend, 50, 50, 80, 180);
            /* vertical lines every 10 world units */
            for (int gx = -200; gx <= 500; gx += 10) {
                float dz0 = world->zmin > 0 ? world->zmin - cam_z : 0.05f - cam_z;
                float dz1 = 0.01f;
                if (dz0 <= 0.001f) dz0 = 0.001f;
                if (dz1 <= 0.001f) continue;
                float s0 = focal / dz0;
                float s1 = focal / dz1;
                int x0 = cx  + (int)((gx - cam_x) * s0);
                int x1 = cx  + (int)((gx - cam_x) * s1);
                int y0 = halfy + (int)((0 - cam_y) * s0);
                int y1 = halfy + (int)((0 - cam_y) * s1);
                SDL_RenderDrawLine(rend, x0, y0, x1, y1);
            }
        }

        /* HUD */
        {
            char hud[160];
            snprintf(hud, sizeof(hud),
                "cam X=%.1f Y=%.1f Z=%.3f  focal=%.3f  objs=%d  Home=reset  TAB=map",
                cam_x, cam_y, cam_z, focal, world->num_objs);
            SDL_SetRenderDrawColor(rend,0,0,0,140);
            SDL_Rect hudbar = {0, 0, ww, 25};
            SDL_RenderFillRect(rend, &hudbar);
            draw_str(rend, 4, 4, hud, 200, 200, 200);
            draw_str(rend, 4, 14, "Shift=2x  Ctrl+up/dn=eye height  Shift+T=grid  B=borders  O=objs  S=sky  +/-=zoom", 140, 140, 140);
        }

        /* tooltip */
        if (hover_idx >= 0)
            draw_tooltip(rend, mx, my, &world->objs[hover_idx]);

        SDL_RenderPresent(rend);
    }

    /* cleanup */
    for (int i = 0; i < world->n_img_files; i++) {
        ImgFile *f = &world->img_files[i];
        for (int j = 0; j < f->nimgs; j++)
            if (f->imgs[j].tex) SDL_DestroyTexture(f->imgs[j].tex);
    }
    free(order);
    free(world);
    SDL_DestroyRenderer(rend);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
