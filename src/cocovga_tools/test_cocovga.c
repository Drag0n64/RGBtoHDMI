/*
 * test_cocovga.c - PC test harness for the CoCoVGA W64 emulation.
 *
 * Builds MC6847 frames exactly as the MC6847digitizer + RGBtoHDMI would put
 * them in the frame buffer, runs the decoder on them and checks the result.
 * Also simulates RGBtoHDMI's multi-buffer capture loop end to end.
 *
 *   make -C src/cocovga_tools test
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "gen6847.h"
#include "../cocovga_bench.h"

static int failures = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

// The register page NitrOS-9's CoVGA driver writes (covdg.asm, VGASetup)
static const uint8_t nitros9_regs[9] = { 0x00, 0x81, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x02 };

// NitrOS-9 CoVDG character translation (VDG-T1 "true lower case" mode)
static uint8_t covdg_code(uint8_t a) {
   if (a & 0x80) return a;
   if (a == '^') return 0x00;
   if (a == '_') return 0x1F;
   if (a == '`') return 0x67;
   if (a >= 0x60) return a - 0x60;
   if (a >= 0x40) return (uint8_t)((a - 0x40) ^ 0x40);
   return a ^ 0x40;
}

static void put_text(uint8_t *vram, int row, int col, const char *s) {
   for (; *s && col < 64; s++, col++) vram[row * 64 + col] = covdg_code((uint8_t)*s);
}

static uint8_t *alloc_fb(const gen_geom_t *g) { return calloc((size_t)g->pitch, g->fb_h); }

static unsigned int rng = 12345;
static unsigned int rnd(void) { rng = rng * 1103515245u + 12345u; return (rng >> 8) & 0xFFFFFF; }

// ---------------------------------------------------------------------
// Independent reference for what CoCoVGA shows for one W64 pixel
// ---------------------------------------------------------------------
static uint8_t ref_w64_pixel(uint8_t b, int x, int y, int font, int css) {
   static const uint8_t sgc[8] = { CV_GREEN, CV_YELLOW, CV_BLUE, CV_RED, CV_BUFF, CV_CYAN, CV_MAGENTA, CV_ORANGE };
   if (b & 0x80) {
      int q = (y < 6 ? 0 : 2) + (x < 4 ? 0 : 1);
      return ((b >> (3 - q)) & 1) ? sgc[(b >> 4) & 7] : CV_BLACK;
   }
   int code = b & 0x3F;
   int t1 = font & 1, lower = font & 2;
   const uint8_t *g;
   int dark_strokes;   // 1: dark strokes on light background
   if (b & 0x40) { g = t1 ? mc6847t1_font[code] : mc6847_font[code]; dark_strokes = 1; }
   else if (lower && code < 32) { g = cv_lower_glyph(code, t1); dark_strokes = 1; }
   else { g = t1 ? mc6847t1_font[code] : mc6847_font[code]; dark_strokes = 0; }
   int stroke = (g[y] >> (7 - x)) & 1;
   uint8_t light = css ? CV_LORANGE : CV_GREEN, dark = css ? CV_DKORANGE : CV_DKGREEN;
   if (dark_strokes) return stroke ? dark : light;
   return stroke ? light : dark;
}

// compare a frame buffer's active area with the expected W64 screen
static int compare_w64(const uint8_t *fb, const gen_geom_t *g, const uint8_t *vram, int font, int css) {
   int bad = 0;
   int px = g->hs / 2, py = g->vs / 2;
   for (int i = 0; i < CV_VRAM_SIZE; i++) {
      int col = i & 63, row = i >> 6;
      for (int y = 0; y < 12; y++)
         for (int x = 0; x < 8; x++) {
            uint8_t want = ref_w64_pixel(vram[i], x, y, font, css);
            for (int yy = 0; yy < py; yy++)
               for (int xx = 0; xx < px; xx++) {
                  int fx = g->ox + (col * 8 + x) * px + xx;
                  int fy = g->oy + (row * 12 + y) * py + yy;
                  if ((fb[fy * g->pitch + fx] & 15) != want) bad++;
               }
         }
   }
   return bad;
}

static const gen_geom_t geoms[] = {
   { 640, 480, 640, 64, 48, 2, 2 },     // MC6847 LumaCode profile, double width + height
   { 640, 480, 640, 62, 46, 2, 2 },     // shifted by a couple of pixels
   { 720, 480, 736, 104, 48, 2, 2 },    // wider frame buffer (scaling borders), padded pitch
   { 640, 240, 640, 64, 24, 2, 1 },     // double height inhibited
   { 320, 240, 320, 32, 24, 1, 1 },     // no doubling
   { 1280, 960, 1280, 128, 96, 4, 4 },  // 4x
};
#define NGEOMS (int)(sizeof(geoms) / sizeof(geoms[0]))

static void make_text_page(uint8_t *t, const uint8_t *regs) {
   memset(t, 0x60, 512);
   if (regs) memcpy(t, regs, 9);
}

static void test_calibration(void) {
   uint8_t text[512];
   make_text_page(text, NULL);
   for (int k = 0; k < NGEOMS; k++) {
      const gen_geom_t *g = &geoms[k];
      uint8_t *fb = alloc_fb(g);
      for (int css = 0; css < 2; css++) {
         gen_frame(fb, g, GEN_TEXT, text, css, k + 1);
         cv_state_t s; cv_init(&s);
         cv_frame_t f = cv_analyze(&s, fb, g->pitch, g->fb_w, g->fb_h);
         CHECK(f == CV_FRAME_TEXT, "geom %d css %d: text frame not recognised (%d)", k, css, f);
         CHECK(s.cal_valid && s.ox == g->ox && s.oy == g->oy && s.hs == g->hs && s.vs == g->vs,
               "geom %d: calibration %d,%d %dx%d expected %d,%d %dx%d", k, s.ox, s.oy, s.hs, s.vs, g->ox, g->oy, g->hs, g->vs);
      }
      free(fb);
   }
}

static void test_register_page(void) {
   uint8_t text[512];
   for (int k = 0; k < NGEOMS; k++) {
      const gen_geom_t *g = &geoms[k];
      uint8_t *fb = alloc_fb(g);
      make_text_page(text, nitros9_regs);
      gen_frame(fb, g, GEN_TEXT, text, 0, 99);
      cv_state_t s; cv_init(&s);
      cv_frame_t f = cv_analyze(&s, fb, g->pitch, g->fb_w, g->fb_h);
      CHECK(f == CV_FRAME_REGISTER, "geom %d: NitrOS-9 register page not recognised (%d)", k, f);
      CHECK(s.regs.font == 0x03 && s.regs.enhanced == 0x02, "geom %d: regs font %02x enh %02x", k, s.regs.font, s.regs.enhanced);
      // reset-all page turns 64 columns off again
      static const uint8_t reset_all[9] = { 0xFF, 0x00, 0, 0, 0, 0, 0, 0, 0 };
      make_text_page(text, reset_all);
      gen_frame(fb, g, GEN_TEXT, text, 0, 0);
      f = cv_analyze(&s, fb, g->pitch, g->fb_w, g->fb_h);
      CHECK(f == CV_FRAME_REGISTER && s.regs.enhanced == 0 && s.regs.font == 0, "geom %d: reset page", k);
      // register page surrounded by black semigraphics (frame can't calibrate
      // on its own): the earlier calibration must be used
      memset(text, 0x80, 512);
      memcpy(text, nitros9_regs, 9);
      gen_frame(fb, g, GEN_TEXT, text, 0, 0);
      f = cv_analyze(&s, fb, g->pitch, g->fb_w, g->fb_h);
      CHECK(f == CV_FRAME_REGISTER && s.regs.enhanced == 0x02, "geom %d: register page with black surroundings", k);
      free(fb);
   }
   // every edit-mask / value combination the decoder must read back
   const gen_geom_t *g = &geoms[0];
   uint8_t *fb = alloc_fb(g);
   int bad = 0;
   for (int font = 0; font < 8; font++)
      for (int enh = 0; enh < 8; enh++)
         for (int extras = 0; extras < 256; extras += 17) {
            uint8_t regs[9] = { 0x00, 0x85, 0x00, (uint8_t)font, 0x00, (uint8_t)extras, 0, 0, (uint8_t)enh };
            make_text_page(text, regs);
            gen_frame(fb, g, GEN_TEXT, text, 0, 0);
            cv_state_t s; cv_init(&s);
            if (cv_analyze(&s, fb, g->pitch, g->fb_w, g->fb_h) != CV_FRAME_REGISTER ||
                s.regs.font != font || s.regs.enhanced != enh || s.regs.extras != extras) bad++;
         }
   CHECK(bad == 0, "%d register value combinations misread", bad);
   free(fb);
}

static void test_no_false_registers(void) {
   const gen_geom_t *g = &geoms[0];
   uint8_t *fb = alloc_fb(g);
   uint8_t text[512];
   int hits = 0, pages = 20000;
   for (int n = 0; n < pages; n++) {
      // typical screens: mostly normal text, some inverse, some semigraphics
      for (int i = 0; i < 512; i++) {
         unsigned int r = rnd() % 100;
         if (r < 60) text[i] = 0x60;                         // space
         else if (r < 90) text[i] = 0x40 | (rnd() & 0x3F);   // normal characters
         else if (r < 96) text[i] = rnd() & 0x3F;            // inverse / lower case
         else text[i] = 0x80 | (rnd() & 0x7F);               // semigraphics
      }
      gen_frame(fb, g, GEN_TEXT, text, 0, 0);
      cv_state_t s; cv_init(&s);
      if (cv_analyze(&s, fb, g->pitch, g->fb_w, g->fb_h) == CV_FRAME_REGISTER) hits++;
   }
   CHECK(hits == 0, "%d of %d random text screens taken for a register page", hits, pages);
   // the "all characters" page (POKE 1024+I,I)
   for (int i = 0; i < 512; i++) text[i] = i < 256 ? i : 0x60;
   gen_frame(fb, g, GEN_TEXT, text, 0, 0);
   cv_state_t s; cv_init(&s);
   CHECK(cv_analyze(&s, fb, g->pitch, g->fb_w, g->fb_h) == CV_FRAME_TEXT, "all-characters page");
   free(fb);
}

static void test_cg2_decode(void) {
   uint8_t vram[6144];
   for (int k = 0; k < NGEOMS; k++) {
      const gen_geom_t *g = &geoms[k];
      uint8_t *fb = alloc_fb(g);
      uint8_t text[512];
      make_text_page(text, nitros9_regs);
      for (int css = 0; css < 2; css++) {
         cv_state_t s; cv_init(&s);
         gen_frame(fb, g, GEN_TEXT, text, 0, 0);
         cv_analyze(&s, fb, g->pitch, g->fb_w, g->fb_h);       // calibrate + registers
         for (int n = 0; n < 20; n++) {
            for (int i = 0; i < 2048; i++) vram[i] = rnd() & 0xFF;
            gen_frame(fb, g, GEN_CG2, vram, css, n + 7);
            cv_frame_t f = cv_analyze(&s, fb, g->pitch, g->fb_w, g->fb_h);
            CHECK(f == CV_FRAME_CG2, "geom %d css %d: CG2 not recognised", k, css);
            CHECK(memcmp(s.vram, vram, 2048) == 0, "geom %d css %d: CG2 bytes differ", k, css);
            CHECK(s.css == css, "geom %d: css %d decoded as %d", k, css, s.css);
         }
         // other modes must not be taken for CG2
         static const gen_mode_t others[] = { GEN_TEXT, GEN_CG1, GEN_CG3, GEN_CG6, GEN_RG6 };
         for (int m = 0; m < 5; m++) {
            for (int n = 0; n < 10; n++) {
               for (int i = 0; i < 6144; i++) vram[i] = rnd() & 0xFF;
               if (others[m] == GEN_TEXT) for (int i = 0; i < 512; i++) vram[i] = 0x40 | (vram[i] & 0x3F);
               gen_frame(fb, g, others[m], vram, css, 0);
               cv_frame_t f = cv_analyze(&s, fb, g->pitch, g->fb_w, g->fb_h);
               CHECK(f != CV_FRAME_CG2, "geom %d: mode %d taken for CG2", k, others[m]);
            }
         }
      }
      free(fb);
   }
}

// ---------------------------------------------------------------------
// End-to-end simulation of RGBtoHDMI's capture loop (rgb_to_fb.S)
// ---------------------------------------------------------------------
static uint8_t *sim_buffers[4];
static uint8_t *sim_buffer(int i) { return sim_buffers[i]; }

typedef struct {
   int num_buffers_param;   // OSD "Num Buffers" (0..3)
   int last, displayed, redirect;
} sim_t;

// optional change to each generated frame before cv_frame sees it
static void (*sim_tamper)(uint8_t *fb, const gen_geom_t *g);

static int sim_capture(sim_t *m, cv_state_t *s, const gen_geom_t *g, gen_mode_t mode, const uint8_t *vram, int css, int *captured_out) {
   int cap;
   if (m->redirect) cap = m->redirect - 1;
   else if (m->last == m->num_buffers_param) cap = 0;
   else cap = (m->last + 1) & 3;
   gen_frame(sim_buffers[cap], g, mode, vram, css, 0);
   if (sim_tamper) sim_tamper(sim_buffers[cap], g);
   int redirect, swap;
   cv_frame_t f = cv_frame(s, cap, m->displayed, 4, sim_buffer, g->pitch, g->fb_w, g->fb_h, &redirect, &swap);
   if (swap >= 0) m->displayed = swap;
   m->last = cap;
   if (!redirect) m->displayed = cap;
   m->redirect = redirect;
   if (captured_out) *captured_out = cap;
   return f;
}

static void test_end_to_end(void) {
   for (int k = 0; k < NGEOMS; k++) {
      const gen_geom_t *g = &geoms[k];
      int w64_possible = g->hs >= 2 && g->vs >= 2;
      for (int nb = 0; nb < 4; nb++) {
         for (int i = 0; i < 4; i++) sim_buffers[i] = alloc_fb(g);
         uint8_t *expect = alloc_fb(g);
         sim_t m = { nb, 0, 0, 0 };
         cv_state_t s; cv_init(&s);
         uint8_t text[512], vram[2048];

         // BASIC screen
         make_text_page(text, NULL);
         for (int i = 0; i < 10; i++) sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
         gen_frame(expect, g, GEN_TEXT, text, 0, 0);
         CHECK(memcmp(sim_buffers[m.displayed], expect, (size_t)g->pitch * g->fb_h) == 0, "geom %d nb %d: BASIC screen", k, nb);

         // NitrOS-9 writes the register page over its screen, one frame
         make_text_page(text, nitros9_regs);
         int f = sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
         CHECK(f == CV_FRAME_REGISTER, "geom %d nb %d: register frame", k, nb);
         if (nb > 0) {
            make_text_page(text, NULL);
            gen_frame(expect, g, GEN_TEXT, text, 0, 0);
            CHECK(memcmp(sim_buffers[m.displayed], expect, (size_t)g->pitch * g->fb_h) == 0,
                  "geom %d nb %d: previous picture not held during register frame", k, nb);
         }

         // then CG2 with a cleared 64x32 screen and a shell session
         memset(vram, 0x60, sizeof(vram));
         put_text(vram, 0, 0, "NitrOS-9/6809 Level 1 V3.3.0");
         put_text(vram, 1, 0, "Dragon 64 with CoCoVGA 64 column text, lower case: abcxyz {|}~_^`");
         for (int i = 0; i < 3; i++) sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL);
         if (w64_possible) {
            CHECK(s.w64_on, "geom %d nb %d: W64 not switched on", k, nb);
            int bad = compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0);
            CHECK(bad == 0, "geom %d nb %d: %d W64 pixels wrong", k, nb, bad);
            // typing: one character per frame
            const char *cmd = "OS9: dir /d0";
            for (int i = 0; cmd[i]; i++) {
               vram[3 * 64 + i] = covdg_code((uint8_t)cmd[i]);
               sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL);
               CHECK(compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0) == 0, "geom %d nb %d: typing step %d", k, nb, i);
            }
            // scroll the whole screen
            memmove(vram, vram + 64, 2048 - 64);
            memset(vram + 2048 - 64, 0x60, 64);
            sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL);
            CHECK(compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0) == 0, "geom %d nb %d: scroll", k, nb);
            // something else scribbles over the display buffer: rolling refresh repairs it
            memset(sim_buffers[m.displayed] + g->oy * g->pitch, CV_RED, (size_t)g->pitch * 12 * g->vs);
            for (int i = 0; i < 40; i++) sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL);
            CHECK(compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0) == 0, "geom %d nb %d: rolling refresh", k, nb);
            // the capture never lands in the buffer on screen while W64 is on
            int cap;
            sim_capture(&m, &s, g, GEN_CG2, vram, 0, &cap);
            CHECK(cap != m.displayed, "geom %d nb %d: capture into displayed buffer", k, nb);
            // the probe after a mode change shows another buffer: W64 must follow the picture
            if (nb >= 1) {
               m.displayed = (m.displayed + 1) & 3;
               sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL);
               CHECK(s.w64_on && m.displayed == s.display_buf, "geom %d nb %d: W64 on wrong buffer", k, nb);
               CHECK(compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0) == 0, "geom %d nb %d: picture after buffer change", k, nb);
            }
            // the screen is cleared: one frame must redraw every character, not just a refresh
            memset(sim_buffers[m.displayed], 0, (size_t)g->pitch * g->fb_h);
            cv_invalidate(&s);
            sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL);
            CHECK(compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0) == 0, "geom %d nb %d: picture after clear", k, nb);
         } else {
            CHECK(!s.w64_on, "geom %d: W64 switched on without double size frame buffer", k);
         }

         // program exits back to the 32 column text screen
         make_text_page(text, NULL);
         memcpy(text, "\x4F\x4B", 2);
         for (int i = 0; i < 3; i++) sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
         gen_frame(expect, g, GEN_TEXT, text, 0, 0);
         CHECK(!s.w64_on && m.redirect == 0, "geom %d nb %d: W64 not left", k, nb);
         CHECK(memcmp(sim_buffers[m.displayed], expect, (size_t)g->pitch * g->fb_h) == 0, "geom %d nb %d: text screen after W64", k, nb);

         for (int i = 0; i < 4; i++) free(sim_buffers[i]);
         free(expect);
      }
   }
}

// ---------------------------------------------------------------------
// Preview: what NitrOS-9 will look like (PPM, MC6847_LC palette)
// ---------------------------------------------------------------------
static void write_ppm(const char *name, const uint8_t *fb, const gen_geom_t *g) {
   static const uint8_t pal[16][3] = {
      {0,0,0},{0,0,0},{0x9A,0x32,0x36},{0xFF,0xB7,0},{0x6B,0x27,0},{0x0D,0x3A,0x06},{0x4C,0x3A,0xB4},{0xBF,0xC8,0xAD},
      {0xD4,0x7F,0},{0xC8,0x4E,0xF0},{0x41,0xAF,0x71},{0,0,0},{0x30,0xD2,0},{0xC1,0xE5,0},{0,0,0},{0,0,0}
   };
   FILE *f = fopen(name, "wb");
   if (!f) return;
   fprintf(f, "P6\n%d %d\n255\n", g->fb_w, g->fb_h);
   for (int y = 0; y < g->fb_h; y++)
      for (int x = 0; x < g->fb_w; x++) fwrite(pal[fb[y * g->pitch + x] & 15], 1, 3, f);
   fclose(f);
}

static void make_previews(void) {
   const gen_geom_t *g = &geoms[0];
   for (int i = 0; i < 4; i++) sim_buffers[i] = alloc_fb(g);
   sim_t m = { 0, 0, 0, 0 };
   cv_state_t s; cv_init(&s);
   uint8_t text[512], vram[2048];
   make_text_page(text, nitros9_regs);
   sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
   memset(vram, 0x60, sizeof(vram));
   static const char *lines[] = {
      "NitrOS-9/6809 Level 1 V3.3.0",
      "Tano Dragon (64K) with CoCoVGA 64x32 text",
      "",
      "OS9: dir -e",
      "",
      "                       Directory of .  2026/10/24 15:42",
      "Owner Last modified    Attributes Sector Bytecount Name",
      "----- ---------------- ---------- ------ --------- ----",
      "    0 2026/10/24 15:40  d-ewrewr      2D       320 CMDS",
      "    0 2026/10/24 15:40  d-ewrewr      43       160 SYS",
      "    0 2026/10/24 15:40  --e-rewr      5A      2741 startup",
      "    0 2026/10/24 15:41  ---wr-wr      61       812 readme.txt",
      "",
      "OS9: list readme.txt",
      "The quick brown fox jumps over the lazy dog.",
      "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG. 0123456789",
      "Symbols: !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~",
      "",
      "OS9: _",
   };
   for (int i = 0; i < (int)(sizeof(lines) / sizeof(lines[0])); i++) put_text(vram, i, 0, lines[i]);
   for (int r = 22; r < 25; r++)
      for (int c = 0; c < 64; c++) vram[r * 64 + c] = 0x80 | ((c / 8) << 4) | (c & 15);
   sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL);
   write_ppm("preview_w64.ppm", sim_buffers[m.displayed], g);
   uint8_t *raw = alloc_fb(g);
   gen_frame(raw, g, GEN_CG2, vram, 0, 0);
   write_ppm("preview_cg2_raw.ppm", raw, g);
   free(raw);
   for (int i = 0; i < 4; i++) free(sim_buffers[i]);
}

static void test_fonts(void) {
   // only the space glyphs are empty; catches rows lost in the tables
   for (int i = 0; i < 64; i++) {
      int sum = 0;
      for (int r = 0; r < 12; r++) sum |= mc6847_font[i][r];
      CHECK((sum == 0) == (i == 0x20), "6847 glyph $%02X empty=%d", i, sum == 0);
   }
   for (int i = 0; i < 96; i++) {
      int sum = 0;
      for (int r = 0; r < 12; r++) sum |= mc6847t1_font[i][r];
      CHECK((sum == 0) == (i == 0x20), "T1 glyph %d empty=%d", i, sum == 0);
   }
   // spot checks against the data sheet: 'A', 'g' descender, '_' and ']'
   static const uint8_t t1_A[12] = { 0x00, 0x08, 0x14, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00 };
   CHECK(memcmp(mc6847t1_font[1], t1_A, 12) == 0, "T1 'A'");
   CHECK(mc6847t1_font[64 + 7][9] == 0x1C, "T1 'g' descender");
   CHECK(mc6847t1_font[95][7] == 0x3E, "T1 '_'");
   // lower case next to the 6847's capitals: same baseline (row 9) and cap
   // height (row 3) as the 6847's 'A'; nothing lost by the move
   for (int c = 0; c < 32; c++) {
      CHECK(mc6847t1_font[64 + c][10] == 0 && mc6847t1_font[64 + c][11] == 0, "T1 lower %d rows 10-11", c);
      CHECK(memcmp(cv_lower_glyph(c, 0) + 2, mc6847t1_font[64 + c], 10) == 0, "lower %d moved down 2 rows", c);
   }
   CHECK(cv_lower_glyph(1, 0)[9] != 0 && cv_lower_glyph(1, 0)[10] == 0 && mc6847_rom_font[1][9] != 0 && mc6847_rom_font[1][10] == 0,
         "'a' and the 6847's 'A' end on row 9");
   CHECK(cv_lower_glyph(2, 0)[3] != 0 && mc6847_rom_font[1][3] != 0 && cv_lower_glyph(2, 0)[2] == 0, "'b' ascender at the 6847's cap height");
   CHECK(cv_lower_glyph(1, 1) == mc6847t1_font[65], "T1 capitals: T1 lower case unchanged");
   CHECK(mc6847_font[0x1D][3] == 0x0E, "6847 ']'");
}

// Work limit: entering W64 and scrolling spread over several frames; a
// half drawn 64 column screen is never shown (Num Buffers >= 1)
static unsigned int fake_now;
static unsigned int fake_usec(void) { fake_now += 7; return fake_now; }

static void test_budget(void) {
   const gen_geom_t *g = &geoms[0];
   for (int mode = 0; mode < 2; mode++) {          // 0: character limit, 1: time limit
      for (int nb = 0; nb < 4; nb += 3) {
         for (int i = 0; i < 4; i++) sim_buffers[i] = alloc_fb(g);
         uint8_t *text_fb = alloc_fb(g);
         sim_t m = { nb, 0, 0, 0 };
         cv_state_t s; cv_init(&s);
         if (mode == 0) s.max_chars = 200;
         else { s.usec = fake_usec; s.budget_us = 300; }
         uint8_t text[512], vram[2048];
         make_text_page(text, NULL);
         memcpy(text, "\x4F\x4B", 2);
         for (int i = 0; i < 6; i++) sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
         gen_frame(text_fb, g, GEN_TEXT, text, 0, 0);
         make_text_page(text, nitros9_regs);
         sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
         memset(vram, 0x60, sizeof(vram));
         for (int r = 0; r < 32; r++) put_text(vram, r, 0, "The quick brown fox jumps over the lazy dog 0123456789");
         int frames = 0, bad_shown = 0;
         while (!s.w64_on && frames < 100) {
            sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL);
            frames++;
            if (!s.w64_on && nb > 0 &&
                memcmp(sim_buffers[m.displayed], text_fb, (size_t)g->pitch * g->fb_h) != 0) bad_shown++;
         }
         CHECK(s.w64_on, "budget %d nb %d: W64 never shown", mode, nb);
         CHECK(mode == 1 || frames == (2048 + 199) / 200, "budget %d nb %d: W64 shown after %d frames", mode, nb, frames);
         CHECK(frames > 1, "budget %d nb %d: work limit not applied", mode, nb);
         CHECK(bad_shown == 0, "budget %d nb %d: %d frames showed something other than the old screen", mode, nb, bad_shown);
         CHECK(compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0) == 0, "budget %d nb %d: W64 picture", mode, nb);
         // scroll the whole screen by one line: catches up within a few frames
         memmove(vram, vram + 64, 2048 - 64);
         memset(vram + 2048 - 64, 0x60, 64);
         int n = 0;
         do {
            sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL);
            n++;
         } while (compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0) != 0 && n < 50);
         CHECK(n <= 12, "budget %d nb %d: scroll took %d frames", mode, nb, n);
         for (int i = 0; i < 4; i++) free(sim_buffers[i]);
         free(text_fb);
      }
   }
}

// CG2 decoding spread over frames (16 rows per frame)
static void test_sliced_decode(void) {
   const gen_geom_t *g = &geoms[0];
   for (int nb = 0; nb < 4; nb += 3) {
      for (int i = 0; i < 4; i++) sim_buffers[i] = alloc_fb(g);
      sim_t m = { nb, 0, 0, 0 };
      cv_state_t s; cv_init(&s);
      s.decode_rows = 16;
      uint8_t text[512], vram[2048];
      make_text_page(text, NULL);
      for (int i = 0; i < 4; i++) sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
      make_text_page(text, nitros9_regs);
      sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
      memset(vram, 0x60, sizeof(vram));
      for (int r = 0; r < 32; r++) put_text(vram, r, 0, "Sliced decoding 0123456789 abcdefghijklmnopqrstuvwxyz");
      int n = 0;
      while (!s.w64_on && n < 20) { sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL); n++; }
      CHECK(s.w64_on && n == 4, "sliced nb %d: W64 shown after %d frames (expected 4)", nb, n);
      CHECK(compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0) == 0, "sliced nb %d: picture", nb);
      // a typed character shows within 4 frames, wherever it is
      for (int k = 0; k < 8; k++) {
         int pos = (k * 523) & 2047;
         vram[pos] = (uint8_t) (0x41 + k);
         int f = 0;
         do { sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL); f++; }
         while (compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0) != 0 && f < 10);
         CHECK(f <= 4, "sliced nb %d: character %d shown after %d frames", nb, k, f);
      }
      // a text frame in between (ride over) must not put text rows into the screen
      make_text_page(text, NULL);
      sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
      for (int i = 0; i < 4; i++) sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL);
      CHECK(s.w64_on && compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0) == 0, "sliced nb %d: after an odd frame", nb);
      for (int i = 0; i < 4; i++) free(sim_buffers[i]);
   }
}

// ---------------------------------------------------------------------
// Benchmark (the one the firmware runs from the Info menu): every row must
// take the path it is meant to measure. PC timings are only a sanity check.
// ---------------------------------------------------------------------
static unsigned int pc_usec(void) {
   struct timespec ts;
   clock_gettime(CLOCK_MONOTONIC, &ts);
   return (unsigned int) (ts.tv_sec * 1000000u + ts.tv_nsec / 1000u);
}

// a clock that runs fast compared with the work (each call is 5 us later)
static unsigned int slow_clock;
static unsigned int slow_usec(void) { return slow_clock += 5; }

static void test_bench(void) {
   static uint8_t mem[CV_BENCH_MEM_SIZE];
   cv_bench_result_t res;
   cv_bench_run(mem, pc_usec, 0, 0, &res);
   CHECK(res.rows >= 10, "bench: %d rows", res.rows);
   for (int i = 0; i < res.rows; i++) {
      const cv_bench_row_t *r = &res.row[i];
      CHECK(r->ok && r->n > 0, "bench row '%s' did not take the expected path", r->name);
      printf("  bench (PC) %-22s %s %5u %5u %5u us\n", r->name, r->group ? "lower" : "     ", r->min_us, r->avg_us, r->max_us);
   }
   // with the Pi's limits and a slow clock (entering W64 takes several
   // frames, as on a Pi Zero W) every row must still take its path
   cv_bench_run(mem, slow_usec, 250, 120, &res);
   int max_frames = 0;
   for (int i = 0; i < res.rows; i++) {
      const cv_bench_row_t *r = &res.row[i];
      CHECK(r->ok && r->n > 0, "bench (slow clock) row '%s' did not take the expected path", r->name);
      if (r->frames > max_frames) max_frames = r->frames;
   }
   CHECK(max_frames > 3, "bench (slow clock): W64 entered in %d frames, clock not slow enough", max_frames);
}

// The mode change to CG2 came after the top border began (as on a PAL
// Dragon, where field sync comes late): the top border is still black
static void tamper_late_border(uint8_t *fb, const gen_geom_t *g) {
   int by0 = g->oy - 24 * g->vs;
   if (by0 < 0) by0 = 0;
   for (int y = by0; y < g->oy; y++) memset(fb + y * g->pitch, CV_BLACK, (size_t)g->fb_w);
}
// a CG2 pixel row torn by a write while the 6847 drew it (first line differs
// at a byte the vertical check samples)
static void tamper_torn_row(uint8_t *fb, const gen_geom_t *g) {
   int row = 10, b = 2;
   uint8_t *p = fb + (g->oy + row * 3 * g->vs) * g->pitch + g->ox + b * 8 * g->hs;
   uint8_t c = (*p == CV_BLUE) ? CV_RED : CV_BLUE;
   for (int x = 0; x < 2 * g->hs; x++) p[x] = c;
}

static void test_entry_holds(void) {
   for (int k = 0; k < NGEOMS; k++) {
      const gen_geom_t *g = &geoms[k];
      if (g->hs < 2 || g->vs < 2) continue;
      for (int t = 0; t < 3; t++) {
         for (int nb = 1; nb < 4; nb++) {
            for (int i = 0; i < 4; i++) sim_buffers[i] = alloc_fb(g);
            uint8_t *expect = alloc_fb(g);
            sim_t m = { nb, 0, 0, 0 };
            cv_state_t s; cv_init(&s);
            uint8_t text[512], vram[2048];
            make_text_page(text, NULL);
            for (int i = 0; i < 5; i++) sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
            make_text_page(text, nitros9_regs);
            sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
            make_text_page(text, NULL);
            sim_capture(&m, &s, g, GEN_TEXT, text, 0, NULL);
            gen_frame(expect, g, GEN_TEXT, text, 0, 0);
            memset(vram, 0x60, sizeof(vram));
            put_text(vram, 0, 0, "entering W64");
            if (t < 2) {
               // one unusable frame: the text stays, then W64 from the next
               sim_tamper = t == 0 ? tamper_late_border : tamper_torn_row;
               int cap;
               sim_capture(&m, &s, g, GEN_CG2, vram, 0, &cap);
               sim_tamper = NULL;
               CHECK(!s.w64_on && memcmp(sim_buffers[m.displayed], expect, (size_t)g->pitch * g->fb_h) == 0,
                     "geom %d nb %d %s: previous picture not held", k, nb, t ? "torn row" : "late border");
               sim_capture(&m, &s, g, GEN_CG2, vram, 0, NULL);
               CHECK(s.w64_on && compare_w64(sim_buffers[m.displayed], g, vram, 0x03, 0) == 0,
                     "geom %d nb %d %s: W64 not entered next frame", k, nb, t ? "torn row" : "late border");
               // the W64 picture has the CG border all round (not the late one)
               uint8_t top = sim_buffers[m.displayed][(g->oy - g->vs) * g->pitch + g->ox];
               CHECK(top == CV_GREEN, "geom %d nb %d: W64 top border %d", k, nb, top);
            } else {
               // a program that shows another CG picture with W64 enabled:
               // only CV_ENTRY_HOLDS frames are held, then it shows
               sim_tamper = tamper_torn_row;
               int cap = -1;
               for (int i = 0; i <= CV_ENTRY_HOLDS; i++) sim_capture(&m, &s, g, GEN_CG2, vram, 0, &cap);
               sim_tamper = NULL;
               CHECK(m.displayed == cap, "geom %d nb %d: unusable CG frames held for ever", k, nb);
            }
            for (int i = 0; i < 4; i++) free(sim_buffers[i]);
            free(expect);
         }
      }
   }
}

int main(void) {
   test_fonts();
   test_calibration();
   test_register_page();
   test_no_false_registers();
   test_cg2_decode();
   test_end_to_end();
   test_entry_holds();
   test_budget();
   test_sliced_decode();
   make_previews();
   test_bench();
   printf("%d checks, %d failures\n", checks, failures);
   return failures ? 1 : 0;
}
