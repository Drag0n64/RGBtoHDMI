/*
 * cocovga.c - CoCoVGA 64-column text emulation for MC6847 LumaCode capture
 *
 * See cocovga.h for an overview. Reference: CoCoVGA "Software Mode
 * Control" documentation (register page 0, 64-column mode W64) and the
 * NitrOS-9 CoVGA driver (level1/coco1/modules/covdg.asm).
 */
#include <string.h>
#include "cocovga.h"

// 6847 semigraphics / CG colour order -> LumaCode code
static const uint8_t sg_colour[8] = {
   CV_GREEN, CV_YELLOW, CV_BLUE, CV_RED, CV_BUFF, CV_CYAN, CV_MAGENTA, CV_ORANGE
};

// CG2 colour sets: code -> (set << 2) | value, 0xFF = not a CG colour
static uint8_t cg_lut[16];

// how many frames without CG2 before W64 is left (rides over glitches)
#define CV_LEAVE_FRAMES 2
// characters redrawn per frame by the rolling refresh
#define CV_REFRESH_CHARS 64
// characters always drawn per frame, even when over the time budget
#define CV_MIN_CHARS 32
// CG2 pixel rows always decoded per frame (a full pass takes <= 8 frames)
#define CV_MIN_DECODE_ROWS 8
// maximum Hamming distance accepted when recognising a text glyph
#define CV_GLYPH_TOLERANCE 4

static int cv_leave_count;

static inline int code_at(const uint8_t *fb, int pitch, int x, int y) {
   return fb[y * pitch + x] & CV_CODE_MASK;
}

static inline int is_dark(int c) {
   return c == CV_BLANK || c == CV_BLACK || c == 11 || c == 14 || c == 15;
}

// The frame buffer is not cached on the Pi, so every single byte read is a
// full memory access (about 0.1 us on a Pi Zero W). Lines that are scanned
// are first copied to cached memory in 32 byte blocks (struct copies, which
// compile to load-multiple bursts). The copy is only valid until the next
// fetch_line() call.
typedef struct { uint32_t w[8]; } cv_block_t;
static cv_block_t cv_line_blocks[4096 / 32 + 2];   // up to 4 KB per line

static const uint8_t *fetch_line(const uint8_t *src, int len) {
   uintptr_t a = (uintptr_t) src & ~(uintptr_t) 31;
   uintptr_t e = (uintptr_t) src + (uintptr_t) len;
   const cv_block_t *p = (const cv_block_t *) a;
   cv_block_t *d = cv_line_blocks;
   while ((uintptr_t) p < e) *d++ = *p++;
   return (const uint8_t *) cv_line_blocks + ((uintptr_t) src - a);
}

void cv_reset_registers(cv_state_t *s) {
   s->regs.font = 0;
   s->regs.artifact = 0x0D;   // CoCoVGA default: colour artifacts, MESS mode
   s->regs.extras = 0x40;     // CoCoVGA default: 10 colour SG palette
   s->regs.enhanced = 0;
}

void cv_init(cv_state_t *s) {
   memset(s, 0, sizeof(*s));
   cv_reset_registers(s);
   memset(cg_lut, 0xFF, sizeof(cg_lut));
   cg_lut[CV_GREEN]   = 0x0 | 0; cg_lut[CV_YELLOW] = 0x0 | 1;
   cg_lut[CV_BLUE]    = 0x0 | 2; cg_lut[CV_RED]    = 0x0 | 3;
   cg_lut[CV_BUFF]    = 0x4 | 0; cg_lut[CV_CYAN]   = 0x4 | 1;
   cg_lut[CV_MAGENTA] = 0x4 | 2; cg_lut[CV_ORANGE] = 0x4 | 3;
   s->display_buf = -1;
   s->build_buf = -1;
   s->full_pos = -1;
   cv_leave_count = 0;
}

// ---------------------------------------------------------------------
// Calibration: find the 256x192 active picture in a text-mode frame.
// In text modes the border is black and every text cell contains green
// or dark green, so the active picture is the box of non-dark pixels.
// ---------------------------------------------------------------------

static int find_box_on_row(const uint8_t *fb, int pitch, int width, int y, int *left, int *right) {
   int l = 0, r = width - 1;
   const uint8_t *line = width <= 4096 ? fetch_line(fb + y * pitch, width) : fb + y * pitch;
   while (l < width && is_dark(line[l] & CV_CODE_MASK)) l++;
   if (l >= width) return 0;
   while (r > l && is_dark(line[r] & CV_CODE_MASK)) r--;
   *left = l; *right = r;
   return 1;
}

static int find_box_on_col(const uint8_t *fb, int pitch, int height, int x, int *top, int *bottom) {
   int t = 0, b = height - 1;
   while (t < height && is_dark(code_at(fb, pitch, x, t))) t++;
   if (t >= height) return 0;
   while (b > t && is_dark(code_at(fb, pitch, x, b))) b--;
   *top = t; *bottom = b;
   return 1;
}

static int calibrate(cv_state_t *s, const uint8_t *fb, int pitch, int width, int height) {
   static const int fractions[4][2] = { {1, 2}, {1, 3}, {2, 3}, {3, 8} };
   for (int i = 0; i < 4; i++) {
      int y = height * fractions[i][0] / fractions[i][1];
      int l, r, t, b;
      if (!find_box_on_row(fb, pitch, width, y, &l, &r)) continue;
      int w = r - l + 1;
      if (w % CV_ACTIVE_W != 0) continue;
      int hs = w / CV_ACTIVE_W;
      if (hs < 1 || hs > 4) continue;
      int found = 0;
      for (int j = 0; j < 4 && !found; j++) {
         int x = l + w * fractions[j][0] / fractions[j][1];
         if (!find_box_on_col(fb, pitch, height, x, &t, &b)) continue;
         int h = b - t + 1;
         if (h % CV_ACTIVE_H != 0) continue;
         int vs = h / CV_ACTIVE_H;
         if (vs < 1 || vs > 4) continue;
         if (y < t || y > b) continue;
         s->ox = l; s->oy = t; s->hs = hs; s->vs = vs;
         found = 1;
      }
      if (found) {
         s->cal_valid = 1;
         s->fb_w = width; s->fb_h = height; s->fb_pitch = pitch;
         return 1;
      }
   }
   return 0;
}

// ---------------------------------------------------------------------
// Text cell decoding (used for the register page)
// Dragon/CoCo wiring: DD7 -> A/S (semigraphics), DD6 -> INV.
// Bytes $40-$7F are dark characters on green, $00-$3F green on dark.
// ---------------------------------------------------------------------

static int popcount8(unsigned int v) {
   v = v - ((v >> 1) & 0x55);
   v = (v & 0x33) + ((v >> 2) & 0x33);
   return (v + (v >> 4)) & 0x0F;
}

// Returns the byte, -1 if the cell can't be decoded, or CV_CELL_SOLID_GREEN
// for a solid green cell, which is either a space ($60) or a fully lit green
// semigraphics block ($8F): the two look identical.
// CoCoVGA's register page 0 is always displayed with CSS = 0, so only the
// green text colours are accepted; orange cells are semigraphics.
#define CV_CELL_SOLID_GREEN (-2)

// fb / pitch / ox / oy / hs / vs describe where the cell's picture is (the
// register page passes a copy of its top 12 dot lines, vs = 1)
static int decode_text_cell(const uint8_t *fb, int pitch, int ox, int oy, int hs, int vs, int col, int row) {
   int x0 = ox + col * 8 * hs;
   int y0 = oy + row * 12 * vs;
   uint8_t dark_bits[12];
   unsigned int seen = 0;              // bit mask of colour codes present
   int sg_code = -1;
   for (int dy = 0; dy < 12; dy++) {
      unsigned int bits = 0;
      const uint8_t *line = fb + (y0 + dy * vs) * pitch;
      for (int dx = 0; dx < 8; dx++) {
         int c = line[x0 + dx * hs] & CV_CODE_MASK;
         seen |= 1u << c;
         if (c == CV_DKGREEN || c == CV_DKORANGE) bits |= 0x80 >> dx;
      }
      dark_bits[dy] = bits;
   }
   unsigned int text0 = (1u << CV_GREEN) | (1u << CV_DKGREEN);
   if (seen == (1u << CV_GREEN)) return CV_CELL_SOLID_GREEN;
   if ((seen & ~text0) == 0) {
      // exact match first (the 6847's output is digital), then tolerant
      for (int i = 0; i < 64; i++) {
         if (memcmp(dark_bits, mc6847_rom_font[i], 12) == 0) return 0x40 | i;
      }
      uint8_t lit_bits[12];
      for (int dy = 0; dy < 12; dy++) lit_bits[dy] = (uint8_t) ~dark_bits[dy];
      for (int i = 0; i < 64; i++) {
         if (memcmp(lit_bits, mc6847_rom_font[i], 12) == 0) return i;
      }
      int best = 99, best_code = -1;
      for (int i = 0; i < 64; i++) {
         int dn = 0, di = 0;
         for (int dy = 0; dy < 12; dy++) {
            dn += popcount8(dark_bits[dy] ^ mc6847_rom_font[i][dy]);
            di += popcount8((~dark_bits[dy] & 0xFF) ^ mc6847_rom_font[i][dy]);
         }
         if (dn < best) { best = dn; best_code = 0x40 | i; }
         if (di < best) { best = di; best_code = i; }
      }
      return best <= CV_GLYPH_TOLERANCE ? best_code : -1;
   }
   // semigraphics 4: black plus at most one of the 8 colours
   unsigned int colours = seen & ~((1u << CV_BLACK) | (1u << CV_BLANK));
   if (colours & (colours - 1)) return -1;          // more than one colour
   int colour = 0;
   if (colours) {
      for (sg_code = 0; sg_code < 8; sg_code++) {
         if (colours == (1u << sg_colour[sg_code])) break;
      }
      if (sg_code == 8) return -1;
      colour = sg_code;
   }
   int quads = 0;
   for (int q = 0; q < 4; q++) {
      int qx = (q & 1) ? 4 : 0, qy = (q & 2) ? 6 : 0, lit = 0;
      for (int dy = 0; dy < 6; dy++) {
         for (int dx = 0; dx < 4; dx++) {
            int c = code_at(fb, pitch, x0 + (qx + dx) * hs, y0 + (qy + dy) * vs);
            if (!is_dark(c)) lit++;
         }
      }
      if (lit > 12) quads |= 8 >> q;   // bit3 = top left ... bit0 = bottom right
   }
   return 0x80 | (colour << 4) | quads;
}

static void apply_register_page(cv_state_t *s, const int *b) {
   int reset = b[0], mask = b[1];
   if (reset & 0x80) s->regs.enhanced = 0;
   if (reset & 0x04) s->regs.extras = 0x40;
   if (reset & 0x02) s->regs.artifact = 0x0D;
   if (reset & 0x01) s->regs.font = 0;
   if (mask & 0x80) s->regs.enhanced = b[8];
   if (mask & 0x04) s->regs.extras = b[5];
   if (mask & 0x02) s->regs.artifact = b[4];
   if (mask & 0x01) s->regs.font = b[3];
}

// CoCoVGA register page 0 starts with: reset, edit mask, reserved(0), font,
// artifact, extras, reserved(0), reserved(0), enhanced modes.
static int check_register_page(cv_state_t *s, const uint8_t *fb, int pitch) {
   // value for a solid green cell at each position ($60 or $8F, see above)
   static const int solid_green[9] = { 0x8F, 0x8F, -1, -1, -1, 0x60, -1, -1, -1 };
   int b[9];
   // copy the top 12 dot lines of cells 0-8 to cached memory (burst reads)
   static uint8_t cells[12][9 * 8 * 8];
   const int span = 9 * 8 * s->hs;
   if (span > (int) sizeof(cells[0])) return 0;
   for (int dy = 0; dy < 12; dy++) {
      memcpy(cells[dy], fetch_line(fb + (s->oy + dy * s->vs) * pitch + s->ox, span), span);
   }
   // the reserved registers 2, 6 and 7 are always 0: check them first, so
   // ordinary text screens are rejected after 3 cells instead of 9
   static const int order[9] = { 2, 6, 7, 0, 1, 3, 4, 5, 8 };
   for (int k = 0; k < 9; k++) {
      int i = order[k];
      b[i] = decode_text_cell(&cells[0][0], sizeof(cells[0]), 0, 0, s->hs, 1, i, 0);
      if (b[i] == CV_CELL_SOLID_GREEN) b[i] = solid_green[i];
      if (b[i] < 0) return 0;
      if (k < 3 && b[i] != 0) return 0;
   }
   if ((b[3] & 0xF8) || (b[4] & 0xF0) || (b[8] & 0xF8)) return 0;
   if ((b[0] | b[1]) == 0) return 0;
   apply_register_page(s, b);
   for (int i = 0; i < 9; i++) s->last_register[i] = b[i];
   s->register_frames++;
   return 1;
}

// ---------------------------------------------------------------------
// CG2 decoding: 128x64 four colour, each pixel 2 dots wide, 3 lines high,
// 32 bytes per row, 2 bits per pixel, leftmost pixel in bits 7-6.
// ---------------------------------------------------------------------

// decode one CG2 pixel row into out[32]; returns the colour sets seen
// (bit 0 green set, bit 1 buff set), -1 if this is not a CG2 row (colours),
// -2 if the row's lines differ (a write during the scan, or CG3 / CG6)
static int decode_cg2_row(const cv_state_t *s, const uint8_t *fb, int pitch, int row, uint8_t *out, int *pairs_differ) {
   const int hs = s->hs, vs = s->vs;
   int sets = 0;
   const uint8_t *l0 = fb + (s->oy + (row * 3 + 0) * vs) * pitch + s->ox;
   const uint8_t *l2 = fb + (s->oy + (row * 3 + 2) * vs) * pitch + s->ox;
   const uint8_t *l1 = fetch_line(fb + (s->oy + (row * 3 + 1) * vs) * pitch + s->ox, 256 * hs);
   for (int b = 0; b < 32; b++) {
      int byte = 0;
      int x = b * 8 * hs;
      for (int p = 0; p < 4; p++, x += 2 * hs) {
         int v = cg_lut[l1[x] & CV_CODE_MASK];
         if (v == 0xFF) return -1;
         sets |= 1 << (v >> 2);
         byte = (byte << 2) | (v & 3);
      }
      // vertical shape check on a quarter of the bytes: all three lines
      // of a CG2 pixel row must be identical
      if ((b & 3) == (row & 3)) {
         int xc = b * 8 * hs;
         if (((l0[xc] ^ l1[xc]) | (l2[xc] ^ l1[xc])) & CV_CODE_MASK) return -2;
      }
      if (((byte >> 6) ^ (byte >> 4)) & 3 || ((byte >> 2) ^ byte) & 3) *pairs_differ = 1;
      out[b] = (uint8_t) byte;
   }
   return sets;
}

static int decode_cg2(cv_state_t *s, const uint8_t *fb, int pitch) {
   static uint8_t rows[64][32];
   const int hs = s->hs, vs = s->vs;
   // the border above the picture must be the CG border colour (cheap first test)
   if (s->oy >= vs) {
      int c = code_at(fb, pitch, s->ox + 4 * hs, s->oy - vs);
      if (c != CV_GREEN && c != CV_BUFF) {
         // the mode may have changed after the top border began: then the
         // border beside the picture already has the CG colour
         if (s->ox >= 4 * hs) {
            int m = code_at(fb, pitch, s->ox - 4 * hs, s->oy + 96 * vs);
            if (m == CV_GREEN || m == CV_BUFF) s->cg_unusable = 1;
         }
         return 0;
      }
   }
   int sets = 0, pairs_differ = 0, n = 0;
   int row = s->decode_row;
   int limit = (s->decode_rows > 0 && s->decode_rows < 64) ? s->decode_rows : 64;
   while (n < 64) {
      int r = decode_cg2_row(s, fb, pitch, row, rows[n], &pairs_differ);
      if (r < 0) {
         if (r == -2) s->cg_unusable = 1;
         return 0;
      }
      sets |= r;
      n++;
      row = (row + 1) & 63;
      if (n >= limit) break;
      if (s->usec && s->decode_budget_us && n >= CV_MIN_DECODE_ROWS &&
          s->usec() - s->t_frame >= s->decode_budget_us) break;
   }
   if (sets == 3) return 0;          // mixed colour sets: not a CG picture
   // CG1 (4 dot wide pixels) never has two different pixels in a pair; a
   // slice of a 64 column screen can look like that (rows of $00 / $FF),
   // so this only decides when W64 is not on yet
   if (!pairs_differ && !s->w64_on) return 0;
   // all rows of this frame are good: commit them
   for (int k = 0, r = s->decode_row; k < n; k++, r = (r + 1) & 63) memcpy(s->vram + r * 32, rows[k], 32);
   s->decode_row = row;
   s->rows_valid += n;
   if (s->rows_valid > 64) s->rows_valid = 64;
   s->css = (sets & 2) ? 1 : 0;
   s->cg2_frames++;
   return 1;
}

cv_frame_t cv_analyze(cv_state_t *s, const uint8_t *fb, int pitch, int width, int height) {
   s->frames++;
   s->cg_unusable = 0;
   if (s->cal_valid && (s->fb_w != width || s->fb_h != height || s->fb_pitch != pitch)) {
      s->cal_valid = 0;
   }
   // CG2 first when 64-column mode is enabled: that is the busy path
   if (s->cal_valid && (s->regs.enhanced & 0x02)) {
      if (decode_cg2(s, fb, pitch)) return CV_FRAME_CG2;
   }
   // text frame? (re)calibrate, then look for the register page
   // (the register page may hold arbitrary data around the register bytes,
   // so if this frame can't calibrate, use the previous calibration)
   int old_ox = s->ox, old_oy = s->oy, old_hs = s->hs, old_vs = s->vs;
   int cal = calibrate(s, fb, pitch, width, height);
   if (cal && (old_ox != s->ox || old_oy != s->oy || old_hs != s->hs || old_vs != s->vs)) {
      // the picture moved: the one on screen was drawn at the old place,
      // so start again from the next captured CG2 frame
      if (s->w64_on || s->build_buf >= 0) cv_stop(s);
      s->shown_valid = 0;
   }
   if (cal || s->cal_valid) {
      if (check_register_page(s, fb, pitch)) return CV_FRAME_REGISTER;
      if (cal) return CV_FRAME_TEXT;
   }
   return CV_FRAME_OTHER;
}

// ---------------------------------------------------------------------
// W64 renderer
// ---------------------------------------------------------------------

static int renderable(const cv_state_t *s) {
   // hs <= 8: draw_char's line buffer holds 8 W64 pixels of up to 4 bytes
   return s->cal_valid && s->hs >= 2 && s->vs >= 2 && !(s->hs & 1) && !(s->vs & 1)
          && s->hs <= 8 && s->vs <= 16;
}

// The T1 table puts lower case on the baseline of its own capitals (row 7);
// the 6847's capitals sit 2 rows lower (rows 3-9). Rows 10-11 of every T1
// lower case glyph are empty, so moving them down loses nothing.
const uint8_t *cv_lower_glyph(int code, int t1_capitals) {
   static uint8_t low[32][12];
   static int ready = 0;
   if (!ready) {
      for (int c = 0; c < 32; c++) {
         for (int y = 0; y < 12; y++) low[c][y] = y >= 2 ? mc6847t1_font[64 + c][y - 2] : 0;
      }
      ready = 1;
   }
   return t1_capitals ? mc6847t1_font[64 + (code & 31)] : low[code & 31];
}

static void draw_char(const cv_state_t *s, uint8_t *fb, int pitch, int index, int value) {
   const int px = s->hs / 2, py = s->vs / 2;
   const int col = index & 63, row = index >> 6;
   uint8_t *base = fb + (s->oy + row * 12 * py) * pitch + s->ox + col * 8 * px;
   uint8_t rows[12];
   uint8_t fg, bg;
   if (value & 0x80) {
      // semigraphics 4: quadrants 4x6 W64 pixels, coloured on black
      int q_top = (value >> 2) & 3, q_bottom = value & 3;
      for (int r = 0; r < 12; r++) {
         int q = r < 6 ? q_top : q_bottom;
         rows[r] = ((q & 2) ? 0xF0 : 0) | ((q & 1) ? 0x0F : 0);
      }
      fg = sg_colour[(value >> 4) & 7];
      bg = CV_BLACK;
   } else {
      int code = value & 0x3F;
      int normal = value & 0x40;
      int t1 = s->regs.font & 1;
      int lower = s->regs.font & 2;
      const uint8_t *glyph;
      int inverse;
      if (normal) {
         glyph = t1 ? mc6847t1_font[code] : mc6847_font[code];
         inverse = 0;
      } else if (lower && code < 32) {
         glyph = cv_lower_glyph(code, t1);    // lower case replaces inverse
         inverse = 0;
      } else {
         glyph = t1 ? mc6847t1_font[code] : mc6847_font[code];
         inverse = 1;
      }
      if (s->regs.extras & 0x20) inverse ^= 1;
      uint8_t light = s->css ? CV_LORANGE : CV_GREEN;   // as the 6847 draws text
      uint8_t dark  = s->css ? CV_DKORANGE : CV_DKGREEN;
      fg = inverse ? light : dark;
      bg = inverse ? dark : light;
      memcpy(rows, glyph, 12);
   }
   if (px == 1 && (((uintptr_t) base | (uintptr_t) pitch) & 1) == 0) {
      // fast path: one W64 pixel per fb pixel, word aligned
      // (4 pixel words for each 4 bit pattern, rebuilt when the colours change)
      static uint32_t nib[16];
      static int nib_fg = -1, nib_bg = -1;
      if (fg != nib_fg || bg != nib_bg) {
         for (int n = 0; n < 16; n++) {
            uint32_t w = 0;
            for (int b = 0; b < 4; b++) {
               uint32_t c = (n & (8 >> b)) ? fg : bg;
               w |= c << (8 * b);           // little endian: leftmost pixel first
            }
            nib[n] = w;
         }
         nib_fg = fg;
         nib_bg = bg;
      }
      if ((((uintptr_t) base | (uintptr_t) pitch) & 3) == 0) {
         for (int r = 0; r < 12; r++) {
            uint32_t w0 = nib[rows[r] >> 4], w1 = nib[rows[r] & 15];
            for (int k = 0; k < py; k++) {
               ((uint32_t *) base)[0] = w0;
               ((uint32_t *) base)[1] = w1;
               base += pitch;
            }
         }
      } else {
         // picture 2 pixels off a word boundary: 16 + 32 + 16 bit stores
         for (int r = 0; r < 12; r++) {
            uint32_t w0 = nib[rows[r] >> 4], w1 = nib[rows[r] & 15];
            uint16_t h0 = (uint16_t) w0, h1 = (uint16_t) (w1 >> 16);
            uint32_t mid = (w0 >> 16) | (w1 << 16);
            for (int k = 0; k < py; k++) {
               ((uint16_t *) base)[0] = h0;
               ((uint32_t *) (base + 2))[0] = mid;
               ((uint16_t *) (base + 6))[0] = h1;
               base += pitch;
            }
         }
      }
      return;
   }
   for (int r = 0; r < 12; r++) {
      uint8_t bits = rows[r];
      uint8_t line[32];
      int n = 0;
      for (int b = 0; b < 8; b++) {
         uint8_t c = (bits & (0x80 >> b)) ? fg : bg;
         for (int k = 0; k < px; k++) line[n++] = c;
      }
      for (int k = 0; k < py; k++) {
         memcpy(base, line, n);
         base += pitch;
      }
   }
}

// work limit (see cv_state_t); checked every 8 characters
static int over_budget(const cv_state_t *s, int drawn) {
   if (s->max_chars && drawn >= s->max_chars) return 1;
   if (s->usec && drawn >= CV_MIN_CHARS && (drawn & 7) == 0) {
      return s->usec() - s->t_frame >= s->budget_us;
   }
   return 0;
}

int cv_render(cv_state_t *s, uint8_t *fb, int pitch, int full) {
   if (!renderable(s)) return 0;
   int look = s->regs.font | (s->regs.extras << 8);
   if (full || !s->shown_valid || s->shown_css != s->css || s->shown_font != look) {
      // (re)start a full redraw; it may take several frames
      if (full || s->full_pos < 0 || s->shown_css != s->css || s->shown_font != look) s->full_pos = 0;
      s->shown_valid = 1;
      s->shown_css = s->css;
      s->shown_font = look;
   }
   int drawn = 0;
   while (s->full_pos >= 0 && s->full_pos < CV_VRAM_SIZE) {
      if (over_budget(s, drawn)) return 0;
      int i = s->full_pos++;
      draw_char(s, fb, pitch, i, s->vram[i]);
      s->shown[i] = s->vram[i];
      drawn++;
   }
   s->full_pos = -1;
   // characters that changed (including ones drawn earlier in a full redraw)
   for (int i = 0; i < CV_VRAM_SIZE; i++) {
      if (s->vram[i] != s->shown[i]) {
         if (over_budget(s, drawn)) return 0;
         draw_char(s, fb, pitch, i, s->vram[i]);
         s->shown[i] = s->vram[i];
         drawn++;
      }
   }
   // rolling refresh repairs the picture if anything else touched it
   for (int n = 0; n < CV_REFRESH_CHARS; n++) {
      if (over_budget(s, drawn)) break;
      int i = s->refresh_pos;
      draw_char(s, fb, pitch, i, s->vram[i]);
      s->refresh_pos = (i + 1) & (CV_VRAM_SIZE - 1);
      drawn++;
   }
   return 1;
}

// ---------------------------------------------------------------------
// Multi-buffer controller
// ---------------------------------------------------------------------

int cv_stop(cv_state_t *s) {
   int was_on = s->w64_on;
   s->w64_on = 0;
   s->shown_valid = 0;
   s->display_buf = -1;
   s->build_buf = -1;
   s->full_pos = -1;
   s->rows_valid = 0;
   cv_leave_count = 0;
   return was_on;
}

void cv_invalidate(cv_state_t *s) {
   // the screen was cleared (mode change): the W64 picture and its CG2
   // border are gone, so start again from the next captured CG2 frame
   cv_stop(s);
}

// a buffer for the next capture that is neither on screen nor the W64 one
static int spare_buffer(int keep, int displayed, int nbuffers) {
   int c = (keep + 1) % nbuffers;
   if (c == displayed) c = (c + 1) % nbuffers;
   if (c == keep) c = (c + 1) % nbuffers;
   return c;
}

cv_frame_t cv_frame(cv_state_t *s, int captured, int displayed, int nbuffers,
                    cv_buffer_fn buffer, int pitch, int width, int height,
                    int *redirect, int *swap_to) {
   *redirect = 0;
   *swap_to = -1;
   if (s->usec) s->t_frame = s->usec();
   if (nbuffers < 2) {
      cv_stop(s);
      return CV_FRAME_OTHER;
   }
   // The W64 picture must stay in the buffer on screen. If something else
   // (e.g. the probe after a mode change) showed another buffer, start over.
   if (s->w64_on && displayed != s->display_buf) {
      cv_stop(s);
   }
   cv_frame_t f = cv_analyze(s, buffer(captured), pitch, width, height);
   // the buffer W64 owns (being built or on screen), -1 if none
   int own = s->w64_on ? s->display_buf : s->build_buf;

   if (f == CV_FRAME_REGISTER) {
      // CoCoVGA keeps showing the previous picture while it reads the
      // register page, so don't flip this frame to the screen.
      if (own >= 0) {
         *redirect = spare_buffer(own, displayed, nbuffers) + 1;
      } else if (captured != displayed) {
         *redirect = captured + 1;
      }
      return f;
   }

   if (f == CV_FRAME_CG2 && (s->regs.enhanced & 0x02) && renderable(s)) {
      cv_leave_count = 0;
      if (!s->w64_on) {
         // enter W64: build the 64x32 screen over the captured CG2 picture
         // (which has the right border), off screen, and show it when
         // complete; with Num Buffers = 0 the capture is already on screen
         int done;
         if (s->build_buf < 0) {
            s->build_buf = captured;
            done = cv_render(s, buffer(captured), pitch, 1);
         } else {
            done = cv_render(s, buffer(s->build_buf), pitch, 0);
         }
         if (done && s->rows_valid >= 64) {   // complete, up to date, every row decoded
            s->w64_on = 1;
            s->display_buf = s->build_buf;
            s->build_buf = -1;
            *swap_to = s->display_buf;
            displayed = s->display_buf;
         }
         own = s->w64_on ? s->display_buf : s->build_buf;
      } else {
         cv_render(s, buffer(s->display_buf), pitch, 0);
      }
      *redirect = spare_buffer(own, displayed, nbuffers) + 1;
      return f;
   }

   if (f != CV_FRAME_OTHER || s->w64_on) s->entry_holds = 0;
   if (f == CV_FRAME_OTHER && s->cg_unusable && !s->w64_on && (s->regs.enhanced & 0x02) &&
       s->entry_holds < CV_ENTRY_HOLDS) {
      // entering W64 (see cg_unusable): keep the previous picture for this
      // frame instead of showing the raw CG2 data
      s->entry_holds++;
      if (own >= 0) {
         *redirect = spare_buffer(own, displayed, nbuffers) + 1;
      } else if (captured != displayed) {
         *redirect = captured + 1;
      }
      return f;
   }

   if (own >= 0) {
      if (++cv_leave_count < CV_LEAVE_FRAMES && (s->regs.enhanced & 0x02)) {
         // ride over a single odd frame
         *redirect = spare_buffer(own, displayed, nbuffers) + 1;
         return f;
      }
      cv_stop(s);       // let this frame be flipped to the screen normally
   }
   return f;
}
