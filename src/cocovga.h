/*
 * cocovga.h - CoCoVGA 64-column text emulation for MC6847 LumaCode capture
 *
 * The MC6847digitizer delivers an exact copy of the 6847 picture, one
 * colour code per 6847 dot. CoCoVGA programs (e.g. NitrOS-9 with the CoVGA
 * driver) switch on 64x32 text by writing a CoCoVGA register page and then
 * selecting the 6847's CG2 graphics mode. This module recognises the
 * register page when the 6847 displays it, reads the 2 kB of character
 * codes back out of the CG2 picture, and draws the 64x32 text screen the
 * way CoCoVGA does.
 *
 * The core (cv_*) is plain C with no RGBtoHDMI dependencies so it can be
 * tested on a PC (see cocovga_tools/).
 */
#ifndef COCOVGA_H
#define COCOVGA_H

#include <stdint.h>

// LumaCode colour codes of the MC6847digitizer (sample1 * 4 + sample2).
// These are the palette indices that land in the 8bpp frame buffer.
#define CV_BLANK      0
#define CV_BLACK      1
#define CV_RED        2
#define CV_LORANGE    3   // light orange: text in the orange colour set
#define CV_DKORANGE   4
#define CV_DKGREEN    5
#define CV_BLUE       6
#define CV_BUFF       7
#define CV_ORANGE     8
#define CV_MAGENTA    9
#define CV_CYAN      10
#define CV_GREEN     12
#define CV_YELLOW    13

#define CV_CODE_MASK 0x0F

#define CV_TEXT_COLS   64
#define CV_TEXT_ROWS   32
#define CV_VRAM_SIZE   (CV_TEXT_COLS * CV_TEXT_ROWS)   // 2048, = CG2 size

// 6847 active picture in dots
#define CV_ACTIVE_W  256
#define CV_ACTIVE_H  192

typedef enum {
   CV_FRAME_OTHER = 0,     // anything we don't care about
   CV_FRAME_TEXT,          // 6847 text/semigraphics screen (origin calibrated)
   CV_FRAME_REGISTER,      // CoCoVGA register page 0 shown in text mode
   CV_FRAME_CG2            // CG2 graphics; character codes decoded
} cv_frame_t;

// CoCoVGA register page 0 values we emulate
#define CV_ENTRY_HOLDS 2

typedef struct {
   uint8_t font;       // reg 3: bit0 = 6847T1 font, bit1 = force lower case
   uint8_t artifact;   // reg 4 (stored, unused)
   uint8_t extras;     // reg 5: bit5 = inverse-video text
   uint8_t enhanced;   // reg 8: bit1 = 64-column text
} cv_regs_t;

typedef struct {
   // calibration: where the 256x192 active picture sits in the frame buffer
   int cal_valid;
   int ox, oy;          // top left of active picture, in fb pixels
   int hs, vs;          // fb pixels per 6847 dot (horizontal, vertical)
   int fb_w, fb_h, fb_pitch;   // fb geometry the calibration belongs to

   cv_regs_t regs;

   // decoded 64x32 screen from the most recent CG2 frame
   uint8_t vram[CV_VRAM_SIZE];
   int css;                     // 0 = green colour set, 1 = buff/orange

   // what is currently drawn in the display buffer
   uint8_t shown[CV_VRAM_SIZE];
   int shown_valid;
   int shown_css;
   int shown_font;
   int refresh_pos;            // rolling refresh position

   // multi-buffer control (see cv_frame)
   int w64_on;                 // W64 picture is being displayed
   int display_buf;            // buffer holding the W64 picture
   int build_buf;              // W64 picture being built (not yet shown), -1 = none
   int full_pos;               // next character of a full redraw, -1 = none

   // work limit per frame (so the hook finishes before the next vsync):
   // drawing stops once usec() - frame start >= budget_us (after at least
   // CV_MIN_CHARS characters) or after max_chars characters; the rest is
   // carried to the next frame. usec == NULL and max_chars == 0: no limit.
   unsigned int (*usec)(void);
   unsigned int budget_us;
   int max_chars;
   unsigned int t_frame;       // usec() at the start of this frame

   // CG2 decoding is spread over frames too: each frame decodes CG2 pixel
   // rows from decode_row on, at most decode_rows (0 = all 64) and, with
   // usec set, until decode_budget_us has passed (but at least
   // CV_MIN_DECODE_ROWS). A typed character shows within 64 / rows frames.
   int decode_rows;
   unsigned int decode_budget_us;
   int decode_row;             // next CG2 pixel row (0-63) to decode
   int rows_valid;             // rows decoded since W64 (re)started, up to 64

   // the last frame looked like a CG picture but could not be used to
   // enter W64: its top border still had the previous mode's colour (the
   // mode changed after the top border began, as on a PAL Dragon, where
   // field sync comes late; the W64 picture keeps the captured border), or
   // a row was torn by a write during the scan. Up to CV_ENTRY_HOLDS such
   // frames in a row keep the previous picture instead of showing raw CG2.
   int cg_unusable;
   int entry_holds;

   // statistics (handy for the OSD / tests)
   unsigned int frames;
   unsigned int register_frames;
   unsigned int cg2_frames;
   unsigned int last_register[9];
} cv_state_t;

// Frame buffer accessor used by cv_frame
typedef uint8_t *(*cv_buffer_fn)(int index);

extern const uint8_t mc6847_font[64][12];
extern const uint8_t mc6847t1_font[96][12];
extern const uint8_t mc6847_rom_font[64][12];   // the 6847's own glyphs (for recognising cells)
// lower case glyph for code $00-$1F: as in the MC6847T1 table when the
// capitals are T1 ones (rows 1-7), else moved down 2 rows to the baseline of
// the 6847's capitals (rows 3-9)
const uint8_t *cv_lower_glyph(int code, int t1_capitals);

void       cv_init(cv_state_t *s);
void       cv_reset_registers(cv_state_t *s);

// Analyse one captured frame (8bpp, LumaCode colour codes).
cv_frame_t cv_analyze(cv_state_t *s, const uint8_t *fb, int pitch, int width, int height);

// Draw the decoded 64x32 screen into fb. full != 0 starts a redraw of every
// character, otherwise only changed characters plus a small rolling
// refresh. Stops at the work limit; returns 1 when fb is complete.
int        cv_render(cv_state_t *s, uint8_t *fb, int pitch, int full);

// Per-frame controller for RGBtoHDMI's multi-buffered capture.
//   captured     : buffer the frame was just captured into
//   displayed    : buffer currently on screen
//   nbuffers     : number of frame buffers available (4)
//   buffer       : returns the address of a buffer
// Outputs:
//   *redirect    : 0 = normal capture/flip, else (capture buffer + 1) and
//                  the frame must NOT be flipped to the screen
//   *swap_to     : -1 = nothing, else make this buffer visible now
// Returns the frame classification.
cv_frame_t cv_frame(cv_state_t *s, int captured, int displayed, int nbuffers,
                    cv_buffer_fn buffer, int pitch, int width, int height,
                    int *redirect, int *swap_to);

// Leave W64 mode (e.g. when the OSD is opened or emulation switched off).
// Returns 1 if the caller should resume normal flipping.
int        cv_stop(cv_state_t *s);

// Mark the picture on screen as lost (the screen was cleared): the next
// W64 frame redraws every character.
void       cv_invalidate(cv_state_t *s);

#endif
