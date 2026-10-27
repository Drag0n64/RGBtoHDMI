# CoCoVGA 64-column text for RGBtoHDMI (MC6847 LumaCode)

This adds to RGBtoHDMI a
software emulation of [CoCoVGA](http://www.cocovga.com/)'s 64x32 text mode
for Dragon and Tandy CoCo machines fitted with c0pperdragon's
[MC6847digitizer](https://github.com/c0pperdragon/LumaCode/wiki/MC6847digitizer).

Programs written for CoCoVGA — in particular **NitrOS-9 with the CoVGA
driver** — show their 64-column, mixed-case text on HDMI exactly as they
would on a CoCoVGA board. No hardware changes, no changes to the digitizer
firmware: only the Pi's SD card.

## How it works

The digitizer sends an exact copy of the 6847 picture (one colour code per
6847 dot). CoCoVGA software switches 64-column mode on in two steps, and
both are visible in that picture:

1. **Register page.** The program writes CoCoVGA register page 0 into
   video memory and shows it for one frame as a normal 6847 text screen.
   The emulation recognises that page (the reserved register bytes always
   show as inverse `@`) and reads the font, extras and enhanced-mode
   registers from the characters on screen. As on CoCoVGA, the previous
   picture stays on screen during that frame.
2. **CG2.** The program then selects the 6847's CG2 mode (128x64, 4
   colours, 2 kB). Each CG2 pixel carries 2 bits, so the 2048 bytes of
   video memory are read back out of the picture without loss. With
   64-column mode enabled they are drawn as 64x32 characters using the
   MC6847 or MC6847T1 character set, with lower case, exactly where the
   6847 picture was.

While 64-column text is on screen, capture goes to a spare frame buffer and
only the characters that changed are redrawn in the buffer on screen (plus a
small rolling refresh).

The work runs between the end of one capture and the next vsync, and if it
runs past that vsync RGBtoHDMI drops a frame and may re-initialise the mode.
So the work per frame is limited (`CV_HOOK_BUDGET_US`, 250 us including
decoding, at least 32 characters per frame): characters that don't fit are
drawn in the next frame. Reading the CG2 picture back is spread too
(`CV_DECODE_BUDGET_US`, 120 us, at least 8 of the 64 CG2 pixel rows per
frame): measured on a Pi Zero W, reading the whole picture takes about
0.5 ms because the frame buffer is not cached, so a typed character appears
within a few frames (each slice is only used if every row in it is good).
Lines that are scanned are read in 32 byte bursts. On entering 64 columns the screen is built in a
frame buffer that is not on screen and shown when it is complete and up to
date (with Num Buffers = 0 the captured frame is already on screen, so the
build is visible for a few frames). Under very fast scrolling the 64 column
screen may lag the 6847 by a few frames and catch up when it stops.

### Supported

| CoCoVGA feature | Status |
|---|---|
| 64x32 text (W64), semigraphics 4 in 64 columns | yes |
| Font register: 6847 / 6847T1 character set, force lower case | yes |
| Extras: inverse-video text | yes |
| Reset / edit-mask handling of register page 0 | yes |
| Leaving W64 (any non-CG2 picture), returning to 32 columns | yes |
| Custom palettes, VG6 16-colour mode, uploaded character sets | not yet |

Not possible without help from the digitizer firmware: CoCoVGA notices the
register page through a "combination lock" on the 6847 mode pins during
vertical blanking, which leaves no trace in the picture. The emulation
therefore recognises register pages by their content instead.

## Fonts

* MC6847: the default character set built into CoCoVGA's own `CCVEDIT.BIN`
  (CoCoVGA 0.92 software).
* MC6847T1: transcribed from Motorola's MC6847T1 data sheet, Figure 24
  "Character ROM Pattern", and checked against an independent
  reconstruction (DamienG's FontStruct): 90 of 93 glyphs identical, the
  remaining 3 re-checked against the data sheet scan.

## Installing

1. Start from an RGBtoHDMI Beta 66 (or later) release on your SD card.
2. Replace `kernelrpi.img` with one built from this source
   (Pi Zero / Zero W / Zero 2 W with the default GPU capture kernel).
3. Copy `src/scripts/Palettes/MC6847_LC.bin` to `/Palettes/` and
   `src/scripts/Profiles/6-12_BIT_YUV_Analog/MC6847/` to the same place
   under `/Profiles/` (these are c0pperdragon's MC6847 LumaCode profile
   files, with `cocovga=1` added).
4. Optional: add `cocovga=0` to `/Profiles/Default.txt` (the firmware
   assumes it if the line is missing).

The setting is **Palette Menu → CoCoVGA 64 Col** (On/Off). It is on in the
MC6847 LumaCode profile and off everywhere else. The emulation only runs
with the 4-bit LumaCode palette control, 8bpp frame buffer and a double
width + height frame buffer (the profile default); it pauses while the
RGBtoHDMI menu is open.

## Building

```
cd src/scripts
cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain-arm-none-eabi-rpi.cmake ../
make
```

Produces `src/scripts/kernelrpi.img` (tested with arm-none-eabi-gcc 13.2).

## PC test harness

`src/cocovga_tools/` renders 6847 frames the way the digitizer and
RGBtoHDMI put them in the frame buffer (text, CG1/2/3/6, RG6, both colour
sets, several frame buffer geometries, random upper pixel bits) and checks
the decoder end to end, including a simulation of RGBtoHDMI's multi-buffered
capture loop:

```
make -C src/cocovga_tools test
```

It also writes `preview_w64.ppm`, a NitrOS-9 style screen as it will appear.

## Files

* `src/cocovga.c`, `src/cocovga.h` — decoder, renderer and buffer control
  (plain C, no RGBtoHDMI dependencies)
* `src/cocovga_font.c` — MC6847 and MC6847T1 character sets
* `src/rgb_to_hdmi.c` — `cocovga_frame_hook()`, called once per frame
* `src/rgb_to_fb.S` — buffer selection / flip helpers (moved out of line;
  the capture loop's size is unchanged)
* `src/osd.c`, `src/osd.h` — the `cocovga` setting

## Frame time check (Info menu)

The CoCoVGA work runs once per frame, between the end of the capture and
the next vsync. If it runs past that vsync, a frame is dropped and
RGBtoHDMI may re-initialise the mode (screen clear, re-probe). The frame
buffers are in non-cacheable memory, so this has to be measured on the Pi.
Two pages in the **Info** menu do that:

* **CoCoVGA Benchmark** - needs no Dragon. Runs every CoCoVGA path on
  generated 6847 frames in uncached RAM (the same memory type as the frame
  buffers, the screen is not touched) and shows min / avg / max time per
  frame in microseconds: 32 column text, register page, entering W64 (full
  redraw, aligned and unaligned), W64 idle / typing / scrolling, graphics,
  black screen. A "!" means a row did not take the path it measures.
* **CoCoVGA Live Timing** - with the Dragon running. Since the page was
  last opened: frames, dropped frames, frame rate, hook time per frame type
  (avg / max); since boot: re-inits caused by timing or sync changes and the
  longest hook time just before them. Both pages show the time budget,
  estimated from the measured sync: lines between the end of the capture
  and the next vsync.

## Verified with XRoar

`src/cocovga_tools/xroar/` runs the emulation on frames from XRoar's MC6847
(see its README). Every frame is checked pixel by pixel against what must be
on screen, decoded independently with XRoar's character ROMs:

* a test ROM driving the 6847 like the CoVGA driver (register page, CG2
  64x32 with typing and scrolling, back to 32 columns) on XRoar's NTSC
  Dragon, with the capture shifted by 0, 1 and 2 dots;
* NitrOS-9 Level 1 with the CoVGA driver booting to its prompt on XRoar's
  NTSC CoCo (1390 64-column frames). The real driver shows its register
  page for one frame with a mode change part way down the screen (green
  border at the top) and its screen contents around the register bytes;
  it is recognised using the previous calibration;
* Dragon 64 BASIC frames (no false register pages);
* the Dragon BASIC test disk (`src/cocovga_tools/dragon/W64TEST.VDK`, all
  tests, both fonts, orange set, inverse video, scroll and full screen
  stress) on XRoar's PAL Dragon 32 with DragonDOS, also with a work limit;
* Num Buffers 0 and 3.

On the PAL Dragon the mode change a program makes right after field sync
lands part way down the top border (the PAL padding lines come before it),
so the first CG2 frame still has a black top border. W64 keeps the border of
the frame it was built on, so that frame is not used: the previous picture
stays for it (instead of one frame of raw CG2 data) and W64 starts from the
next frame. The same is done for a first CG2 frame with a row torn by a
write during the scan, for at most `CV_ENTRY_HOLDS` (2) frames in a row.

This found that the 6847's own character set differs from CoCoVGA's in
'3' (2 dot middle stroke); cells drawn by the 6847 are recognised with the
6847's glyphs (`mc6847_rom_font`). XRoar's 'E' and 'F' have a 4 dot middle
bar, but a real MC6847digitizer capture showed 3 dots, as in CoCoVGA's set,
so those follow the real chip.

## Frame buffer depth and HDMI audio

The emulation works on the 8 bpp frame buffer of the MC6847 LumaCode
profile. RGBtoHDMI switches to 16 bpp when Audio Capture and Optimise
Profile are both on, so the profiles set `optimise=0` (the 50 Hz profile
also `audio_capture=1`): HDMI audio from the digitizer works with Optimise
off (checked on a Dragon 32). The Info pages say when the emulation can't
run and why.

## References

* CoCoVGA documentation: Software Mode Control, Theory
* NitrOS-9 `level1/coco1/modules/covdg.asm` (CoVGA register page, character
  translation)
* Motorola MC6847T1 data sheet
* c0pperdragon, LumaCode wiki: MC6847digitizer colour encoding
