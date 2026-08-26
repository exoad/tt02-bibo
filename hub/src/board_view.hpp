// Interactive 2D board diagrams.
//
// The central region of the app can show something other than the lidar map:
// a to-scale drawing of a board, with every pin hoverable. It is a live
// reference for docs/wiring.md that cannot drift out of date the way a diagram
// in a document does, because the project's own pin assignments are compiled in
// beside the physical pinout.
//
// Adding another board later means adding another entry here, not another
// screen.
#pragma once

#include "shared.hpp"

#include "imgui.h"

namespace board {

enum class Which
{
    WHICH_PICO2_W = 0,
    WHICH_COUNT
};

// Display name, for the tab that selects it.
const Char* name(Which w);

// What is known about the board RIGHT NOW, so the drawing can show the state of
// the real thing rather than a diagram of one.
//
// Every field is explicitly three-state where it can be. "We have not asked" and
// "it is off" are different facts and the drawing distinguishes them: an unknown
// LED is drawn dark and hollow, an off LED is drawn dark and filled. Defaulting
// unknowns to off would make a board nobody has talked to look like a board that
// answered and said no.
struct Live
{
    enum class Tri : UInt8 { TRI_UNKNOWN, TRI_NO, TRI_YES };
    enum class Led : UInt8 { LED_UNKNOWN, LED_OFF, LED_ON, LED_BLINK };

    Bool link       = false;   // USB CDC line is open
    Bool bootsel    = false;   // board is in BOOTSEL mass-storage mode
    Bool fwPresent = false;   // a program is on the flash

    Tri  cyw43 = Tri::TRI_UNKNOWN; // wireless chip up. The LED cannot light without it

    Led   led    = Led::LED_UNKNOWN;
    Float32 ledHz = 0.0f;       // only meaningful when led == Blink

    // Servo and ESC pulse widths, from control firmware that reports them.
    Bool havePwm = false;
    Int32  servoUs = 0;
    Int32  escUs   = 0;
};

// Draws the board into `size` of layout space, centred and scaled to fit while
// keeping its aspect ratio. Handles its own hover interaction; nothing is
// returned because nothing outside needs to know what the cursor is over.
Void draw(Which w, const ImVec2& size, const Live& live);

} // namespace board
