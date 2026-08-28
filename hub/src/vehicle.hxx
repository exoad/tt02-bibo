// ---------------------------------------------------------------------------
// The car and the sensor, in millimetres. ONE definition, shared by the flat
// map and the 3D scene.
//
// They used to be two copies - one in radar.cxx, one in scene3d.cxx - which is
// how they came to disagree with the kit and with each other. A dimension that
// three renderers, a corridor calculation and a configuration-space erosion all
// measure against is not a local constant.
//
// SOURCES, because a number nobody can check is a number nobody can correct:
//
//   Car     Tamiya item 58631, "1/10 R/C 4WD Subaru Impreza Monte-Carlo '99
//           (TT-02 Chassis)" - the kit this project is built on.
//           tamiya.com/english/products/58631/index.htm, "Specifications":
//               Length 442 mm, Width 186 mm, Height 140 mm,
//               Wheelbase 257 mm, Tire Width/Diameter 27/69 mm (front & rear)
//
//   Sensor  Slamtec RPLIDAR C1 datasheet, as recorded in lidar/README.md:
//               55.6 x 55.6 x 41.3 mm
//
// Earlier revisions of this file carried 430 x 190 x 135 and 64 x 26 mm tyres,
// which were remembered rather than looked up. Four of the six were wrong.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hxx"

namespace vehicle {

// ---- the car ----------------------------------------------------------
inline constexpr Float32 CAR_LEN_MM       = 442.0f;
inline constexpr Float32 CAR_WID_MM       = 186.0f;   // across the tyres
inline constexpr Float32 CAR_HEIGHT_MM    = 140.0f;
inline constexpr Float32 CAR_WHEELBASE_MM = 257.0f;
inline constexpr Float32 CAR_TYRE_DIA_MM  = 69.0f;
inline constexpr Float32 CAR_TYRE_WID_MM  = 27.0f;

// DERIVED, not guessed. Tamiya publishes the overall width and the tyre width
// but not the tread, and the TT-02 offers two tread settings. The stated 186 mm
// is measured across the tyres, so the centre-to-centre tread is
//
//     186 - 27 = 159 mm
//
// which is self-consistent with both published figures. If the car is ever set
// to its wider tread this becomes wrong, and it will be wrong VISIBLY - the
// wheels will sit outside the shell.
inline constexpr Float32 CAR_TREAD_MM = CAR_WID_MM - CAR_TYRE_WID_MM;

// ---- the sensor -------------------------------------------------------
inline constexpr Float32 C1_BASE_MM = 55.6f;
inline constexpr Float32 C1_TALL_MM = 41.3f;

// ---- where the sensor sits on the car ---------------------------------
//
// ALL THREE OF THESE ARE ASSUMPTIONS. The C1 is not mounted yet; it is on a
// desk beside the car. They are written down here, with this warning, rather
// than left implicit in a renderer, so that measuring the real mount is a
// three-line edit and so that nothing downstream can mistake them for facts.
//
//   AHEAD  along the car, + toward the nose. Zero = over the middle.
//   LATERAL across the car. Zero = on the centreline.
//   BASE   the underside of the sensor above the floor. Assumed to sit on the
//          roof of a 140 mm shell.
inline constexpr Float32 C1_MOUNT_AHEAD_MM   = 0.0f;
inline constexpr Float32 C1_MOUNT_LATERAL_MM = 0.0f;
inline constexpr Float32 C1_MOUNT_BASE_MM    = CAR_HEIGHT_MM;

// The height of the SCAN PLANE above the floor - the horizontal slice the
// device actually measures. Roughly two thirds up the body, where the optics
// sit; also an assumption, and the reason every return in the 3D view is drawn
// at this height rather than at an invented one.
inline constexpr Float32 C1_OPTICAL_Z_MM = C1_TALL_MM * 0.66f;
inline constexpr Float32 C1_SCAN_Z_MM    = C1_MOUNT_BASE_MM + C1_OPTICAL_Z_MM;

} // namespace vehicle
