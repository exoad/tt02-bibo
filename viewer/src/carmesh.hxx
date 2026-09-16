// carmesh - the car as a textured mesh for the 3D view, loaded from a Wavefront
// OBJ beside the executable (viewer/assets/car, not tracked: the model is the
// operator's download, not the repository's). Absent or unreadable, the view
// keeps its wire box and says why in the log; a model is never required.
//
// PURE: parses text into triangles in the WORLD frame (scene.hxx: X right, Y
// forward, Z up, metres) and names the texture file; no file it reads reaches a
// GPU here. The suite feeds it OBJ text by hand.
//
// The model comes in its own units and axes. It is fitted, not trusted: its
// longest horizontal extent becomes the TT-02's length, its up axis becomes Z,
// and it is stood on the floor at the car's origin. Which end is the nose is a
// constant a person confirms by looking.
#pragma once

#include "shared.hxx"

namespace carmesh
{
  // The TT-02 with its shell on, matching scene.cxx's box.
  constexpr Float32 CAR_LENGTH_M = 0.44f;
  constexpr Float32 CAR_FLOOR_M = 0.02f;

  // OBJ exporters put the model's forward along -Z (Blender's default). +1
  // maps the model's +Z to the car's nose, -1 its -Z. Confirmed by eye.
  constexpr Float32 MODEL_FORWARD_SIGN = -1.0f;

  struct Vertex
  {
      Float32 x = 0.0f;   // world metres
      Float32 y = 0.0f;
      Float32 z = 0.0f;
      Float32 u = 0.0f;   // texture, origin top-left as the GPU samples it
      Float32 v = 0.0f;
  };

  struct Triangle
  {
      Array<Vertex, 3> at = {};
  };

  struct Mesh
  {
      Vec<Triangle> triangles;
      Str textureFile;    // from the MTL's map_Kd, relative to the OBJ; empty when none
      Str name;           // the first `o` line, for the log

      // The roof over the origin, world metres: the highest vertex within
      // ROOF_PATCH_M of the car's centre, or the highest anywhere when the
      // model has none there. Where the lidar hat sits.
      Float32 roofZ = 0.0f;
  };

  constexpr Float32 ROOF_PATCH_M = 0.08f;

  // OBJ text (and the MTL's, when there is one) into a fitted mesh. Polygons
  // are fanned into triangles; faces without texture coordinates get (0, 0).
  // False, with why, when there is not a single face.
  [[nodiscard]] Bool parse(const Str& obj, const Str& mtl, Mesh* out, Str& why);

  // Reads the files and parses them. `objPath` is a full path; the MTL named
  // by mtllib is read from beside it when present.
  [[nodiscard]] Bool load(const Str& objPath, Mesh* out, Str& why);

  // The directory part of a path, with its trailing separator, or empty.
  [[nodiscard]] Str directoryOf(const Str& path);
}
