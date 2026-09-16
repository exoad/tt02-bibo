#include "carmesh.hxx"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace carmesh
{
  namespace
  {
    struct Raw
    {
        Float32 x = 0.0f;
        Float32 y = 0.0f;
        Float32 z = 0.0f;
    };

    struct Uv
    {
        Float32 u = 0.0f;
        Float32 v = 0.0f;
    };

    // One "a/b/c" face corner into a position index and a texture index, both
    // 1-based in OBJ and negative when counted from the end. False when the
    // position is missing.
    [[nodiscard]] Bool corner(const Str& tok, Size nPos, Size nUv, Size* pos, Size* uv, Bool* hasUv)
    {
        const Size slash = tok.find('/');
        const Str p = tok.substr(0, slash);
        Str t;
        if(slash != Str::npos)
        {
            const Size second = tok.find('/', slash + 1u);
            t = tok.substr(slash + 1u, second == Str::npos ? Str::npos : second - slash - 1u);
        }
        const long pi = std::strtol(p.c_str(), nullptr, 10);
        if(pi == 0)
        {
            return false;
        }
        const long pr = pi > 0 ? pi - 1 : static_cast<long>(nPos) + pi;
        if(pr < 0 || static_cast<Size>(pr) >= nPos)
        {
            return false;
        }
        *pos = static_cast<Size>(pr);
        *hasUv = false;
        if(!t.empty())
        {
            const long ti = std::strtol(t.c_str(), nullptr, 10);
            const long tr = ti > 0 ? ti - 1 : static_cast<long>(nUv) + ti;
            if(ti != 0 && tr >= 0 && static_cast<Size>(tr) < nUv)
            {
                *uv = static_cast<Size>(tr);
                *hasUv = true;
            }
        }
        return true;
    }

    [[nodiscard]] Vec<Str> words(const Str& line)
    {
        Vec<Str> out;
        Size at = 0;
        while(at < line.size())
        {
            while(at < line.size() && (line[at] == ' ' || line[at] == '\t' || line[at] == '\r'))
            {
                ++at;
            }
            Size end = at;
            while(end < line.size() && line[end] != ' ' && line[end] != '\t' && line[end] != '\r')
            {
                ++end;
            }
            if(end > at)
            {
                out.push_back(line.substr(at, end - at));
            }
            at = end;
        }
        return out;
    }

    [[nodiscard]] Bool readAll(const Str& path, Str* text)
    {
        std::FILE* f = std::fopen(path.c_str(), "rb");
        if(f == nullptr)
        {
            return false;
        }
        Array<Char, 4096> chunk = {};
        for(;;)
        {
            const Size n = std::fread(chunk.data(), 1, chunk.size(), f);
            if(n == 0u)
            {
                break;
            }
            text->append(chunk.data(), n);
        }
        std::fclose(f);
        return true;
    }
  }

  Str directoryOf(const Str& path)
  {
      const Size cut = path.find_last_of("/\\");
      return cut == Str::npos ? Str() : path.substr(0, cut + 1u);
  }

  Bool parse(const Str& obj, const Str& mtl, Mesh* out, Str& why)
  {
      if(out == nullptr)
      {
          why = "nowhere to put the mesh";
          return false;
      }
      Vec<Raw> positions;
      Vec<Uv> uvs;
      struct Face
      {
          Array<Size, 3> pos = {};
          Array<Size, 3> uv = {};
          Array<Bool, 3> hasUv = {};
      };
      Vec<Face> faces;
      Str name;
      Size at = 0;
      while(at <= obj.size())
      {
          Size end = obj.find('\n', at);
          if(end == Str::npos)
          {
              end = obj.size();
          }
          const Vec<Str> w = words(obj.substr(at, end - at));
          at = end + 1u;
          if(w.empty())
          {
              continue;
          }
          if(w[0] == "v" && w.size() >= 4u)
          {
              positions.push_back(Raw{ std::strtof(w[1].c_str(), nullptr), std::strtof(w[2].c_str(), nullptr), std::strtof(w[3].c_str(), nullptr) });
          }
          else if(w[0] == "vt" && w.size() >= 3u)
          {
              uvs.push_back(Uv{ std::strtof(w[1].c_str(), nullptr), std::strtof(w[2].c_str(), nullptr) });
          }
          else if(w[0] == "o" && w.size() >= 2u && name.empty())
          {
              name = w[1];
          }
          else if(w[0] == "f" && w.size() >= 4u)
          {
              // A fan from the first corner: a quad is two triangles, and so on.
              Vec<Size> p;
              Vec<Size> t;
              Vec<Bool> has;
              for(Size i = 1; i < w.size(); ++i)
              {
                  Size pi = 0;
                  Size ti = 0;
                  Bool h = false;
                  if(!corner(w[i], positions.size(), uvs.size(), &pi, &ti, &h))
                  {
                      why = "a face names a vertex that does not exist";
                      return false;
                  }
                  p.push_back(pi);
                  t.push_back(ti);
                  has.push_back(h);
              }
              for(Size i = 1; i + 1u < p.size(); ++i)
              {
                  Face f;
                  f.pos = { p[0], p[i], p[i + 1u] };
                  f.uv = { t[0], t[i], t[i + 1u] };
                  f.hasUv = { has[0], has[i], has[i + 1u] };
                  faces.push_back(f);
              }
          }
      }
      if(faces.empty())
      {
          why = "no faces in the model";
          return false;
      }
      // Fit: bounds in the model's own frame, Y up.
      Raw lo = positions[0];
      Raw hi = positions[0];
      for(const Raw& r : positions)
      {
          lo.x = std::min(lo.x, r.x);
          lo.y = std::min(lo.y, r.y);
          lo.z = std::min(lo.z, r.z);
          hi.x = std::max(hi.x, r.x);
          hi.y = std::max(hi.y, r.y);
          hi.z = std::max(hi.z, r.z);
      }
      const Float32 spanX = hi.x - lo.x;
      const Float32 spanZ = hi.z - lo.z;
      const Float32 longest = std::max(spanX, spanZ);
      if(longest <= 0.0f)
      {
          why = "the model has no size";
          return false;
      }
      const Float32 scale = CAR_LENGTH_M / longest;
      // The long axis is the car's Y. A model whose length runs along its X
      // is turned a quarter so it does too.
      const Bool lengthAlongX = spanX > spanZ;
      const Float32 cx = 0.5f * (lo.x + hi.x);
      const Float32 cz = 0.5f * (lo.z + hi.z);
      Mesh m;
      m.name = name;
      for(const Face& f : faces)
      {
          Triangle tri;
          for(Size i = 0; i < 3u; ++i)
          {
              const Raw& r = positions[f.pos[i]];
              const Float32 mx = r.x - cx;
              const Float32 mz = r.z - cz;
              Vertex v;
              if(lengthAlongX)
              {
                  v.x = -mz * scale;
                  v.y = MODEL_FORWARD_SIGN * mx * scale;
              }
              else
              {
                  v.x = mx * scale;
                  v.y = MODEL_FORWARD_SIGN * mz * scale;
              }
              v.z = (r.y - lo.y) * scale + CAR_FLOOR_M;
              if(f.hasUv[i])
              {
                  v.u = uvs[f.uv[i]].u;
                  v.v = 1.0f - uvs[f.uv[i]].v;   // OBJ's origin is bottom-left
              }
              tri.at[i] = v;
          }
          m.triangles.push_back(tri);
      }
      Float32 roofAnywhere = -1.0e9f;
      Float32 roofOverOrigin = -1.0e9f;
      for(const Triangle& t : m.triangles)
      {
          for(const Vertex& v : t.at)
          {
              roofAnywhere = std::max(roofAnywhere, v.z);
              if(std::fabs(v.x) <= ROOF_PATCH_M && std::fabs(v.y) <= ROOF_PATCH_M)
              {
                  roofOverOrigin = std::max(roofOverOrigin, v.z);
              }
          }
      }
      m.roofZ = roofOverOrigin > -1.0e8f ? roofOverOrigin : roofAnywhere;
      // The texture: the first map_Kd in the MTL, if any.
      Size mat = 0;
      while(mat < mtl.size())
      {
          Size end = mtl.find('\n', mat);
          if(end == Str::npos)
          {
              end = mtl.size();
          }
          const Vec<Str> w = words(mtl.substr(mat, end - mat));
          mat = end + 1u;
          if(w.size() >= 2u && w[0] == "map_Kd")
          {
              m.textureFile = w[w.size() - 1u];
              break;
          }
      }
      *out = m;
      return true;
  }

  Bool load(const Str& objPath, Mesh* out, Str& why)
  {
      Str obj;
      if(!readAll(objPath, &obj))
      {
          why = "cannot read " + objPath;
          return false;
      }
      // mtllib names the material file beside the model.
      Str mtl;
      const Size lib = obj.find("mtllib ");
      if(lib != Str::npos)
      {
          Size end = obj.find('\n', lib);
          if(end == Str::npos)
          {
              end = obj.size();
          }
          const Vec<Str> w = words(obj.substr(lib, end - lib));
          if(w.size() >= 2u)
          {
              if(!readAll(directoryOf(objPath) + w[1], &mtl))
              {
                  mtl.clear();
              }
          }
      }
      return parse(obj, mtl, out, why);
  }
}
