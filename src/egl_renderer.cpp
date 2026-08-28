// Headless EGL + OpenGL robot-depth renderer (NVIDIA default; any EGL device).
//
// Renders posed URDF meshes to an offscreen FBO with a depth attachment and
// reads back a linearized per-pixel depth in the camera optical frame. Only
// compiled when EGL/OpenGL are found (DSF_HAVE_EGL); make_egl_renderer() returns
// nullptr if init fails at runtime so the node falls back to the CPU rasterizer.
//
// NOTE: This path must be validated on the target GPU/driver. The math and GL
// state are standard, but driver quirks (surfaceless context support, GL version)
// are only observable on device.
#include "depth_self_filter/renderer.hpp"

#ifndef DSF_HAVE_EGL

namespace depth_self_filter
{
std::unique_ptr<IRenderer> make_egl_renderer() { return nullptr; }
}  // namespace depth_self_filter

#else

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// --- Minimal GL entry-point loader (core >=3.3 symbols via eglGetProcAddress) --
namespace
{
typedef void (*PFN)();
template <typename T>
static void load(T & fn, const char * n) { fn = reinterpret_cast<T>(eglGetProcAddress(n)); }

// Forward-declared function pointers (subset we use).
PFNGLGENFRAMEBUFFERSPROC glGenFramebuffers_;
PFNGLBINDFRAMEBUFFERPROC glBindFramebuffer_;
PFNGLGENRENDERBUFFERSPROC glGenRenderbuffers_;
PFNGLBINDRENDERBUFFERPROC glBindRenderbuffer_;
PFNGLRENDERBUFFERSTORAGEPROC glRenderbufferStorage_;
PFNGLFRAMEBUFFERRENDERBUFFERPROC glFramebufferRenderbuffer_;
PFNGLCHECKFRAMEBUFFERSTATUSPROC glCheckFramebufferStatus_;
PFNGLGENVERTEXARRAYSPROC glGenVertexArrays_;
PFNGLBINDVERTEXARRAYPROC glBindVertexArray_;
PFNGLGENBUFFERSPROC glGenBuffers_;
PFNGLBINDBUFFERPROC glBindBuffer_;
PFNGLBUFFERDATAPROC glBufferData_;
PFNGLCREATESHADERPROC glCreateShader_;
PFNGLSHADERSOURCEPROC glShaderSource_;
PFNGLCOMPILESHADERPROC glCompileShader_;
PFNGLGETSHADERIVPROC glGetShaderiv_;
PFNGLGETSHADERINFOLOGPROC glGetShaderInfoLog_;
PFNGLCREATEPROGRAMPROC glCreateProgram_;
PFNGLATTACHSHADERPROC glAttachShader_;
PFNGLLINKPROGRAMPROC glLinkProgram_;
PFNGLGETPROGRAMIVPROC glGetProgramiv_;
PFNGLUSEPROGRAMPROC glUseProgram_;
PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocation_;
PFNGLUNIFORMMATRIX4FVPROC glUniformMatrix4fv_;
PFNGLENABLEVERTEXATTRIBARRAYPROC glEnableVertexAttribArray_;
PFNGLVERTEXATTRIBPOINTERPROC glVertexAttribPointer_;
PFNGLDRAWBUFFERSPROC glDrawBuffers_;

bool load_gl()
{
  load(glGenFramebuffers_, "glGenFramebuffers");
  load(glBindFramebuffer_, "glBindFramebuffer");
  load(glGenRenderbuffers_, "glGenRenderbuffers");
  load(glBindRenderbuffer_, "glBindRenderbuffer");
  load(glRenderbufferStorage_, "glRenderbufferStorage");
  load(glFramebufferRenderbuffer_, "glFramebufferRenderbuffer");
  load(glCheckFramebufferStatus_, "glCheckFramebufferStatus");
  load(glGenVertexArrays_, "glGenVertexArrays");
  load(glBindVertexArray_, "glBindVertexArray");
  load(glGenBuffers_, "glGenBuffers");
  load(glBindBuffer_, "glBindBuffer");
  load(glBufferData_, "glBufferData");
  load(glCreateShader_, "glCreateShader");
  load(glShaderSource_, "glShaderSource");
  load(glCompileShader_, "glCompileShader");
  load(glGetShaderiv_, "glGetShaderiv");
  load(glGetShaderInfoLog_, "glGetShaderInfoLog");
  load(glCreateProgram_, "glCreateProgram");
  load(glAttachShader_, "glAttachShader");
  load(glLinkProgram_, "glLinkProgram");
  load(glGetProgramiv_, "glGetProgramiv");
  load(glUseProgram_, "glUseProgram");
  load(glGetUniformLocation_, "glGetUniformLocation");
  load(glUniformMatrix4fv_, "glUniformMatrix4fv");
  load(glEnableVertexAttribArray_, "glEnableVertexAttribArray");
  load(glVertexAttribPointer_, "glVertexAttribPointer");
  load(glDrawBuffers_, "glDrawBuffers");
  return glGenFramebuffers_ && glCreateProgram_ && glUniformMatrix4fv_;
}

const char * kVertPlane =
  "#version 330 core\n"
  "layout(location=0) in vec3 pos;\n"
  "uniform mat4 mvp;\n"
  "out vec3 vpos;\n"
  // Positions arrive already in the camera optical frame, so the varying IS the
  // eye-space position and needs no extra matrix.
  "void main(){ vpos = pos; gl_Position = mvp * vec4(pos,1.0); }\n";
// Emit the exact plane of the triangle covering this fragment.
//
// vpos is interpolated perspective-correctly, so for a planar triangle its
// screen-space derivatives are two independent tangent vectors OF THAT PLANE and
// their cross product is the exact face normal — no per-triangle attribute, no
// geometry shader, no vertex duplication. d follows as n . vpos because the
// interpolated position lies on the plane.
//
// Caveat: derivatives are evaluated per 2x2 quad, so a fragment straddling two
// triangles of different orientation (an interior crease, e.g. where two box
// faces meet) can mix them for a ~1 px band. Silhouettes are already excluded by
// the consumer's depth-edge gate; creases are not, which is why render_probe
// measures the EGL-vs-CPU plane disagreement rather than assuming it is zero.
const char * kFragPlane =
  "#version 330 core\n"
  "in vec3 vpos;\n"
  "layout(location=1) out vec4 plane;\n"
  "void main(){\n"
  "  vec3 g = cross(dFdx(vpos), dFdy(vpos));\n"
  "  float L = length(g);\n"
  "  if (L < 1e-20) { plane = vec4(0.0); return; }\n"
  "  vec3 n = g / L;\n"
  "  if (dot(n, vpos) > 0.0) n = -n;\n"   // face the camera (+z forward)
  "  plane = vec4(n, dot(n, vpos));\n"
  "}\n";

const char * kVert =
  "#version 330 core\n"
  "layout(location=0) in vec3 pos;\n"
  "uniform mat4 mvp;\n"
  "void main(){ gl_Position = mvp * vec4(pos,1.0); }\n";
const char * kFrag =
  "#version 330 core\n"
  "void main(){}\n";  // depth-only; color is unused
}  // namespace

namespace depth_self_filter
{
namespace
{

class EglRenderer : public IRenderer
{
public:
  bool init()
  {
    // Headless NVIDIA: enumerate EGL devices and open a device-platform display
    // (no X server). Fall back to EGL_DEFAULT_DISPLAY for Mesa/other setups.
    dpy_ = open_device_display();
    if (dpy_ == EGL_NO_DISPLAY) dpy_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (dpy_ == EGL_NO_DISPLAY) return false;
    if (!eglInitialize(dpy_, nullptr, nullptr)) return false;
    if (!eglBindAPI(EGL_OPENGL_API)) return false;

    const EGLint cfg_attribs[] = {
      EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
      EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
      EGL_NONE};
    EGLConfig cfg;
    EGLint n = 0;
    if (!eglChooseConfig(dpy_, cfg_attribs, &cfg, 1, &n) || n < 1) return false;

    const EGLint ctx_attribs[] = {
      EGL_CONTEXT_MAJOR_VERSION, 3,
      EGL_CONTEXT_MINOR_VERSION, 3,
      EGL_NONE};
    ctx_ = eglCreateContext(dpy_, cfg, EGL_NO_CONTEXT, ctx_attribs);
    if (ctx_ == EGL_NO_CONTEXT) return false;
    if (!eglMakeCurrent(dpy_, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx_)) return false;

    if (!load_gl()) return false;

    // Diagnostics: which GPU/driver actually backs this context.
    const GLubyte * rend = glGetString(GL_RENDERER);
    const GLubyte * vend = glGetString(GL_VENDOR);
    std::fprintf(stderr, "[depth_self_filter] EGL context: %s / %s\n",
      vend ? reinterpret_cast<const char *>(vend) : "?",
      rend ? reinterpret_cast<const char *>(rend) : "?");

    if (!build_program()) return false;

    glGenVertexArrays_(1, &vao_);
    glGenBuffers_(1, &vbo_);
    glGenBuffers_(1, &ebo_);
    return true;
  }

  ~EglRenderer() override
  {
    if (dpy_ != EGL_NO_DISPLAY) {
      eglMakeCurrent(dpy_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
      if (ctx_ != EGL_NO_CONTEXT) eglDestroyContext(dpy_, ctx_);
      eglTerminate(dpy_);
    }
  }

  bool render(
    const CameraModel & cam,
    const std::vector<RenderItem> & items,
    std::vector<float> & depth_out) override
  {
    if (!ensure_fbo(cam.width, cam.height)) return false;
    glBindFramebuffer_(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, cam.width, cam.height);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glClearColor(0, 0, 0, 0);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram_(prog_);

    const Eigen::Matrix4f P = projection(cam);
    for (const auto & item : items) {
      if (!item.mesh || item.mesh->indices.empty()) continue;
      draw(P, cam, item, mvp_loc_);
    }

    read_depth(cam, depth_out);
    glBindFramebuffer_(GL_FRAMEBUFFER, 0);
    return true;
  }

  // Same rasterization, additionally reading back the exact plane per pixel.
  bool render_planes(
    const CameraModel & cam,
    const std::vector<RenderItem> & items,
    std::vector<float> & depth_out,
    std::vector<float> & plane_out) override
  {
    if (!prog_plane_ || !glDrawBuffers_) return false;   // caller falls back
    if (!ensure_fbo(cam.width, cam.height)) return false;
    glBindFramebuffer_(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, cam.width, cam.height);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    const GLenum bufs[2] = {GL_NONE, GL_COLOR_ATTACHMENT1};
    glDrawBuffers_(2, bufs);
    glClearColor(0, 0, 0, 0);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram_(prog_plane_);

    const Eigen::Matrix4f P = projection(cam);
    for (const auto & item : items) {
      if (!item.mesh || item.mesh->indices.empty()) continue;
      draw(P, cam, item, mvp_plane_loc_);
    }

    read_depth(cam, depth_out);
    read_planes(cam, depth_out, plane_out);

    const GLenum back[2] = {GL_COLOR_ATTACHMENT0, GL_NONE};
    glDrawBuffers_(2, back);
    glBindFramebuffer_(GL_FRAMEBUFFER, 0);
    return true;
  }

  const char * name() const override { return "egl"; }

private:
  // Try EGL_EXT_platform_device: enumerate EGL devices and open a surfaceless
  // display on a *hardware* device. Multiple vendors can be present (e.g. the
  // NVIDIA GPU plus a Mesa software device), so we prefer a device exposing
  // EGL_EXT_device_drm (a real render node) over a software rasterizer.
  static EGLDisplay open_device_display()
  {
    auto queryDevices = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(
      eglGetProcAddress("eglQueryDevicesEXT"));
    auto getPlatformDisplay = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
      eglGetProcAddress("eglGetPlatformDisplayEXT"));
    auto queryDeviceString = reinterpret_cast<PFNEGLQUERYDEVICESTRINGEXTPROC>(
      eglGetProcAddress("eglQueryDeviceStringEXT"));
    if (!queryDevices || !getPlatformDisplay) return EGL_NO_DISPLAY;

    EGLDeviceEXT devices[16];
    EGLint num = 0;
    if (!queryDevices(16, devices, &num) || num < 1) return EGL_NO_DISPLAY;

    int chosen = 0;
    if (queryDeviceString) {
      for (int i = 0; i < num; ++i) {
        const char * ext = queryDeviceString(devices[i], EGL_EXTENSIONS);
        if (ext && std::strstr(ext, "EGL_EXT_device_drm")) { chosen = i; break; }
      }
    }
    return getPlatformDisplay(EGL_PLATFORM_DEVICE_EXT, devices[chosen], nullptr);
  }

  // GL projection from pinhole intrinsics, optical frame (+x right, +y down,
  // +z forward). We feed optical-frame eye coords directly and map them so that
  //   ndc_x = 2u/w - 1,  ndc_y = 2v/h - 1   (u,v = pixel coords)
  // Because image v is y-DOWN while GL's window origin is bottom-left, this
  // ndc_y already lands image-top at window-row 0 — glReadPixels then returns
  // rows in image order, so read_depth does NOT flip vertically.
  static Eigen::Matrix4f projection(const CameraModel & c)
  {
    const float n = c.near, f = c.far, w = c.width, h = c.height;
    Eigen::Matrix4f P = Eigen::Matrix4f::Zero();
    P(0, 0) = 2.f * c.fx / w;
    P(0, 2) = 2.f * c.cx / w - 1.f;
    P(1, 1) = 2.f * c.fy / h;
    P(1, 2) = 2.f * c.cy / h - 1.f;
    P(2, 2) = (f + n) / (f - n);       // optical +z forward -> +z clip
    P(2, 3) = -2.f * f * n / (f - n);
    P(3, 2) = 1.f;
    return P;
  }

  void draw(
    const Eigen::Matrix4f & P, const CameraModel &, const RenderItem & item,
    GLint mvp_loc)
  {
    const Mesh & m = *item.mesh;
    // Positions transformed to camera optical frame on CPU (small meshes); the
    // shader only applies the projection. Keeps the vertex format trivial.
    std::vector<float> verts(m.vertices.size() * 3);
    for (size_t i = 0; i < m.vertices.size(); ++i) {
      const Eigen::Vector3f p = item.T_cam_link * m.vertices[i];
      verts[3 * i + 0] = p.x();
      verts[3 * i + 1] = p.y();
      verts[3 * i + 2] = p.z();
    }
    glBindVertexArray_(vao_);
    glBindBuffer_(GL_ARRAY_BUFFER, vbo_);
    glBufferData_(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(),
      GL_DYNAMIC_DRAW);
    glBindBuffer_(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData_(GL_ELEMENT_ARRAY_BUFFER, m.indices.size() * sizeof(uint32_t),
      m.indices.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray_(0);
    glVertexAttribPointer_(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glUniformMatrix4fv_(mvp_loc, 1, GL_FALSE, P.data());  // Eigen is col-major
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m.indices.size()),
      GL_UNSIGNED_INT, nullptr);
  }

  void read_depth(const CameraModel & cam, std::vector<float> & out)
  {
    const int w = cam.width, h = cam.height;
    std::vector<float> zbuf(static_cast<size_t>(w) * h);
    glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, zbuf.data());
    out.assign(static_cast<size_t>(w) * h, 0.f);
    const float n = cam.near, f = cam.far;
    for (int y = 0; y < h; ++y) {
      // No vertical flip: the projection maps image-top to GL window-row 0, so
      // glReadPixels already returns rows in top-left image order (see above).
      for (int x = 0; x < w; ++x) {
        const float d = zbuf[static_cast<size_t>(y) * w + x];
        if (d >= 1.0f) continue;  // cleared background -> invalid (0)
        const float z_ndc = 2.f * d - 1.f;
        const float z_eye = 2.f * n * f / (f + n - z_ndc * (f - n));
        out[static_cast<size_t>(y) * w + x] = z_eye;
      }
    }
  }

  // Read attachment 1 and zero it wherever no geometry was drawn, so the plane
  // buffer's validity exactly matches the depth buffer's.
  void read_planes(
    const CameraModel & cam, const std::vector<float> & depth,
    std::vector<float> & out)
  {
    const size_t n = static_cast<size_t>(cam.width) * cam.height;
    out.assign(n * 4, 0.f);
    glReadBuffer(GL_COLOR_ATTACHMENT1);
    glReadPixels(0, 0, cam.width, cam.height, GL_RGBA, GL_FLOAT, out.data());
    for (size_t i = 0; i < n; ++i) {
      if (depth[i] <= 0.f) {
        for (int k = 0; k < 4; ++k) out[i * 4 + k] = 0.f;
      }
    }
  }

  bool ensure_fbo(int w, int h)
  {
    if (w == fbo_w_ && h == fbo_h_ && fbo_) return true;
    if (!fbo_) {
      glGenFramebuffers_(1, &fbo_);
      glGenRenderbuffers_(1, &color_rb_);
      glGenRenderbuffers_(1, &plane_rb_);
      glGenRenderbuffers_(1, &depth_rb_);
    }
    glBindFramebuffer_(GL_FRAMEBUFFER, fbo_);
    glBindRenderbuffer_(GL_RENDERBUFFER, color_rb_);
    glRenderbufferStorage_(GL_RENDERBUFFER, GL_RGBA8, w, h);
    glFramebufferRenderbuffer_(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
      GL_RENDERBUFFER, color_rb_);
    // Attachment 1 carries the per-pixel plane (nx, ny, nz, d) at full float
    // precision — 8-bit would be useless for a metric offset.
    glBindRenderbuffer_(GL_RENDERBUFFER, plane_rb_);
    glRenderbufferStorage_(GL_RENDERBUFFER, GL_RGBA32F, w, h);
    glFramebufferRenderbuffer_(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1,
      GL_RENDERBUFFER, plane_rb_);
    glBindRenderbuffer_(GL_RENDERBUFFER, depth_rb_);
    glRenderbufferStorage_(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, w, h);
    glFramebufferRenderbuffer_(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
      GL_RENDERBUFFER, depth_rb_);
    const bool ok =
      glCheckFramebufferStatus_(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    fbo_w_ = w; fbo_h_ = h;
    return ok;
  }

  bool build_program()
  {
    prog_ = link(kVert, kFrag);
    if (!prog_) return false;
    mvp_loc_ = glGetUniformLocation_(prog_, "mvp");
    // Plane output is optional: if it fails to build we simply report no planes
    // and callers fall back, rather than losing the renderer entirely.
    prog_plane_ = link(kVertPlane, kFragPlane);
    if (prog_plane_) mvp_plane_loc_ = glGetUniformLocation_(prog_plane_, "mvp");
    return true;
  }

  GLuint link(const char * vsrc, const char * fsrc)
  {
    GLuint vs = compile(GL_VERTEX_SHADER, vsrc);
    GLuint fs = compile(GL_FRAGMENT_SHADER, fsrc);
    if (!vs || !fs) return 0;
    GLuint p = glCreateProgram_();
    glAttachShader_(p, vs);
    glAttachShader_(p, fs);
    glLinkProgram_(p);
    GLint ok = 0;
    glGetProgramiv_(p, GL_LINK_STATUS, &ok);
    return ok ? p : 0;
  }

  GLuint compile(GLenum type, const char * src)
  {
    GLuint s = glCreateShader_(type);
    glShaderSource_(s, 1, &src, nullptr);
    glCompileShader_(s);
    GLint ok = 0;
    glGetShaderiv_(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
      char log[512];
      glGetShaderInfoLog_(s, sizeof(log), nullptr, log);
      std::fprintf(stderr, "[depth_self_filter] shader compile failed: %s\n", log);
      return 0;
    }
    return s;
  }

  EGLDisplay dpy_ = EGL_NO_DISPLAY;
  EGLContext ctx_ = EGL_NO_CONTEXT;
  GLuint prog_ = 0, prog_plane_ = 0, vao_ = 0, vbo_ = 0, ebo_ = 0;
  GLuint fbo_ = 0, color_rb_ = 0, plane_rb_ = 0, depth_rb_ = 0;
  GLint mvp_loc_ = -1, mvp_plane_loc_ = -1;
  int fbo_w_ = -1, fbo_h_ = -1;
};

}  // namespace

std::unique_ptr<IRenderer> make_egl_renderer()
{
  auto r = std::make_unique<EglRenderer>();
  if (!r->init()) return nullptr;
  return r;
}

}  // namespace depth_self_filter

#endif  // DSF_HAVE_EGL
