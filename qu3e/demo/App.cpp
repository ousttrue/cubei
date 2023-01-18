#include "App.h"
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif
#include <imgui.h>
#include "Demo.h"
#include "stb_image.h"
#include <GLFW/glfw3.h>
#include <gl/GLU.h>

int mouseX;
int mouseY;
bool mouseLeftDown;
bool mouseRightDown;
static GLuint fontTex;
int windowWidth;
int windowHeight;

// This is the main rendering function that you have to implement and provide to
// ImGui (via setting up 'RenderDrawListsFn' in the ImGuiIO structure) If text
// or lines are blurry when integrating ImGui in your engine:
// - in your Render function, try translating your projection matrix by
// (0.5f,0.5f) or (0.375f,0.375f)
// - try adjusting ImGui::GetIO().PixelCenterOffset to 0.5f or 0.375f
static void ImImpl_RenderDrawLists(ImDrawList **const cmd_lists,
                                   int cmd_lists_count) {
  if (cmd_lists_count == 0)
    return;

  // We are using the OpenGL fixed pipeline to make the example code simpler to
  // read! A probable faster way to render would be to collate all vertices from
  // all cmd_lists into a single vertex buffer. Setup render state:
  // alpha-blending enabled, no face culling, no depth testing, scissor enabled,
  // vertex/texcoord/color pointers.
  glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_SCISSOR_TEST);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  // Setup texture
  glBindTexture(GL_TEXTURE_2D, fontTex);
  glEnable(GL_TEXTURE_2D);

  // Setup orthographic projection matrix
  const float width = ImGui::GetIO().DisplaySize.x;
  const float height = ImGui::GetIO().DisplaySize.y;
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0.0f, width, height, 0.0f, -1.0f, +1.0f);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  // Render command lists
  for (int n = 0; n < cmd_lists_count; n++) {
    const ImDrawList *cmd_list = cmd_lists[n];
    const unsigned char *vtx_buffer =
        (const unsigned char *)&cmd_list->vtx_buffer.front();
    glVertexPointer(2, GL_FLOAT, sizeof(ImDrawVert),
                    (void *)(vtx_buffer + offsetof(ImDrawVert, pos)));
    glTexCoordPointer(2, GL_FLOAT, sizeof(ImDrawVert),
                      (void *)(vtx_buffer + offsetof(ImDrawVert, uv)));
    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(ImDrawVert),
                   (void *)(vtx_buffer + offsetof(ImDrawVert, col)));

    int vtx_offset = 0;
    for (size_t cmd_i = 0; cmd_i < cmd_list->commands.size(); cmd_i++) {
      const ImDrawCmd *pcmd = &cmd_list->commands[cmd_i];
      glScissor((int)pcmd->clip_rect.x, (int)(height - pcmd->clip_rect.w),
                (int)(pcmd->clip_rect.z - pcmd->clip_rect.x),
                (int)(pcmd->clip_rect.w - pcmd->clip_rect.y));
      glDrawArrays(GL_TRIANGLES, vtx_offset, pcmd->vtx_count);
      vtx_offset += pcmd->vtx_count;
    }
  }
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);

  // Restore modified state
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glPopAttrib();
}

void mouse_button_callback(GLFWwindow *window, int button, int action,
                           int mods) {
  if (action == GLFW_PRESS) {
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT: {
      mouseLeftDown = true;
      DemosMouseLeftDown(mouseX, mouseY);
    } break;
    case GLFW_MOUSE_BUTTON_RIGHT: {
      mouseRightDown = true;
    } break;
    }
  } else if (action == GLFW_RELEASE) {
    switch (button) {
    case GLFW_MOUSE_BUTTON_LEFT: {
      mouseLeftDown = false;
    } break;
    case GLFW_MOUSE_BUTTON_RIGHT: {
      mouseRightDown = false;
    } break;
    }
  }
}

static void cursor_position_callback(GLFWwindow *window, double xpos,
                                     double ypos) {
  mouseX = static_cast<int>(xpos);
  mouseY = static_cast<int>(ypos);
  ImGuiIO &io = ImGui::GetIO();
  io.MousePos = ImVec2((float)mouseX, (float)mouseY);
}

namespace Camera {
float position[3] = {0.0f, 5.0f, 20.0f};
float target[3] = {0.0f, 0.0f, 0.0f};
}; // namespace Camera

namespace Light {
float ambient[4] = {1.0f, 1.0f, 1.0f, 0.5f};
float diffuse[4] = {0.2f, 0.4f, 0.7f, 1.0f};
float specular[4] = {1.0f, 1.0f, 1.0f, 1.0f};
} // namespace Light

void key_callback(GLFWwindow *window, int key, int scancode, int action,
                  int mods) {
  const float increment = 0.2f;

  if (action == GLFW_PRESS) {
    switch (key) {
    case 27:
      exit(0);
      break;
    case 'p':
      DemosTogglePause();
      break;
    case ' ':
      DemosSingleStep();
      break;
    case 'w':
      Camera::position[2] -= increment;
      Camera::target[2] -= increment;
      break;
    case 's':
      Camera::position[2] += increment;
      Camera::target[2] += increment;
      break;
    case 'a':
      Camera::position[0] -= increment;
      Camera::target[0] -= increment;
      break;
    case 'd':
      Camera::position[0] += increment;
      Camera::target[0] += increment;
      break;
    case 'q':
      Camera::position[1] -= increment;
      Camera::target[1] -= increment;
      break;
    case 'e':
      Camera::position[1] += increment;
      Camera::target[1] += increment;
      break;
    default:
      // printf( "%d\n", key );
      break;
    }
    DemosKeyDown(key);
  } else if (action == GLFW_RELEASE) {
    DemosKeyUp(key);
  }

  ImGuiIO &io = ImGui::GetIO();
  io.KeysDown[key] = true;

  // int mod = glutGetModifiers();
  io.KeyCtrl = (mods & GLFW_MOD_CONTROL) != 0;
  io.KeyShift = (mods & GLFW_MOD_SHIFT) != 0;
}

void window_size_callback(GLFWwindow *window, int width, int height) {
  if (height <= 0)
    height = 1;

  windowWidth = width;
  windowHeight = height;

  float aspectRatio = (float)width / (float)height;
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0f, aspectRatio, 0.1f, 10000.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(Camera::position[0], Camera::position[1], Camera::position[2],
            Camera::target[0], Camera::target[1], Camera::target[2], 0.0f, 1.0f,
            0.0f);
}

void UpdateImGui(float dt) {
  ImGuiIO &io = ImGui::GetIO();

  io.DeltaTime = dt;
  io.DisplaySize.x = float(windowWidth);
  io.DisplaySize.y = float(windowHeight);
  io.MousePos = ImVec2((float)mouseX, (float)mouseY + 8);
  io.MouseDown[0] = mouseLeftDown;
  io.MouseDown[1] = mouseRightDown;

  // Start the frame
  ImGui::NewFrame();
}

void InitImGui(int w, int h) {
  ImGuiIO &io = ImGui::GetIO();
  io.DisplaySize = ImVec2(
      (float)w,
      (float)h); // Display size, in pixels. For clamping windows positions.
  io.DeltaTime = 1.0f / 60.0f; // Time elapsed since last frame, in seconds (in
                               // this sample app we'll override this every
                               // frame because our timestep is variable)
  io.PixelCenterOffset = 0.0f; // Align OpenGL texels
  io.KeyMap[ImGuiKey_Tab] = 9; // Keyboard mapping. ImGui will use those
                               // indices to peek into the io.KeyDown[] array.
  io.KeyMap[ImGuiKey_LeftArrow] = GLFW_KEY_LEFT;
  io.KeyMap[ImGuiKey_RightArrow] = GLFW_KEY_RIGHT;
  io.KeyMap[ImGuiKey_UpArrow] = GLFW_KEY_UP;
  io.KeyMap[ImGuiKey_DownArrow] = GLFW_KEY_DOWN;
  io.KeyMap[ImGuiKey_Home] = GLFW_KEY_HOME;
  io.KeyMap[ImGuiKey_End] = GLFW_KEY_END;
  io.KeyMap[ImGuiKey_Delete] = 127;
  io.KeyMap[ImGuiKey_Backspace] = 8;
  io.KeyMap[ImGuiKey_Enter] = 13;
  io.KeyMap[ImGuiKey_Escape] = 27;
  io.KeyMap[ImGuiKey_A] = 'a';
  io.KeyMap[ImGuiKey_C] = 'c';
  io.KeyMap[ImGuiKey_V] = 'v';
  io.KeyMap[ImGuiKey_X] = 'x';
  io.KeyMap[ImGuiKey_Y] = 'y';
  io.KeyMap[ImGuiKey_Z] = 'z';

  io.RenderDrawListsFn = ImImpl_RenderDrawLists;
  // io.SetClipboardTextFn = ImImpl_SetClipboardTextFn;
  // io.GetClipboardTextFn = ImImpl_GetClipboardTextFn;

  glGenTextures(1, &fontTex);
  glBindTexture(GL_TEXTURE_2D, fontTex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  // Default font (embedded in code)
  const void *png_data;
  unsigned int png_size;
  ImGui::GetDefaultFontData(NULL, NULL, &png_data, &png_size);
  int tex_x, tex_y, tex_comp;
  void *tex_data =
      stbi_load_from_memory((const unsigned char *)png_data, (int)png_size,
                            &tex_x, &tex_y, &tex_comp, 0);
  IM_ASSERT(tex_data != NULL);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex_x, tex_y, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, tex_data);
  stbi_image_free(tex_data);

  // Disable usage of .ini file
  ImGui::GetIO().IniSavingRate = -1.0f;
}

int InitApp(int argc, char **argv) {
  // Starting width / height of the window
  const uint32_t kWindowWidth = 1000;
  const uint32_t kWindowHeight = 600;

  // Initialize GLUT
  if (!glfwInit()) {
    return -1;
  }

  // Setup the window
  auto window = glfwCreateWindow(kWindowWidth, kWindowHeight,
                                 "qu3e Physics by Randy Gaul", NULL, NULL);
  if (!window) {
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  glfwSetWindowSizeCallback(window, window_size_callback);
  glfwSetKeyCallback(window, key_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, cursor_position_callback);

  // Setup all the open-gl states we want to use (ones that don't change in
  // the lifetime of the application) Note: These can be changed anywhere, but
  // generally we don't change the back buffer color
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  // Used FFP to setup lights
  float floats[4];
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)Light::ambient[i];
  glLightfv(GL_LIGHT0, GL_AMBIENT, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)Light::diffuse[i];
  glLightfv(GL_LIGHT0, GL_DIFFUSE, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)Light::specular[i];
  glLightfv(GL_LIGHT0, GL_SPECULAR, floats);
  for (int i = 0; i < 4; ++i)
    floats[i] = (float)Camera::position[i];
  glLightfv(GL_LIGHT0, GL_POSITION, floats);
  glEnable(GL_LIGHT0);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

  int w, h;
  glfwGetFramebufferSize(window, &w, &h);
  InitImGui(w, h);

  while (!glfwWindowShouldClose(window)) { // message loop
    glfwPollEvents();
    glfwGetFramebufferSize(window, &w, &h);

    DemosUpdate();
    window_size_callback(window, w, h);
    UpdateImGui(1.0f / 60.0f);
    DemosGui();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    DemosRender();
    ImGui::Render();

    glfwSwapBuffers(window);
  }

  glfwTerminate();
  return 0;
}
