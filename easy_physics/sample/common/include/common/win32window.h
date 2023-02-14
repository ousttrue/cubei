#pragma once
#include <Windows.h>
#include <stdexcept>
#include <string>

class WglContext {
  PIXELFORMATDESCRIPTOR pfd = {
      sizeof(PIXELFORMATDESCRIPTOR),
      1,
      PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
      PFD_TYPE_RGBA,
      32,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      32,
      0,
      0,
      PFD_MAIN_PLANE,
      0,
      0,
      0,
      0,
  };

  HWND hWnd_ = {};
  HDC s_hDC = {};
  HGLRC s_hRC = {};

public:
  WglContext(HWND hWnd) {
    hWnd_ = hWnd;
    s_hDC = GetDC(hWnd);
    if (!s_hDC) {
      throw std::runtime_error("GetDC");
    }

    int pixelformat = ChoosePixelFormat(s_hDC, &pfd);
    if (pixelformat == 0) {
      throw std::runtime_error("ChoosePixelFormat Failed....");
    }

    if (SetPixelFormat(s_hDC, pixelformat, &pfd) == FALSE) {
      throw std::runtime_error("SetPixelFormat Failed....");
    }

    s_hRC = wglCreateContext(s_hDC);
    if (!s_hRC) {
      throw std::runtime_error("Creating HGLRC Failed....");
    }

    wglMakeCurrent(s_hDC, s_hRC);

    // Set Vsync
    BOOL(WINAPI * wglSwapIntervalEXT)(int) = NULL;
    wglSwapIntervalEXT =
        (BOOL(WINAPI *)(int))wglGetProcAddress("wglSwapIntervalEXT");
    if (wglSwapIntervalEXT) {
      wglSwapIntervalEXT(1);
    }
  }
  ~WglContext() {
    if (s_hRC) {
      wglMakeCurrent(0, 0);
      wglDeleteContext(s_hRC);
    }
    if (s_hDC) {
      ReleaseDC(hWnd_, s_hDC);
    }
  }

  void MakeCurrent() { wglMakeCurrent(s_hDC, s_hRC); }
  void SwapBuffers() { ::SwapBuffers(s_hDC); }
};

class Win32Window {
  WNDCLASS wc = {
      .style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC,
      .lpfnWndProc = (WNDPROC)WndProc,
      .cbClsExtra = 0,
      .cbWndExtra = 0,
      .hInstance = {},
      .hIcon = LoadIcon(NULL, IDI_WINLOGO),
      .hCursor = LoadCursor(NULL, IDC_ARROW),
      .hbrBackground = NULL,
      .lpszMenuName = NULL,
      .lpszClassName = "WIN32_WINDOW",
  };
  ATOM atom_ = {};
  HWND s_hWnd = NULL;
  WglContext *wgl_ = nullptr;
  int s_screenWidth = 0;
  int s_screenHeight = 0;

public:
  Win32Window(HINSTANCE hInstance, const char *title, int width, int height) {
    wc.hInstance = hInstance;
    atom_ = RegisterClass(&wc);
    if (!atom_) {
      throw std::runtime_error("RegisterClass");
    }

    RECT rect{
        .left = 0,
        .top = 0,
        .right = width,
        .bottom = height,
    };
    AdjustWindowRectEx(&rect, WS_OVERLAPPEDWINDOW, FALSE,
                       WS_EX_APPWINDOW | WS_EX_WINDOWEDGE);
    s_hWnd = CreateWindowEx(
        WS_EX_APPWINDOW | WS_EX_WINDOWEDGE, (const char *)atom_, title,
        WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN, 0, 0,
        rect.right - rect.left, rect.bottom - rect.top, NULL, NULL,
        wc.hInstance, this);
    if (!s_hWnd) {
      throw std::runtime_error("CreateWindowEx");
    }
    s_screenWidth = width;
    s_screenHeight = height;

    wgl_ = new WglContext(s_hWnd);

    ShowWindow(s_hWnd, SW_SHOW);
    SetForegroundWindow(s_hWnd);
    SetFocus(s_hWnd);
  }
  ~Win32Window() {
    if (wgl_) {
      delete wgl_;
    }
    if (s_hWnd) {
      DestroyWindow(s_hWnd);
    }
    UnregisterClass((const char *)atom_, wc.hInstance);
  }

  HWND createWindow() {}

  LRESULT Proc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
    case WM_SYSCOMMAND: {
      switch (wParam) {
      case SC_SCREENSAVE:
      case SC_MONITORPOWER:
        return 0;
      }
      break;
    }

    case WM_CLOSE:
      PostQuitMessage(0);
      return 0;

    case WM_SIZE:
      s_screenWidth = LOWORD(lParam);
      s_screenHeight = HIWORD(lParam);
      return 0;
    }

    return DefWindowProc(s_hWnd, uMsg, wParam, lParam);
  }

  static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam,
                                  LPARAM lParam) {
    if (message == WM_CREATE) {
      auto p = reinterpret_cast<LPCREATESTRUCT>(lParam);
      SetWindowLongPtr(hWnd, GWLP_USERDATA,
                       reinterpret_cast<LONG_PTR>(p->lpCreateParams));
      return 0;
    }

    auto w =
        reinterpret_cast<Win32Window *>(GetWindowLongPtr(hWnd, GWLP_USERDATA));
    if (w) {
      return w->Proc(hWnd, message, wParam, lParam);
    } else {
      return DefWindowProcA(hWnd, message, wParam, lParam);
    }
  }

  std::tuple<int, int> GetSize() const {
    return {s_screenWidth, s_screenHeight};
  }

  void MakeCurrent() { wgl_->MakeCurrent(); }
  void SwapBuffers() { wgl_->SwapBuffers(); }
};
