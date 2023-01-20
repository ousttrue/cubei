#pragma once
#include "GLTypes.h"
#include <chrono>
#include <memory>
#include <q3.h>
#include <vector>

class App {
  std::vector<std::shared_ptr<struct Demo>> demos_;
  int currentDemo_ = 3;
  char sceneFileName_[256] = {0};
  int lastDemo_ = -1;
  std::chrono::high_resolution_clock::time_point time_;

  std::unique_ptr<class q3Scene> scene_;
  std::unique_ptr<class q3ContactManager> contactManager_;
  std::unique_ptr<class q3Render> renderer_;
  // Is frame by frame stepping enabled?
  bool paused_ = false;
  // Can the simulation take a step, while paused is enabled?
  bool singleStep_ = false;

  Camera camera_;
  // Globals for running the scene
  q3Env env_ = {
      .m_iterations = 10,
      .m_allowSleep = true,
      .m_enableFriction = true,
  };

public:
  App(struct GLFWwindow *window);
  ~App();
  void KeyDown(char key);
  void Frame(int w, int h);

private:
  void DemosMouseLeftDown(int x, int y);
  void DemosKeyDown(unsigned char key);
  void DemosKeyUp(unsigned char key);
  void DemosTogglePause();
  void DemosSingleStep();
  void DemosSceneDump();
};
