#pragma once
#include "debug/q3Render.h"
#include <chrono>
#include <memory>
#include <vector>

class App {
  std::vector<std::shared_ptr<struct Demo>> demos_;
  int currentDemo_ = 3;
  char sceneFileName_[256] = {0};
  int lastDemo_ = -1;
  std::unique_ptr<class q3Render> renderer_;

  std::chrono::high_resolution_clock::time_point time_;
  std::chrono::nanoseconds dt_ = std::chrono::nanoseconds(1000000000 / 60);
  std::unique_ptr<class q3Scene> scene_;
  // Is frame by frame stepping enabled?
  bool paused_ = false;
  // Can the simulation take a step, while paused is enabled?
  bool singleStep_ = false;
  // Globals for running the scene
  bool enableSleep_ = true;
  bool enableFriction_ = true;
  int velocityIterations_ = 10;

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
