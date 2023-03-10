#pragma once
#include <common/PhysicsScene.h>
#include <functional>
#include <memory>
#include <vector>

struct PhysicsSceneSelector {
  bool simulating_ = false;
  int sceneId_ = 0;

  using SceneGen = std::function<std::shared_ptr<PhysicsScene>()>;
  std::vector<SceneGen> generators_;

  std::shared_ptr<PhysicsScene> scene_;

public:
  PhysicsSceneSelector(std::span<const SceneGen> generators);
  void Reset();
  void Next();
  void Toggle() { simulating_ = !simulating_; }
  void Run() { simulating_ = true; }
  void Fire(const EasyPhysics::EpxVector3 &wp1,
            const EasyPhysics::EpxVector3 &wp2);
  void Update();
  DrawDataSpan DrawData() const;
  std::string Title() const;
};
