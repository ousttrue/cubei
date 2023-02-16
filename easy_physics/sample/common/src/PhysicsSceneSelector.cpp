#include <common/PhysicsSceneSelector.h>
#include <common/ScreenCamera.h>
#include <format>

PhysicsSceneSelector::PhysicsSceneSelector(std::span<SceneGen> generators)
    : generators_(generators.begin(), generators.end()) {
  Reset();
}
void PhysicsSceneSelector::Reset() {
  sceneId_ = sceneId_ % generators_.size();
  scene_ = generators_[sceneId_]();
  scene_->CreateFireBody();
}
void PhysicsSceneSelector::Next() {
  ++sceneId_;
  Reset();
}
void PhysicsSceneSelector::Fire(const ScreenCamera &camera, int x, int y,
                                int width, int height) {
  auto wp1 = camera.GetWorldPosition({(float)x, (float)y, 0.0f}, width, height);
  auto wp2 = camera.GetWorldPosition({(float)x, (float)y, 1.0f}, width, height);
  scene_->PhysicsFire(wp1, normalize(wp2 - wp1) * 50.0f);
}
void PhysicsSceneSelector::Update() {
  if (simulating_) {
    scene_->Simulate();
  }
}
DrawDataSpan PhysicsSceneSelector::DrawData() const {
  return scene_->GetDrawData();
}
std::string PhysicsSceneSelector::Title() const {
  return std::format("[{}/{}] {}", sceneId_+1, generators_.size(),
                     scene_->title_);
}
