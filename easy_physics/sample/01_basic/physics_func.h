#pragma once
#include <common/PhysicsSceneSelector.h>
#include <memory>

std::shared_ptr<PhysicsScene> createSceneTwoBox();
std::shared_ptr<PhysicsScene> createSceneFriction();
std::shared_ptr<PhysicsScene> createSceneRestitution();
std::shared_ptr<PhysicsScene> createSceneGeometries();

const inline PhysicsSceneSelector::SceneGen generators[] = {
    createSceneTwoBox,
    createSceneFriction,
    createSceneRestitution,
    createSceneGeometries,
};
