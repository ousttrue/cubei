#pragma once
#include <common/PhysicsSceneSelector.h>
#include <memory>

std::shared_ptr<PhysicsScene> createSceneCompound();
std::shared_ptr<PhysicsScene> createSceneDaruma();
std::shared_ptr<PhysicsScene> createSceneStackingPole();
std::shared_ptr<PhysicsScene> createSceneStackingWall();

const inline PhysicsSceneSelector::SceneGen generators[] = {
    createSceneCompound,
    createSceneDaruma,
    createSceneStackingPole,
    createSceneStackingWall,
};
