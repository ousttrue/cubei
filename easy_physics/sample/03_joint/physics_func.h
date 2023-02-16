#pragma once
#include <common/PhysicsSceneSelector.h>
#include <memory>

std::shared_ptr<PhysicsScene> createSceneBallJoint();
std::shared_ptr<PhysicsScene> createSceneHingeJoint();
std::shared_ptr<PhysicsScene> createSceneFixedJoint();
std::shared_ptr<PhysicsScene> createSceneGearJoint();
std::shared_ptr<PhysicsScene> createSceneChain();

const inline PhysicsSceneSelector::SceneGen generators[] = {
    createSceneBallJoint, createSceneHingeJoint, createSceneFixedJoint,
    createSceneGearJoint, createSceneChain,
};
