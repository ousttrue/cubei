#include "q3Render.h"
#include "../broadphase/q3BroadPhase.h"
#include "../dynamics/q3Body.h"
#include "../dynamics/q3ContactManager.h"
#include "../scene/q3Scene.h"

void q3Render::RenderScene(const class q3Scene *scene) {
  for (auto body : scene->m_bodyList) {
    body->Render(this);
  }
}

void q3Render::RenderContact(const class q3ContactManager *contactManager) {
  contactManager->RenderContacts(this);
  contactManager->m_broadphase.m_tree.Render(this);
}
