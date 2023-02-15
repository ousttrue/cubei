#include <common/ScreenCamera.h>

ScreenCamera::ScreenCamera() {
  s_viewPos = EpxMatrix3::rotationY(s_viewRadY) *
              EpxMatrix3::rotationX(s_viewRadX) *
              EpxVector3(0, 0, s_viewRadius);

  s_lightPos = EpxMatrix3::rotationY(s_lightRadY) *
               EpxMatrix3::rotationX(s_lightRadX) *
               EpxVector3(0, 0, s_lightRadius);
}

ScreenCamera::~ScreenCamera() {}

std::tuple<const float *, const float *>
ScreenCamera::UpdateProjectionView(int width, int height) {
  // update projection
  s_pMat = EpxMatrix4::perspective(3.1415f / 4.0f, (float)width / (float)height,
                                   0.1f, 1000.0f);

  // create view matrix
  s_vMat =
      EpxMatrix4::lookAt(EpxPoint3(s_viewTgt + s_viewPos), EpxPoint3(s_viewTgt),
                         EpxVector3(0.0f, 1.0f, 0.0f));

  return {(const float *)&s_pMat, (const float *)&s_vMat};
}

EpxVector3 ScreenCamera::GetWorldPosition(const EpxVector3 &screenPos,
                                           int width, int height) const {
  auto mvp = s_pMat * s_vMat;
  auto mvpInv = inverse(mvp);

  EpxVector4 wp(screenPos, 1.0f);

  wp[0] /= (0.5f * (float)width);
  wp[1] /= (0.5f * (float)height);

  float w = mvpInv[0][3] * wp[0] + mvpInv[1][3] * wp[1] + mvpInv[2][3] * wp[2] +
            mvpInv[3][3];

  wp = mvpInv * wp;
  wp /= w;

  return wp.getXYZ();
}

EpxVector3 ScreenCamera::GetScreenPosition(const EpxVector3 &worldPos,
                                            int width, int height) const {
  EpxVector4 sp(worldPos, 1.0f);

  EpxMatrix4 mvp;
  mvp = s_vMat;

  sp = mvp * sp;
  sp /= (float)sp[3];
  sp[0] *= (0.5f * (float)width);
  sp[1] *= (0.5f * (float)height);

  return sp.getXYZ();
}
