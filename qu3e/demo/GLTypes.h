#pragma once
#include <DirectXMath.h>
#include <cmath>
#include <plog/Log.h>
#include <stdint.h>

struct Light {
  DirectX::XMFLOAT4 uLightAmbient = {1.0f, 1.0f, 1.0f, 0.5f};
  DirectX::XMFLOAT4 uLightDiffuse = {0.2f, 0.4f, 0.7f, 1.0f};
  DirectX::XMFLOAT4 uLightSpecular = {1.0f, 1.0f, 1.0f, 1.0f};
  DirectX::XMFLOAT4 uLightPosition = {0, 10, 0, 0};
};

class Camera {
  // projection
  int width_ = 1;
  int height_ = 1;
  float fovY_ = DirectX::XMConvertToRadians(45.0f);

  // view
  float shiftX_ = 0;
  float shiftY_ = -5;
  float distance_ = 20;
  float yaw_ = 0;
  float pitch_ = 0;

  bool isDirty_ = true;

public:
  // DirectX::XMFLOAT3 position = {0.0f, 5.0f, 20.0f};
  // DirectX::XMFLOAT3 target = {0.0f, 0.0f, 0.0f};
  DirectX::XMFLOAT4X4 projection = {
      1, 0, 0, 0, //
      0, 1, 0, 0, //
      0, 0, 1, 0, //
      0, 0, 0, 1, //
  };
  DirectX::XMFLOAT4X4 view = {
      1, 0, 0, 0, //
      0, 1, 0, 0, //
      0, 0, 1, 0, //
      0, 0, 0, 1, //
  };

  void Resize(int width, int height) {
    if (width == width_ && height == height_) {
      return;
    }
    width_ = width;
    height_ = height;
    isDirty_ = true;
  }

  void Update() {
    if (!isDirty_) {
      return;
    }
    isDirty_ = false;

    {
      float aspectRatio = (float)width_ / (float)(height_ <= 0 ? 1 : height_);
      auto m =
          DirectX::XMMatrixPerspectiveFovRH(fovY_, aspectRatio, 0.1f, 10000.0f);
      DirectX::XMStoreFloat4x4(&projection, m);
    }
    {
      // DirectX::XMFLOAT3 yup = {0, 1, 0};
      // auto m = DirectX::XMMatrixLookAtRH(DirectX::XMLoadFloat3(&position),
      //                                    DirectX::XMLoadFloat3(&target),
      //                                    DirectX::XMLoadFloat3(&yup));
      auto m_yaw = DirectX::XMMatrixRotationY(yaw_);
      auto m_pitch = DirectX::XMMatrixRotationX(pitch_);
      auto m_shift = DirectX::XMMatrixTranslation(shiftX_, shiftY_, -distance_);
      auto m = m_yaw * m_pitch * m_shift;
      DirectX::XMStoreFloat4x4(&view, m);
    }
  }

  void YawPitch(float yaw, float pitch) {
    yaw_ += yaw;
    pitch_ += pitch;
    isDirty_ = true;
  }

  void Shift(float dx, float dy) {
    auto factor = std::tan(fovY_ / 2.0) * 2.0 * distance_ / (float)height_;
    shiftX_ += (factor * dx);
    shiftY_ -= (factor * dy);
    isDirty_ = true;
    // PLOG_DEBUG << (factor * x);
  }

  void Dolly(float d) {
    if (d > 0) {
      distance_ *= 0.9f;
      isDirty_ = true;
    } else if (d < 0) {
      distance_ *= 1.1f;
      isDirty_ = true;
    }
  }
};
