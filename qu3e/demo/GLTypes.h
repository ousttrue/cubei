#pragma once
#include <DirectXMath.h>
#include <stdint.h>

struct Light {
  float ambient[4] = {1.0f, 1.0f, 1.0f, 0.5f};
  float diffuse[4] = {0.2f, 0.4f, 0.7f, 1.0f};
  float specular[4] = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct Camera {
  DirectX::XMFLOAT3 position = {0.0f, 5.0f, 20.0f};
  DirectX::XMFLOAT3 target = {0.0f, 0.0f, 0.0f};
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

  void Update(float aspectRatio) {
    {
      auto m = DirectX::XMMatrixPerspectiveFovRH(
          DirectX::XMConvertToRadians(45.0f), aspectRatio, 0.1f, 10000.0f);
      DirectX::XMStoreFloat4x4(&projection, m);
    }
    {
      DirectX::XMFLOAT3 yup = {0, 1, 0};
      auto m = DirectX::XMMatrixLookAtRH(DirectX::XMLoadFloat3(&position),
                                         DirectX::XMLoadFloat3(&target),
                                         DirectX::XMLoadFloat3(&yup));
      DirectX::XMStoreFloat4x4(&view, m);
    }
  }
};
