#include "./tbox_sdk/lib/include/tbox_sdk.h"
#include <iostream>

  // 登录状态回调
  void debug_login(uint8_t statu) {
      std::cout << "登录状态: " << (int)statu << std::endl;
  }

  // 状态上报回调
  int callback(uint8_t cmd, const void* data, int data_len) {
      // 处理状态上报
      return 0;
  }

  int main() {
      // 1. 注册回调函数
      tbox_sdk_register_callback(callback);

      // 2. SDK初始化（需要厂商提供的token）
      tbox_sdk_init(debug_login);

      // 等待连接成功（状态为 0x08 AUTH_SUCCESS）

      // 3. 拉起刹车
      int result = tbox_sdk_brake_control(0);
      if (result == 0) {
          std::cout << "刹车已拉起" << std::endl;
      } else {
          std::cout << "刹车拉起失败" << std::endl;
      }

      // 等待一段时间...

      // 4. 释放刹车
      result = tbox_sdk_brake_control(1);
      if (result == 0) {
          std::cout << "刹车已释放" << std::endl;
      } else {
          std::cout << "刹车释放失败" << std::endl;
      }

      return 0;
  }