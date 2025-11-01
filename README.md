# XMCU SDK

A cross-platform SDK for various microcontroller units (MCUs) with a focus on ease of use and modularity.

## 示例

运行`xmake menuconfig`配置MCU厂商与参数后, 直接运行`xmake`编译示例工程.

```
hello_world
├── main.c
└── xmake.lua
```

xmake.lua:
```lua
--[[ 
    SDK与工程分离管理, 设置XMCU_SDK_PATH环境变量指向SDK路径
 ]]
includes(os.getenv("XMCU_SDK_PATH"))

target("firmware")
    set_kind("binary")
    add_rules("xmcu") -- 这条是关键
    add_files("*.c")
```

## Toolchain

### Arm Toolchain for Embedded(ATfE)

参考: https://developer.arm.com/documentation/107976/20-1-0/C-and-C---libraries/Automatically-selecting-libraries-with-multilib
