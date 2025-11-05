# xHive SDK

A cross-platform SDK for various microcontroller units (MCUs) with a focus on ease of use and modularity.

## 特性

- 支持多种MCU构架: ARM Cortex-M, RISC-V等
- 基于xmake的构建系统集成
- 基于xmake命名空间支持同时在PC上进行纯逻辑代码的单元测试同时编译输出MCU固件
- 自动化启动代码生成
- 自动化链接脚本生成

## License

- GPLv3
- CLA
- Commercial License

### 许可证说明

1. 本项目采用 GPLv3 许可证。任何对本项目的修改，无论是在内部使用还是随产品分发，都必须遵循 GPLv3 协议开源。如果您希望在不开源修改的情况下在商业产品中使用或集成本项目，请联系我们获取商业许可证。
2. 许可不涉及引用的第三方库或厂商提供的驱动代码
3. GPLv3 许可证的完整文本可以在 [GNU 官网](https://www.gnu.org/licenses/gpl-3.0.html) 找到。
4. 本项目接受 CLA 贡献。所有贡献者必须签署 CLA，以确保代码的版权和许可符合项目的要求。

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
    SDK与工程分离管理, 设置XHIVE_SDK_PATH环境变量指向SDK路径
 ]]
includes(os.getenv("XHIVE_SDK_PATH"))

target("firmware")
    set_kind("binary")
    add_rules("xhive::rules") -- 这条是关键
    add_files("*.c")
```

## Toolchain

### Arm Toolchain for Embedded(ATfE)

参考: https://developer.arm.com/documentation/107976/20-1-0/C-and-C---libraries/Automatically-selecting-libraries-with-multilib

