# libdivsufsort 的 xHive 集成

## 不生成头文件

只维护 SDK 的 Xmake 对象库集成，也可直接用 C99 编译器编译。两种方式使用同一组普通头文件：

- `libdivsufsort/include/divsufsort_types.h`：共享的字节、状态类型及声明标记。
- `libdivsufsort/include/divsufsort.h`：32 位索引 API。
- `libdivsufsort/include/divsufsort64.h`：64 位索引 API。

不需要 `.h.cmake`、`config.h.in`、`configure_file()` 生成的头文件，也没有 Lua 读取模板、字符串替换或写出头文件的步骤。库代码直接使用 C99 标准头文件。

保留上游许可证、算法和五个示例源码。不再提供 CMake、CPack、pkg-config、安装/卸载或自动 DLL 导出配置。

## 类型与 ABI

| 类型 | 固定位宽 | 说明 |
| --- | --- | --- |
| `sauchar_t` | 无符号 8 位 | 输入、输出字节 |
| `saint_t` | 有符号 32 位 | 两套 API 共用的状态码、字符值 |
| `saidx_t` | 有符号 32 位 | `divsufsort.h` 的索引和长度 |
| `saidx64_t` | 有符号 64 位 | `divsufsort64.h` 的索引和长度 |

两套头文件可以在同一个 C/C++ 翻译单元中按任意顺序包含。公共头不通过宏重命名调用者的函数，也不让 `saidx_t` 随编译选项改变位宽。

- 默认编译库源码，提供 `divsufsort()`、`divbwt()` 等 32 位索引符号。
- 编译库源码时定义 `BUILD_DIVSUFSORT64`，通过私有类型别名和函数名映射提供 `divsufsort64()`、`divbwt64()` 等符号。
- 若需同时链接两套 API，应将库源码分别编译为两组对象；不能复用同一组对象。
- `BUILD_DIVSUFSORT64` 是库内部编译选项，不需要传播给应用。
- `PRIdSAINT_T` 始终对应 `PRId32`；64 位模式只改变索引的格式宏，不改变状态类型。
- GCC 裸机模式与部分 libc 组合需要先提供 libc 的类型元数据。公共头在可用时包含 `sys/types.h`，再使用标准整数与格式宏；不手写类型替代品或猜测格式字符串。

应用工程继续通过 `CONFIG_DIVSUFSORT_ENABLE_64BIT` 选择索引宽度。SDK 独立模式默认构建 32 位索引库，测试会分别编译并验证两套 API。

64 位索引不等于可以突破主机地址空间限制。调用者仍须保证长度、`n+1` 及缓冲区分配大小可表示；此次修改未重写上游算法或增加统一的分配溢出保护。

## 直接编译

以下示例假定当前目录中存在使用 `divsufsort.h` 的 `app.c`：

```sh
sdk=/path/to/xhive
cc -std=c99 \
    -I"$sdk/libs/divsufsort/libdivsufsort/include" \
    "$sdk"/libs/divsufsort/libdivsufsort/lib/*.c app.c -o app
```

使用 64 位 API 时，将应用改为包含 `divsufsort64.h`、调用 `*64` 函数，并在编译库源码时增加 `-DBUILD_DIVSUFSORT64`。不需要预先运行 Xmake 或 CMake 生成文件。

版本字符串在 `divsufsort_version()` 中直接定义，保持 SDK 之前返回的 `2.0.1`，不再通过构建系统传入。

`DIVSUFSORT_API` 默认为空，仅保留一个可覆盖的声明标记。不维护额外的共享库导出策略。

## 测试

从 SDK 根目录执行：

```sh
xmake f -y -m debug --xhive_tests=y --xhive_cross_tests=n
xmake test 'test_divsufsort/default' 'xhive_test_divsufsort/divsufsort' -v
```

- C++ 用例将两套 C 库一起链接，覆盖类型/格式宏、后缀排序、BWT、搜索及参数边界。
- 构建回归使用全新的临时工程分别选择 32/64 位库，检查 C99/C++11 头文件兼容性、公有包含路径和运行结果，并断言构建目录中没有生成头文件。
- 两项测试均属于 `xhive` 主机组。
- 启用 SDK 的 `xhive_cross_tests` 后，`xhive_test_riscv_divsufsort/riscv_divsufsort` 另检查 RV32/RV64 与两种索引宽度的四种组合，使用 `-ffreestanding` 编译，不访问硬件。

公共头记录了上游实现的边界行为，包括零长度仍要求非空的必需指针，以及 `inverse_bw_transform*()` 对长度不超过 1 的输入不写输出的现有行为。

## 保留的示例

`libdivsufsort/examples/` 保留 `suftest`、`mksary`、`sasearch`、`bwt`、`unbwt` 五个示例。它们使用 32 位索引 API，可直接编译，不额外维护一套构建系统。以下命令从 SDK 根目录执行：

```sh
lib="$PWD/libs/divsufsort/libdivsufsort"
mkdir -p build/divsufsort-examples
for name in suftest mksary sasearch bwt unbwt; do
    cc -std=c99 -O2 -I"$lib/include" \
        "$lib"/lib/*.c "$lib/examples/$name.c" \
        -o "build/divsufsort-examples/$name"
done
```

示例的固定 `lfs.h` 在 Windows 使用 `_fseeki64`/`_ftelli64`，在 POSIX 使用带 64 位 `off_t` 的 `fseeko`/`ftello`。该头必须先于系统头包含。编译时定义 `DIVSUFSORT_NO_LFS` 可选择 ISO C 的 `fseek`/`ftell`，文件偏移范围受 `long` 位宽限制。

Windows 大文件 I/O 分支尚需 Windows 工具链验证；当前运行验证在 Linux 上完成。
