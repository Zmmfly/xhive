# 项目开发规范

本文规定本仓库的 Agent 行为约束、Git 提交信息、Doxygen 注释格式，以及 SDK 主机构建与测试流程。新增或修改本项目维护的代码时应遵守这些规范；不要为了统一格式而批量改写无关代码或第三方原始代码。

## 0. Agent 行为约束

本节适用于在本仓库工作的所有 Agent，优先级高于后续流程性约定。约束针对 GPT-6 系列在编码任务中反复出现的失败模式：为想象出的失败加防御、为假设的未来加抽象、用模糊措辞代替判断。规则本身按同一标准执行——不要为了显得更严谨而在本节堆叠同义条目。

### 0.1 诚实且明确

- 结论必须与已观察到的证据一致。没运行的测试、没验证的行为写“未验证”，不要用“应该”“通常”“理论上”遮掩。
- 是就是，不是就不是。评估代码、方案或他人改动时直接给出对错和依据；不为了缓和语气弱化判断，也不为了迎合而附和。
- 禁止似是而非的模糊免责说明。不以“视实际情况而定”“建议根据需求调整”“可能存在问题”充当回答主体；能判断就下判断，不能判断就说明缺哪条信息、如何取得。
- 真实限制要讲具体：写明触发条件、后果和已知边界。这属于事实说明，不是免责套话。
- 标明事实来源层级：有命令输出、日志、代码或文档依据的写“已确认”；由证据推出的写“推断”并给出依据；其余写“未知”。
- 失败和未完成的工作如实报告并说明影响范围；不用“已完成”描述只做了一半的事。

### 0.2 先有问题，再有设计

- 动手前先回答两个问题：要解决的具体问题是什么？不引入这层设计会出什么错？答不出来就不做。
- 只实现当前需求。不为想象中的复用、扩展、兼容或“以后可能要”预留抽象层、配置项、接口和开关。
- 选能满足需求的最小方案。新增模块、抽象或依赖必须说明它消除了什么具体复杂度；说不清就改用更直接的写法。
- 需求本身可疑时直接指出，不用放大的实现去“稳妥覆盖”它。
- 优先判断现有代码够不够用，而不是重写。能从现有代码直接读到的事实，不再造一层封装去获得。

### 0.3 最小化改动

- 默认最小改动：只改达成目标所必需的代码，不顺手格式化、重命名、重排、批量替换或“统一风格”。
- 无绝对必要不做成片修改与重构。重构须有明确收益（消除具体缺陷或阻塞点），说明影响范围和验证方式，并单独提交。
- 改动范围超出目标所需时先停下确认，不做“既然改到这里就一起改”的扩张；发现的无关问题只在回复中指出，不顺手修改。
- 保持既有结构、命名和周边写法，不按个人偏好重写现有实现。

### 0.4 不留冗余与死代码

- 不为逻辑上不可能出现的条件写检查。上游已保证的前置条件、类型系统已保证的取值、本函数刚写入的状态都不重复判断；确需保留时写明具体触发路径。
- 不沿调用链层层重复同一种校验。参数校验放在边界做一次，内部函数信任已校验的输入。
- 不写用不到的分支、不会被走到的回退路径、无人读取的日志和注释掉的旧代码；发现即删除，不以“保留以备后用”留档。
- 每段防御性代码都要对应一个真实可发生的失败；没有对应失败路径的“保险”就是冗余。
- 文档和注释只写事实、约定和理由，不复述代码、不堆同义条目、不为同一规则写多处副本。

## 1. Git 提交规范

### 1.1 语言与基本要求

- **整个 commit message 必须使用英文**，包括标题、正文、章节名称和说明；文件路径、符号名称保持原样。
- **使用 Markdown 格式组织正文**：使用二级标题分节，使用 `-` 列出条目，使用反引号标记命令、路径和代码符号。
- 内容必须结构化、条目清晰，说明“做了什么、为什么做、如何验证”。避免空泛描述，例如 `update code`、`fix issues`、`some changes`。
- 每次提交围绕一个逻辑完整的主题。无关修改应拆分提交，不要将用户已有的工作区修改顺带提交。
- 提交信息必须与实际 diff 和验证结果一致，不能把计划执行的检查写成已经通过，也不能把软件模拟测试描述为实板验证。

### 1.2 标题格式

使用 Conventional Commits 风格：

```text
<type>(<scope>): <imperative summary>
```

- `type` 使用小写；常用值为 `feat`、`fix`、`docs`、`refactor`、`perf`、`test`、`build`、`ci`、`chore`、`revert`。
- `scope` 表示主要影响范围，例如 `gpio`、`openecos`、`build`、`agent`；确实没有合适范围时可以省略。
- 摘要使用英文祈使句，说明实际变化，例如 `add`、`fix`、`remove`、`document`。
- 标题不超过 72 个字符，不以句号结尾，不在标题前添加 Markdown 的 `#`。
- 标题与正文之间保留一个空行。

### 1.3 正文结构

正文默认使用以下章节；每节使用简洁条目，不重复堆砌同一信息：

- `## Summary`：说明提交目的及必要背景。
- `## Changes`：列出主要实现或文档变化。
- `## Validation`：列出实际执行的检查、命令和结果。未执行时明确说明原因，例如 `Not run (documentation-only change).`。
- `## Notes`：可选，说明兼容性、已知限制、迁移步骤或尚未验证的事项。没有相关内容时省略。

存在不兼容变更时，按 Conventional Commits 使用 `!` 和/或 `BREAKING CHANGE:` footer，并给出必要的迁移说明。

示例：

```markdown
docs(agent): define commit and Doxygen standards

## Summary
- Establish consistent commit messages and API documentation.

## Changes
- Require English commit messages with structured Markdown sections.
- Define Doxygen conventions for public APIs and hardware side effects.

## Validation
- Review the document structure and examples.
- Run `git diff --check`: passed.
- Build and runtime tests: not run (documentation-only change).
```

### 1.4 提交前检查与范围控制

- 检查 `git status`、工作区 diff 和暂存区 diff，确认提交范围。
- 优先按明确路径暂存文件，避免使用宽泛的暂存操作混入无关修改。
- 提交前执行 `git diff --cached --check`，并根据代码风险进行适当验证。
- 仅在用户要求提交时创建提交；不要自动推送远端，也不要擅自改写已有提交历史。
- 提交完成后报告提交哈希、标题，以及仍留在工作区的未提交修改。

## 2. Doxygen 注释规范

### 2.1 适用范围与位置

- **对外公开的函数声明必须有 Doxygen 注释**，通常放在头文件中，并紧邻对应声明之前。
- 使用 `/** ... */` 多行块注释；每行内容以 ` *` 开头。
- 注释内容使用英文；语义准确、简洁，不只是重复函数名称。
- 头文件记录调用契约，源文件说明算法、实现原因和硬件规避措施，避免复制两份容易失去同步的完整接口说明。
- 对复杂的内部函数、非直观算法和重要数据结构，也应补充必要的 Doxygen 或实现注释。
- 修改接口或行为时，同步修改对应注释。不要保留过时的参数名、单位或返回值说明。

### 2.2 函数注释结构

按以下顺序组织，适用的项目必须填写：

1. `@brief`：一句话说明功能。
2. 可选的详细说明：补充调用前提、算法或操作顺序。
3. `@param`：按函数声明中的顺序逐一说明所有参数。
4. `@return`：说明非 `void` 函数的返回类型或返回值含义。
5. `@retval`：在返回状态码或离散结果时，逐项说明实际可能出现的值及触发条件。
6. `@pre`、`@post`、`@note`、`@warning`：按需说明前置条件、后置条件、副作用及风险。

各组之间使用空注释行分隔：

```c
/**
 * @brief Perform an operation.
 *
 * @param[in] input Input value.
 * @param[out] output Destination for the result.
 * @return status_t Operation status.
 */
```

- 优先使用 `@param[in]`、`@param[out]`、`@param[in,out]` 表达参数方向。
- 参数名称必须与声明完全一致，不遗漏参数，不记录不存在的参数。
- 指针参数必须说明用途、可空条件，以及相关的缓冲区容量或生命周期要求。
- 长度、频率、时间、计数、地址和掩码参数必须说明单位、范围或编码方式，避免只写 `length`、`value`、`data pointer`。
- `void` 函数不写 `@return`；无参数函数不写空的 `@param`。
- 不虚构错误码或行为。失败时输出是否有效、是否存在部分完成，应在注释中说明。

### 2.3 外设驱动的额外要求

对适用的驱动接口，必须记录以下信息，不能仅描述正常成功路径：

- 初始化、时钟、引脚方向、context 所有权等前置条件。
- 阻塞或非阻塞行为；超时参数的单位、零值语义、预算覆盖范围。
- 并发与中断限制，包括是否可重入、是否需要调用者串行化。
- MMIO 副作用，例如读清零、FIFO 出队、写一清零、影子状态更新。
- 返回成功表示“已提交数据”还是“硬件操作已完成”。
- 失败后的硬件状态、部分传输和恢复要求。
- 与硬件缺陷相关的规避方式及限制；区分已确认的事实、参考实现和推断。
- 涉及 GPIO、PSRAM 或共享 QSPI 时，说明稳定等待、保留引脚、代码/栈位置等实际约束。未实现的保护措施不能写成已经具备。

### 2.4 函数示例

下面的注释格式适用于 `c2_rng_fill()`，同时说明参数方向、单位、可空条件和实际返回值：

```c
/**
 * @brief Fill a buffer with pseudo-random data.
 *
 * @param[in] reg Peripheral register block. Must not be NULL.
 * @param[out] data Destination buffer with capacity for at least length bytes.
 *                  May be NULL only when length is zero.
 * @param[in] length Number of bytes to write. Zero performs no MMIO access.
 * @return c2_status_t Operation status.
 * @retval C2_OK The requested buffer range was filled, or length was zero.
 * @retval C2_ERROR_INVALID_ARGUMENT reg is NULL, or data is NULL with a
 *                                  nonzero length.
 *
 * @pre Seed the generator with a nonzero value to avoid the zero-lock state.
 * @note Samples are stored in little-endian byte order. Unused bytes from
 *       the final sample are discarded.
 * @warning The hardware uses an LFSR and is not suitable for cryptographic use.
 */
c2_status_t c2_rng_fill(C2_RNG_TypeDef *reg, void *data, size_t length);
```

### 2.5 类型、成员与宏

- 公开的结构体、枚举和重要配置宏应说明用途及取值含义。
- 成员或枚举值可使用 `/**< ... */` 形式的行尾注释。
- 寄存器成员应注明偏移、访问属性、有效位宽及特殊副作用；未公开的信息明确标记为未知，不凭惯例补全。

```c
/**
 * @brief Software-owned GPIO state.
 */
typedef struct {
    uint32_t output_shadow;     /**< Authoritative output-latch bitmap. */
    uint32_t input_mask_shadow; /**< Direction bitmap: 1 = input, 0 = output. */
} gpio_shadow_example_t;
```

## 3. SDK 主机构建与测试（Host）

### 3.1 入口与依赖

- **在 xhive SDK 仓库根目录执行命令**，不要为了测试 SDK 而依赖某个板卡应用目录。
- SDK 根目录是独立的主机工程：不读取板卡 `.config`，不运行 `genconfig`，不加载 MCU/RTOS 固件目标，也不要求嵌入式交叉工具链。
- 不要为主机测试复制板卡 `.config`、运行 `menuconfig`、选择 `-p cross`，或自动下载交叉工具链。
- 基本环境：Xmake ≥ 3.0.0、主机 C/C++ 工具链；运行完整主机测试还需要 Python ≥ 3.10，以及 `PATH` 中 GCC/Clang 兼容的 `cc`、`c++`。
- GoogleTest 由 Xmake 管理，首次配置可能需要下载。普通非交互配置使用 `-y`，但不能把下载失败、工具缺失或未执行的测试记为通过。
- 当前流程已在 Linux 验证；其他主机平台，尤其默认 MSVC 环境，不应未经验证就声称兼容所有测试脚本。

### 3.2 标准主机流程

```sh
cd /path/to/xhive

# 显式选择 SDK 根工程，启用主机测试，关闭交叉检查。
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n

# 构建默认主机库。
xmake b

# 构建并运行主机测试组。
xmake test -g xhive -v
```

- `xmake` 与 `xmake b` 构建默认库目标，**不会自动运行测试**。
- 测试目标是非默认目标；`xmake test` 会构建所选测试需要的目标并执行测试。
- `xhive_tests` 控制主机测试；`xhive_cross_tests` 控制可选交叉检查。切换用途时显式设置这两个选项，避免沿用不合适的配置。
- 若只构建库、不测试，可使用以下配置；此时不需要 GoogleTest 或 Python：

```sh
xmake f -P . -y -m debug --xhive_tests=n --xhive_cross_tests=n
xmake b
```

### 3.3 按改动范围选择测试

优先运行最窄的相关选择器，再根据风险扩大范围。以下命令均从 SDK 根目录执行：

```sh
# base64 / bitmap。目标名为 <库名>_test_<相对路径>，用例名为源文件 basename。
xmake test -v 'base64_test_base64_decode/base64_decode'
xmake test -v 'bitmap_test_bitmap/bitmap'

# divsufsort 双索引 API 用例，以及两个独立 C/C++ 消费者构建检查。
xmake test -v 'divsufsort_test_divsufsort/divsufsort' 'divsufsort_test_consumer*/consumer'

# C2 主机 MMIO mock、头文件和链接检查；不要求 RISC-V 编译器。
xmake test -v 'xhive_test_c2/host'

# SDK 入口、配置隔离及可选目标选择。
xmake test -v 'xhive_test_sdk_entry/sdk_entry' 'xhive_test_target_selection/target_selection'
```

- 使用通配符选择器时加引号，避免 shell 提前展开；不要把测试分组名称当作目标名称。
- **把 `-g`、`-v` 等选项放在测试选择器之前。** Xmake 3.0.8 会将首个选择器后的参数当作更多选择器，导致后置的分组过滤和详细输出选项失效。
- 修改公共构建规则、共享头文件、目标选择或多个库时，运行完整 `xmake test -g xhive -v`。
- 修改生产代码或构建配置时，同时运行 `xmake b`，不能仅以测试目标编译成功代替默认构建检查。
- 仅修改文档时可不重跑构建与测试，但应校验命令、链接和 `git diff --check`，并明确记录未重跑原因。
- 验证结果必须区分主机运行、交叉编译/链接和实板验证；主机 mock 与目标对象编译不能算实板通过。
- 库测试都是 `xhive_host` 命名空间下的普通非默认目标，选择器可省略命名空间前缀；目标名由库名与测试文件相对路径组成，用例名是源文件 basename，分组统一为 `xhive`。

#### 3.3.1 在 SDK 根目录只测试一个库

推荐保持 SDK 工程上下文，用测试选择器筛选库。先执行 3.2 中的配置命令；迭代时无需先构建整个 SDK，`xmake test` 会构建所选测试及其依赖，不运行其他库的测试。

```sh
# base64 的全部 3 个主机测试入口：编码、解码、输入合法性检查。
xmake test -g xhive -v 'base64_test_*/*'

# bitmap 的主机测试入口。
xmake test -g xhive -v 'bitmap_test_*/*'

# divsufsort 的全部 3 个主机测试入口：双索引 API 与两个 C/C++ 消费者。
xmake test -g xhive -v 'divsufsort_test_*/*'

# 只编译某个测试二进制，不运行测试。
xmake b xhive_host::base64_test_base64_decode
```

- 上面的命令把 `-g xhive` 与库选择器取交集，即使工程启用了交叉检查，也只运行所选库的主机测试。不要把 `-g xhive` 移到选择器后面。
- 自动发现的目标名称为 `<库名>_test_<相对 test/ 的路径，去扩展名并将分隔符转下划线>`，用例名是源文件 basename，因此 `base64_test_*/*` 一次覆盖 base64 的 3 个测试入口。
- Xmake 汇总中的测试数量是已注册的测试入口数量，不是 GoogleTest 内部 `TEST`/`TYPED_TEST` 的用例数量。记录验证结果时说明是哪一层计数，并确认没有意外扩大或缩小选择范围。
- 当前库测试共用 `xhive` 分组，**没有按库名设置独立分组**；`-g xhive` 是完整主机组，不能假定 `-g base64` 或 `xmake test base64` 等价于测试 base64 库。
- 单库筛选仍会加载 SDK 的公共项目配置，不等于只加载该库的构建脚本。新增或改名测试后，应根据 `libs/<库>/test/` 下的文件核对选择器是否完整（测试定义在各库的 `xmake.lua` 中，不再有 `libs/<库>/tests.lua`）。
- 如果输出 `nothing to test`，应检查 `xhive_tests` 配置及选择器，**不能将其记为测试通过**。
- 若只想编译某个库而不运行测试，可执行 `xmake b xhive_host::base64` 或 `xmake b xhive_host::divsufsort`。库目标名与测试选择器不是同一种名称；头文件库的构建成功也不能代替用例验证。
- 单库测试适合迭代；提交前的默认构建和扩大回归范围仍按 3.3 的要求执行。

#### 3.3.2 使用库自己的独立测试入口（可选）

三个库的 `xmake.lua` 都在自身上下文加载测试定义，因此 `base64`、`bitmap`、`divsufsort` 都支持独立的 `-P` 入口。独立模式只加载该库工程，不定义 SDK 根工程专用选项（如 `--xhive_tests`、`--xhive_cross_tests`），交叉对象目标也只在 SDK 根工程下注册。

下面以 base64 为例，从 SDK 根目录执行（POSIX shell）。用子 shell 同时隔离当前目录和配置缓存目录，避免把工程选择或配置写回 SDK 根目录：

```sh
component="$PWD/libs/base64"
(cd "$component" && XMAKE_CONFIGDIR="$component" xmake f -P . -y -m debug && XMAKE_CONFIGDIR="$component" xmake test -P . -v)
```

- 把 `libs/base64` 换成 `libs/bitmap` 或 `libs/divsufsort` 即可测试对应库。
- 独立模式只传主机配置参数（`-y -m debug`），**不要**传 SDK 根工程选项（`--xhive_tests` 等）。
- 库测试目标只在 Linux/Windows/macOS 主机注册，独立入口继承同样的平台判断。
- 不要省略 `XMAKE_CONFIGDIR`，也不要在 SDK 根目录外混用两种配置；日常 SDK 开发仍优先使用 3.3.1。

### 3.4 可选交叉检查

只有任务涉及相应交叉编译行为、且已确认所需工具可用时，才启用交叉检查。需要支持所测 RV32/RV64 组合的 RISC-V GCC，以及带 RISC-V 后端的 Clang。

```sh
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=y \
    --xhive_riscv_gcc=/path/to/bin/riscv-none-elf-gcc \
    --xhive_clang=clang
# 运行完整交叉检查组。
xmake test -g xhive_cross -v

# 或只检查 divsufsort 的 RV32/RV64 × 32/64 位索引编译矩阵。
xmake test -g xhive_cross -v 'divsufsort_test_rv*_index*/compile'

# 完成后恢复主机测试配置。
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
```

启用交叉检查后，不带分组的 `xmake test` 会运行所有已注册测试。只做主机验证时应使用 `-g xhive`，而不是无条件运行全部测试。测试不得隐式烧录、打开串口或访问板卡。

`xhive_cross` 当前包含 RISC-V 编译参数与启动/链接模板回归、divsufsort 的编译专用对象目标（`divsufsort_test_rv{32,64}_index{32,64}/compile`，`build_should_pass = true`，只编译、不在主机执行）以及 C2 完整软件检查。

### 3.5 工程缓存与临时目录安全

- Xmake 3.0.8 的 `-P` 可能把工程选择写入调用目录的 `.xmake` 缓存。若 SDK 命令曾被重定向到其他工程，在 SDK 根目录重新执行 3.2 中带 `-P .` 的配置命令。
- 检查是否继承了指向其他工程的 `XMAKE_CONFIGDIR` 等环境设置；不要将外部应用的配置混入 SDK 主机配置。
- 从 SDK 目录操作外部应用时，同时指定应用工程目录和缓存目录，按命令设置环境变量，不要全局切换：

```sh
app=/absolute/path/to/application
XHIVE_SDK_PATH="$PWD" XMAKE_CONFIGDIR="$app" xmake -P "$app"
```

- 测试创建的临时工程必须位于 SDK 之外；启动子 Xmake 时将 `curdir` 和 `XMAKE_CONFIGDIR` 都指向该临时工程，避免污染父工程缓存或争用父工程锁。
- Lua 测试使用包含 `-xhive-` 标记的独占临时目录，并在清理时调用 `modules/xhive/testing.lua` 的 `remove_workdir()`；Python 测试优先使用 `tempfile.TemporaryDirectory`。
- **禁止用 Xmake 的递归 `os.rm` 清理测试目录树，也不要创建指向真实 SDK、已有工程或用户数据的测试目录链接。** 特殊路径测试只可引用测试自行创建的临时数据。
- 清理函数拒绝 SDK 本身、其祖先/后代以及链接形式的工作目录根节点。遇到拒绝应检查路径，不能绕过保护或改用宽泛删除命令。

更完整的操作说明见 [README.zh_CN.md](README.zh_CN.md)。

### 3.6 新增库测试的组织约定

新增或扩展库测试时沿用现有约定：不引入新的测试 DSL、不生成测试程序、不为普通库测试引入额外编译器或代码生成步骤。

- 测试目标在库自己的 `libs/<库>/xmake.lua` 中声明，位置在生产 `target(...)` 之外。SDK 模式下受主机平台、`xhive_host` 作用域及 `xhive_tests` 开关约束；库独立模式下按主机平台注册。
- 用 `for _, file in ipairs(os.files("test/**.cc"))` 扫描 `test/**.cc`，**只排除 `test/main.cc`**；其余每个 `.cc` 生成一个独立的非默认二进制目标。
- 自动发现的目标名称为 `<库名>_test_<相对 test/ 的路径，去扩展名并将分隔符转下划线>`；注册的用例名是源文件 basename；分组统一为 `xhive`。
- 每个目标都编译自己的源文件与共享的 `test/main.cc`，通过 `add_deps` 依赖库目标，并添加 `gtest` 包；共享 `main.cc` 只做 `InitGoogleTest`/`RUN_ALL_TESTS`，用例源文件里不要再写 `main`。
- 新增用例等于新增一个 `.cc` 文件；不需要修改 SDK 根工程，也不需要额外的生成或注册步骤。
- 需要纯 C/C++ 消费者、头文件或链接检查时，把 fixture 作为普通源文件签入库的 `test/compile/`，用普通二进制目标构建（参见 divsufsort 的 `test/compile/consumer.c`、`headers.cpp`）。这类检查不依赖 GoogleTest。

下面是模式示例（库名以 `example` 代指）：

```lua
target("example")
    set_kind("object")
    set_default(false)
    add_files("src/*.c")
    add_includedirs("include", {public = true})
    -- Capture the qualified name before leaving the library target scope.
    local library_scope = interp_get_scopename()
target_end()

-- Declare test targets outside the production target.
local standalone = path.normalize(os.projectdir()) == path.normalize(os.scriptdir())
if is_plat("linux", "windows", "macosx") and
   ((standalone and library_scope == "example") or
    (library_scope == "xhive_host::example" and has_config("xhive_tests"))) then
    add_requires("gtest")
    for _, file in ipairs(os.files("test/**.cc")) do
        local relative = path.relative(file, "test")
        if relative ~= "main.cc" then
            local suffix = relative:gsub("%.cc$", ""):gsub("[/\\]", "_")
            target("example_test_" .. suffix)
                set_kind("binary")
                set_default(false)
                set_languages("c++17")
                add_files(file, "test/main.cc")
                add_deps("example")
                add_packages("gtest")
                add_tests(path.basename(file), {
                    group = "xhive",
                    realtime_output = false,
                    run_timeout = 120 * 1000
                })
            target_end()
        end
    end
end
```

- SDK 模式中，GoogleTest 与库的主机测试目标只在 `xhive_host` 作用域、且 `xhive_tests` 打开时注册；`xhive_embed` 不会重复注册测试二进制或引入 GoogleTest 依赖。
- SDK 模式中，`xhive_tests=n` 不注册主机测试目标与 GoogleTest 依赖；再设置 `xhive_cross_tests=n` 后，构建只需要主机 C/C++ 工具链，不需要 Python 或交叉编译器。
- `tests/` 目录只保留 SDK 构建系统回归脚本（`sdk_entry`、`target_selection`、`riscv_toolset`/`riscv_startup`），它是既有约定，**不是**新增库测试的模板。
- C2 软件的包装目标声明在 `vendor/openecos/starrysky/c2/tests/xmake.lua`，选择器仍是 `xhive_test_c2/host` 与 `xhive_test_c2_full/full`；C2 的 Python 测试套件继续保留。
