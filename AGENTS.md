# 项目开发规范

本文规定本仓库的 Git 提交信息和 Doxygen 注释格式。新增或修改本项目维护的代码时应遵守这些规范；不要为了统一格式而批量改写无关代码或第三方原始代码。

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
