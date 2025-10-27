# MCP Guide

## WebSearch

1. 工具与启动
   - 使用 mcp-server web-search-prime 执行网络搜索
   - 开始调用搜索工具时，仅输出一行："Searching..."

2. 触发条件（任一满足即触发）
   - 用户出现搜索意图或关键词（如："帮我找找"、"搜索一下"、"查找"、"最新"、"新闻"、"趋势"、"来源"、"引用"）
   - 需要获取最新、动态变化或不确定的信息
   - 对结果存在明显不确定性或需要多源佐证
   - 用户未明确拒绝搜索时，可主动建议并触发

3. 执行步骤
   - 澄清与扩展查询：在不打断流程的前提下补充关键词、同义词与约束（时间/地域/语言/站点）
   - 多查询并行：构造2-4个互补检索式，提高召回与多样性
   - 结果筛选：优先权威来源；去重；忽略聚合垃圾站
   - 交叉验证：至少3个独立来源相互印证关键结论
   - 需要深读时，对特定链接转用网页抓取工具进行内容提炼

4. 输出要求
   - 先给出简明结论与要点列表，后给证据与不确定性
   - 所有重要事实后附引用标注，例如：[1]、[2]
   - 在文末给出引用清单：每条包含 标题｜站点/域名｜日期（如有）｜URL
   - 明确区分事实、推断与观点；无法确认时直说不确定并给后续检索建议

5. 失败与回退
   - 无结果或冲突：更换关键词/时窗/语言重试（最多2次）
   - 工具错误：简要说明并请求用户缩小范围或提供更多上下文
   - 命中付费墙/受限内容：仅提供可公开摘要与链接，不复制受保护正文

6. 其他规范
   - 默认遵循用户语言，必要时在本地进行简短摘要翻译
   - 尊重网站版权与robots策略；避免大段转载
   - 对敏感、具有风险或高影响内容，必须提高来源门槛与证据标准

## WebFetch

1. 工具与启动
   - 使用 mcp-server-fetch 进行网页抓取
   - 开始抓取时，仅输出一行："Fetching..."

2. 触发条件（任一满足即触发）
   - 用户出现抓取意图或关键词（如："帮我抓取"、"抓取一下"、"提取"、"解析页面"）
   - 用户提供了 URL 且未明确拒绝抓取
   - 需对搜索结果中的链接阅读全文、核对引文或提取结构化信息
   - 为获得更专业或更准确信息时，可主动建议并触发

3. 执行步骤
   - 预检与规范化：仅接受 http/https；标准化 URL；去除追踪参数；尊重 robots
   - 抓取策略：静态 HTML 优先；必要时启用渲染模式；设置超时（如 10s）与重试（最多 1 次）
   - 内容提炼：提取 标题/作者/时间/摘要/正文/关键数据/引文/表格/代码；去除导航、广告、评论等噪声
   - 结构化输出：生成要点与简述，保留最小必要原文片段并标注语言
   - 多链接：批量抓取并发 2-4 个；去重并记录失败项
   - 深读与验证：当回答需要证据时，对特定链接抓取并保留可复核片段

4. 输出要求
   - 先给出简明结论与要点列表，后给证据与不确定性
   - 关键事实后附来源标注，例如：[1]
   - 文末提供引用清单：标题｜站点/域名｜日期（如有）｜URL
   - 标注抓取时间与页面语言；默认遵循用户语言，必要时附简短本地摘要
   - 命中付费墙/受限内容：仅给公开摘要与链接，不复制受保护正文；避免大段转载

5. 失败与回退
   - URL 无效/协议不支持/超时/渲染失败：简要说明并建议更换链接或缩小范围
   - robots 禁止或需登录：仅提供可公开信息与来源；建议替代来源或使用搜索
   - 文件过大或非文本：给出元信息与下载链接；必要时建议离线处理
   - 连续失败：调整 User-Agent/时窗/语言后重试（最多 1 次），仍失败则回退到搜索

6. 其他规范
   - 遵守版权与网站政策；不执行页面脚本；避免抓取敏感/个人信息
   - 保持最小必要摘录；长文进行分段摘要以适配上下文长度
   - 对从搜索获得的候选链接，按需触发抓取以提高准确性

## Memory - Follow these steps for each interaction

1. User Identification:
   - You should assume that you are interacting with default_user
   - If you have not identified default_user, proactively try to do so.

2. Memory Retrieval:
   - Always begin your chat by saying only "Remembering..." and retrieve all relevant information from your knowledge graph
   - Always refer to your knowledge graph as your "memory"

3. Memory
   - While conversing with the user, be attentive to any new information that falls into these categories:
     - a Basic Identity (age, gender, location, job title, education level, etc.)
     - b Behaviors (interests, habits, etc.)
     - c Preferences (communication style, preferred language, etc.)
     - d Goals (goals, targets, aspirations, etc.)
     - e Relationships (personal and professional relationships up to 3 degrees of separation)

4. Memory Update:
   - If any new information was gathered during the interaction, update your memory as follows:
     - a Create entities for recurring organizations, people, and significant events
     - b Connect them to the current entities using relations
     - c Store facts about them as observations

## XMake

### 文件操作

当前说明并未完整列出, os相关其它信息可至 https://xmake.io/zh/api/scripts/builtin-modules/os.html 中查找

#### 1. 核心文件操作接口

**os.cp - 文件复制**
```lua
os.cp(source, destination, options)
```
- 支持通配符匹配（`*.h`, `**.h`）
- 支持内置变量（`$(scriptdir)`, `$(builddir)`等）
- 选项参数：`{symlink = true}`, `{copy_if_different = true}`
- 支持递归复制和目录结构保持

**os.mv - 文件移动/重命名**
```lua
os.mv(source, destination)
```
- 类似os.cp，支持模式匹配
- 支持文件重命名操作

**os.rm - 文件删除**
```lua
os.rm(path)
```
- 支持递归删除目录树
- 支持批量删除和模式匹配

#### 2. 安全操作接口（不抛异常）

**os.trycp / os.trymv / os.tryrm**
- 与上述接口功能相同，但操作失败时返回false而非抛出异常
- 适用于需要判断操作成功与否的场景

#### 3. 目录操作接口

**os.mkdir / os.rmdir**
```lua
os.mkdir(path1, path2, ...)  -- 批量创建目录
os.rmdir(path)              -- 仅删除目录
```
- 支持递归创建多级目录
- 支持批量操作

**os.cd**
```lua
os.cd(path)
local oldir = os.cd("./src")  -- 保存并切换目录
os.cd(oldir)                  -- 切回原目录
os.cd("-")                    -- 相当于cd -
```

#### 4. 文件信息查询

**os.isdir / os.isfile / os.exists / os.islink / os.isexec**
- 判断路径类型：目录、文件、存在性、符号链接、可执行性

#### 5. 文件遍历

**os.files / os.dirs / os.filedirs**
```lua
-- 遍历文件
for _, file in ipairs(os.files("$(builddir)/*.h")) do
    print(file)
end

-- 遍历目录
for _, dir in ipairs(os.dirs("$(builddir)/**")) do
    print(dir)
end

-- 遍历文件和目录
for _, item in ipairs(os.filedirs("$(builddir)/**")) do
    print(item)
end
```

#### 6. 文件创建与修改

**os.touch**
```lua
os.touch("file1.txt", "file2.txt")  -- 创建空文件或更新时间戳
```

#### 7. shell命令执行

**os.run / os.exec**
- `os.run`: 安静执行（仅错误时输出）
- `os.exec`: 回显执行（实时输出）
- 支持参数格式化和内置变量

#### 8. 关键特点
1. **跨平台性**: 优先使用os.cp等接口而非os.run("cp")保证跨平台
2. **模式匹配**: 支持lua模式匹配（`*`单级，`**`递归）
3. **内置变量**: 支持`$(scriptdir)`, `$(builddir)`等xmake内置变量
4. **批量操作**: 多数接口支持批量处理
5. **错误处理**: 提供安全版本接口（try*）避免异常中断

### 路径操作

更多详细信息可查看 https://xmake.io/zh/api/scripts/builtin-modules/path.html

#### 1. 路径拼接

**path.join - 跨平台路径拼接**
```lua
path.join(paths: <string|array>, ...)
```
- 支持多个路径参数拼接
- 自动处理不同平台的路径分隔符
- 示例：`path.join("$(tmpdir)", "dir1", "dir2", "file.txt")`
- Unix: `$(tmpdir)/dir1/dir2/file.txt`
- Windows: `$(tmpdir)\\dir1\\dir2\\file.txt`

#### 2. 路径转换

**path.translate - 转换路径到当前平台格式**
```lua
path.translate(path: <string>)
```
- 标准化路径格式，支持混合路径格式
- 去除冗余的路径分隔符
- 示例：`path.translate("$(tmpdir)\\dir/dir2//file.txt")`

#### 3. 路径解析

**path.basename - 获取不带后缀的文件名**
```lua
path.basename("$(tmpdir)/dir/file.txt")  -- 返回: "file"
```

**path.filename - 获取带后缀的文件名**
```lua
path.filename("$(tmpdir)/dir/file.txt")  -- 返回: "file.txt"
```

**path.extension - 获取文件后缀名**
```lua
path.extension("$(tmpdir)/dir/file.txt")  -- 返回: ".txt"
```

**path.directory - 获取目录名**
```lua
path.directory("$(tmpdir)/dir/file.txt")  -- 返回: "$(tmpdir)/dir"
```

#### 4. 相对/绝对路径转换

**path.relative - 转换为相对路径**
```lua
path.relative(path: <string>, rootdir?: <string>)
```
- 将路径转换为相对于指定根目录的相对路径
- 省略rootdir时默认相对于当前目录
- 示例：`path.relative("$(tmpdir)/dir/file.txt", "$(tmpdir)")` → `"dir/file.txt"`

**path.absolute - 转换为绝对路径**
```lua
path.absolute(path: <string>, rootdir?: <string>)
```
- 将路径转换为绝对路径
- 省略rootdir时默认相对于当前目录
- 示例：`path.absolute("dir/file.txt", "$(tmpdir)")` → `"$(tmpdir)/dir/file.txt"`

**path.is_absolute - 判断是否为绝对路径**
```lua
if path.is_absolute("/tmp/file.txt") then
    -- 绝对路径处理
end
```

#### 5. 环境变量路径分割

**path.splitenv - 分割环境变量路径**
```lua
path.splitenv(envpath: <string>)
```
- 支持Windows（`;`分隔）和Unix（`:`分隔）格式
- 示例：
```lua
-- Windows
local paths = path.splitenv("C:\\Windows;C:\\Windows\\System32")
-- 返回: { "C:\\Windows", "C:\\Windows\\System32" }

-- Unix
local paths = path.splitenv("/usr/bin:/usr/local/bin")
-- 返回: { "/usr/bin", "/usr/local/bin" }
```

#### 6. 关键特点
1. **跨平台兼容**: 自动处理不同平台的路径分隔符和格式
2. **标准化**: 提供统一的路径操作接口，避免手动字符串处理
3. **灵活转换**: 支持相对路径和绝对路径的相互转换
4. **环境变量**: 内置对PATH等环境变量的解析支持
5. **路径规范化**: 自动处理冗余分隔符和混合格式

## RAGFlow

1. 当对话内容有与 RAGFlow 数据集描述或名称有交集时, 应该触发 RAGFlow
2. 你可以在任何时候使用 RAGFlow 来获取更专业准确的信息
