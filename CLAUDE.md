# MCP Guide

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

**io.readfile - 一次性读取完整文件内容**
```lua
local content = io.readfile("file.txt")
```

**io.writefile - 一次性写入完整文件内容**
```lua
io.writefile("file.txt", content)
```

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
