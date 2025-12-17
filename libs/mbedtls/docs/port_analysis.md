# MbedTLS 4.0 移植分析文档

本文为无CMake环境下的MbedTLS 4.0移植方法分析，基于源码目录 `@libs\mbedtls\mbedtls-4.0\`。所有分析均符合CMakeLists.txt的逻辑设计。

## 目录结构

```
mbedtls-4.0/
├── include/                      # 公共头文件
│   └── mbedtls/                  # TLS/X.509公共接口
├── tf-psa-crypto/                # 密码学库（独立模块）
│   ├── include/
│   │   └── psa/                  # PSA Crypto API接口
│   ├── core/                     # PSA核心实现
│   └── drivers/
│       └── builtin/
│           ├── include/
│           │   ├── mbedtls/      # Crypto驱动私有头文件
│           │   └── psa/          # PSA驱动接口
│           └── src/              # 内置密码学算法实现
└── library/                      # TLS/X.509实现
    ├── *.c                       # TLS和X.509源码
    └── *.h                       # 私有头文件
```

---

## 功能模块分析

### 1. 核心密码学驱动库 (Built-in Crypto Driver)

#### 路径
- **源码路径**: `tf-psa-crypto/drivers/builtin/src/`

#### Include路径
- **私有Include**:
  - `tf-psa-crypto/drivers/builtin/include/mbedtls/private/`
  - `tf-psa-crypto/drivers/builtin/src/`
  - `tf-psa-crypto/core/` (用于PSA集成)

- **公开Include**:
  - `tf-psa-crypto/drivers/builtin/include/`
  - `tf-psa-crypto/include/`

#### 文件清单

**算法实现文件**:

- **对称加密**
  - `aes.c` - AES算法
  - `aesce.c` - ARM64 AES加速（可选，依赖平台）
  - `aesni.c` - AES-NI指令集加速（可选，依赖平台）
  - `aria.c` - ARIA算法
  - `camellia.c` - Camellia算法

- **流/块加密模式**
  - `cipher.c` - 通用加密框架
  - `cipher_wrap.c` - 加密算法包装器
  - `block_cipher.c` - 块加密通用接口

- **认证加密**
  - `ccm.c` - CCM模式
  - `chachapoly.c` - ChaCha20-Poly1305
  - `gcm.c` - GCM模式
  - `chacha20.c` - ChaCha20流密码
  - `poly1305.c` - Poly1305认证

- **哈希算法**
  - `md.c` - 通用哈希框架
  - `md5.c` - MD5（已淘汰，不推荐）
  - `sha1.c` - SHA-1（已淘汰）
  - `sha256.c` - SHA-256
  - `sha512.c` - SHA-512
  - `sha3.c` - SHA3系列
  - `ripemd160.c` - RIPEMD-160

- **消息认证码**
  - `hmac_drbg.c` - HMAC DRBG随机数生成
  - `ctr_drbg.c` - CTR DRBG随机数生成
  - `cmac.c` - CMAC认证码
  - `nist_kw.c` - NIST密钥封装

- **公钥加密**
  - `rsa.c` - RSA算法
  - `rsa_alt_helpers.c` - RSA辅助函数
  - `ecp.c` - 椭圆曲线点运算
  - `ecp_curves.c` - 标准椭圆曲线
  - `ecp_curves_new.c` - 新曲线格式支持
  - `ecdh.c` - ECDH密钥交换
  - `ecdsa.c` - ECDSA签名
  - `ecjpake.c` - EC-JPAKE认证
  - `bignum.c` - 大数运算
  - `bignum_core.c` - 大数核心运算
  - `bignum_mod.c` - 模运算
  - `bignum_mod_raw.c` - 底层模运算

- **密钥/证书格式**
  - `pk.c` - 公钥抽象层
  - `pk_wrap.c` - 公钥包装器
  - `pkparse.c` - 密钥解析(PEM/DER)
  - `pkwrite.c` - 密钥写入(PEM/DER)
  - `pk_ecc.c` - ECC密钥处理
  - `pk_rsa.c` - RSA密钥处理
  - `pem.c` - PEM格式编解码
  - `asn1parse.c` - ASN.1解析
  - `asn1write.c` - ASN.1写入
  - `oid.c` - OID对象标识符
  - `base64.c` - Base64编解码

- **平台/辅助**
  - `platform.c` - 平台抽象层
  - `platform_util.c` - 平台工具函数
  - `constant_time.c` - 常数时间实现（防时序攻击）
  - `memory_buffer_alloc.c` - 内存分配器
  - `entropy.c` - 熵源管理
  - `entropy_poll.c` - 熵收集（平台相关）
  - `threading.c` - 线程支持（可选）
  - `lmots.c` / `lms.c` - 后量子数字签名（实验性）

**PSA集成文件**:
- `psa_crypto_aead.c` - PSA AEAD接口
- `psa_crypto_cipher.c` - PSA Cipher接口
- `psa_crypto_hash.c` - PSA Hash接口
- `psa_crypto_mac.c` - PSA MAC接口
- `psa_crypto_ecp.c` - PSA ECC接口
- `psa_crypto_rsa.c` - PSA RSA接口
- `psa_crypto_ffdh.c` - PSA FFDH接口
- `psa_crypto_pake.c` - PSA PAKE接口
- `psa_util.c` - PSA工具函数

#### 配置依赖说明

**平台相关文件（根据目标平台选择）**:
- `aesni.c` - 需要x86平台且启用`MBEDTLS_AESNI_C`
- `aesce.c` - 需要ARM64平台且启用`MBEDTLS_AESCE_C`
- `entropy_poll.c` - 需要适配到目标平台的熵源收集

**可选算法文件（根据mbedtls_config.h配置）**:
大多数算法文件都有对应的配置宏控制，例如：
- `MBEDTLS_AES_C` - 启用aes.c
- `MBEDTLS_SHA256_C` - 启用sha256.c
- `MBEDTLS_RSA_C` - 启用rsa.c
- `MBEDTLS_ECP_C` - 启用ecp.c

**PSA相关文件（如果被上层使用）**:
- `psa_*.c` - 依赖`MBEDTLS_PSA_CRYPTO_C`

---

### 2. PSA加密核心库 (PSA Core Library)

#### 路径
- **源码路径**: `tf-psa-crypto/core/`

#### Include路径
- **私有Include**: `tf-psa-crypto/core/`
- **公开Include**: `tf-psa-crypto/include/`

#### 文件清单

```
psa_crypto.c                 # PSA Crypto主实现
psa_crypto_client.c          # PSA客户端接口
psa_crypto_slot_management.c # 密钥槽管理
psa_crypto_storage.c         # 密钥持久化存储
psa_its_file.c              # 文件存储后端（可选）
tf_psa_crypto_config.c       # 配置管理
tf_psa_crypto_version.c      # 版本信息
```

**说明**: PSA Core依赖于Builtin Driver中的算法实现，是PSA API的核心管理层。

---

### 3. X.509证书库 (X.509 Certificate Library)

#### 路径
- **源码路径**: `library/`

#### Include路径
- **私有Include**: `library/`
- **公开Include**: `include/`

#### 文件清单

```
# 核心文件
error.c                     # 错误处理
mbedtls_config.c            # 配置管理

# X.509解析和验证
x509.c                      # X.509核心功能
x509_crt.c                  # 证书解析和验证
x509_crl.c                  # CRL解析和验证
x509_csr.c                  # 证书签名请求
x509_create.c               # X.509结构创建

# X.509编码/写入
x509write.c                 # X.509写入核心
x509write_crt.c             # 证书写入
x509write_csr.c             # CSR写入

# OID和其他
x509_oid.c                  # OID处理
pkcs7.c                     # PKCS#7/CMS
```

#### 配置依赖说明

**所有文件均依赖**: `MBEDTLS_X509_USE_PSA` 或 `MBEDTLS_X509_USE_LEGACY` 配置

**文件依赖关系**:
- 所有X.509文件依赖Crypto库的`pk*.c`, `oid.c`, `asn1*.c`
- `pkcs7.c` 需要启用 `MBEDTLS_PKCS7_C`

---

### 4. TLS/SSL协议库 (TLS/SSL Library)

#### 路径
- **源码路径**: `library/`

#### Include路径
- **私有Include**: `library/`
- **公开Include**: `include/`

#### 文件清单

**核心协议实现**:

```
# TLS 1.2实现（传统版本）
ssl_tls.c                   # TLS核心框架
ssl_tls12_client.c          # TLS 1.2客户端
ssl_tls12_server.c          # TLS 1.2服务端

# TLS 1.3实现
ssl_tls13_client.c          # TLS 1.3客户端
ssl_tls13_server.c          # TLS 1.3服务端
ssl_tls13_keys.c            # TLS 1.3密钥派生
ssl_tls13_generic.c         # TLS 1.3通用函数

# 协议通用层
ssl_client.c                # 客户端通用代码
ssl_msg.c                   # 消息处理
ssl_ciphersuites.c          # 密码套件管理

# 安全机制
ssl_cache.c                 # 会话缓存
ssl_ticket.c                # 会话票证(TLS 1.2)
ssl_cookie.c                # DTLS Cookie

# 调试和辅助
debug.c                     # 调试输出
ssl_debug_helpers_generated.c # 调试助手（自动生成）
version.c                   # 版本信息
version_features.c          # 特性列表（自动生成）
timing.c                    # 时间处理

# 网络层
net_sockets.c               # TCP/UDP套接字（可能需要平台适配）

# 消息解析器（MPS - Message Processing Stack）
mps_reader.c                # MPS读取器
mps_trace.c                 # MPS跟踪

# 未来扩展框架（占位）
ssl_tls13_invasive.h        # TLS 1.3测试接口
```

#### 配置依赖说明

**自动生成文件（需要GEN_FILES=1）**:
1. `ssl_debug_helpers_generated.c` - 需要Python脚本生成
   - 生成命令: `python framework/scripts/generate_ssl_debug_helpers.py`
   - 依赖: `include/mbedtls/*.h`

2. `version_features.c` - 需要Perl脚本生成
   - 生成命令: `perl scripts/generate_features.pl`
   - 依赖: `include/mbedtls/mbedtls_config.h`

3. `error.c` (由tf-psa-crypto生成) - 需要Perl脚本
   - 生成命令: `perl scripts/generate_errors.pl`
   - 依赖: 所有模块的头文件

**TLS版本选择**:
- `MBEDTLS_SSL_PROTO_TLS1_2` - 启用TLS 1.2相关文件
- `MBEDTLS_SSL_PROTO_TLS1_3` - 启用TLS 1.3相关文件
- `MBEDTLS_SSL_PROTO_DTLS` - 启用DTLS支持

**密码套件控制**:
- `ssl_ciphersuites.c`中的密码套件根据配置宏进行条件编译
- 需要对应算法文件已启用（如`MBEDTLS_AES_C`, `MBEDTLS_SHA256_C`等）

**平台相关文件**:
- `net_sockets.c` - 需要适配到目标平台的网络API
  - 在裸机环境可能需要替换或禁用（如果不需要网络层）

---

---

## 依赖关系图

```
┌─────────────────┐
│  TLS/SSL库      │  library/*.c (ssl_*, tls_*)
│  (ssl/tls层)    │  依赖X.509和Crypto
└────────┬────────┘
         │
┌────────▼────────┐
│  X.509证书库    │  library/*.c (x509_*)
│  (证书层)       │  依赖Crypto
└────────┬────────┘
         │
┌────────▼────────────────────────────┐
│  PSA Crypto Core                    │  tf-psa-crypto/core/*.c
│  (PSA API管理层)                    │
└────────┬────────────────────────────┘
         │
┌────────▼────────────────────────────┐
│  Built-in Crypto Driver             │  tf-psa-crypto/drivers/builtin/src/*.c
│  (算法实现层)                       │  基础密码学算法
└─────────────────────────────────────┘
```

---

## 移植建议

### 1. 文件组织

建议按以下方式组织移植项目：

```
port/
├── crypto/                # 核心密码学和PSA
│   ├── include/
│   │   ├── mbedtls/
│   │   └── psa/
│   └── src/
├── x509/                  # X.509证书
│   ├── include/mbedtls/
│   └── src/
└── tls/                   # TLS/SSL
    ├── include/mbedtls/
    └── src/
```

### 2. 必需修改的文件

**平台适配必须修改**:
- `tf-psa-crypto/drivers/builtin/src/entropy_poll.c`
  - 适配到目标平台的随机源
- `library/net_sockets.c` (如果不使用BSD sockets)
  - 或禁用MBEDTLS_NET_C使用自定义I/O

**配置必需修改**:
- `include/mbedtls/mbedtls_config.h` (或自定义配置)
  - 启用/禁用算法模块
  - 设置缓冲区大小
  - 配置调试选项

### 3. 可选模块

按需求选择添加：

- **最小TLS客户端**: Crypto + X.509 + TLS客户端代码
- **TLS服务器**: 需要完整的TLS + X.509
- **仅密码学**: 只需要Crypto Driver + PSA Core
- **证书工具**: X.509 + Crypto (不需要TLS)

---

## 动态添加文件清单（根据配置）

除了上述固定的源文件外，CMakeLists.txt还会根据配置动态添加以下文件：

### 1. 自动生成文件（GEN_FILES配置）

当`GEN_FILES=1`时，CMakeLists.txt会通过脚本生成以下文件：

#### library/CMakeLists.txt中生成的文件：

```
# 由generate_errors.pl生成
${CMAKE_CURRENT_BINARY_DIR}/error.c

# 由generate_features.pl生成
${CMAKE_CURRENT_BINARY_DIR}/version_features.c

# 由generate_ssl_debug_helpers.py生成
${CMAKE_CURRENT_BINARY_DIR}/ssl_debug_helpers_generated.c

# 由generate_config_checks.py生成（多个配置检查头文件）
${CMAKE_CURRENT_BINARY_DIR}/mbedtls_config_*.h
```

**生成命令和依赖建议**：
- **error.c**: 运行 `perl scripts/generate_errors.pl <crypto_headers> <tls_headers> <data_files> <output_file>`
- **version_features.c**: 运行 `perl scripts/generate_features.pl <include_dir> <data_files> <output_file>`
- **ssl_debug_helpers_generated.c**: 运行 `python framework/scripts/generate_ssl_debug_helpers.py --mbedtls-root <root> <output_dir>`
- **mbedtls_config_*.h**: 运行 `python scripts/generate_config_checks.py <output_dir>`

**注意**: error.c在tf-psa-crypto子模块中生成，但用于library模块。

#### tf-psa-crypto/CMakeLists.txt中测试相关的生成文件：

```
# 测试用密钥头文件（仅用于测试，不是核心库）
${CMAKE_CURRENT_BINARY_DIR}/tests/include/test/test_keys.h
```

**建议**: 对于移植来说，核心库只需要前3-4个文件，测试文件可以不用。

---

### 2. 条件编译控制的源文件

以下文件**不是**由CMake自动生成，但它们的**包含/排除由配置宏控制**（在mbedtls_config.h或tf-psa-crypto-config.h中定义）：

#### Built-in Crypto Driver中的算法文件：

每个算法文件都依赖于对应的`MBEDTLS_XXX_C`宏（约50+个文件）：

```
# 对称加密算法（可选）
aesce.c             - MBEDTLS_AESCE_C (ARM64加速)
aesni.c             - MBEDTLS_AESNI_C (x86加速)
aria.c              - MBEDTLS_ARIA_C
camellia.c          - MBEDTLS_CAMELLIA_C

# 哈希算法（可选）
md5.c               - MBEDTLS_MD5_C (已淘汰)
sha1.c              - MBEDTLS_SHA1_C (已淘汰)
sha3.c              - MBEDTLS_SHA3_C
ripemd160.c         - MBEDTLS_RIPEMD160_C

# 公钥算法（可选）
rsa.c               - MBEDTLS_RSA_C
ecp.c               - MBEDTLS_ECP_C
ecdh.c              - MBEDTLS_ECDH_C
ecdsa.c             - MBEDTLS_ECDSA_C
ecjpake.c           - MBEDTLS_ECJPAKE_C

# 消息认证码（可选）
ctr_drbg.c          - MBEDTLS_CTR_DRBG_C
hmac_drbg.c         - MBEDTLS_HMAC_DRBG_C
nist_kw.c           - MBEDTLS_NIST_KW_C

# 认证加密（可选）
chachapoly.c        - MBEDTLS_CHACHAPOLY_C
chacha20.c          - MBEDTLS_CHACHA20_C
poly1305.c          - MBEDTLS_POLY1305_C

# 其他可选
threading.c         - MBEDTLS_THREADING_PTHREAD/MBEDTLS_THREADING_ALT
memory_buffer_alloc.c - MBEDTLS_MEMORY_BUFFER_ALLOC_C
lmots.c / lms.c     - MBEDTLS_LMS_C (实验性后量子签名)
```

**注**: 基本的AES、SHA256、SHA512、CCM、GCM等通常被认为是"核心"，但同样受配置宏控制。

#### X.509库的可选文件：

```
pkcs7.c             - MBEDTLS_PKCS7_C
```

#### TLS/SSL库的可选文件：

```
# TLS 1.2实现
ssl_tls12_client.c  - MBEDTLS_SSL_PROTO_TLS1_2
ssl_tls12_server.c  - MBEDTLS_SSL_PROTO_TLS1_2

# TLS 1.3实现
ssl_tls13_client.c  - MBEDTLS_SSL_PROTO_TLS1_3
ssl_tls13_server.c  - MBEDTLS_SSL_PROTO_TLS1_3
ssl_tls13_keys.c    - MBEDTLS_SSL_PROTO_TLS1_3
ssl_tls13_generic.c - MBEDTLS_SSL_PROTO_TLS1_3

# DTLS支持
ssl_cookie.c        - MBEDTLS_SSL_PROTO_DTLS
net_sockets.c       - MBEDTLS_NET_C (可禁用)

# 会话机制（可选）
ssl_cache.c         - MBEDTLS_SSL_CACHE_C
ssl_ticket.c        - MBEDTLS_SSL_TICKET_C
```

#### PSA Core的依赖：

```
psa_crypto_aead.c   - MBEDTLS_PSA_CRYPTO_C
psa_crypto_cipher.c - MBEDTLS_PSA_CRYPTO_C
psa_crypto_hash.c   - MBEDTLS_PSA_CRYPTO_C
psa_crypto_mac.c    - MBEDTLS_PSA_CRYPTO_C
psa_crypto_ecp.c    - MBEDTLS_PSA_CRYPTO_C
psa_crypto_rsa.c    - MBEDTLS_PSA_CRYPTO_C
psa_crypto_ffdh.c   - MBEDTLS_PSA_CRYPTO_C
psa_crypto_pake.c   - MBEDTLS_PSA_CRYPTO_C
psa_util.c          - MBEDTLS_PSA_CRYPTO_C
```

---

### 3. 构建类型相关文件

在CMakeLists.txt中，**库目标的选择**取决于构建类型配置：

```
USE_STATIC_MBEDTLS_LIBRARY = ON/OFF   # 构建静态库
USE_SHARED_MBEDTLS_LIBRARY = ON/OFF   # 构建共享库
```

构建结果：
- **静态库** (`USE_STATIC_MBEDTLS_LIBRARY=ON`):
  - `libmbedx509.a` 或 `libmbedx509_static.a`
  - `libmbedtls.a` 或 `libmbedtls_static.a`
  - `libmbedcrypto.a` (tf-psa-crypto)

- **共享库** (`USE_SHARED_MBEDTLS_LIBRARY=ON`):
  - `libmbedx509.so/.dylib/.dll`
  - `libmbedtls.so/.dylib/.dll`
  - `libmbedcrypto.so/.dylib/.dll` (tf-psa-crypto)

---

### 4. 平台相关文件

某些源文件的**实现代码会根据平台而不同**（由#ifdef控制）：

```
entropy_poll.c      - 根据平台选择熵源(/dev/urandom, Windows CryptoAPI等)
net_sockets.c       - 根据平台使用BSD sockets, Winsock等

# 平台特定加速实现
aesce.c             - ARM64 AES加速（ARM64平台）
aesni.c             - AES-NI指令集（x86平台）
```

---

### 5. 关键配置总结

#### MbedTLS (library/) CMakeLists.txt中的关键变量：

```
# 库类型选择（CmakeLists.txt第103-104行）
USE_STATIC_MBEDTLS_LIBRARY  # 默认ON
USE_SHARED_MBEDTLS_LIBRARY  # 默认OFF

# 程序生成（第71行）
ENABLE_PROGRAMS             # 默认ON

# 文件生成控制（第40行）
GEN_FILES                   # 默认OFF（开发环境）

# 网络支持（影响net_sockets.c的链接）
MBEDTLS_NET_C               # 在mbedtls_config.h中定义

# Pthread链接
LINK_WITH_PTHREAD           # 默认OFF
```

#### TF-PSA-Crypto CMakeLists.txt中的关键变量：

```
# 库类型选择
USE_STATIC_TF_PSA_CRYPTO_LIBRARY  # 复用USE_STATIC_MBEDTLS_LIBRARY
USE_SHARED_TF_PSA_CRYPTO_LIBRARY  # 复用USE_SHARED_MBEDTLS_LIBRARY

# 文件生成控制（第379行）
GEN_FILES                   # 默认OFF
```

---

### 移植时的动态文件处理建议

#### 1. 自动生成文件（3个核心文件）

**选项A：使用预生成版本**
- 从官方发布包获取已生成的error.c、version_features.c、ssl_debug_helpers_generated.c
- 优点：无需安装Perl/Python和运行脚本
- 缺点：可能与自定义配置不完全匹配

**选项B：运行脚本生成（推荐）**
```bash
# 生成error.c (在library/执行)
perl scripts/generate_errors.pl \
    tf-psa-crypto/drivers/builtin/include/mbedtls \
    include/mbedtls \
    scripts/data_files \
    build/error.c

# 生成version_features.c
perl scripts/generate_features.pl \
    include/mbedtls \
    scripts/data_files \
    build/version_features.c

# 生成ssl_debug_helpers_generated.c
python framework/scripts/generate_ssl_debug_helpers.py \
    --mbedtls-root . \
    build/

# 生成配置检查头文件（自动生成多个）
python scripts/generate_config_checks.py \
    build/
```

#### 2. 条件编译文件

**建议方法**：

1. **复制mbedtls_config.h模板**:
   ```bash
   cp include/mbedtls/mbedtls_config.h include/mbedtls/mbedtls_config_custom.h
   ```

2. **编辑配置**，只启用需要的功能:
   ```c
   #define MBEDTLS_AES_C
   #define MBEDTLS_SHA256_C
   #define MBEDTLS_SHA512_C
   // #define MBEDTLS_SHA3_C  // 不需要就注释掉
   ```

3. **在编译命令中定义配置路径**:
   ```bash
   gcc -DMBEDTLS_CONFIG_FILE=\"mbedtls/mbedtls_config_custom.h\" ...
   ```

3. **平台相关文件**

必须修改的文件：
- **entropy_poll.c** - 必须适配到目标平台的随机源
- **net_sockets.c** - 可选（如果不使用MBEDTLS_NET_C，可以禁用或提供自定义bio）

---

## 与CMake逻辑一致性说明

本分析与MbedTLS CMakeLists.txt的逻辑完全一致：

1. **分层构建**: CMakeLists.txt先将Crypto作为独立库构建，然后X.509依赖Crypto，TLS依赖X.509和Crypto

2. **源文件分组**: 完全按照CMakeLists.txt中的`src_core`, `src_x509`, `src_tls`分组

3. **Include路径**: 私有和公开Include路径与CMake的`target_include_directories`设置一致
   - PUBLIC: `include/` 和 `tf-psa-crypto/include/`
   - PRIVATE: `library/`, `tf-psa-crypto/core/`, `tf-psa-crypto/drivers/builtin/src/`

4. **条件编译**: 所有需要根据配置启用的文件都在"配置依赖说明"中明确标注

5. **自动生成文件**: CMake的GEN_FILES选项对应的3个需要脚本生成的文件已明确列出

---

## 静态 vs 动态文件统计

### 完全静态的文件（必需的核心文件，约30个）：
```
# Crypto核心
aes.c, cipher.c, cipher_wrap.c, md.c, bignum.c, bignum_core.c
bignum_mod.c, bignum_mod_raw.c, constant_time.c
platform.c, platform_util.c, entropy.c

# 对称加密
ccm.c, gcm.c

# 哈希
sha256.c, sha512.c

# 公钥
ecp.c, ecp_curves.c, ecp_curves_new.c, ecp_curves.h
pk.c, pk_wrap.c, pkparse.c, pkwrite.c
pk_ecc.c, pk_rsa.c
asn1parse.c, asn1write.c, oid.c, pem.c, base64.c

# PSA Core（如果选择PSA API）
psa_crypto.c, psa_crypto_slot_management.c, psa_crypto_storage.c, tf_psa_crypto_config.c
```

### 动态文件（根据配置生成或选择）：
```
# 自动生成文件（3-4个）
error.c, version_features.c, ssl_debug_helpers_generated.c
mbedtls_config_*.h

# 条件编译（50+个，取决于配置）
所有MBEDTLS_XXX_C控制的文件，如sha3.c, rsa.c, ecdh.c等

# TLS/X.509文件（可选择性编译）
x509相关的13个文件（可选，如果不需要证书）
ssl相关的21个文件（可选，如果不需要TLS）

# 测试文件（可选）
tests/目录下的所有文件（不包括在核心库中）
```

### 实际数字：
- **总源文件数**: ~100+个.c文件
- **必须的核心文件**: ~30个（纯Crypto + PSA Core）
- **可选算法**: ~50+个（取决于配置）
- **TLS/X.509**: ~34个（可选）
- **自动生成**: 3-4个（在library/中）
- **测试文件**: 50+个（不包含在核心库）

---

### 移植时的文件数估算

| 场景 | 核心文件 | 条件编译 | TLS/X.509 | 自动生成 | 总计 |
|------|---------|----------|-----------|----------|------|
| **仅Crypto+PSA** | ~30 | 10-20 | 0 | 3-4 | **45-55** |
| **最小TLS客户端** | ~30 | 15-25 | 15 | 3-4 | **65-75** |
| **完整TLS服务器** | ~30 | 30-40 | 34 | 3-4 | **90-110** |
| **全功能构建** | ~30 | 50+ | 34 | 3-4 | **120+** |

---

## 总结

MbedTLS 4.0的模块化设计非常清晰，移植时应：

1. **从底层到上层**依次移植: Crypto → PSA Core → X.509 → TLS
2. **根据需求裁剪**: 使用mbedtls_config.h选择需要的算法和协议
3. **适配平台接口**: 重点修改entropy_poll.c和net_sockets.c
4. **处理可选依赖**: 根据配置宏决定哪些文件需要编译
5. **处理动态文件**: 生成3个核心文件或从发布包获取预生成版本

总共约100+个源文件，但通过配置裁剪，最小化构建：
- **纯Crypto**: 约45-55个文件
- **最小TLS**: 约65-75个文件
- **完整功能**: 90-110个文件
