# 贡献 Axon

感谢您对 Axon 的贡献兴趣！本文档为所有贡献者提供必要信息。

**[English Version](CONTRIBUTING.md)** | 中文版文档

## 目录

- [行为准则](#行为准则) - 详见 [CODE_OF_CONDUCT_ZH.md](CODE_OF_CONDUCT_ZH.md)
- [开始贡献](#开始贡献)
- [开发环境搭建](#开发环境搭建)
- [贡献指南](#贡献指南)
- [开发工作流](#开发工作流)
- [代码规范](#代码规范)
- [测试](#测试)
- [构建](#构建)
- [提交更改](#提交更改)
- [报告问题](#报告问题)
- [社区准则](#社区准则)

---

## 行为准则

**详见 [CODE_OF_CONDUCT_ZH.md](CODE_OF_CONDUCT_ZH.md)** 获取完整的行为准则。

---

## 开始贡献

### 贡献方式

有多种方式可以为 Axon 做出贡献：

1. **报告错误** - 提交错误报告帮助我们改进
2. **建议功能** - 请求对您有用的功能
3. **编写代码** - 修复错误或实现功能
4. **改进文档** - 帮助让 Axon 更易于理解
5. **审查拉取请求** - 帮助审查和测试贡献
6. **回答问题** - 在 GitHub 讨论中帮助其他用户
7. **分享用例** - 告诉我们您如何使用 Axon

### 首次贡献者

我们欢迎首次贡献者！可以从以下开始：
- 标记为 `good first issue` 的议题
- 标记为 `help wanted` 的议题
- 文档改进
- 错误修复

---

## 开发环境搭建

### 前置要求

**系统依赖：**
- CMake 3.12+
- GCC 7+ 或 Clang 6+（支持 C++17）
- Boost 1.71+ (`libboost-all-dev`)
- yaml-cpp (`libyaml-cpp-dev`)
- OpenSSL (`libssl-dev`)
- zstd (`libzstd-dev`) - 可选，用于 MCAP 压缩
- lz4 (`liblz4-dev`) - 可选，用于 MCAP 压缩

**ROS 环境：**
- ROS 1 Noetic 或 ROS 2（Humble/Jazzy/Rolling）

**开发工具：**
- clang-format（必需，用于代码格式化）
- clang-format-14 或更高版本推荐
- clang-tidy（可选，用于静态分析）
- lcov（用于覆盖率报告）
- cppcheck（可选，用于静态分析）

### 初始设置

1. **Fork 并克隆仓库：**
```bash
# 首先在 GitHub 上 Fork 仓库
git clone --recurse-submodules https://github.com/YOUR_USERNAME/Axon.git
cd Axon
git remote add upstream https://github.com/ArcheBase/Axon.git
```

2. **配置 git 钩子：**
```bash
git config core.hooksPath githooks
```

3. **配置 ROS 环境：**
```bash
# ROS 1
source /opt/ros/noetic/setup.bash

# 或 ROS 2
source /opt/ros/<distro>/setup.bash
```

4. **安装依赖：**
```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libboost-all-dev \
    libyaml-cpp-dev \
    libssl-dev \
    libzstd-dev \
    liblz4-dev \
    clang-format \
    clang-tidy \
    lcov \
    cppcheck
```

---

## 贡献指南

### 许可证

通过向 Axon 贡献，您同意您的贡献将在 **Mulan PSL v2** 许可证下获得许可。

所有源文件必须包含以下许可证头：

```cpp
// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.
```

### 我们欢迎的贡献

- 错误修复
- 新功能（请先在 issue 中讨论）
- 性能改进
- 文档改进
- 测试添加
- 代码重构（保持功能）

### 我们通常不接受

- 未经讨论的重大更改
- 不符合项目目标的功能
- 未经事先批准的大型重构
- 降低代码覆盖率的更改
- 对专有软件的依赖

### 设计理念

Axon 遵循以下设计原则：
1. **以任务为中心**：一个任务 = 一个 MCAP 文件
2. **无锁设计**：SPSC 队列实现零拷贝消息处理
3. **车队就绪**：通过 HTTP RPC API 服务器控制录制
4. **崩溃恢复**：状态持久化和恢复的 S3 上传器
5. **插件架构**：中间件无关核心，支持 ROS1/ROS2 插件

请确保您的贡献与这些原则保持一致。

---

## 开发工作流

### 分支策略

- **`main`** - 受保护分支，所有 PR 必须以此为目标
- **`develop`** - 功能集成分支（如适用）
- **功能分支** - 命名如 `feature/your-feature` 或 `fix/your-bug-fix`
- **发布分支** - 命名如 `release/vX.Y.Z`

### 创建功能分支

```bash
git checkout main
git fetch upstream
git rebase upstream/main
git checkout -b feature/your-feature-name
```

### 提交消息格式

遵循 [约定式提交](https://www.conventionalcommits.org/zh-hans/)：

```
<type>(<scope>): <description>

[可选 body]

[可选 footer]
```

**类型：**
- `feat`: 新功能
- `fix`: 错误修复
- `docs`: 文档更改
- `style`: 代码风格更改（格式化等）
- `refactor`: 代码重构
- `perf`: 性能改进
- `test`: 添加或更新测试
- `chore`: 维护任务
- `ci`: CI/CD 更改

**范围：**
- `recorder`: 主录制器应用程序
- `mcap`: MCAP 写入库
- `uploader`: S3 上传库
- `logging`: 日志库
- `ros1`: ROS1 中间件
- `ros2`: ROS2 中间件
- `http`: HTTP RPC API
- `test`: 测试代码
- `docs`: 文档
- `build`: 构建系统

**示例：**
```
feat(recorder): 添加暂停/恢复功能

实现活动录制会话的暂停和恢复操作。
在暂停期间维护 SPSC 队列状态，并在恢复时
自动恢复消息消费。

Closes #123
```

```
fix(uploader): 解决重试处理器中的内存泄漏

重试处理器未正确清理失败的上传
尝试，导致内存随时间积累。此修复
确保所有错误路径中的资源正确清理。

Fixes #145
```

### 进行更改

1. 遵循我们的[代码规范](#代码规范)编写代码
2. 为新功能添加测试
3. 在本地运行测试
4. 格式化代码
5. 使用约定式提交消息提交
6. 推送到您的 fork
7. 创建拉取请求

---

## 代码规范

### 预提交钩子

Axon 使用预提交钩子来强制执行代码质量标准：

```bash
# 配置钩子
git config core.hooksPath githooks

# 预提交钩子将：
# - 检查暂存文件的 clang-format 合规性
# - 如果需要格式化则使提交失败
```

**如果需要格式化：**
```bash
# 格式化暂存文件
git clang-format

# 或格式化所有文件
make format

# 或格式化单个文件
clang-format -i path/to/file.cpp
```

**跳过钩子（不推荐）：**
```bash
git commit --no-verify -m "message"
```

### 格式化指南

Axon 使用基于 Google 风格的 [clang-format](.clang-format)：

| 设置 | 值 |
|---------|-------|
| BasedOnStyle | Google |
| ColumnLimit | 100 |
| PointerAlignment | Left (`int* ptr`) |
| TabWidth | 2 |
| UseTab | Never |

### 头文件包含顺序

按以下优先级排序包含文件：

1. **Axon 包含** - `#include <axon/...>`
2. **第三方包含** - `#include <boost/...>`
3. **标准库** - `#include <memory>`
4. **本地包含** - `#include "..."`

示例：
```cpp
// Axon 包含
#include <axon/axon_log_init.hpp>

// 第三方包含
#include <boost/asio.hpp>
#include <mcap/writer.hpp>

// 标准库
#include <memory>
#include <string>

// 本地包含
#include "my_header.hpp"
```

### C++ 最佳实践

**通用：**
- 使用 RAII 进行资源管理
- 优先使用 `std::unique_ptr` 和 `std::shared_ptr` 而非裸指针
- 尽可能使用 `constexpr` 和 `const`
- 为重写的虚函数使用 `override` 关键字
- 在适当的地方标记函数 `noexcept`
- 大对象通过 `const` 引用传递
- 使用 `enum class` 而非普通 `enum`

**线程安全：**
- 记录线程安全保证
- 将 `std::mutex` 与 `std::lock_guard` 或 `std::unique_lock` 一起使用
- 对于 SPSC 队列，使用无锁实现
- 通过建立锁定顺序避免死锁
- 尽可能优先使用原子操作而非锁

**内存管理：**
- 优先使用栈分配而非堆
- 使用智能指针管理所有权
- 注意移动语义 - 在适当的地方使用 `std::move`
- 避免使用 `shared_ptr` 的循环引用
- 使用 `std::make_unique` 和 `std::make_shared`

**性能：**
- 避免过早优化
- 优化前先分析
- 使用适当的数据结构
- 最小化热路径中的分配
- 考虑缓存局部性

### 文档

- 所有公共 API 必须有文档注释
- 使用 Doxygen 风格注释（`///` 或 `/** */`）
- 记录线程安全保证
- 为复杂 API 包含使用示例
- 保持注释与代码更改同步

示例：
```cpp
/// @brief 线程安全的 MCAP 写入器包装器
///
/// 此类为向 MCAP 文件写入消息提供线程安全接口。
/// 它为每个主题使用无锁 SPSC 队列以实现
/// 零拷贝消息处理。
///
/// @threadsafe{所有方法都是线程安全的}
/// @note 写入器不得在操作进行期间被销毁
class McapWriterWrapper {
public:
    /// @brief 写入消息到 MCAP 文件
    /// @param topic 主题名称
    /// @param message 要写入的消息数据
    /// @param timestamp 纳秒级消息时间戳
    /// @return 写入成功返回 true，否则返回 false
    bool write(const std::string& topic,
               const std::vector<uint8_t>& message,
               uint64_t timestamp);
};
```

---

## 测试

### 测试理念

- **测试驱动开发**：在代码之前或同时编写测试
- **覆盖率**：保持 >80% 代码覆盖率
- **隔离性**：测试应该独立且可以任何顺序运行
- **速度**：单元测试应该快速运行（每个 < 0.1s）
- **清晰性**：测试应该作为文档

### 本地运行测试

**快速测试（仅 C++ 核心库）：**
```bash
make test
```

**使用 Docker 运行完整测试套件（无需本地 ROS）：**
```bash
make docker-test-all
```

**特定测试类别：**
```bash
make test-core                    # C++ 核心库测试
make test-mcap                    # MCAP 写入器测试
make test-uploader                # 上传器测试
make docker-test-ros1             # ROS1 测试
make docker-test-ros2-humble      # ROS2 Humble 测试
make docker-test-ros2-jazzy       # ROS2 Jazzy 测试
make docker-test-ros2-rolling     # ROS2 Rolling 测试
```

**运行特定测试：**
```bash
cd build
ctest -R test_mcap_writer -V
ctest -R test_state_machine --output-on-failure
```

### 测试结构

```
axon_<module>/
├── src/
│   └── ...
├── include/axon/
│   └── ...
└── test/
    ├── unit/                     # 单元测试
    │   ├── test_*.cpp
    │   └── test_*.hpp
    ├── integration/              # 集成测试
    │   └── test_*.cpp
    └── mocks/                   # 测试替身
        └── *_mock*.hpp
```

### 编写测试

**单元测试：**
```cpp
#include <gtest/gtest.h>
#include <axon/my_class.hpp>

namespace axon::testing {

TEST(MyClassTest, ConstructionCreatesValidObject) {
    MyClass obj(42);
    EXPECT_EQ(obj.value(), 42);
}

TEST(MyClassTest, SetValueUpdatesValue) {
    MyClass obj(0);
    obj.set_value(100);
    EXPECT_EQ(obj.value(), 100);
}

TEST(MyClassDeathTest, NullPointerThrows) {
    MyClass obj(nullptr);
    EXPECT_THROW(obj.use(), std::runtime_error);
}

}  // namespace axon::testing
```

**测试命名：**
- 使用 `TEST(TestSuite, TestName)` 格式
- TestSuite: 被测试的类/模块名称
- TestName: 正在测试的内容（使用 `Should_期望行为_When_状态UnderTest` 格式）

### 覆盖率报告

```bash
make coverage-html
open ../coverage/html/index.html
```

**覆盖率目标：**
- 核心库：>80%
- 关键路径（状态机、录制）：>90%
- 新代码：合并前必须有测试

### 测试要求

**对于 PR：**
- 所有测试必须通过
- 新功能必须包含测试
- 覆盖率不应降低
- 适用情况下测试必须线程安全
- 为外部依赖使用模拟（S3、HTTP 等）

---

## 构建

### 快速构建

```bash
make build
```

### 构建模式

```bash
make release  # 优化构建（默认）
make debug    # 带符号的调试构建 (-g -O0)
```

### 清理构建

```bash
make clean
make build
```

### 构建单个组件

```bash
make build-core                   # C++ 核心库
make build-ros1                  # ROS1 中间件
make build-ros2                  # ROS2 中间件
make app                         # 主应用程序
```

### Docker 构建

```bash
make docker-build
```

---

## 提交更改

### 拉取请求流程

1. **更新您的分支：**
   ```bash
   git fetch upstream
   git rebase upstream/main
   ```

2. **确保所有检查通过：**
   - CI 测试通过
   - 代码已格式化（预提交钩子）
   - 覆盖率得到维护或改进
   - 文档已更新

3. **创建拉取请求：**
   - 使用描述性标题（约定式提交格式）
   - 填写 PR 模板
   - 链接相关议题
   - 如适用，添加 `closes #123` 或 `fixes #123`

4. **PR 描述模板：**
   ```markdown
   ## 摘要
   更改的简要描述

   ## 动机
   为什么需要此更改

   ## 更改
   包含文件链接的详细技术描述

   ## 测试
   - [ ] 单元测试通过
   - [ ] 集成测试通过
   - [ ] 手动测试完成

   ## 检查清单
   - [ ] 代码遵循风格指南
   - [ ] 自我审查完成
   - [ ] 为复杂代码添加注释
   - [ ] 文档已更新
   - [ ] 未生成新警告
   - [ ] 测试已添加/更新
   - [ ] 所有测试通过
   ```

5. **请求审查：**
   - 标记相关维护者
   - 及时处理审查反馈
   - 保持 PR 专注（一个 PR 一个功能）

6. **批准后：**
   - 如需要则压缩提交
   - 确保 CI 通过
   - 等待维护者合并

### 审查流程

- **自动检查**：CI 运行测试、检查器和覆盖率
- **同行审查**：至少一名维护者必须批准
- **审查时间表**：期望在 3-5 个工作日内完成审查
- **迭代**：处理反馈并更新您的 PR

### 合并标准

PR 在以下情况下合并：
- 所有 CI 检查通过
- 至少一名维护者批准
- 无未解决的审查评论
- 覆盖率得到维护
- 文档完整

---

## 项目结构

### 目录布局

```
Axon/
├── .clang-format              # 代码格式化规则
├── .github/                   # GitHub Actions CI 工作流
│   └── workflows/
├── cmake/                     # CMake 模块
│   ├── ClangFormat.cmake
│   └── FindMcap.cmake
├── core/                      # C++ 共享库
│   ├── axon_logging/          # 日志基础设施
│   ├── axon_mcap/             # MCAP 写入器包装器
│   └── axon_uploader/         # S3 边缘上传器
├── docs/                      # 文档
│   └── designs/               # 设计文档
├── docker/                    # Docker 文件
├── githooks/                  # 共享 git 钩子
├── middlewares/               # ROS 中间件插件
│   ├── ros1/                  # ROS1 插件
│   └── ros2/                  # ROS2 插件
├── apps/                      # 主应用程序
│   ├── axon_recorder/         # HTTP RPC 录制器
│   └── plugin_example/        # 插件示例
├── tools/                     # 工具和实用程序
│   └── axon_panel/            # Web 控制面板
└── scripts/                   # 实用脚本
```

### 核心库

| 库 | 目的 | 关键类 |
|---------|---------|-------------|
| `axon_logging` | 结构化日志 | `axon_log_init`、`axon_console_sink`、`axon_file_sink` |
| `axon_mcap` | MCAP 操作 | `McapWriterWrapper`、`McapValidator` |
| `axon_uploader` | S3 上传 | `EdgeUploader`、`S3Client`、`UploadQueue` |

### 中间件插件

| 插件 | ROS 版本 | 目的 |
|--------|-------------|---------|
| `ros1_plugin` | ROS 1 Noetic | ROS1 集成 |
| `ros2_plugin` | ROS 2 Humble/Jazzy/Rolling | ROS2 集成 |

---

## 报告问题

### 报告前

1. **搜索现有议题** - 检查问题是否已被报告
2. **检查文档** - 查看 [README.md](README.md) 和设计文档
3. **尝试最新版本** - 您的问题可能已修复

### 错误报告

使用错误报告模板并包括：

**必需信息：**
- Axon 版本
- ROS 发行版（Noetic/Humble/Jazzy/Rolling）
- 操作系统和版本
- 重现步骤
- 期望与实际行为
- 相关日志
- 最小可重现示例

**示例：**
```markdown
## 描述
录制器在暂停/恢复循环后完成录制时崩溃

## 环境
- Axon 版本: v0.2.0
- ROS: Humble
- 操作系统: Ubuntu 22.04

## 重现步骤
1. 开始录制: `POST /rpc/config` 然后执行 `POST /rpc/begin`
2. 暂停录制: `POST /rpc/pause`
3. 恢复录制: `POST /rpc/resume`
4. 完成录制: `POST /rpc/finish`
5. 发生崩溃

## 期望行为
录制干净完成，MCAP 文件被最终确定

## 实际行为
`worker_thread_pool.cpp:142` 中发生段错误

## 日志
[包含相关日志输出]

## 其他上下文
没有暂停/恢复循环时工作正常
```

### 功能请求

使用功能请求模板并包括：

- **问题陈述**：这解决了什么问题？
- **建议方案**：应该如何工作？
- **考虑的替代方案**：您还考虑了哪些其他方法？
- **其他上下文**：用例、示例、模型

### 安全问题

**请勿公开报告安全问题。**

通过电子邮件将安全问题发送至：security@archebase.com

---

## 社区准则

### 沟通渠道

- **GitHub 议题**：错误报告和功能请求
- **GitHub 讨论**：问题和一般讨论
- **拉取请求**：代码贡献
- **电子邮件**：contact@archebase.com 用于行政事务

### 获取帮助

1. **先搜索**：检查现有议题和讨论
2. **具体描述**：包含代码、错误消息和环境详细信息
3. **耐心等待**：维护者自愿贡献时间
4. **尊重他人**：尊重地对待每个人

### 认可

贡献者将：
- 列入 [CONTRIBUTORS.md](CONTRIBUTORS.md)
- 在发行说明中提及重大贡献
- 在持续贡献后有资格成为维护者

---

## 设计理念

### 核心原则

1. **以任务为中心**：一个任务 = 一个具有完整生命周期的 MCAP 文件
2. **无锁设计**：SPSC 队列实现零拷贝消息处理
3. **车队就绪**：通过 HTTP RPC API 服务器控制录制
4. **崩溃恢复**：具有状态持久化和恢复的 S3 上传器
5. **插件架构**：中间件无关核心，支持 ROS1/ROS2 插件

### 架构决策

- **HTTP 而非 ROS 服务**：减少耦合，启用 Web 接口
- **SPSC 队列**：无锁、缓存友好的消息传递
- **状态机**：显式状态转换防止无效操作
- **MCAP 格式**：机器人数据的现代灵活容器

---

## 其他资源

### 文档

- [README.md](README.md) - 项目概述和使用
- [ARCHITECTURE.md](ARCHITECTURE.md) - 系统架构
- [docs/designs/rpc-api-design.md](docs/designs/rpc-api-design.md) - HTTP RPC API 规范
- [docs/designs/frontend-design.md](docs/designs/frontend-design.md) - Web 面板架构

### 外部参考

- [Mulan PSL v2 许可证](LICENSE)
- [ROS 1 文档](http://wiki.ros.org/noetic)
- [ROS 2 文档](https://docs.ros.org/)
- [MCAP 格式](https://mcap.dev/)
- [约定式提交](https://www.conventionalcommits.org/zh-hans/)

---

## 许可证

通过向 Axon 贡献，您同意您的贡献将在 **Mulan PSL v2** 许可证下获得许可。详见 [LICENSE](LICENSE)。

---

## 有问题？

- 检查现有的 [GitHub 议题](https://github.com/ArcheBase/Axon/issues)
- 开始 [GitHub 讨论](https://github.com/ArcheBase/Axon/discussions)
- 查看 [ARCHITECTURE.md](ARCHITECTURE.md) 了解系统设计
- 查看 [README.md](README.md) 了解使用信息

---

感谢您对 Axon 的贡献！您的贡献让 Axon 对每个人都更好。🚀
