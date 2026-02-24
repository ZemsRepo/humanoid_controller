# Author Linter 工具使用说明

这是一个用于自动检查和更新C++文件作者信息的工具。

## 功能特性

- ✅ 自动检查C++头文件和源文件是否包含作者信息
- ✅ 自动添加缺失的作者信息
- ✅ 自动更新现有的作者信息
- ✅ 支持递归处理目录
- ✅ 支持试运行模式（dry-run）
- ✅ 可集成到pre-commit hooks

## 安装

确保Python 3.6+已安装，工具无需额外依赖。

```bash
chmod +x utils/author_linter.py
```

## 使用方法

### 1. 检查文件是否有作者信息

```bash
# 检查单个文件
python3 utils/author_linter.py --check --author "Fulong Yin" --org "IO-AI.tech" src/tasks/holosoma_locomotion/Command.cpp

# 检查整个目录
python3 utils/author_linter.py --check --author "Fulong Yin" --org "IO-AI.tech" src/

# 检查多个路径
python3 utils/author_linter.py --check --author "Fulong Yin" --org "IO-AI.tech" src/ include/
```

### 2. 自动修复（添加/更新作者信息）

```bash
# 修复单个文件
python3 utils/author_linter.py --fix --author "Fulong Yin" --org "IO-AI.tech" src/tasks/holosoma_locomotion/Command.cpp

# 修复整个目录
python3 utils/author_linter.py --fix --author "Fulong Yin" --org "IO-AI.tech" src/

# 试运行模式（不实际修改文件，只显示将要做的更改）
python3 utils/author_linter.py --fix --dry-run --author "Fulong Yin" --org "IO-AI.tech" src/
```

### 3. 命令行选项

```
--author NAME          作者名称（必需）
--org, --organization NAME  组织名称（必需）
--check                检查模式（默认）
--fix                  自动修复模式
--dry-run              试运行模式（不实际修改文件）
--recursive, -r        递归处理子目录（默认启用）
--no-recursive         不递归处理子目录
```

## 作者信息格式

工具会在文件开头添加以下格式的作者信息：

```cpp
//
// Author: Fulong Yin
// Organization: IO-AI.tech
//
```

作者信息会被插入在 `#pragma once` 之前，或者在第一个非注释代码行之前。

## 使用便捷脚本

项目提供了两个便捷的bash脚本：

### check_authors.sh - 检查作者信息

```bash
# 使用默认配置
./utils/check_authors.sh

# 检查特定目录
./utils/check_authors.sh src/ include/

# 使用环境变量覆盖作者信息
AUTHOR="Your Name" ORG="Your Org" ./utils/check_authors.sh
```

### fix_authors.sh - 修复作者信息

```bash
# 自动修复所有文件
./utils/fix_authors.sh

# 修复特定目录
./utils/fix_authors.sh src/ include/

# 试运行模式
./utils/fix_authors.sh --dry-run
```

## 使用 Makefile

项目提供了Makefile集成：

```bash
# 检查所有文件
make -f utils/Makefile.author check

# 自动修复
make -f utils/Makefile.author fix

# 试运行
make -f utils/Makefile.author fix-dry-run

# 自定义作者信息
make -f utils/Makefile.author check AUTHOR="Your Name" ORG="Your Org"
```

## 使用 CMake 目标

如果使用CMake构建系统，可以使用以下目标：

```bash
# 检查作者信息
cmake --build build --target check_authors

# 修复作者信息
cmake --build build --target fix_authors

# 自定义作者信息（在配置CMake时）
cmake -DAUTHOR_LINTER_AUTHOR="Your Name" -DAUTHOR_LINTER_ORG="Your Org" ..
```

## 集成到 Pre-commit Hooks

### 方法1: 使用pre-commit框架

1. 安装pre-commit（如果还没有）：
```bash
pip install pre-commit
```

2. 在项目根目录创建或编辑 `.pre-commit-config.yaml`，添加：

```yaml
repos:
  - repo: local
    hooks:
      - id: author-linter
        name: Author Linter
        entry: bash -c 'python3 src/humanoid_controller/utils/author_linter.py --check --author "Fulong Yin" --org "IO-AI.tech" --recursive src/'
        language: system
        pass_filenames: false
        always_run: true
        stages: [commit]
```

3. 安装hook：
```bash
pre-commit install
```

### 方法2: 使用Git hooks

创建 `.git/hooks/pre-commit`：

```bash
#!/bin/bash
python3 src/humanoid_controller/utils/author_linter.py --check --author "Fulong Yin" --org "IO-AI.tech" --recursive src/ include/
if [ $? -ne 0 ]; then
    echo "错误: 某些文件缺少作者信息"
    echo "运行以下命令自动修复:"
    echo "python3 src/humanoid_controller/utils/author_linter.py --fix --author \"Fulong Yin\" --org \"IO-AI.tech\" --recursive src/ include/"
    exit 1
fi
```

## 示例

### 检查所有源文件

```bash
python3 utils/author_linter.py --check --author "Fulong Yin" --org "IO-AI.tech" src/ include/
```

输出示例：
```
✓ Command.cpp
✓ Controller.cpp
✗ 缺少作者信息: Observation.cpp
✗ 缺少组织信息: Policy.cpp

检查完成: 2/4 个文件通过检查
警告: 2 个文件缺少作者信息
```

### 自动修复所有文件

```bash
python3 utils/author_linter.py --fix --author "Fulong Yin" --org "IO-AI.tech" src/ include/
```

输出示例：
```
无需更新: Command.cpp
已更新: Observation.cpp
已更新: Policy.cpp

处理完成: 3/3 个文件
```

## 注意事项

1. 工具会识别多种作者信息格式，包括：
   - `// Author: ...`
   - `// Created by ...`
   - `/* Author: ... */`
   - `/* Created by ... */`

2. 工具会自动更新现有的作者信息，确保格式统一。

3. 建议在提交代码前运行检查，或集成到CI/CD流程中。

4. 如果文件已经有作者信息但格式不同，工具会将其更新为标准格式。

## 故障排除

### 编码错误

如果遇到编码错误，确保文件使用UTF-8编码。工具默认使用UTF-8读取文件。

### 权限问题

确保脚本有执行权限：
```bash
chmod +x utils/author_linter.py
```

### 批量更新后检查

更新大量文件后，建议使用git diff检查更改：
```bash
git diff src/ include/
```

