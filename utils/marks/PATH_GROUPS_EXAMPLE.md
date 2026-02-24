# 路径分组配置示例

## 概述

Author Linter 支持根据文件路径为不同的任务（tasks）配置不同的作者信息。这对于多人协作的项目非常有用。

## 配置文件格式

在 `author_config.yaml` 中，你可以使用 `path_groups` 来配置不同路径的作者信息：

```yaml
# 默认作者信息（用于没有匹配到特定路径的文件）
author: "Fulong Yin"
organization: "IO-AI.tech"

# 按路径分组的作者配置
path_groups:
  # holosoma_locomotion 任务
  - path_pattern: "**/tasks/holosoma_locomotion/**"
    author: "Fulong Yin"
    organization: "IO-AI.tech"
  
  # motion_tracking 任务
  - path_pattern: "**/tasks/motion_tracking/**"
    author: "Fulong Yin"
    organization: "IO-AI.tech"
  
  # unitree_locomotion 任务
  - path_pattern: "**/tasks/unitree_locomotion/**"
    author: "Fulong Yin"
    organization: "IO-AI.tech"
```

## 路径模式说明

路径模式支持以下通配符：

- `**` - 匹配任意层级的目录（包括零个或多个目录）
- `*` - 匹配单个目录层级中的任意字符（不包括路径分隔符）
- `?` - 匹配单个字符

### 示例

```yaml
path_groups:
  # 匹配所有 holosoma_locomotion 目录下的文件
  - path_pattern: "**/tasks/holosoma_locomotion/**"
    author: "Author A"
    organization: "Org A"
  
  # 匹配所有 utils 目录下的文件
  - path_pattern: "**/utils/**"
    author: "Author B"
    organization: "Org B"
  
  # 匹配 src 目录下所有文件
  - path_pattern: "src/**"
    author: "Author C"
    organization: "Org C"
  
  # 匹配特定文件
  - path_pattern: "**/common.h"
    author: "Author D"
    organization: "Org D"
```

## 匹配顺序

路径模式按配置顺序进行匹配，**第一个匹配的模式会被使用**。因此，应该将更具体的模式放在前面，更通用的模式放在后面。

### 正确示例

```yaml
path_groups:
  # 更具体的模式在前
  - path_pattern: "**/tasks/holosoma_locomotion/Command.h"
    author: "Specific Author"
    organization: "Specific Org"
  
  # 更通用的模式在后
  - path_pattern: "**/tasks/holosoma_locomotion/**"
    author: "General Author"
    organization: "General Org"
```

## 使用示例

### 1. 检查特定任务的文件

```bash
python3 utils/marks/author_linter.py --check --config utils/marks/author_config.yaml \
  include/humanoid_controller/tasks/holosoma_locomotion/
```

### 2. 修复所有任务的文件

```bash
python3 utils/marks/author_linter.py --fix --config utils/marks/author_config.yaml \
  include/humanoid_controller/tasks/
```

### 3. 检查所有文件（会自动应用不同的作者配置）

```bash
python3 utils/marks/author_linter.py --check --config utils/marks/author_config.yaml \
  include/ src/
```

## 实际应用场景

### 场景1：不同任务由不同开发者负责

```yaml
path_groups:
  - path_pattern: "**/tasks/holosoma_locomotion/**"
    author: "Developer A"
    organization: "Team Alpha"
  
  - path_pattern: "**/tasks/motion_tracking/**"
    author: "Developer B"
    organization: "Team Beta"
  
  - path_pattern: "**/tasks/unitree_locomotion/**"
    author: "Developer C"
    organization: "Team Gamma"
```

### 场景2：第三方代码和自有代码区分

```yaml
path_groups:
  # 第三方代码
  - path_pattern: "**/third_party/**"
    author: "Third Party"
    organization: "External"
  
  # 自有代码使用默认作者
  # （不需要配置，会使用默认的 author 和 organization）
```

### 场景3：不同模块使用不同组织

```yaml
path_groups:
  - path_pattern: "**/tasks/holosoma_locomotion/**"
    author: "Fulong Yin"
    organization: "IO-AI.tech"
  
  - path_pattern: "**/tasks/motion_tracking/**"
    author: "Fulong Yin"
    organization: "IO-AI.tech"
  
  - path_pattern: "**/utils/**"
    author: "Fulong Yin"
    organization: "IO-AI.tech"
```

## 验证配置

你可以使用以下命令验证路径匹配是否正确：

```bash
# 检查特定文件会使用哪个作者
python3 utils/marks/author_linter.py --check --config utils/marks/author_config.yaml \
  include/humanoid_controller/tasks/holosoma_locomotion/Command.h
```

输出会显示文件使用的作者信息。

## 注意事项

1. **路径匹配是大小写敏感的**：确保路径模式与实际路径大小写一致
2. **使用正斜杠**：路径模式中应使用 `/` 而不是 `\`
3. **相对路径**：路径模式是相对于项目根目录的
4. **顺序很重要**：更具体的模式应该放在更通用的模式之前

