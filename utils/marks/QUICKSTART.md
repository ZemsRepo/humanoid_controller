# Author Linter 快速参考

## 🚀 快速开始

### 最简单的方式（使用配置文件）

1. 编辑 `utils/marks/author_config.yaml` 设置作者信息
2. 运行检查：
   ```bash
   python3 utils/marks/author_linter.py --check --config utils/marks/author_config.yaml src/ include/
   ```
3. 自动修复：
   ```bash
   python3 utils/marks/author_linter.py --fix --config utils/marks/author_config.yaml src/ include/
   ```

### 使用便捷脚本

```bash
# 检查
./utils/marks/check_authors.sh

# 修复
./utils/marks/fix_authors.sh

# 试运行
./utils/marks/fix_authors.sh --dry-run
```

### 使用 Makefile

```bash
make -f utils/marks/Makefile.author check
make -f utils/marks/Makefile.author fix
```

### 使用 CMake

```bash
cmake --build build --target check_authors
cmake --build build --target fix_authors
```

## 📋 常用命令

| 操作 | 命令 |
|------|------|
| 检查单个文件 | `python3 utils/marks/author_linter.py --check --author "Name" --org "Org" file.cpp` |
| 检查目录 | `python3 utils/marks/author_linter.py --check --author "Name" --org "Org" src/` |
| 自动修复 | `python3 utils/marks/author_linter.py --fix --author "Name" --org "Org" src/` |
| 试运行 | `python3 utils/marks/author_linter.py --fix --dry-run --author "Name" --org "Org" src/` |

## ⚙️ 配置文件示例

`utils/marks/author_config.yaml`:
```yaml
author: "Fulong Yin"
organization: "IO-AI.tech"
```

## 📝 作者信息格式

工具会自动添加以下格式：

```cpp
//
// Author: Fulong Yin
// Organization: IO-AI.tech
//
```

## 🔗 更多信息

详细文档请参考 `utils/marks/README_author_linter.md`

