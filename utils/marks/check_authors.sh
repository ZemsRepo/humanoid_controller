#!/bin/bash
# 便捷脚本：检查所有C++文件的作者信息

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTHOR_LINTER="${SCRIPT_DIR}/author_linter.py"
CONFIG_FILE="${SCRIPT_DIR}/author_config.yaml"

# 默认作者信息
AUTHOR="${AUTHOR:-Fulong Yin}"
ORG="${ORG:-IO-AI.tech}"

# 检查脚本是否存在
if [ ! -f "$AUTHOR_LINTER" ]; then
    echo "错误: 找不到 author_linter.py" >&2
    exit 1
fi

# 如果配置文件存在，使用配置文件
if [ -f "$CONFIG_FILE" ]; then
    python3 "$AUTHOR_LINTER" --check --config "$CONFIG_FILE" "$@"
else
    python3 "$AUTHOR_LINTER" --check --author "$AUTHOR" --org "$ORG" "$@"
fi

