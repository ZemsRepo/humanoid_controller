#!/bin/bash
# 便捷脚本：自动修复所有C++文件的作者信息

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

# 解析参数
DRY_RUN=false
PATHS=()

while [[ $# -gt 0 ]]; do
    case $1 in
        --dry-run|-n)
            DRY_RUN=true
            shift
            ;;
        *)
            PATHS+=("$1")
            shift
            ;;
    esac
done

# 如果没有指定路径，使用默认路径
if [ ${#PATHS[@]} -eq 0 ]; then
    PATHS=(src include)
fi

# 构建命令
CMD_ARGS=("--fix")
if [ "$DRY_RUN" = true ]; then
    CMD_ARGS+=("--dry-run")
fi

# 如果配置文件存在，使用配置文件
if [ -f "$CONFIG_FILE" ]; then
    python3 "$AUTHOR_LINTER" "${CMD_ARGS[@]}" --config "$CONFIG_FILE" "${PATHS[@]}"
else
    python3 "$AUTHOR_LINTER" "${CMD_ARGS[@]}" --author "$AUTHOR" --org "$ORG" "${PATHS[@]}"
fi

