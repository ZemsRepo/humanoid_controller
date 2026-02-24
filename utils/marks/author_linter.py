#!/usr/bin/env python3
"""
Author Linter Tool
自动检查和添加/更新C++头文件和源文件的作者信息
"""

import argparse
import fnmatch
import re
import sys
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False


class AuthorLinter:
    """作者信息检查器和更新器"""

    # 支持的C++文件扩展名
    CPP_EXTENSIONS = {'.h', '.hpp', '.cpp', '.cc', '.cxx', '.c'}

    # 作者信息注释模式
    AUTHOR_PATTERNS = [
        r'//\s*Author:\s*(.+)',
        r'//\s*Created by (.+)',
        r'/\*\s*Author:\s*(.+?)\s*\*/',
        r'/\*\s*Created by (.+?)\s*\*/',
    ]

    # 组织信息注释模式
    ORGANIZATION_PATTERNS = [
        r'//\s*Organization:\s*(.+)',
        r'//\s*Company:\s*(.+)',
        r'/\*\s*Organization:\s*(.+?)\s*\*/',
        r'/\*\s*Company:\s*(.+?)\s*\*/',
    ]

    def __init__(self, author: str, organization: str, path_groups: Optional[List[Dict]] = None, base_path: Optional[Path] = None, source_url: Optional[str] = None):
        """
        初始化Author Linter

        Args:
            author: 默认作者名称
            organization: 默认组织名称
            path_groups: 路径分组配置列表，每个元素包含 path_pattern, author, organization, source_url
            base_path: 基础路径，用于将相对路径转换为绝对路径进行匹配
            source_url: 默认原始链接（可选）
        """
        self.default_author = author
        self.default_organization = organization
        self.default_source_url = source_url or ""
        self.path_groups = path_groups or []
        self.base_path = base_path or Path.cwd()
        
    def _match_path_pattern(self, path_str: str, pattern: str) -> bool:
        """
        匹配路径模式，支持 ** 通配符
        
        Args:
            path_str: 要匹配的路径字符串
            pattern: 模式字符串，支持 ** 和 * 通配符
            
        Returns:
            是否匹配
        """
        # 将路径统一为正斜杠
        path_str = path_str.replace('\\', '/')
        pattern = pattern.replace('\\', '/')
        
        # 处理 ** 模式：匹配任意层级的目录
        if '**' in pattern:
            # 先转义所有特殊字符
            regex_pattern = re.escape(pattern)
            # 然后将转义后的 \*\* 替换为 .* (匹配任意字符包括斜杠)
            regex_pattern = regex_pattern.replace(r'\*\*', '.*')
            # 处理单个转义的 \* (在非**的情况下) 替换为 [^/]* (匹配非斜杠字符)
            regex_pattern = regex_pattern.replace(r'\*', '[^/]*')
            # 处理转义的 \? 替换为 . (匹配单个字符)
            regex_pattern = regex_pattern.replace(r'\?', '.')
            regex_pattern = '^' + regex_pattern + '$'
            try:
                return bool(re.match(regex_pattern, path_str))
            except re.error:
                # 如果正则表达式有问题，回退到简单匹配
                return fnmatch.fnmatch(path_str, pattern)
        else:
            # 使用fnmatch进行简单匹配
            return fnmatch.fnmatch(path_str, pattern)
    
    def get_author_for_path(self, file_path: Path) -> Tuple[str, str, str]:
        """
        根据文件路径获取对应的作者、组织和链接信息
        
        Args:
            file_path: 文件路径
            
        Returns:
            (author, organization, source_url): 作者、组织和原始链接信息
        """
        # 将文件路径转换为相对于base_path的路径字符串
        try:
            if file_path.is_absolute():
                rel_path = file_path.relative_to(self.base_path)
            else:
                rel_path = file_path
            path_str = str(rel_path).replace('\\', '/')  # 统一使用正斜杠
        except ValueError:
            # 如果无法计算相对路径，使用绝对路径
            path_str = str(file_path).replace('\\', '/')
        
        # 按顺序检查每个路径模式
        for group in self.path_groups:
            pattern = group.get('path_pattern', '')
            if pattern and self._match_path_pattern(path_str, pattern):
                return (
                    group.get('author', self.default_author),
                    group.get('organization', self.default_organization),
                    group.get('source_url', self.default_source_url)
                )
        
        # 如果没有匹配到，返回默认值
        return self.default_author, self.default_organization, self.default_source_url
    
    def get_author_header(self, file_path: Path) -> str:
        """
        获取文件对应的作者信息头
        
        Args:
            file_path: 文件路径
            
        Returns:
            作者信息头字符串（不包含末尾换行）
        """
        author, organization, source_url = self.get_author_for_path(file_path)
        header_lines = [
            "//",
            f"// Author: {author}",
            f"// Organization: {organization}",
        ]
        if source_url:
            header_lines.append(f"// Source: {source_url}")
        header_lines.append("//")
        return "\n".join(header_lines)

    def is_cpp_file(self, file_path: Path) -> bool:
        """检查文件是否为C++文件"""
        return file_path.suffix in self.CPP_EXTENSIONS

    def has_author_info(self, content: str) -> Tuple[bool, Optional[int]]:
        """
        检查文件是否包含作者信息

        Returns:
            (has_author, line_number): 是否有作者信息，以及作者信息所在的行号
        """
        lines = content.split('\n')
        for i, line in enumerate(lines[:10]):  # 只检查前10行
            for pattern in self.AUTHOR_PATTERNS:
                if re.search(pattern, line, re.IGNORECASE):
                    return True, i
        return False, None

    def has_organization_info(self, content: str) -> bool:
        """检查文件是否包含组织信息"""
        for pattern in self.ORGANIZATION_PATTERNS:
            if re.search(pattern, content[:500], re.IGNORECASE):
                return True
        return False

    def get_existing_author_block(self, content: str) -> Optional[Tuple[int, int]]:
        """
        获取现有作者信息块的位置

        Returns:
            (start_line, end_line): 作者信息块的起始和结束行号，如果没有则返回None
        """
        lines = content.split('\n')
        start_line = None
        end_line = None

        for i, line in enumerate(lines[:15]):  # 检查前15行
            stripped = line.strip()
            # 检查是否是注释开始
            if stripped.startswith('//') or stripped.startswith('/*'):
                if start_line is None:
                    start_line = i
                end_line = i
            # 如果遇到非注释行，且已经有开始行，则结束
            elif start_line is not None and stripped and not stripped.startswith('*'):
                break

        if start_line is not None and end_line is not None:
            # 检查这个块是否包含作者信息
            block_content = '\n'.join(lines[start_line:end_line + 1])
            if self.has_author_info(block_content)[0]:
                return start_line, end_line

        return None

    def update_author_info(self, content: str, file_path: Path) -> str:
        """
        更新文件中的作者信息

        Args:
            content: 文件内容
            file_path: 文件路径（用于确定使用哪个作者配置）

        Returns:
            更新后的文件内容
        """
        lines = content.split('\n')
        has_author, author_line = self.has_author_info(content)
        has_org = self.has_organization_info(content)

        # 如果已有作者信息，更新它
        if has_author:
            author_block = self.get_existing_author_block(content)
            if author_block:
                start, end = author_block
                # 移除旧的作者信息块
                new_lines = lines[:start] + lines[end + 1:]
                # 在#pragma once之前插入新的作者信息
                return self.insert_author_header('\n'.join(new_lines), file_path)
            else:
                # 如果找不到完整的块，尝试替换单行
                if author_line is not None:
                    # 移除旧行
                    new_lines = lines[:author_line] + lines[author_line + 1:]
                    return self.insert_author_header('\n'.join(new_lines), file_path)

        # 如果没有作者信息，直接插入
        return self.insert_author_header(content, file_path)

    def insert_author_header(self, content: str, file_path: Path) -> str:
        """
        在文件开头插入作者信息头

        Args:
            content: 文件内容
            file_path: 文件路径（用于确定使用哪个作者配置）

        Returns:
            插入作者信息后的文件内容
        """
        lines = content.split('\n')

        # 找到#pragma once或第一个非空行的位置
        insert_pos = 0
        for i, line in enumerate(lines):
            stripped = line.strip()
            if stripped.startswith('#pragma once'):
                insert_pos = i
                break
            elif stripped and not stripped.startswith('//') and not stripped.startswith('/*'):
                insert_pos = i
                break

        # 跳过文件开头的空行
        while insert_pos < len(lines) and not lines[insert_pos].strip():
            insert_pos += 1

        # 获取对应的作者信息头
        author_header = self.get_author_header(file_path)
        header_lines = author_header.strip().split('\n')
        # 只保留一行空行
        new_lines = header_lines + [''] + lines[insert_pos:]
        return '\n'.join(new_lines)

    def check_file(self, file_path: Path) -> Tuple[bool, str]:
        """
        检查文件是否有正确的作者信息

        Returns:
            (is_valid, message): 是否有效，以及消息
        """
        if not file_path.exists():
            return False, f"文件不存在: {file_path}"

        if not self.is_cpp_file(file_path):
            return True, "跳过非C++文件"

        try:
            content = file_path.read_text(encoding='utf-8')
        except UnicodeDecodeError:
            return False, f"无法读取文件（编码错误）: {file_path}"

        # 获取该文件应该使用的作者信息
        expected_author, expected_org, _ = self.get_author_for_path(file_path)
        
        has_author, _ = self.has_author_info(content)
        has_org = self.has_organization_info(content)

        if not has_author:
            return False, f"缺少作者信息: {file_path} (期望: {expected_author})"
        elif not has_org:
            return False, f"缺少组织信息: {file_path} (期望: {expected_org})"
        else:
            # 检查作者信息是否匹配（可选，可以更严格）
            return True, f"✓ {file_path.name}"

    def fix_file(self, file_path: Path, dry_run: bool = False) -> Tuple[bool, str]:
        """
        修复文件的作者信息

        Args:
            file_path: 文件路径
            dry_run: 是否为试运行模式（不实际修改文件）

        Returns:
            (success, message): 是否成功，以及消息
        """
        if not file_path.exists():
            return False, f"文件不存在: {file_path}"

        if not self.is_cpp_file(file_path):
            return True, "跳过非C++文件"

        try:
            content = file_path.read_text(encoding='utf-8')
        except UnicodeDecodeError:
            return False, f"无法读取文件（编码错误）: {file_path}"

        updated_content = self.update_author_info(content, file_path)
        
        # 获取该文件应该使用的作者信息
        expected_author, expected_org, _ = self.get_author_for_path(file_path)

        if content == updated_content:
            return True, f"无需更新: {file_path.name} ({expected_author})"

        if not dry_run:
            file_path.write_text(updated_content, encoding='utf-8')
            return True, f"已更新: {file_path.name} ({expected_author})"
        else:
            return True, f"[试运行] 将更新: {file_path.name} ({expected_author})"

    def process_directory(self, directory: Path, recursive: bool = True, fix: bool = False,
                         dry_run: bool = False) -> List[Tuple[bool, str]]:
        """
        处理目录中的所有C++文件

        Args:
            directory: 目录路径
            recursive: 是否递归处理子目录
            fix: 是否自动修复
            dry_run: 是否为试运行模式

        Returns:
            处理结果列表
        """
        results = []
        pattern = '**/*' if recursive else '*'

        for file_path in directory.glob(pattern):
            if file_path.is_file() and self.is_cpp_file(file_path):
                if fix:
                    success, message = self.fix_file(file_path, dry_run)
                else:
                    success, message = self.check_file(file_path)
                results.append((success, message))

        return results


def load_config(config_path: Optional[Path] = None) -> Optional[dict]:
    """加载配置文件"""
    if not HAS_YAML:
        if config_path:
            print("警告: 未安装PyYAML，无法加载配置文件。使用 'pip install pyyaml' 安装。", file=sys.stderr)
        return None

    if config_path is None:
        # 尝试在当前目录和脚本目录查找配置文件
        script_dir = Path(__file__).parent
        possible_paths = [
            Path('author_config.yaml'),
            script_dir / 'author_config.yaml',
            Path('.author_config.yaml'),
        ]
        for path in possible_paths:
            if path.exists():
                config_path = path
                break

    if config_path and config_path.exists():
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"警告: 无法加载配置文件 {config_path}: {e}", file=sys.stderr)
    return None


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='C++文件作者信息检查器和更新器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 检查所有文件
  %(prog)s --check --author "Fulong Yin" --org "IO-AI.tech" src/

  # 自动修复所有文件
  %(prog)s --fix --author "Fulong Yin" --org "IO-AI.tech" src/

  # 使用配置文件
  %(prog)s --check --config utils/author_config.yaml src/

  # 试运行（不实际修改文件）
  %(prog)s --fix --dry-run --author "Fulong Yin" --org "IO-AI.tech" src/
        """
    )

    parser.add_argument('paths', nargs='*', type=Path, default=[],
                        help='要处理的文件或目录路径（默认：src/ include/）')
    parser.add_argument('--author',
                        help='作者名称（如果使用配置文件则不需要）')
    parser.add_argument('--org', '--organization', dest='organization',
                        help='组织名称（如果使用配置文件则不需要）')
    parser.add_argument('--config', type=Path,
                        help='配置文件路径（可选）')
    parser.add_argument('--check', action='store_true',
                        help='检查模式（默认）')
    parser.add_argument('--fix', action='store_true',
                        help='自动修复模式')
    parser.add_argument('--dry-run', action='store_true',
                        help='试运行模式（不实际修改文件）')
    parser.add_argument('--recursive', '-r', action='store_true', default=True,
                        help='递归处理子目录（默认启用）')
    parser.add_argument('--no-recursive', dest='recursive', action='store_false',
                        help='不递归处理子目录')

    args = parser.parse_args()

    # 加载配置
    config = load_config(args.config)
    author = args.author
    organization = args.organization
    path_groups = None
    base_path = Path.cwd()

    # 从配置文件获取默认值和路径分组
    source_url = None
    if config:
        if not author and 'author' in config:
            author = config['author']
        if not organization and 'organization' in config:
            organization = config['organization']
        if 'source_url' in config:
            source_url = config.get('source_url', '')
        if 'path_groups' in config:
            path_groups = config['path_groups']
        # 确定基础路径（用于路径匹配）
        if args.config:
            base_path = args.config.parent.resolve()
        else:
            # 尝试找到配置文件所在目录
            script_dir = Path(__file__).parent
            possible_configs = [
                Path('author_config.yaml'),
                script_dir / 'author_config.yaml',
                Path('.author_config.yaml'),
            ]
            for config_path in possible_configs:
                if config_path.exists():
                    base_path = config_path.parent.resolve()
                    break

    # 验证必需参数
    if not author:
        parser.error("必须指定 --author 或使用配置文件")
    if not organization:
        parser.error("必须指定 --org 或使用配置文件")

    # 默认路径
    if not args.paths:
        default_paths = [Path('src'), Path('include')]
        args.paths = [p for p in default_paths if p.exists()]

    if not args.paths:
        parser.error("没有找到要处理的文件或目录")

    linter = AuthorLinter(author, organization, path_groups, base_path, source_url)

    all_results = []
    for path in args.paths:
        if not path.exists():
            print(f"错误: 路径不存在: {path}", file=sys.stderr)
            continue

        if path.is_file():
            if args.fix:
                success, message = linter.fix_file(path, args.dry_run)
            else:
                success, message = linter.check_file(path)
            all_results.append((success, message))
        elif path.is_dir():
            results = linter.process_directory(path, args.recursive, args.fix, args.dry_run)
            all_results.extend(results)

    # 输出结果
    success_count = sum(1 for success, _ in all_results if success)
    total_count = len(all_results)

    for success, message in all_results:
        if success:
            print(message)
        else:
            print(f"✗ {message}", file=sys.stderr)

    # 统计信息
    failed_count = total_count - success_count
    if args.fix:
        print(f"\n处理完成: {success_count}/{total_count} 个文件")
    else:
        print(f"\n检查完成: {success_count}/{total_count} 个文件通过检查")
        if failed_count > 0:
            print(f"警告: {failed_count} 个文件缺少作者信息")

    # 返回退出码
    sys.exit(0 if failed_count == 0 else 1)


if __name__ == '__main__':
    main()

