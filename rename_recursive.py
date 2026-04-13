import os
import sys
import argparse
import tempfile


def safe_rename_case_change(old_path, new_path):
    """
    安全处理仅大小写不同的重命名（Windows 兼容）
    策略：old -> temp -> new
    """
    if os.path.exists(new_path) and old_path.lower() != new_path.lower():
        # 真正的不相关文件冲突
        return False, "目标文件已存在（不同文件）"

    # 生成临时文件名
    dir_name = os.path.dirname(old_path)
    base_name = os.path.basename(old_path)
    temp_name = f"__temp_rename_{base_name}_{os.urandom(4).hex()}"
    temp_path = os.path.join(dir_name, temp_name)

    try:
        # 步骤 1: 重命名为临时名
        os.rename(old_path, temp_path)
        # 步骤 2: 重命名为最终小写名
        os.rename(temp_path, new_path)
        return True, None
    except Exception as e:
        # 如果第二步失败，尝试恢复原状
        if os.path.exists(temp_path) and not os.path.exists(old_path):
            try:
                os.rename(temp_path, old_path)
            except:
                pass
        return False, str(e)


def rename_files_recursive(directory, dry_run=False, include_dirs=False):
    """
    递归将指定目录下所有文件的文件名改为小写形式（支持 Windows 大小写转换）
    """
    if not os.path.exists(directory):
        print(f"❌ 错误：路径 '{directory}' 不存在")
        return (0, 0, 0, 0)

    if not os.path.isdir(directory):
        print(f"❌ 错误：'{directory}' 不是有效的目录")
        return (0, 0, 0, 0)

    renamed_count = 0
    skipped_count = 0
    error_count = 0
    dir_count = 0

    abs_directory = os.path.abspath(directory)
    print(f"\n{'=' * 60}")
    print(f"📁 开始处理母目录: {abs_directory}")
    print(f"🔍 模式: {'试运行 (不实际执行)' if dry_run else '实际执行'}")
    print(f"📂 包含子目录重命名: {'是' if include_dirs else '否 (仅文件)'}")
    print(f"{'=' * 60}\n")

    for root, dirs, files in os.walk(abs_directory, topdown=True):
        dir_count += 1
        current_dir_display = os.path.relpath(root, abs_directory)
        if current_dir_display == '.':
            current_dir_display = '[根目录]'

        print(f"\n📂 正在处理目录: {current_dir_display}")
        print("-" * 50)

        # 处理文件
        for filename in files:
            old_path = os.path.join(root, filename)
            new_filename = filename.lower()

            if filename == new_filename:
                skipped_count += 1
                continue

            new_path = os.path.join(root, new_filename)

            # 检查是否只是大小写不同（Windows 特殊情况）
            is_case_only_change = (filename.lower() == new_filename.lower())

            # 检查目标是否存在（且不是同一个文件的大小写变体）
            if os.path.exists(new_path) and not is_case_only_change:
                print(f"  ⚠️  [跳过] '{filename}' -> 目标 '{new_filename}' 已存在（不同文件）")
                error_count += 1
                continue

            if dry_run:
                if is_case_only_change:
                    print(f"  📝 [预览-大小写转换] '{filename}' -> '{new_filename}'")
                else:
                    print(f"  📝 [预览] '{filename}' -> '{new_filename}'")
                renamed_count += 1
            else:
                if is_case_only_change:
                    # 仅大小写不同：使用临时文件策略
                    success, error = safe_rename_case_change(old_path, new_path)
                    if success:
                        print(f"  ✅ [成功-大小写转换] '{filename}' -> '{new_filename}'")
                        renamed_count += 1
                    else:
                        print(f"  ❌ [失败] '{filename}': {error}")
                        error_count += 1
                else:
                    # 普通重命名
                    try:
                        os.rename(old_path, new_path)
                        print(f"  ✅ [成功] '{filename}' -> '{new_filename}'")
                        renamed_count += 1
                    except Exception as e:
                        print(f"  ❌ [失败] '{filename}': {e}")
                        error_count += 1

        # 处理目录名（类似逻辑）
        if include_dirs:
            dirs_to_rename = []
            for dirname in dirs[:]:
                new_dirname = dirname.lower()
                if dirname != new_dirname:
                    dirs_to_rename.append((dirname, new_dirname))

            for dirname, new_dirname in dirs_to_rename:
                old_dir_path = os.path.join(root, dirname)
                new_dir_path = os.path.join(root, new_dirname)

                is_case_only_change = (dirname.lower() == new_dirname.lower())

                if os.path.exists(new_dir_path) and not is_case_only_change:
                    print(f"  ⚠️  [跳过目录] '{dirname}/' -> 目标已存在")
                    error_count += 1
                    continue

                if dry_run:
                    print(f"  📝 [预览目录] '{dirname}/' -> '{new_dirname}/'")
                    idx = dirs.index(dirname)
                    dirs[idx] = new_dirname
                    renamed_count += 1
                else:
                    if is_case_only_change:
                        success, error = safe_rename_case_change(old_dir_path, new_dir_path)
                        if success:
                            print(f"  ✅ [成功目录-大小写转换] '{dirname}/' -> '{new_dirname}/'")
                            idx = dirs.index(dirname)
                            dirs[idx] = new_dirname
                            renamed_count += 1
                        else:
                            print(f"  ❌ [失败目录] '{dirname}/': {error}")
                            error_count += 1
                    else:
                        try:
                            os.rename(old_dir_path, new_dir_path)
                            print(f"  ✅ [成功目录] '{dirname}/' -> '{new_dirname}/'")
                            idx = dirs.index(dirname)
                            dirs[idx] = new_dirname
                            renamed_count += 1
                        except Exception as e:
                            print(f"  ❌ [失败目录] '{dirname}/': {e}")
                            error_count += 1

    print(f"\n{'=' * 60}")
    print(f"📊 处理完成统计:")
    print(f"   扫描的目录数: {dir_count}")
    print(f"   成功重命名: {renamed_count}")
    print(f"   跳过 (已为小写): {skipped_count}")
    print(f"   错误/冲突: {error_count}")
    print(f"{'=' * 60}")

    return (renamed_count, skipped_count, error_count, dir_count)


def confirm_execution(directory, include_dirs):
    """请求用户确认"""
    print(f"\n⚠️  警告：即将修改 '{directory}' 下的所有文件名")
    if include_dirs:
        print("⚠️  注意：目录名也将被修改！这可能导致路径变化！")
    print("\n建议操作：")
    print("  1. 先使用 --dry-run 参数预览将要进行的更改")
    print("  2. 确保重要文件已备份")

    while True:
        choice = input("\n是否继续? [yes/no/dry-run]: ").strip().lower()
        if choice in ['yes', 'y']:
            return 'execute'
        elif choice in ['no', 'n']:
            return 'cancel'
        elif choice in ['dry-run', 'dry', 'd']:
            return 'dry-run'
        else:
            print("请输入 'yes', 'no' 或 'dry-run'")


def main():
    parser = argparse.ArgumentParser(
        description='递归将目录下所有文件名改为小写形式（支持 Windows 大小写转换）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  python rename_recursive.py /path/to/directory
  python rename_recursive.py "C:\\Users\\Name\\Documents" --dry-run
  python rename_recursive.py ./myfolder --include-dirs
        """
    )

    parser.add_argument('directory', nargs='?', help='目标母目录路径')
    parser.add_argument('--dry-run', '-d', action='store_true',
                        help='试运行模式：只显示将要执行的操作，不实际修改')
    parser.add_argument('--include-dirs', '-i', action='store_true',
                        help='同时重命名目录名（谨慎使用！）')
    parser.add_argument('--yes', '-y', action='store_true',
                        help='跳过确认提示，直接执行（危险！）')

    args = parser.parse_args()

    target_dir = args.directory
    if not target_dir:
        target_dir = input("请输入要处理的母目录路径: ").strip().strip('"\'')

    dry_run = args.dry_run
    if not dry_run and not args.yes and not args.include_dirs:
        decision = confirm_execution(target_dir, args.include_dirs)
        if decision == 'cancel':
            print("❎ 操作已取消")
            return 0
        elif decision == 'dry-run':
            dry_run = True

    rename_files_recursive(target_dir, dry_run=dry_run, include_dirs=args.include_dirs)

    if dry_run:
        print(f"\n💡 这是试运行模式，没有实际修改任何文件。")
        print(f"   如需正式执行，请去掉 --dry-run 参数或输入 'yes'")

    return 0


if __name__ == "__main__":
    sys.exit(main())