#!/usr/bin/env python3
"""
导航控制模块 - 统一测试入口

这是项目的主要测试工具，统一了所有测试命令。建议优先使用此脚本而非直接调用pytest。

快速开始：
    python run_tests.py --help          # 查看完整帮助
    python run_tests.py --quick         # 快速验证（推荐日常使用）
    python run_tests.py --all           # 完整测试+报告（推荐提交前）
    
报告位置：
    • JSON报告：reports/test-results.json
    • HTML报告：reports/test-report.html  
    • 覆盖率：htmlcov/index.html
    
高级调试请直接使用 pytest 命令（见文档 4.2 的5.2节）
"""

import argparse
import subprocess
import sys
import os
import time
from pathlib import Path

def run_command(cmd, description, timeout=300):
    """运行命令并显示结果"""
    print(f"\n{'='*60}")
    print(f"🏃 {description}")
    print(f"{'='*60}")
    print(f"命令: {' '.join(cmd)}")
    print("-" * 60)
    
    start_time = time.time()
    try:
        result = subprocess.run(
            cmd, 
            timeout=timeout,
            capture_output=False,  # 实时显示输出
            text=True
        )
        
        duration = time.time() - start_time
        
        if result.returncode == 0:
            print(f"\n✅ {description} 成功完成! (耗时: {duration:.1f}s)")
        else:
            print(f"\n❌ {description} 失败! (耗时: {duration:.1f}s)")
            
        return result.returncode == 0
        
    except subprocess.TimeoutExpired:
        print(f"\n⏰ {description} 超时 (>{timeout}s)")
        return False
    except Exception as e:
        print(f"\n💥 {description} 出错: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description="🤖 导航控制模块统一测试工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
推荐使用方式:
  python run_tests.py --quick            # 日常快速验证
  python run_tests.py --all              # 提交前完整验证（生成所有报告）
  python run_tests.py --unit --coverage  # 单元测试+覆盖率分析
  python run_tests.py --module planning  # 调试特定模块

vs pytest原生命令:
  本脚本 = pytest + 自动报告生成 + 友好界面
  直接使用pytest请参考文档《4.2 连调：导航与控制》第5.2节
        """
    )
    
    parser.add_argument('--all', action='store_true', help='运行所有测试')
    parser.add_argument('--unit', action='store_true', help='运行单元测试')
    parser.add_argument('--integration', action='store_true', help='运行集成测试')
    parser.add_argument('--performance', action='store_true', help='运行性能测试')
    parser.add_argument('--robustness', action='store_true', help='运行鲁棒性测试')
    parser.add_argument('--coverage', action='store_true', help='生成覆盖率报告')
    parser.add_argument('--quick', action='store_true', help='快速测试（跳过慢速测试）')
    parser.add_argument('--module', type=str, help='测试特定模块 (path_planner, motion_planner, etc.)')
    parser.add_argument('--verbose', '-v', action='store_true', help='详细输出')
    parser.add_argument('--parallel', '-j', type=int, help='并行测试进程数')
    
    args = parser.parse_args()
    
    # 确保在项目根目录
    project_root = Path(__file__).parent
    os.chdir(project_root)
    
    print("🤖 导航控制模块测试工具")
    print(f"📁 工作目录: {project_root}")
    print(f"🐍 Python版本: {sys.version}")
    
    # 检查环境
    if not Path("tests").exists():
        print("❌ 未找到tests目录!")
        return 1
    
    if not Path("requirements.txt").exists():
        print("❌ 未找到requirements.txt!")
        return 1
    
    # 构建pytest命令
    pytest_cmd = [sys.executable, "-m", "pytest"]
    
    # 默认参数
    if args.verbose:
        pytest_cmd.append("-v")
    else:
        pytest_cmd.append("-q")
    
    # 并行测试
    if args.parallel:
        pytest_cmd.extend(["-n", str(args.parallel)])
    
    # 覆盖率
    if args.coverage or args.all:
        pytest_cmd.extend([
            "--cov=.", 
            "--cov-report=term-missing",
            "--cov-report=html:htmlcov"
        ])
    
    # 生成报告
    if args.all:
        pytest_cmd.extend([
            "--html=reports/test-report.html",
            "--self-contained-html",
            "--json-report",
            "--json-report-file=reports/test-results.json"
        ])
        # 确保reports目录存在
        Path("reports").mkdir(exist_ok=True)
    
    success_count = 0
    total_tests = 0
    
    # 根据参数选择测试
    if args.all:
        total_tests += 1
        if run_command(pytest_cmd + ["tests/"], "所有测试", 600):
            success_count += 1
            
    elif args.module:
        total_tests += 1
        test_file = f"tests/test_{args.module}.py"
        if Path(test_file).exists():
            if run_command(pytest_cmd + [test_file], f"{args.module}模块测试"):
                success_count += 1
        else:
            print(f"❌ 未找到测试文件: {test_file}")
            
    else:
        # 分类运行测试
        if args.unit:
            total_tests += 1
            cmd = pytest_cmd + ["-m", "unit", "tests/"]
            if args.quick:
                cmd.extend(["-m", "not slow"])
            if run_command(cmd, "单元测试"):
                success_count += 1
        
        if args.integration:
            total_tests += 1
            if run_command(pytest_cmd + ["-m", "integration", "tests/"], "集成测试"):
                success_count += 1
        
        if args.performance:
            total_tests += 1
            if run_command(pytest_cmd + ["-m", "performance", "tests/"], "性能测试"):
                success_count += 1
        
        if args.robustness:
            total_tests += 1
            if run_command(pytest_cmd + ["-m", "robustness", "tests/"], "鲁棒性测试"):
                success_count += 1
        
        # 如果没有指定任何测试类型，运行所有单元测试
        if not any([args.unit, args.integration, args.performance, args.robustness]):
            total_tests += 1
            cmd = pytest_cmd + ["tests/"]
            if args.quick:
                cmd.extend(["-m", "not slow"])
            if run_command(cmd, "默认测试（所有单元测试）"):
                success_count += 1
    
    # 总结
    print(f"\n{'='*60}")
    print(f"📊 测试总结")
    print(f"{'='*60}")
    print(f"成功: {success_count}/{total_tests}")
    
    if success_count == total_tests:
        print("🎉 所有测试都通过了！")
        if args.coverage or args.all:
            print("📋 覆盖率报告: htmlcov/index.html")
        if args.all:
            print("📄 详细报告: reports/test-results.json")
            print("🌐 可视化报告: reports/test-report.html")
        print("\n💡 提示: 如需高级调试，请参考文档《4.2 连调：导航与控制》第5.2节使用pytest命令")
        return 0
    else:
        print("❌ 部分测试失败，请检查上面的输出")
        print("🔧 调试建议: 使用 pytest tests/test_xxx.py::test_func -v -s 精确定位问题")
        return 1

if __name__ == "__main__":
    sys.exit(main())
