#!/bin/bash
set -euo pipefail

# 编译脚本 - 在 Apple Silicon (M4) 上强制使用 x86_64 架构编译
# 使用 Ninja Multi-Config 生成器，支持 Debug / Release / RelWithDebInfo 配置选择
# 项目: Moer-lite

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
TARGET_DIR="${SCRIPT_DIR}/target"

# 默认配置
BUILD_TYPE="${1:-Release}"

# 验证配置参数
case "${BUILD_TYPE}" in
    Debug|Release|RelWithDebInfo)
        ;;
    *)
        echo "Usage: $0 [Debug|Release|RelWithDebInfo]"
        echo "  Default: Release"
        exit 1
        ;;
esac

echo "==> Build config: ${BUILD_TYPE}"

# Force x86_64 architecture
export CMAKE_OSX_ARCHITECTURES=x86_64

# Clean old build (comment out for incremental builds)
# echo "==> Cleaning old build..."
# rm -rf "${BUILD_DIR}"
# rm -rf "${TARGET_DIR}"

# Create build directory
mkdir -p "${BUILD_DIR}"

echo "==> Configuring CMake (target: x86_64, generator: Ninja Multi-Config)..."
cd "${BUILD_DIR}"
cmake .. \
    -G "Ninja Multi-Config" \
    -DCMAKE_OSX_ARCHITECTURES=x86_64 \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "==> Building (${BUILD_TYPE})..."
cmake --build . --config "${BUILD_TYPE}"

echo "==> Build complete!"
echo "    Binary: ${TARGET_DIR}/bin/Moer"
echo ""
echo "    Run with TBB:          ./target/bin/Release/Moer <scene-dir>"
echo "    Run without TBB:       ./target/bin/Release/Moer --no-tbb <scene-dir>"
