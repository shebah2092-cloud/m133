#!/bin/bash
# ======================================================================
#  Cross-compile acados core libraries for arm64-v8a (Android)
#
#  Builds libacados.a, libblasfeo.a, libhpipm.a from acados-main/
#  source using Android NDK cmake toolchain.
#
#  Prerequisites:
#    - Android NDK 27 installed
#    - acados-main/ with source code
#
#  Output:
#    AndroidApp/app/src/main/cpp/acados_arm64/lib/libacados.a
#    AndroidApp/app/src/main/cpp/acados_arm64/lib/libblasfeo.a
#    AndroidApp/app/src/main/cpp/acados_arm64/lib/libhpipm.a
# ======================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ---- Find NDK ----
if [ -n "${ANDROID_NDK_HOME:-}" ]; then
    NDK="$ANDROID_NDK_HOME"
elif [ -d "$HOME/Android/Sdk/ndk/27.2.12479018" ]; then
    NDK="$HOME/Android/Sdk/ndk/27.2.12479018"
else
    echo "ERROR: Cannot find Android NDK 27. Set ANDROID_NDK_HOME." >&2
    exit 1
fi

TOOLCHAIN="$NDK/build/cmake/android.toolchain.cmake"
if [ ! -f "$TOOLCHAIN" ]; then
    echo "ERROR: CMake toolchain not found: $TOOLCHAIN" >&2
    exit 1
fi

ACADOS_SRC="$REPO_ROOT/acados-main"
OUT_DIR="$REPO_ROOT/AndroidApp/app/src/main/cpp/acados_arm64/lib"
BUILD_DIR="/tmp/acados_arm64_build_$$"

mkdir -p "$BUILD_DIR" "$OUT_DIR"

echo "Building acados ARM64 libraries..."
echo "  NDK:    $NDK"
echo "  Source: $ACADOS_SRC"
echo "  Output: $OUT_DIR"

cd "$BUILD_DIR"

cmake "$ACADOS_SRC" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DANDROID_ABI=arm64-v8a \
    -DANDROID_PLATFORM=android-26 \
    -DCMAKE_BUILD_TYPE=Release \
    -DACADOS_WITH_QPOASES=OFF \
    -DACADOS_WITH_DAQP=OFF \
    -DACADOS_WITH_OSQP=OFF \
    -DACADOS_WITH_QPDUNES=OFF \
    -DACADOS_WITH_HPMPC=OFF \
    -DACADOS_WITH_OOQP=OFF \
    -DTARGET=GENERIC \
    -DBLASFEO_TARGET=GENERIC \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    2>&1 | tail -20

echo ""
echo "Compiling (this may take a few minutes)..."
make -j$(nproc) acados blasfeo hpipm 2>&1 | tail -20

# Copy libraries
for lib in libacados.a libblasfeo.a libhpipm.a; do
    found=$(find "$BUILD_DIR" -name "$lib" -type f 2>/dev/null | head -1)
    if [ -n "$found" ]; then
        cp "$found" "$OUT_DIR/$lib"
        echo "  Copied: $lib ($(stat -c%s "$OUT_DIR/$lib") bytes)"
    else
        echo "  WARNING: $lib not found in build directory"
    fi
done

rm -rf "$BUILD_DIR"

echo ""
echo "Done! ARM64 acados libraries updated in $OUT_DIR"
echo "Now run: ./scripts/build_m130_solvers_arm64.sh && cd AndroidApp && ./gradlew assembleDebug"
