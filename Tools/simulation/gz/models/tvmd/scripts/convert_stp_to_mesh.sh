#!/bin/bash
###############################################################################
# TVMD STP to Gazebo Mesh Converter
#
# 此腳本將 STP/STEP CAD 模型轉換為 Gazebo 可用的網格格式
#
# 使用方法:
#   ./convert_stp_to_mesh.sh input.stp output_name [format]
#
# 參數:
#   input.stp    - 輸入的 STP/STEP 檔案
#   output_name  - 輸出檔案名稱（不含副檔名）
#   format       - 輸出格式: stl, dae, obj (預設: dae)
#
# 範例:
#   ./convert_stp_to_mesh.sh ~/tvmd_model.stp tvmd_complete dae
#
###############################################################################

set -e  # 遇到錯誤即停止

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 函數：印出訊息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 檢查參數
if [ $# -lt 2 ]; then
    print_error "參數不足"
    echo "使用方法: $0 <input.stp> <output_name> [format]"
    echo "範例: $0 tvmd_model.stp tvmd_complete dae"
    exit 1
fi

INPUT_FILE="$1"
OUTPUT_NAME="$2"
FORMAT="${3:-dae}"  # 預設使用 DAE 格式

# 檢查輸入檔案是否存在
if [ ! -f "$INPUT_FILE" ]; then
    print_error "找不到輸入檔案: $INPUT_FILE"
    exit 1
fi

# 取得腳本目錄
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MESH_DIR="$SCRIPT_DIR/../meshes/custom"

# 建立輸出目錄
mkdir -p "$MESH_DIR"

print_info "開始轉換 STP 模型..."
print_info "輸入檔案: $INPUT_FILE"
print_info "輸出名稱: $OUTPUT_NAME"
print_info "輸出格式: $FORMAT"

# 檢查可用的轉換工具
CONVERTER=""

if command -v freecad &> /dev/null; then
    CONVERTER="freecad"
    print_info "使用 FreeCAD 進行轉換"
elif command -v blender &> /dev/null; then
    CONVERTER="blender"
    print_info "使用 Blender 進行轉換"
else
    print_error "找不到轉換工具 (FreeCAD 或 Blender)"
    echo ""
    echo "請安裝其中一個工具："
    echo "  FreeCAD:  sudo apt-get install freecad"
    echo "  Blender:  sudo snap install blender --classic"
    exit 1
fi

# 轉換函數
convert_with_freecad() {
    local input="$1"
    local output="$2"
    local format="$3"

    # 建立臨時 Python 腳本
    local temp_script=$(mktemp /tmp/freecad_convert_XXXXXX.py)

    cat > "$temp_script" << 'PYTHON_SCRIPT'
import FreeCAD
import Mesh
import sys
import os

# 取得參數
input_file = sys.argv[1]
output_file = sys.argv[2]
output_format = sys.argv[3]

print(f"載入 STP 檔案: {input_file}")
doc = FreeCAD.open(input_file)

# 取得所有形狀
shapes = []
for obj in doc.Objects:
    if hasattr(obj, 'Shape'):
        shapes.append(obj.Shape)

if not shapes:
    print("錯誤：STP 檔案中沒有找到任何形狀")
    sys.exit(1)

print(f"找到 {len(shapes)} 個形狀，開始轉換...")

# 建立網格物件
mesh_obj = doc.addObject("Mesh::Feature", "ConvertedMesh")
mesh_obj.Mesh = Mesh.Mesh()

# 轉換所有形狀為網格
# 精度參數：0.01 = 非常高, 0.1 = 高, 1.0 = 低
for i, shape in enumerate(shapes):
    print(f"轉換形狀 {i+1}/{len(shapes)}...")
    mesh_obj.Mesh.addFacets(shape.tessellate(0.1))

# 匯出
print(f"匯出為 {output_format.upper()} 格式...")
if output_format == 'stl':
    Mesh.export([mesh_obj], output_file)
elif output_format == 'dae':
    # DAE 匯出需要特殊處理
    try:
        import importDAE
        importDAE.export([mesh_obj], output_file)
    except ImportError:
        print("警告：無法匯出 DAE 格式，改用 STL")
        output_file = output_file.replace('.dae', '.stl')
        Mesh.export([mesh_obj], output_file)
elif output_format == 'obj':
    # OBJ 匯出
    Mesh.export([mesh_obj], output_file)
else:
    print(f"不支援的格式: {output_format}")
    sys.exit(1)

print(f"轉換完成！輸出檔案: {output_file}")

# 輸出網格統計
print(f"網格統計:")
print(f"  面數: {mesh_obj.Mesh.CountFacets}")
print(f"  點數: {mesh_obj.Mesh.CountPoints}")
PYTHON_SCRIPT

    # 執行 FreeCAD 轉換
    print_info "執行 FreeCAD 轉換..."
    freecad "$temp_script" "$input" "$output" "$format"

    # 清理臨時檔案
    rm -f "$temp_script"
}

convert_with_blender() {
    local input="$1"
    local output="$2"
    local format="$3"

    print_warning "Blender 不直接支援 STEP 匯入"
    print_info "請先使用 FreeCAD 或線上工具將 STP 轉為 OBJ"
    print_info "然後使用 Blender 進行材質和優化"

    # TODO: 實作 Blender 轉換流程
    return 1
}

# 執行轉換
OUTPUT_FILE="$MESH_DIR/${OUTPUT_NAME}.${FORMAT}"

case "$CONVERTER" in
    freecad)
        convert_with_freecad "$INPUT_FILE" "$OUTPUT_FILE" "$FORMAT"
        ;;
    blender)
        convert_with_blender "$INPUT_FILE" "$OUTPUT_FILE" "$FORMAT"
        ;;
    *)
        print_error "未知的轉換工具: $CONVERTER"
        exit 1
        ;;
esac

# 檢查輸出檔案
if [ -f "$OUTPUT_FILE" ]; then
    print_success "轉換成功！"
    print_info "輸出檔案: $OUTPUT_FILE"

    # 顯示檔案大小
    FILE_SIZE=$(du -h "$OUTPUT_FILE" | cut -f1)
    print_info "檔案大小: $FILE_SIZE"

    echo ""
    echo "下一步："
    echo "1. 檢查網格方向和尺寸："
    echo "   gz sim -v 4 $OUTPUT_FILE"
    echo ""
    echo "2. 修改 model.sdf 使用此網格："
    echo "   <uri>model://tvmd/meshes/custom/${OUTPUT_NAME}.${FORMAT}</uri>"
    echo ""
    echo "3. 調整縮放比例（如果需要）："
    echo "   <scale>0.001 0.001 0.001</scale>  <!-- mm to m -->"

else
    print_error "轉換失敗，找不到輸出檔案"
    exit 1
fi
