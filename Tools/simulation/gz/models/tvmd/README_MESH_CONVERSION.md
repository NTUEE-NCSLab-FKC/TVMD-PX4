# TVMD Mesh Conversion Guide

## 將 STP/STEP CAD 模型匯入 Gazebo

### 方法 1：使用 FreeCAD (推薦)

#### 安裝
```bash
sudo apt-get install freecad freecad-python3
```

#### GUI 轉換
1. 開啟 FreeCAD
2. File → Open → 選擇 `tvmd_complete.stp`
3. File → Export → 選擇格式：
   - `.stl` - 簡單幾何（推薦用於碰撞）
   - `.dae` - 包含顏色材質（推薦用於視覺）

#### 命令列轉換
```bash
# 建立轉換腳本
cat > convert_stp.py << 'EOF'
import FreeCAD
import Mesh
import sys

if len(sys.argv) < 3:
    print("Usage: freecad convert_stp.py <input.stp> <output.stl>")
    sys.exit(1)

input_file = sys.argv[1]
output_file = sys.argv[2]

# 載入 STP
doc = FreeCAD.open(input_file)

# 合併所有物件
shapes = [obj.Shape for obj in doc.Objects if hasattr(obj, 'Shape')]

# 轉換為網格 (0.1 = 高精度, 1.0 = 低精度)
mesh_obj = doc.addObject("Mesh::Feature", "Mesh")
mesh_obj.Mesh = Mesh.Mesh()

for shape in shapes:
    mesh_obj.Mesh.addFacets(shape.tessellate(0.1))

# 匯出
Mesh.export([mesh_obj], output_file)
print(f"Converted {input_file} to {output_file}")
EOF

# 執行轉換
freecad convert_stp.py tvmd_complete.stp meshes/tvmd_full.stl
```

### 方法 2：使用 Blender

#### 安裝
```bash
sudo snap install blender --classic
```

#### 轉換流程
1. 先將 STP 轉為 OBJ（使用 FreeCAD 或線上工具）
2. Blender → File → Import → Wavefront (.obj)
3. 調整模型：
   - 縮放到正確尺寸
   - 旋轉到正確方向（Gazebo: Z-up）
   - 設定原點（3D cursor → Set Origin）
4. File → Export → Collada (.dae)

### 方法 3：線上轉換

訪問以下網站：
- https://www.meshconvert.com/
- https://products.aspose.app/3d/conversion/step-to-dae

上傳 `.stp` 檔案，下載 `.dae` 或 `.stl`

---

## 整合到 Gazebo

### 步驟 1：放置網格檔案

```bash
# 將轉換後的檔案放到正確位置
cp tvmd_full.dae Tools/simulation/gz/models/tvmd/meshes/
cp tvmd_collision.stl Tools/simulation/gz/models/tvmd/meshes/
```

### 步驟 2：修改 model.sdf

#### 選項 A：完全替換視覺化（單一模型）

```xml
<visual name='base_link_visual'>
  <pose>0 0 0 0 0 0</pose>
  <geometry>
    <mesh>
      <scale>0.001 0.001 0.001</scale>
      <uri>model://tvmd/meshes/tvmd_full.dae</uri>
    </mesh>
  </geometry>
  <material>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <ambient>0.5 0.5 0.5 1</ambient>
  </material>
</visual>
```

#### 選項 B：保留分離模型（模組化）

保留 4 個模組的獨立視覺化：
```xml
<!-- 保留原有結構，僅替換 STL 檔案 -->
<visual name='base_link_fixed_joint_lump__module1_body_link_visual_2'>
  <pose>0.185 0.16 0 0 0 1.5707963267948959</pose>
  <geometry>
    <mesh>
      <scale>0.001 0.001 0.001</scale>
      <uri>model://tvmd/meshes/agent/module_new.dae</uri>
    </mesh>
  </geometry>
</visual>
```

### 步驟 3：簡化碰撞模型（重要！）

```xml
<!-- 使用簡單幾何體提升性能 -->
<collision name='base_link_collision'>
  <pose>0 0 0.05 0 0 0</pose>
  <geometry>
    <box>
      <size>0.5 0.4 0.1</size>  <!-- 根據實際尺寸調整 -->
    </box>
  </geometry>
</collision>
```

---

## 常見問題

### Q1: 模型尺寸不對？
```xml
<!-- 調整 scale 參數 -->
<scale>0.001 0.001 0.001</scale>  <!-- 1mm → 1m -->
<scale>1.0 1.0 1.0</scale>        <!-- 保持原尺寸 -->
```

### Q2: 模型方向錯誤？
```xml
<!-- 調整 pose 中的旋轉（roll pitch yaw） -->
<pose>0 0 0 1.5707 0 0</pose>  <!-- 繞 X 軸旋轉 90° -->
<pose>0 0 0 0 1.5707 0</pose>  <!-- 繞 Y 軸旋轉 90° -->
<pose>0 0 0 0 0 1.5707</pose>  <!-- 繞 Z 軸旋轉 90° -->
```

### Q3: 模型太複雜導致模擬變慢？

**視覺化優化：**
```bash
# 使用 Blender 簡化網格
blender --background --python - << 'EOF'
import bpy
bpy.ops.import_mesh.stl(filepath="tvmd_full.stl")
bpy.ops.object.modifier_add(type='DECIMATE')
bpy.context.object.modifiers["Decimate"].ratio = 0.5  # 減少 50% 面數
bpy.ops.object.modifier_apply(modifier="Decimate")
bpy.ops.export_scene.gltf(filepath="tvmd_simplified.glb")
EOF
```

**碰撞優化：**
- 永遠使用基本幾何體（box, cylinder, sphere）
- 如果必須用網格，使用極簡化版本

### Q4: 顏色/材質沒有顯示？

使用 DAE 格式並確保包含材質：
```xml
<visual>
  <geometry>
    <mesh>
      <uri>model://tvmd/meshes/tvmd_full.dae</uri>
    </mesh>
  </geometry>
  <!-- 如果 DAE 包含材質，可省略此部分 -->
  <material>
    <script>
      <uri>file://media/materials/scripts/gazebo.material</uri>
      <name>Gazebo/Grey</name>
    </script>
  </material>
</visual>
```

---

## 測試流程

```bash
# 1. 轉換模型
freecad convert_stp.py your_model.stp meshes/tvmd_full.stl

# 2. 修改 model.sdf（使用上述範例）

# 3. 啟動 Gazebo 驗證
gz sim -v 4 -r Tools/simulation/gz/models/tvmd/model.sdf

# 4. 檢查是否正常顯示和碰撞
```

---

## 進階：動態模組視覺化

如果想讓 4 個模組的螺旋槳獨立旋轉：

```xml
<!-- 每個螺旋槳使用獨立的 visual + joint -->
<link name='module1_prop1_link'>
  <visual name='module1_prop1_link_visual'>
    <geometry>
      <mesh>
        <uri>model://tvmd/meshes/propeller_detailed.dae</uri>
      </mesh>
    </geometry>
  </visual>
</link>
```

這樣在模擬時，螺旋槳會隨著 joint 旋轉而視覺化旋轉。
