# TVMD 自訂 STP 模型整合指南

完整流程將您的 CAD STP 模型整合到 Gazebo SITL 模擬中。

## 📋 快速流程概覽

```
STP/STEP 檔案
    ↓ (轉換)
STL/DAE 網格檔案
    ↓ (放置)
meshes/custom/ 目錄
    ↓ (修改)
model.sdf 參考新網格
    ↓ (測試)
Gazebo 模擬
```

---

## 🚀 步驟 1：準備您的 STP 檔案

建議分離檔案結構：
- `tvmd_base.stp` - 主機身/導航模組
- `tvmd_module.stp` - 單一代理模組（可重複使用4次）
- `tvmd_gimbal.stp` - 萬向節框架
- `tvmd_actuator.stp` - 執行器外殼
- `tvmd_propeller.stp` - 螺旋槳

或者：
- `tvmd_complete.stp` - 完整組裝模型

---

## 🔧 步驟 2：轉換 STP 為網格格式

### 方法 A：使用自動化腳本（推薦）

```bash
cd /home/user/TVMD-PX4/Tools/simulation/gz/models/tvmd

# 轉換完整模型
./scripts/convert_stp_to_mesh.sh ~/Downloads/tvmd_complete.stp tvmd_complete dae

# 轉換個別組件
./scripts/convert_stp_to_mesh.sh ~/Downloads/tvmd_module.stp tvmd_module dae
./scripts/convert_stp_to_mesh.sh ~/Downloads/tvmd_propeller.stp tvmd_prop dae
```

輸出位置：`meshes/custom/tvmd_complete.dae`

### 方法 B：手動使用 FreeCAD

```bash
# 安裝 FreeCAD
sudo apt-get install freecad

# 開啟 GUI
freecad ~/Downloads/tvmd_complete.stp

# 在 FreeCAD 中：
# 1. File → Export
# 2. 選擇格式：
#    - COLLADA (*.dae) - 推薦用於視覺化（支援材質）
#    - STL (*.stl) - 推薦用於碰撞（簡單）
# 3. 儲存到: meshes/custom/tvmd_complete.dae
```

### 方法 C：線上轉換

訪問：https://www.meshconvert.com/
1. 上傳 `.stp` 檔案
2. 選擇輸出格式：DAE 或 STL
3. 下載並放到 `meshes/custom/`

---

## 📐 步驟 3：檢查網格尺寸和方向

```bash
# 使用 Gazebo 直接預覽網格
gz sim -v 4 meshes/custom/tvmd_complete.dae
```

**常見問題：**

### 尺寸錯誤（太大或太小）
```xml
<!-- 在 model.sdf 中調整 scale -->
<scale>0.001 0.001 0.001</scale>  <!-- mm → m (常見) -->
<scale>0.01 0.01 0.01</scale>     <!-- cm → m -->
<scale>1.0 1.0 1.0</scale>        <!-- 已經是 m -->
```

### 方向錯誤（上下顛倒、旋轉）
```xml
<!-- 調整 pose 中的旋轉角度 (roll pitch yaw) -->
<pose>0 0 0 0 0 0</pose>          <!-- 無旋轉 -->
<pose>0 0 0 3.14159 0 0</pose>    <!-- 翻轉 180° -->
<pose>0 0 0 0 0 1.5708</pose>     <!-- Z軸旋轉 90° -->
```

---

## 📝 步驟 4：修改 model.sdf

有兩種整合方式：

### 選項 A：完整替換（單一模型）

如果您有完整的組裝模型：

```xml
<link name='base_link'>
  <!-- 碰撞：使用簡化幾何體 -->
  <collision name='base_link_collision'>
    <pose>0 0 0.05 0 0 0</pose>
    <geometry>
      <box>
        <size>0.5 0.4 0.1</size>
      </box>
    </geometry>
  </collision>

  <!-- 視覺化：使用完整 STP 轉換模型 -->
  <visual name='base_link_visual'>
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <mesh>
        <scale>0.001 0.001 0.001</scale>
        <uri>model://tvmd/meshes/custom/tvmd_complete.dae</uri>
      </mesh>
    </geometry>
    <material>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <ambient>0.5 0.5 0.5 1</ambient>
    </material>
  </visual>
</link>
```

### 選項 B：模組化替換（保留關節）

如果您有分離的組件模型：

```bash
# 在現有 model.sdf 中找到並替換：

# 1. 主機身
<visual name='base_link_visual'>
  <geometry>
    <mesh>
      <scale>0.001 0.001 0.001</scale>
      <!-- 從這裡： -->
      <uri>model://tvmd/meshes/navigator/NavigatorModule-Sim.stl</uri>
      <!-- 改為： -->
      <uri>model://tvmd/meshes/custom/tvmd_base.dae</uri>
    </mesh>
  </geometry>
</visual>

# 2. 模組 1-4（每個模組都替換）
<visual name='base_link_fixed_joint_lump__module1_body_link_visual_2'>
  <pose>0.185 0.16 0 0 0 1.5707963267948959</pose>
  <geometry>
    <mesh>
      <scale>0.001 0.001 0.001</scale>
      <!-- 從這裡： -->
      <uri>model://tvmd/meshes/agent/base_link.stl</uri>
      <!-- 改為： -->
      <uri>model://tvmd/meshes/custom/tvmd_module.dae</uri>
    </mesh>
  </geometry>
</visual>

# 3. 螺旋槳
<visual name='module1_prop1_link_visual'>
  <pose>0 0 0 1.5707963267948959 0 0</pose>
  <geometry>
    <mesh>
      <scale>0.001 0.001 0.001</scale>
      <!-- 從這裡： -->
      <uri>model://tvmd/meshes/agent/Propeller_9047.stl</uri>
      <!-- 改為： -->
      <uri>model://tvmd/meshes/custom/tvmd_prop.dae</uri>
    </mesh>
  </geometry>
</visual>
```

---

## ⚡ 步驟 5：優化性能

### 碰撞模型優化（重要！）

**永遠使用簡單幾何體進行碰撞檢測：**

```xml
<!-- ❌ 不推薦：使用複雜網格 -->
<collision name='bad_collision'>
  <geometry>
    <mesh>
      <uri>model://tvmd/meshes/custom/tvmd_complete.dae</uri>
    </mesh>
  </geometry>
</collision>

<!-- ✅ 推薦：使用簡單幾何體 -->
<collision name='good_collision'>
  <geometry>
    <box>
      <size>0.5 0.4 0.1</size>  <!-- 近似飛機尺寸 -->
    </box>
  </geometry>
</collision>
```

### 視覺化模型簡化

如果模型太複雜導致模擬變慢：

```bash
# 使用 Blender 簡化網格
blender --background --python - << 'EOF'
import bpy

# 匯入模型
bpy.ops.wm.collada_import(filepath="meshes/custom/tvmd_complete.dae")

# 選擇所有物件
bpy.ops.object.select_all(action='SELECT')

# 添加簡化修改器
for obj in bpy.context.selected_objects:
    if obj.type == 'MESH':
        mod = obj.modifiers.new(name='Decimate', type='DECIMATE')
        mod.ratio = 0.5  # 減少 50% 面數
        bpy.ops.object.modifier_apply(modifier='Decimate')

# 匯出簡化後的模型
bpy.ops.wm.collada_export(filepath="meshes/custom/tvmd_complete_simplified.dae")
EOF
```

---

## 🧪 步驟 6：測試整合

### 測試 1：純 Gazebo 測試

```bash
# 直接在 Gazebo 中載入模型
cd /home/user/TVMD-PX4/Tools/simulation/gz/models/tvmd
gz sim -v 4 -r model.sdf
```

檢查項目：
- [ ] 模型正常顯示
- [ ] 尺寸正確
- [ ] 方向正確
- [ ] 顏色/材質正確（如果使用 DAE）
- [ ] 關節可動作（按 GUI 中的 play）

### 測試 2：PX4 SITL 完整測試

```bash
cd /home/user/TVMD-PX4

# 啟動 SITL
make px4_sitl gz_tvmd

# 或
PX4_SIM_MODEL=tvmd make px4_sitl gz
```

檢查項目：
- [ ] Gazebo 正常啟動
- [ ] TVMD 模型載入
- [ ] 馬達旋轉動畫
- [ ] 伺服馬達動作
- [ ] PX4 正常連接

---

## 🎨 進階：材質和顏色設定

### 使用 DAE 內建材質

如果您的 DAE 檔案包含材質：

```xml
<visual name='base_link_visual'>
  <geometry>
    <mesh>
      <uri>model://tvmd/meshes/custom/tvmd_complete.dae</uri>
    </mesh>
  </geometry>
  <!-- 不需要額外的 material 標籤 -->
</visual>
```

### 自訂材質

```xml
<visual name='base_link_visual'>
  <geometry>
    <mesh>
      <uri>model://tvmd/meshes/custom/tvmd_complete.stl</uri>
    </mesh>
  </geometry>
  <material>
    <script>
      <uri>file://media/materials/scripts/gazebo.material</uri>
      <name>Gazebo/DarkGrey</name>  <!-- 可選: Grey, Red, Blue, etc. -->
    </script>
    <!-- 或使用 RGBA 顏色 -->
    <diffuse>0.5 0.5 0.5 1</diffuse>   <!-- 漫反射 -->
    <ambient>0.3 0.3 0.3 1</ambient>   <!-- 環境光 -->
    <specular>0.1 0.1 0.1 1</specular> <!-- 高光 -->
    <emissive>0 0 0 1</emissive>       <!-- 自發光 -->
  </material>
</visual>
```

---

## 📊 檔案大小和性能建議

| 網格複雜度 | 面數 | 檔案大小 | 適用場景 |
|----------|------|---------|---------|
| 超高精度 | >100k | >10MB | 渲染、展示 |
| 高精度 | 10k-100k | 1-10MB | ✅ 視覺化（推薦） |
| 中等精度 | 1k-10k | 100KB-1MB | 一般模擬 |
| 低精度 | <1k | <100KB | ✅ 碰撞檢測（推薦） |

**最佳實踐：**
- 視覺化網格：10k-50k 面
- 碰撞網格：簡單幾何體（box, cylinder）

---

## 🔧 故障排除

### 問題 1：模型不顯示

```bash
# 檢查路徑
ls -la meshes/custom/tvmd_complete.dae

# 檢查 Gazebo 日誌
gz sim -v 4 model.sdf 2>&1 | grep -i error
```

### 問題 2：模型太大/太小

修改 `<scale>` 參數：
- CAD 是 mm → 使用 `0.001`
- CAD 是 cm → 使用 `0.01`
- CAD 是 m → 使用 `1.0`

### 問題 3：顏色全黑

```xml
<!-- 添加環境光 -->
<material>
  <ambient>0.5 0.5 0.5 1</ambient>
  <diffuse>0.8 0.8 0.8 1</diffuse>
</material>
```

### 問題 4：模擬變慢

1. 簡化視覺化網格（使用 Blender Decimate）
2. 碰撞使用基本幾何體
3. 減少 `<update_rate>` 如果不需要高頻率

---

## 📚 參考資源

- [Gazebo SDF Format](http://sdformat.org/)
- [FreeCAD 文件](https://wiki.freecad.org/)
- [Blender 文件](https://docs.blender.org/)
- [STL 格式說明](https://en.wikipedia.org/wiki/STL_(file_format))
- [COLLADA (DAE) 格式](https://www.khronos.org/collada/)

---

## 📞 需要協助？

如果遇到問題，請提供：
1. STP 檔案來源（哪個 CAD 軟體）
2. 轉換使用的工具和版本
3. Gazebo 錯誤訊息
4. 截圖（如果可能）
