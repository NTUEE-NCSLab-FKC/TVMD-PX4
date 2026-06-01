# TVMD-PX4 Log Topics 欄位說明文件

本文件整理 PX4 ulog 飛行日誌所記錄的所有 Topic（主題）、欄位名稱、資料型態、單位及物理意義。

---

## 座標系定義

PX4 使用兩種主要座標系，TVMD 控制器內部另有 NWU：

```
NED（地球固定）           FRD（機體，PX4 標準）    NWU（機體，TVMD 內部）
  N = +x（北）              F = +x（前）              F = +x（前）
  E = +y（東）              R = +y（右）              L = +y（左，Y 反向）
  D = +z（向下）            D = +z（向下）            U = +z（向上，Z 反向）
```

**座標系轉換（機體座標內）：** `NWU = (FRD_x, −FRD_y, −FRD_z)`

### 座標系速查表

```
Topic                               主要量               座標系
──────────────────────────────────────────────────────────────────
vehicle_attitude.q                  姿態四元數            FRD → NED
vehicle_angular_velocity.xyz        角速度                FRD 機體（rad/s）
vehicle_acceleration.xyz            加速度（含重力）       FRD 機體（m/s²）
sensor_combined.gyro_rad            陀螺儀               FRD 機體（rad/s）
sensor_combined.accelerometer_m_s2 加速度計              FRD 機體（m/s²）
vehicle_magnetometer.magnetometer_ga磁場                  FRD 機體（Gauss）
vehicle_imu.delta_angle             角度增量              FRD 機體（rad）
vehicle_imu.delta_velocity          速度增量              FRD 機體（m/s）
──────────────────────────────────────────────────────────────────
vehicle_local_position.x/y/z        位置                  NED（m，z向下為正）
vehicle_local_position.vx/vy/vz     速度                  NED（m/s）
vehicle_local_position.ax/ay/az     加速度                NED（m/s²）
vehicle_local_position.heading      偏航角                NED（rad，從北順時針）
vehicle_global_position             緯度/經度/高度         WGS84
trajectory_setpoint.position[3]     位置設定點            NED（m）
trajectory_setpoint.yaw             偏航設定點            NED（rad）
wind.windspeed_north/east           風速                  NED 水平分量（m/s）
──────────────────────────────────────────────────────────────────
vehicle_attitude_setpoint.q_d       期望姿態四元數         FRD → NED
vehicle_attitude_setpoint.thrust_body 推力指令            FRD 機體（[2]<0=向上）
vehicle_rates_setpoint.roll/pitch/yaw 角速率設定點 ①     FRD 機體（rad/s）
vehicle_thrust_setpoint.xyz         推力設定點            FRD 機體（正規化）
vehicle_torque_setpoint.xyz         力矩設定點            FRD 機體（正規化）
──────────────────────────────────────────────────────────────────
[TVMD 特有]
control_allocation_meta_data.control_sp[6]  控制設定點    FRD 正規化
  索引：[0]=Roll力矩 [1]=Pitch力矩 [2]=Yaw力矩
        [3]=Fx     [4]=Fy     [5]=Fz（向下為正）
control_allocation_meta_data.f_x/y/z[16]   各模組力      模組局部 NWU（N，z向上）
attitude_planner_meta_data.q_d              期望姿態      FRD → NED
attitude_planner_meta_data.sat_thrust_body  飽和推力 ②   NWU 機體（z向上）
attitude_planner_meta_data.coplan_vector    旋轉軸        NWU 機體
──────────────────────────────────────────────────────────────────
```

> ① `vehicle_rates_setpoint` 的 .msg 原始行內註解寫「body angular rates in NED frame」，
>    為**誤導性描述**：roll/pitch/yaw 是 FRD 機體軸角速率，與地球座標系無關。
>
> ② `attitude_planner_meta_data.sat_thrust_body` 的 .msg 原始標注為「body NED frame」，
>    但程式碼（`pfa_att_control.cpp:217`）在 NWU 空間計算後直接寫入，**實際為 NWU 座標系**。
>    若需轉換至 FRD：`FRD = (sat[0], −sat[1], −sat[2])`。

---

---

## 日誌錄製 Profile

| Profile 名稱 | 說明 |
|---|---|
| `DEFAULT` | 預設模式，包含飛控所有核心 Topic |
| `HIGH_RATE` | 高速模式，適用競速/快速機動分析 |
| `ESTIMATOR_REPLAY` | EKF2 重播輸入資料，供離線估算器重播 |
| `THERMAL_CALIBRATION` | 熱校正模式，原始 IMU 資料以 100 ms 間隔記錄 |
| `SENSOR_COMPARISON` | 多感測器比對，記錄四組加速度計/陀螺儀/氣壓計/磁力計 |
| `SYSTEM_IDENTIFICATION` | 系統識別，IMU 及控制輸出以最大速率記錄 |
| `DEBUG_TOPICS` | 偵錯主題（debug_array、debug_vect 等） |
| `VISION_AND_AVOIDANCE` | 視覺里程計與避障障礙物資料 |
| `RAW_IMU_GYRO_FIFO` | 原始陀螺儀 FIFO 資料流 |
| `RAW_IMU_ACCEL_FIFO` | 原始加速度計 FIFO 資料流 |
| `MAVLINK_TUNNEL` | MAVLink Tunnel 訊息 |

---

## Topic 一覽表

### 1. 姿態與角速度

#### `vehicle_attitude` / `estimator_attitude`
**記錄間隔：** 50 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_sample` | uint64 | µs | 原始資料的採樣時間戳 |
| `q[4]` | float32[4] | — | 從機體座標系（FRD）到地球座標系（NED）的四元數旋轉，順序為 [w, x, y, z] |
| `delta_q_reset[4]` | float32[4] | — | 上次重置事件時四元數的變化量 |
| `quat_reset_counter` | uint8 | — | 四元數重置計數器（單調遞增） |

> **FRD**：前-右-下（Forward-Right-Down）機體座標系；**NED**：北-東-下地理座標系。

---

#### `vehicle_angular_velocity`
**記錄間隔：** 20 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_sample` | uint64 | µs | 資料採樣時間戳 |
| `xyz[3]` | float32[3] | rad/s | 已偏差補償的機體 FRD 座標系 X/Y/Z 軸角速度（滾轉/俯仰/偏航） |
| `xyz_derivative[3]` | float32[3] | rad/s² | 機體 FRD 座標系 X/Y/Z 軸角加速度 |

---

#### `vehicle_acceleration`
**記錄間隔：** 50 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_sample` | uint64 | µs | 原始資料採樣時間戳 |
| `xyz[3]` | float32[3] | m/s² | 已偏差補償的機體 FRD 座標系 X/Y/Z 軸加速度（含重力） |

---

### 2. 位置與速度

#### `vehicle_local_position` / `estimator_local_position`
**記錄間隔：** 100 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_sample` | uint64 | µs | 資料採樣時間戳 |
| `xy_valid` | bool | — | x/y 位置是否有效 |
| `z_valid` | bool | — | z 位置是否有效 |
| `v_xy_valid` | bool | — | vx/vy 速度是否有效 |
| `v_z_valid` | bool | — | vz 速度是否有效 |
| `x` | float32 | m | NED 座標系北方位置（相對 EKF2 起始原點） |
| `y` | float32 | m | NED 座標系東方位置 |
| `z` | float32 | m | NED 座標系向下位置（負值代表高度） |
| `delta_xy[2]` | float32[2] | m | 上次水平位置重置的增量 |
| `xy_reset_counter` | uint8 | — | 水平位置重置計數器 |
| `delta_z` | float32 | m | 上次垂直位置重置增量 |
| `z_reset_counter` | uint8 | — | 垂直位置重置計數器 |
| `vx` | float32 | m/s | NED 座標系北方速度 |
| `vy` | float32 | m/s | NED 座標系東方速度 |
| `vz` | float32 | m/s | NED 座標系向下速度 |
| `z_deriv` | float32 | m/s | z 位置的時間導數（向下速度） |
| `delta_vxy[2]` | float32[2] | m/s | 上次水平速度重置增量 |
| `vxy_reset_counter` | uint8 | — | 水平速度重置計數器 |
| `delta_vz` | float32 | m/s | 上次垂直速度重置增量 |
| `vz_reset_counter` | uint8 | — | 垂直速度重置計數器 |
| `ax` | float32 | m/s² | NED 北方加速度 |
| `ay` | float32 | m/s² | NED 東方加速度 |
| `az` | float32 | m/s² | NED 向下加速度 |
| `heading` | float32 | rad | NED 切平面上的尤拉偏航角，範圍 [-π, +π] |
| `unaided_heading` | float32 | rad | 僅由陀螺積分產生的偏航角（無輔助感測器） |
| `delta_heading` | float32 | rad | 上次偏航重置增量 |
| `heading_reset_counter` | uint8 | — | 偏航重置計數器 |
| `heading_good_for_control` | bool | — | 偏航估算品質是否足夠用於飛行控制 |
| `xy_global` | bool | — | x/y 是否有全球參考座標（ref_lat/ref_lon） |
| `z_global` | bool | — | z 是否有全球高度參考 |
| `ref_timestamp` | uint64 | µs | 參考原點設定時間戳 |
| `ref_lat` | float64 | deg | 參考點緯度（WGS84） |
| `ref_lon` | float64 | deg | 參考點經度（WGS84） |
| `ref_alt` | float32 | m | 參考點 AMSL 高度 |
| `dist_bottom` | float32 | m | 機身底部到地面的距離 |
| `dist_bottom_valid` | bool | — | 地面距離是否有效 |
| `dist_bottom_sensor_bitfield` | uint8 | — | 感測器來源位元遮罩（1=測距儀，2=光流） |
| `eph` | float32 | m | 水平位置誤差標準差（1-σ） |
| `epv` | float32 | m | 垂直位置誤差標準差 |
| `evh` | float32 | m/s | 水平速度誤差標準差 |
| `evv` | float32 | m/s | 垂直速度誤差標準差 |
| `dead_reckoning` | bool | — | 是否正在進行純慣性推算定位 |
| `vxy_max` | float32 | m/s | 水平速度限制（0 表示無限制） |
| `vz_max` | float32 | m/s | 垂直速度限制（0 表示無限制） |
| `hagl_min` | float32 | m | 離地最小高度限制（0 表示無限制） |
| `hagl_max` | float32 | m | 離地最大高度限制（0 表示無限制） |

---

#### `vehicle_global_position` / `estimator_global_position`
**記錄間隔：** 200 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_sample` | uint64 | µs | 資料採樣時間戳 |
| `lat` | float64 | deg | 緯度（WGS84） |
| `lon` | float64 | deg | 經度（WGS84） |
| `alt` | float32 | m | AMSL 高度（平均海平面） |
| `alt_ellipsoid` | float32 | m | 橢球面高度 |
| `delta_alt` | float32 | m | 上次高度重置增量 |
| `lat_lon_reset_counter` | uint8 | — | 水平位置重置計數器 |
| `alt_reset_counter` | uint8 | — | 高度重置計數器 |
| `eph` | float32 | m | 水平位置誤差標準差 |
| `epv` | float32 | m | 垂直位置誤差標準差 |
| `terrain_alt` | float32 | m | 地形高度（WGS84） |
| `terrain_alt_valid` | bool | — | 地形高度是否有效 |
| `dead_reckoning` | bool | — | 是否為慣性推算位置 |

---

### 3. 感測器原始資料

#### `sensor_combined`
**記錄間隔：** 連續（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `gyro_rad[3]` | float32[3] | rad/s | FRD 機體座標系 X/Y/Z 軸平均角速度（陀螺儀採樣週期內） |
| `gyro_integral_dt` | uint32 | µs | 陀螺儀採樣週期 |
| `accelerometer_timestamp_relative` | int32 | µs | `timestamp + accelerometer_timestamp_relative` = 加速度計時間戳（`0x7FFFFFFF` 表示無效） |
| `accelerometer_m_s2[3]` | float32[3] | m/s² | FRD 機體座標系 X/Y/Z 軸平均加速度（加速度計採樣週期內） |
| `accelerometer_integral_dt` | uint32 | µs | 加速度計採樣週期 |
| `accelerometer_clipping` | uint8 | — | 位元遮罩：加速度計飽和指示（bit0=X, bit1=Y, bit2=Z） |
| `gyro_clipping` | uint8 | — | 位元遮罩：陀螺儀飽和指示（bit0=X, bit1=Y, bit2=Z） |
| `accel_calibration_count` | uint8 | — | 加速度計校正變更計數器 |
| `gyro_calibration_count` | uint8 | — | 陀螺儀校正變更計數器 |

---

#### `vehicle_imu`（多實例，最多 4 組）
**記錄間隔：** 500 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_sample` | uint64 | µs | 資料採樣時間戳 |
| `accel_device_id` | uint32 | — | 加速度計唯一裝置 ID |
| `gyro_device_id` | uint32 | — | 陀螺儀唯一裝置 ID |
| `delta_angle[3]` | float32[3] | rad | 積分週期內 FRD 機體座標系 X/Y/Z 角增量 |
| `delta_velocity[3]` | float32[3] | m/s | 積分週期內 FRD 機體座標系 X/Y/Z 速度增量 |
| `delta_angle_dt` | uint16 | µs | 角度增量積分週期 |
| `delta_velocity_dt` | uint16 | µs | 速度增量積分週期 |
| `delta_angle_clipping` | uint8 | — | 陀螺儀飽和位元遮罩 |
| `delta_velocity_clipping` | uint8 | — | 加速度計飽和位元遮罩 |
| `accel_calibration_count` | uint8 | — | 加速度計校正變更計數器 |
| `gyro_calibration_count` | uint8 | — | 陀螺儀校正變更計數器 |

---

#### `vehicle_imu_status`（多實例，最多 4 組）
**記錄間隔：** 1000 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `accel_device_id` | uint32 | — | 加速度計唯一裝置 ID |
| `gyro_device_id` | uint32 | — | 陀螺儀唯一裝置 ID |
| `accel_clipping[3]` | uint32[3] | — | X/Y/Z 軸加速度計總飽和次數 |
| `gyro_clipping[3]` | uint32[3] | — | X/Y/Z 軸陀螺儀總飽和次數 |
| `accel_error_count` | uint32 | — | 加速度計錯誤計數 |
| `gyro_error_count` | uint32 | — | 陀螺儀錯誤計數 |
| `accel_rate_hz` | float32 | Hz | 加速度計發布頻率 |
| `gyro_rate_hz` | float32 | Hz | 陀螺儀發布頻率 |
| `accel_raw_rate_hz` | float32 | Hz | 加速度計原始採樣頻率 |
| `gyro_raw_rate_hz` | float32 | Hz | 陀螺儀原始採樣頻率 |
| `accel_vibration_metric` | float32 | m/s² | 加速度計高頻振動強度指標 |
| `gyro_vibration_metric` | float32 | rad/s | 陀螺儀高頻振動強度指標 |
| `delta_angle_coning_metric` | float32 | rad² | 平均角度錐化修正量 |
| `mean_accel[3]` | float32[3] | m/s² | 自上次發布以來加速度計平均讀值 |
| `mean_gyro[3]` | float32[3] | rad/s | 自上次發布以來陀螺儀平均讀值 |
| `var_accel[3]` | float32[3] | (m/s²)² | 自上次發布以來加速度計變異數 |
| `var_gyro[3]` | float32[3] | (rad/s)² | 自上次發布以來陀螺儀變異數 |
| `temperature_accel` | float32 | °C | 加速度計溫度 |
| `temperature_gyro` | float32 | °C | 陀螺儀溫度 |

---

#### `vehicle_magnetometer`（多實例，最多 4 組）
**記錄間隔：** 200 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_sample` | uint64 | µs | 資料採樣時間戳 |
| `device_id` | uint32 | — | 磁力計唯一裝置 ID |
| `magnetometer_ga[3]` | float32[3] | Gauss | FRD 機體座標系 X/Y/Z 軸磁場強度 |
| `calibration_count` | uint8 | — | 校正變更計數器 |

---

#### `vehicle_air_data`
**記錄間隔：** 200 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_sample` | uint64 | µs | 資料採樣時間戳 |
| `baro_device_id` | uint32 | — | 所選氣壓計唯一裝置 ID |
| `baro_alt_meter` | float32 | m | 以氣壓計計算的 AMSL 高度（依 QNH 修正） |
| `baro_temp_celcius` | float32 | °C | 氣壓計溫度 |
| `baro_pressure_pa` | float32 | Pa | 絕對氣壓 |
| `rho` | float32 | kg/m³ | 空氣密度 |
| `calibration_count` | uint8 | — | 校正變更計數器 |

---

#### `sensor_gps` / `vehicle_gps_position`（多實例）
**記錄間隔：** 500 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `device_id` | uint32 | — | GPS 唯一裝置 ID |
| `latitude_deg` | float64 | deg | 緯度（WGS84，可達公分級 RTK 精度） |
| `longitude_deg` | float64 | deg | 經度（WGS84） |
| `altitude_msl_m` | float64 | m | AMSL 高度 |
| `altitude_ellipsoid_m` | float64 | m | 橢球面高度 |
| `s_variance_m_s` | float32 | m/s | GPS 速度精度估算 |
| `c_variance_rad` | float32 | rad | GPS 航向精度估算 |
| `fix_type` | uint8 | — | 定位型態（0-1=無定位，2=2D，3=3D，4=DGPS，5=RTK浮點，6=RTK固定，8=外插） |
| `eph` | float32 | m | GPS 水平位置精度 |
| `epv` | float32 | m | GPS 垂直位置精度 |
| `hdop` | float32 | — | 水平精度因子（HDOP） |
| `vdop` | float32 | — | 垂直精度因子（VDOP） |
| `noise_per_ms` | int32 | — | 每毫秒 GPS 雜訊 |
| `automatic_gain_control` | uint16 | — | AGC 監測值 |
| `jamming_state` | uint8 | — | 干擾狀態（0=未知，1=正常，2=警告，3=嚴重） |
| `jamming_indicator` | int32 | — | 干擾強度指標 |
| `spoofing_state` | uint8 | — | 欺騙偵測狀態（0=未知，1=無，2=疑似，3=多訊號） |
| `vel_m_s` | float32 | m/s | GPS 地速 |
| `vel_n_m_s` | float32 | m/s | GPS 北向速度 |
| `vel_e_m_s` | float32 | m/s | GPS 東向速度 |
| `vel_d_m_s` | float32 | m/s | GPS 向下速度 |
| `cog_rad` | float32 | rad | 對地航向（移動方向，非機頭方向），範圍 [-π, π] |
| `vel_ned_valid` | bool | — | NED 速度是否有效 |
| `time_utc_usec` | uint64 | µs | UTC 時間戳（由 GPS 模組提供，冷啟動後可能為 0） |
| `satellites_used` | uint8 | — | 使用衛星數量 |
| `heading` | float32 | rad | 雙天線 GPS 機體 XYZ 軸相對 NED 的偏航角（無效為 NaN） |
| `heading_offset` | float32 | rad | 雙天線陣列的機體偏置偏航角 |
| `heading_accuracy` | float32 | rad | 偏航角精度 |
| `rtcm_injection_rate` | float32 | Hz | RTCM 訊息注入速率 |
| `selected_rtcm_instance` | uint8 | — | 使用的 RTCM 修正訊號 uORB 實例 |
| `rtcm_crc_failed` | bool | — | RTCM 訊息 CRC 失敗 |
| `rtcm_msg_used` | uint8 | — | RTCM 訊息使用狀態（0=未知，1=未使用，2=已使用） |

---

#### `airspeed`
**記錄間隔：** 1000 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `indicated_airspeed_m_s` | float32 | m/s | 指示空速（IAS） |
| `true_airspeed_m_s` | float32 | m/s | 真實空速（TAS），已濾波 |
| `air_temperature_celsius` | float32 | °C | 空氣溫度（-1000 表示未知） |
| `confidence` | float32 | — | 感測器信心度（0–1） |

---

### 4. 控制指令與致動器

#### `vehicle_attitude_setpoint` / `mc_virtual_attitude_setpoint` / `fw_virtual_attitude_setpoint`
**記錄間隔：** 50 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `roll_body` | float32 | rad | 期望機體滾轉角（NED 座標系，FW 可為 NaN） |
| `pitch_body` | float32 | rad | 期望機體俯仰角 |
| `yaw_body` | float32 | rad | 期望機體偏航角 |
| `yaw_sp_move_rate` | float32 | rad/s | 偏航設定點移動速率（由操作員命令） |
| `q_d[4]` | float32[4] | — | 期望四元數（用於四元數姿態控制） |
| `thrust_body[3]` | float32[3] | — | 機體 NED 座標系正規化推力指令 [-1, 1]（多旋翼：[0,0]為橫向，[2]為垂直；固定翼：[0]為油門） |
| `reset_integral` | bool | — | 是否重置 roll/pitch/yaw 積分項 |
| `fw_control_yaw_wheel` | bool | — | 固定翼是否以前輪轉向控制偏航（跑道起飛） |

---

#### `vehicle_rates_setpoint`
**記錄間隔：** 20 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `roll` | float32 | rad/s | 滾轉角速度設定點 |
| `pitch` | float32 | rad/s | 俯仰角速度設定點 |
| `yaw` | float32 | rad/s | 偏航角速度設定點 |
| `thrust_body[3]` | float32[3] | — | 機體 NED 座標系正規化推力指令 [-1, 1] |
| `reset_integral` | bool | — | 是否重置積分項 |

---

#### `vehicle_thrust_setpoint`（多實例，最多 2 組）
**記錄間隔：** 20 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_sample` | uint64 | µs | 資料採樣時間戳 |
| `xyz[3]` | float32[3] | — | 機體 X/Y/Z 軸推力設定點 [-1, 1] |

---

#### `vehicle_torque_setpoint`（多實例，最多 2 組）
**記錄間隔：** 20 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_sample` | uint64 | µs | 資料採樣時間戳 |
| `xyz[3]` | float32[3] | — | 機體 X/Y/Z 軸力矩設定點（正規化） |

---

#### `actuator_motors`
**記錄間隔：** 100 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_sample` | uint64 | µs | 資料採樣時間戳 |
| `reversible_flags` | uint16 | — | 可反轉電機的位元遮罩 |
| `control[12]` | float32[12] | — | 電機控制指令（12 個電機）。1=最大正推力，-1=最大負推力（不支援時映射為 NaN），NaN=解除武裝（停止） |

---

#### `actuator_servos`
**記錄間隔：** 100 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `control[8]` | float32[8] | — | 舵機控制指令，8 個通道，正規化 [-1, 1] |

---

#### `actuator_outputs`（多實例，最多 3 組）
**記錄間隔：** 100 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `noutputs` | uint32 | — | 有效輸出通道數 |
| `output[16]` | float32[16] | — | 16 通道輸出值（以各輸出的自然單位表示，例如 PWM 為 µs） |

---

#### `actuator_armed`
**記錄間隔：** 連續（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `armed` | bool | — | 系統是否已解除安全（armed） |
| `prearmed` | bool | — | 致動器安全是否解除但電機尚未 armed |
| `ready_to_arm` | bool | — | 系統是否準備好解除安全 |
| `lockdown` | bool | — | 是否強制鎖定致動器（緊急或 HIL） |
| `manual_lockdown` | bool | — | 手動油門切斷開關是否啟用 |
| `force_failsafe` | bool | — | 致動器是否強制進入失效保護位置 |
| `in_esc_calibration_mode` | bool | — | 是否在 ESC 校正模式 |

---

### 5. 系統狀態

#### `vehicle_status`
**記錄間隔：** 連續（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `armed_time` | uint64 | µs | 解除安全時間戳 |
| `takeoff_time` | uint64 | µs | 起飛時間戳 |
| `arming_state` | uint8 | — | 武裝狀態（1=DISARMED，2=ARMED） |
| `latest_arming_reason` | uint8 | — | 最近武裝原因編碼（0–13） |
| `latest_disarming_reason` | uint8 | — | 最近解除武裝原因編碼 |
| `nav_state_timestamp` | uint64 | µs | 當前導航狀態啟用時間戳 |
| `nav_state_user_intention` | uint8 | — | 用戶選擇的飛行模式 |
| `nav_state` | uint8 | — | 當前有效飛行模式（0=Manual，1=AltCtl，2=PosCtl，3=Mission，4=Loiter，5=RTL，10=Acro，14=Offboard，15=Stabilized，17=Takeoff，18=Land，19=FollowTarget，20=PrecLand，21=Orbit，22=VTOL Takeoff等） |
| `failure_detector_status` | uint16 | — | 故障偵測位元遮罩（bit0=Roll，1=Pitch，2=Alt，3=Ext，4=ESC，5=Battery，6=ImbalancedProp，7=Motor） |
| `hil_state` | uint8 | — | HIL 模擬模式（0=OFF，1=ON） |
| `vehicle_type` | uint8 | — | 載具型態（0=未知，1=旋翼，2=固定翼，3=地面車輛，4=飛艇） |
| `failsafe` | bool | — | 是否處於失效保護狀態 |
| `failsafe_and_user_took_over` | bool | — | 失效保護中且操作員已接管 |
| `gcs_connection_lost` | bool | — | 地面站資料鏈路斷開 |
| `gcs_connection_lost_counter` | uint8 | — | GCS 連線中斷事件計數 |
| `high_latency_data_link_lost` | bool | — | 高延遲資料鏈路（如衛星通訊）中斷 |
| `is_vtol` | bool | — | 是否為 VTOL 載具 |
| `is_vtol_tailsitter` | bool | — | 是否為尾坐式 VTOL（過渡時旋轉 90°） |
| `in_transition_mode` | bool | — | VTOL 是否正在過渡 |
| `in_transition_to_fw` | bool | — | VTOL 是否正在從多旋翼轉向固定翼 |
| `system_type` | uint8 | — | MAVLink MAV_TYPE 系統型態 |
| `system_id` | uint8 | — | MAVLink system ID |
| `component_id` | uint8 | — | MAVLink component ID |
| `safety_button_available` | bool | — | 是否有安全按鈕 |
| `safety_off` | bool | — | 安全鎖是否解除 |
| `power_input_valid` | bool | — | 電源輸入是否有效 |
| `usb_connected` | bool | — | USB 是否已連接 |
| `avoidance_system_required` | bool | — | 避障系統是否需要啟用 |
| `avoidance_system_valid` | bool | — | 避障系統是否正常 |
| `pre_flight_checks_pass` | bool | — | 所有飛前檢查是否通過 |

---

#### `vehicle_land_detected`
**記錄間隔：** 連續（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `freefall` | bool | — | 是否處於自由落體 |
| `ground_contact` | bool | — | 是否接觸地面但未完全降落（第1階段） |
| `maybe_landed` | bool | — | 是否可能已降落（第2階段） |
| `landed` | bool | — | 是否確認已降落（第3階段） |
| `in_ground_effect` | bool | — | 是否處於地面效應（氣壓高估） |
| `in_descend` | bool | — | 是否正在下降 |
| `has_low_throttle` | bool | — | 是否低油門 |
| `vertical_movement` | bool | — | 是否有垂直運動 |
| `horizontal_movement` | bool | — | 是否有水平運動 |
| `rotational_movement` | bool | — | 是否有旋轉運動 |
| `close_to_ground_or_skipped_check` | bool | — | 是否接近地面或跳過檢查 |
| `at_rest` | bool | — | 是否靜止 |

---

#### `vehicle_control_mode`
**記錄間隔：** 連續（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `flag_armed` | bool | — | 已武裝（等同 actuator_armed.armed） |
| `flag_multicopter_position_control_enabled` | bool | — | 多旋翼位置控制是否啟用 |
| `flag_control_manual_enabled` | bool | — | 手動輸入是否混入 |
| `flag_control_auto_enabled` | bool | — | 機載自動駕駛是否啟用 |
| `flag_control_offboard_enabled` | bool | — | Offboard 控制是否啟用 |
| `flag_control_rates_enabled` | bool | — | 角速度穩定化是否啟用 |
| `flag_control_attitude_enabled` | bool | — | 姿態穩定化是否啟用 |
| `flag_control_acceleration_enabled` | bool | — | 加速度控制是否啟用 |
| `flag_control_velocity_enabled` | bool | — | 水平速度控制是否啟用 |
| `flag_control_position_enabled` | bool | — | 位置控制是否啟用 |
| `flag_control_altitude_enabled` | bool | — | 高度控制是否啟用 |
| `flag_control_climb_rate_enabled` | bool | — | 爬升率控制是否啟用 |
| `flag_control_termination_enabled` | bool | — | 飛行終止是否啟用 |
| `flag_control_allocation_enabled` | bool | — | 控制分配是否啟用 |

---

#### `cpuload`
**記錄間隔：** 連續（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `load` | float32 | — | 處理器負載（0 到 1） |
| `ram_usage` | float32 | — | RAM 使用率（0 到 1） |

---

#### `system_power`
**記錄間隔：** 500 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `voltage5v_v` | float32 | V | 5V 周邊電源匯流排電壓 |
| `sensors3v3[4]` | float32[4] | V | 感測器 3.3V 電源匯流排電壓（4 路） |
| `sensors3v3_valid` | uint8 | — | 感測器 3.3V 電源有效性位元遮罩 |
| `usb_connected` | uint8 | — | USB 是否連接（1=是） |
| `brick_valid` | uint8 | — | 電源磚（BrickN）電源良好位元遮罩 |
| `usb_valid` | uint8 | — | USB 電源是否有效（1=是） |
| `servo_valid` | uint8 | — | 舵機電源是否良好（1=是） |
| `periph_5v_oc` | uint8 | — | 5V 周邊過電流（1=是） |
| `hipower_5v_oc` | uint8 | — | 高功率 5V 周邊過電流（1=是） |
| `comp_5v_valid` | uint8 | — | 伴侶電腦 5V 電源有效 |
| `can1_gps1_5v_valid` | uint8 | — | CAN1/GPS1 5V 電源有效 |

---

### 6. 電池

#### `battery_status`（多實例，最多 2 組）
**記錄間隔：** 200 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `connected` | bool | — | 電池是否連接（依電壓閾值判斷） |
| `voltage_v` | float32 | V | 電池電壓（0 表示未知） |
| `voltage_filtered_v` | float32 | V | 濾波後電池電壓 |
| `current_a` | float32 | A | 電池電流（-1 表示未知） |
| `current_filtered_a` | float32 | A | 濾波後電池電流 |
| `current_average_a` | float32 | A | 平均電池電流 |
| `discharged_mah` | float32 | mAh | 已放電量（-1 表示未知） |
| `remaining` | float32 | — | 剩餘電量比例（1 到 0，-1 未知） |
| `scale` | float32 | — | 功率縮放係數（≥1，-1 未知） |
| `time_remaining_s` | float32 | s | 依平均放電率預測的剩餘使用時間（NaN 表示未知） |
| `temperature` | float32 | °C | 電池溫度（NaN 表示未知） |
| `cell_count` | uint8 | — | 電池芯數量 |
| `source` | uint8 | — | 電源來源（0=電源模組，1=外部，2=ESC） |
| `priority` | uint8 | — | 零起始優先順序（對應電源控制器連接埠） |
| `capacity` | uint16 | mAh | 電池實際容量 |
| `cycle_count` | uint16 | — | 放電循環次數 |
| `average_time_to_empty` | uint16 | min | 依平均放電率預測剩餘時間 |
| `serial_number` | uint16 | — | 電池序號 |
| `state_of_health` | uint16 | % | 健康狀態（FullChargeCapacity/DesignCapacity × 100%） |
| `max_error` | uint16 | % | 電量計算最大誤差估計（1–100%） |
| `id` | uint8 | — | 電池 ID（1 起始，應在載具生命週期內唯一且一致） |
| `voltage_cell_v[14]` | float32[14] | V | 各電芯電壓（0 表示未知） |
| `max_cell_voltage_delta` | float32 | V | 各電芯電壓最大差值 |
| `is_powering_off` | bool | — | 即將斷電指示 |
| `is_required` | bool | — | 是否在解除安全前必須連接 |
| `faults` | uint16 | — | 智慧電池故障位元遮罩 |
| `custom_faults` | uint32 | — | 製造商自定義故障位元遮罩 |
| `warning` | uint8 | — | 電池警告等級（0=無，1=低電，2=嚴重，3=緊急降落，4=故障，6=不健康，7=充電中） |
| `mode` | uint8 | — | 電池模式（0=正常，1=自動放電，2=熱插拔） |
| `average_power` | float32 | W | 當前放電平均功率 |
| `available_energy` | float32 | Wh | 預測剩餘電量 |
| `full_charge_capacity_wh` | float32 | Wh | 補償後滿充容量 |
| `remaining_capacity_wh` | float32 | Wh | 補償後剩餘容量 |
| `design_capacity` | float32 | Wh | 設計容量 |
| `average_time_to_full` | uint16 | min | 預測充滿所需時間 |
| `over_discharge_count` | uint16 | — | 過放電次數 |
| `nominal_voltage` | float32 | V | 電池組標稱電壓 |

---

### 7. 估算器（EKF2）

#### `estimator_status`（多實例）
**記錄間隔：** 200 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `output_tracking_error[3]` | float32[3] | rad, m/s, m | 輸出預測器追蹤誤差（角度、速度、位置） |
| `gps_check_fail_flags` | uint16 | — | GPS 檢查失敗位元遮罩（bit0=定位型態不足，1=衛星不足，2=PDOP超限，3=水平誤差超限，4=垂直誤差超限，5=速度誤差超限，6=水平漂移超限，7=垂直漂移超限，8=水平速度超限，9=垂直速度差異超限） |
| `control_mode_flags` | uint64 | — | EKF 邏輯狀態位元遮罩（bit0=傾斜校準完成，1=偏航校準完成，2=GPS 融合中，3=光流融合中，4=磁偏航融合中，5=3軸磁融合中，6=磁偏角融合中，7=在飛行中...等共29位元） |
| `filter_fault_flags` | uint32 | — | EKF 內部故障位元遮罩（共18個故障標記） |
| `pos_horiz_accuracy` | float32 | m | 相對估算原點的水平位置精度（1-σ） |
| `pos_vert_accuracy` | float32 | m | 相對估算原點的垂直位置精度（1-σ） |
| `innovation_check_flags` | uint16 | — | 創新一致性檢查位元遮罩（bit0=速度拒絕，1=水平位置拒絕，2=垂直位置拒絕，3=磁X拒絕，4=磁Y拒絕，5=磁Z拒絕，6=偏航拒絕，7=空速拒絕，8=側滑拒絕，9=離地高度拒絕，10=光流X拒絕，11=光流Y拒絕） |
| `mag_test_ratio` | float32 | — | 磁力計最大創新/測試閾值比 |
| `vel_test_ratio` | float32 | — | 速度最大創新/測試閾值比 |
| `pos_test_ratio` | float32 | — | 水平位置最大創新/測試閾值比 |
| `hgt_test_ratio` | float32 | — | 垂直位置創新/測試閾值比 |
| `tas_test_ratio` | float32 | — | 真實空速創新/測試閾值比 |
| `hagl_test_ratio` | float32 | — | 離地高度創新/測試閾值比 |
| `beta_test_ratio` | float32 | — | 合成側滑創新/測試閾值比 |
| `solution_status_flags` | uint16 | — | 可用於飛控的輸出品質位元遮罩（bit0=姿態良好，1=水平速度良好，2=垂直速度良好，3=相對水平位置良好，4=絕對水平位置良好，5=絕對垂直位置良好，6=離地高度良好，7=固定位置模式，8=可提供相對位置，9=可提供絕對位置，10=偵測到GPS故障，11=偵測到加速度計不良） |
| `reset_count_vel_ne` | uint8 | — | 水平速度重置次數 |
| `reset_count_vel_d` | uint8 | — | 垂直速度重置次數 |
| `reset_count_pos_ne` | uint8 | — | 水平位置重置次數 |
| `reset_count_pod_d` | uint8 | — | 垂直位置重置次數 |
| `reset_count_quat` | uint8 | — | 四元數重置次數 |
| `time_slip` | float32 | s | EKF 慣性計算相對系統時鐘累積偏移 |
| `pre_flt_fail_innov_heading` | bool | — | 飛前偏航創新檢查失敗 |
| `pre_flt_fail_innov_vel_horiz` | bool | — | 飛前水平速度創新檢查失敗 |
| `pre_flt_fail_innov_vel_vert` | bool | — | 飛前垂直速度創新檢查失敗 |
| `pre_flt_fail_innov_height` | bool | — | 飛前高度創新檢查失敗 |
| `pre_flt_fail_mag_field_disturbed` | bool | — | 飛前磁場干擾偵測 |
| `accel_device_id` | uint32 | — | EKF 使用的加速度計裝置 ID |
| `gyro_device_id` | uint32 | — | EKF 使用的陀螺儀裝置 ID |
| `baro_device_id` | uint32 | — | EKF 使用的氣壓計裝置 ID |
| `mag_device_id` | uint32 | — | EKF 使用的磁力計裝置 ID |
| `mag_inclination_deg` | float32 | deg | 估算磁傾角 |
| `mag_inclination_ref_deg` | float32 | deg | 參考磁傾角 |
| `mag_strength_gs` | float32 | Gauss | 估算磁場強度 |
| `mag_strength_ref_gs` | float32 | Gauss | 參考磁場強度 |

---

#### `estimator_states`（多實例）
**記錄間隔：** 1000 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `states[24]` | float32[24] | 混合 | EKF 內部狀態向量（24 個狀態：四元數4、速度3、位置3、陀螺偏差3、加速度偏差3、磁場地球座標3、磁場機體座標3、風速2，共24） |
| `n_states` | uint8 | — | 實際使用狀態數 |
| `covariances[23]` | float32[23] | 混合 | 協方差矩陣對角元素（各狀態不確定度） |

---

#### `estimator_innovations` / `estimator_innovation_variances` / `estimator_innovation_test_ratios`
**記錄間隔：** 500 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `gps_hvel[2]` | float32[2] | m/s | GPS 水平速度創新（創新值/創新方差） |
| `gps_vvel` | float32 | m/s | GPS 垂直速度創新 |
| `gps_hpos[2]` | float32[2] | m | GPS 水平位置創新 |
| `gps_vpos` | float32 | m | GPS 垂直位置創新 |
| `ev_hvel[2]` | float32[2] | m/s | 外部視覺水平速度創新 |
| `ev_vvel` | float32 | m/s | 外部視覺垂直速度創新 |
| `ev_hpos[2]` | float32[2] | m | 外部視覺水平位置創新 |
| `ev_vpos` | float32 | m | 外部視覺垂直位置創新 |
| `rng_vpos` | float32 | m | 測距儀高度創新 |
| `baro_vpos` | float32 | m | 氣壓計高度創新 |
| `aux_hvel[2]` | float32[2] | m/s | 輔助水平速度創新（降落目標量測） |
| `flow[2]` | float32[2] | rad/s | 光學流量創新 |
| `terr_flow[2]` | float32[2] | rad/s | 地形估算光學流量創新 |
| `heading` | float32 | rad | 偏航/航向創新 |
| `mag_field[3]` | float32[3] | Gauss | 地球磁場創新 |
| `gravity[3]` | float32[3] | m/s² | 加速度計重力向量創新 |
| `drag[2]` | float32[2] | m/s² | 阻力比力創新 |
| `airspeed` | float32 | m/s | 空速創新 |
| `beta` | float32 | rad | 合成側滑創新 |
| `hagl` | float32 | m | 離地高度創新 |
| `hagl_rate` | float32 | m/s | 離地高度變化率創新 |

---

#### `estimator_sensor_bias`（多實例）
**記錄間隔：** 連續（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `gyro_device_id` | uint32 | — | 陀螺儀裝置 ID |
| `gyro_bias[3]` | float32[3] | rad/s | 機體座標系陀螺儀在線偏差 |
| `gyro_bias_limit` | float32 | rad/s | 陀螺儀在線偏差最大幅度限制 |
| `gyro_bias_variance[3]` | float32[3] | (rad/s)² | 陀螺儀偏差方差 |
| `gyro_bias_valid` | bool | — | 陀螺儀偏差是否有效 |
| `gyro_bias_stable` | bool | — | 陀螺儀偏差是否穩定（可用於校正） |
| `accel_device_id` | uint32 | — | 加速度計裝置 ID |
| `accel_bias[3]` | float32[3] | m/s² | 機體座標系加速度計在線偏差 |
| `accel_bias_limit` | float32 | m/s² | 加速度計在線偏差最大幅度限制 |
| `accel_bias_variance[3]` | float32[3] | (m/s²)² | 加速度計偏差方差 |
| `accel_bias_valid` | bool | — | 加速度計偏差是否有效 |
| `accel_bias_stable` | bool | — | 加速度計偏差是否穩定 |
| `mag_device_id` | uint32 | — | 磁力計裝置 ID |
| `mag_bias[3]` | float32[3] | Gauss | 機體座標系磁力計在線偏差 |
| `mag_bias_limit` | float32 | Gauss | 磁力計在線偏差最大幅度限制 |
| `mag_bias_variance[3]` | float32[3] | Gauss² | 磁力計偏差方差 |
| `mag_bias_valid` | bool | — | 磁力計偏差是否有效 |
| `mag_bias_stable` | bool | — | 磁力計偏差是否穩定 |

---

### 8. 控制器狀態

#### `rate_ctrl_status`（多實例，最多 2 組）
**記錄間隔：** 200 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `rollspeed_integ` | float32 | — | 滾轉角速度控制器積分項 |
| `pitchspeed_integ` | float32 | — | 俯仰角速度控制器積分項 |
| `yawspeed_integ` | float32 | — | 偏航角速度控制器積分項 |
| `wheel_rate_integ` | float32 | — | 前輪轉向速率積分項（固定翼選用） |

---

#### `control_allocator_status`（多實例，最多 2 組）
**記錄間隔：** 200 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `torque_setpoint_achieved` | bool | — | 力矩設定點是否成功分配 |
| `unallocated_torque[3]` | float32[3] | — | 未能分配的力矩（設定值減分配值） |
| `thrust_setpoint_achieved` | bool | — | 推力設定點是否成功分配 |
| `unallocated_thrust[3]` | float32[3] | — | 未能分配的推力 |
| `actuator_saturation[16]` | int8[16] | — | 各致動器飽和狀態（0=正常，1=上動態飽和，2=上限飽和，-1=下動態飽和，-2=下限飽和） |
| `handled_motor_failure_mask` | uint16 | — | 已從分配矩陣移除的故障電機位元遮罩 |

---

#### `hover_thrust_estimate`
**記錄間隔：** 100 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `hover_thrust` | float32 | — | 估算懸停推力（[0.1, 0.9]，即佔最大推力之比例） |
| `hover_thrust_var` | float32 | — | 懸停推力估算方差 |
| `accel_innov` | float32 | m/s² | 最近一次加速度融合創新值 |
| `accel_innov_var` | float32 | (m/s²)² | 加速度融合創新方差 |
| `accel_innov_test_ratio` | float32 | — | 正規化創新平方測試比 |
| `accel_noise_var` | float32 | (m/s²)² | 由創新殘差估算的垂直加速度雜訊方差 |
| `valid` | bool | — | 估算是否有效 |

---

#### `failure_detector_status`
**記錄間隔：** 100 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `fd_roll` | bool | — | 滾轉角異常故障 |
| `fd_pitch` | bool | — | 俯仰角異常故障 |
| `fd_alt` | bool | — | 高度異常故障 |
| `fd_ext` | bool | — | 外部故障觸發 |
| `fd_arm_escs` | bool | — | ESC 武裝故障 |
| `fd_battery` | bool | — | 電池故障 |
| `fd_imbalanced_prop` | bool | — | 螺旋槳不平衡故障 |
| `fd_motor` | bool | — | 電機故障 |
| `imbalanced_prop_metric` | float32 | — | 螺旋槳不平衡低通濾波指標 |
| `motor_failure_mask` | uint16 | — | 關鍵電機故障位元遮罩 |

---

### 9. 導航與路徑規劃

#### `trajectory_setpoint`
**記錄間隔：** 200 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `position[3]` | float32[3] | m | NED 座標系位置設定點（NaN 表示不控制該軸） |
| `velocity[3]` | float32[3] | m/s | NED 座標系速度設定點 |
| `acceleration[3]` | float32[3] | m/s² | NED 座標系加速度設定點 |
| `jerk[3]` | float32[3] | m/s³ | NED 座標系加加速度（僅供記錄） |
| `yaw` | float32 | rad | 期望偏航角，範圍 [-π, +π] |
| `yawspeed` | float32 | rad/s | NED 座標系 z 軸偏航角速度 |

---

#### `position_setpoint_triplet`
**記錄間隔：** 200 ms（預設）

包含三個 `PositionSetpoint`（previous、current、next），各包含全球（WGS84）座標位置航點定義，用於自動飛行任務規劃。

---

#### `wind` / `estimator_wind`（多實例）
**記錄間隔：** 1000 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `windspeed_north` | float32 | m/s | 北方風速分量 |
| `windspeed_east` | float32 | m/s | 東方風速分量 |
| `variance_north` | float32 | (m/s)² | 北方風速估算誤差方差（0=無不確定度） |
| `variance_east` | float32 | (m/s)² | 東方風速估算誤差方差 |
| `tas_innov` | float32 | m/s | 真實空速創新值 |
| `tas_innov_var` | float32 | (m/s)² | 真實空速創新方差 |
| `beta_innov` | float32 | rad | 側滑量測創新值 |
| `beta_innov_var` | float32 | rad² | 側滑創新方差 |

---

### 10. 手動操控輸入

#### `manual_control_setpoint`
**記錄間隔：** 200 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `valid` | bool | — | 資料是否有效 |
| `data_source` | uint8 | — | 輸入來源（1=RC，2-7=MAVLink 實例 0-5） |
| `roll` | float32 | — | 滾轉搖桿 [-1, 1]（負=左，正=右） |
| `pitch` | float32 | — | 俯仰搖桿 [-1, 1]（正=前進，負仰頭） |
| `yaw` | float32 | — | 偏航搖桿 [-1, 1]（正=順時針） |
| `throttle` | float32 | — | 油門搖桿 [-1, 1]（-1=最小，1=最大） |
| `flaps` | float32 | — | 襟翼開關/旋鈕 [-1, 1] |
| `aux1`~`aux6` | float32 | — | 輔助通道 1–6 [-1, 1] |
| `sticks_moving` | bool | — | 搖桿是否正在移動 |

---

#### `input_rc`
**記錄間隔：** 500 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `timestamp_last_signal` | uint64 | µs | 最後有效接收時間 |
| `channel_count` | uint8 | — | 實際接收到的通道數 |
| `rssi` | int32 | — | 接收訊號強度（<0=未定義，0=無訊號，100=滿格） |
| `rc_failsafe` | bool | — | RC 失效保護旗標（發射機故障或失距） |
| `rc_lost` | bool | — | RC 接收器連線狀態（True=無訊號幀） |
| `rc_lost_frame_count` | uint16 | — | 遺失 RC 訊框數 |
| `rc_total_frame_count` | uint16 | — | 總 RC 訊框數 |
| `values[18]` | uint16[18] | µs | 各通道脈衝寬度（最多 18 通道，如 SBUS） |
| `input_source` | uint8 | — | 輸入來源類型（PPM/SBUS/Spektrum/CRSF 等） |
| `link_quality` | int8 | % | 鏈路品質（0–100%，-1=無效） |
| `rssi_dbm` | float32 | dBm | 實際 RSSI（NaN=無效） |

---

### 11. TVMD 特有 Topic

#### `control_allocation_meta_data`
**記錄間隔：** 連續（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `control_sp[6]` | float32[6] | — | 6 自由度控制設定點（力/力矩） |
| `allocated_control[6]` | float32[6] | — | 實際分配出的 6 自由度控制量 |
| `f_x[16]` | float32[16] | — | 4 次迭代 × 4 個智能體的 X 軸力分量 |
| `f_y[16]` | float32[16] | — | 4 次迭代 × 4 個智能體的 Y 軸力分量 |
| `f_z[16]` | float32[16] | — | 4 次迭代 × 4 個智能體的 Z 軸力分量 |
| `t_x[16]` | float32[16] | — | 4 次迭代 × 4 個智能體的 X 軸力矩分量 |
| `t_y[16]` | float32[16] | — | 4 次迭代 × 4 個智能體的 Y 軸力矩分量 |
| `t_z[16]` | float32[16] | — | 4 次迭代 × 4 個智能體的 Z 軸力矩分量 |
| `t_min[16]` | float32[16] | — | 4 次迭代 × 4 個智能體的最小力矩 |
| `saturated_idx[4]` | int8[4] | — | 4 個智能體的飽和索引 |
| `increment[4]` | float32[4] | — | 4 個智能體的增量 |

> **說明：** 此 Topic 為 TVMD（Tilted-rotor Vehicle with Multiple Degrees-of-freedom）專用的分散式控制分配演算法中繼資料。記錄了每次迭代中各智能體計算的力/力矩分配結果，用於分析 IFO（Internal Force Optimizer）的收斂過程。

---

#### `attitude_planner_meta_data`
**記錄間隔：** 連續（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `q_d[4]` | float32[4] | — | 四元數姿態控制的期望四元數 |
| `sat_thrust_body[3]` | float32[3] | — | 機體 NED 座標系正規化飽和推力指令 [-1, 1] |
| `coplan_vector[3]` | float32[3] | — | 機體 NWU 座標系共面向量 k |
| `theta` | float32 | rad | 偏轉姿態角 |
| `is_feasible` | uint8 | — | 參考姿態是否可行（1=可行，0=不可行） |
| `bisection_count` | uint8 | — | 二分法迭代次數 |

> **說明：** 此 Topic 為 TVMD 姿態規劃器的中繼資料，記錄了 PFA（Planar Force Allocation）控制器在計算期望姿態時的二分法求解過程，包含共面向量、偏轉角及可行性判斷。

---

#### `offboard_control_mode`
**記錄間隔：** 100 ms（預設）

| 欄位 | 型態 | 單位 | 說明 |
|---|---|---|---|
| `timestamp` | uint64 | µs | 自系統啟動以來的時間 |
| `position` | bool | — | Offboard 位置控制是否啟用 |
| `velocity` | bool | — | Offboard 速度控制是否啟用 |
| `acceleration` | bool | — | Offboard 加速度控制是否啟用 |
| `attitude` | bool | — | Offboard 姿態控制是否啟用 |
| `body_rate` | bool | — | Offboard 機體角速率控制是否啟用 |
| `actuator` | bool | — | Offboard 直接致動器控制是否啟用 |

---

## 附錄：日誌 Profile 包含的 Topic 摘要

### DEFAULT Profile（完整清單）

**單實例 Topic（含記錄間隔）：**

| Topic | 間隔 (ms) | 說明 |
|---|---|---|
| `action_request` | 連續 | 動作請求 |
| `actuator_armed` | 連續 | 武裝狀態 |
| `actuator_controls_status_0` | 300 | 致動器控制狀態 |
| `actuator_motors` | 100 | 電機控制指令 |
| `actuator_servos` | 100 | 舵機控制指令 |
| `airspeed` | 1000 | 空速 |
| `airspeed_validated` | 200 | 驗證後空速 |
| `attitude_planner_meta_data` | 連續 | 姿態規劃器中繼資料（TVMD） |
| `autotune_attitude_control_status` | 100 | 自動調參狀態 |
| `camera_capture` | 連續 | 相機觸發捕獲 |
| `camera_trigger` | 連續 | 相機觸發 |
| `cellular_status` | 200 | 蜂巢網路狀態 |
| `commander_state` | 連續 | 指揮官狀態機 |
| `control_allocation_meta_data` | 連續 | 控制分配中繼資料（TVMD） |
| `cpuload` | 連續 | CPU 負載 |
| `estimator_baro_bias` | 500 | 氣壓計偏差 |
| `estimator_event_flags` | 連續 | EKF 事件旗標 |
| `estimator_gnss_hgt_bias` | 500 | GNSS 高度偏差 |
| `estimator_gps_status` | 1000 | EKF GPS 狀態 |
| `estimator_innovation_test_ratios` | 500 | 創新測試比 |
| `estimator_innovation_variances` | 500 | 創新方差 |
| `estimator_innovations` | 500 | 創新值 |
| `estimator_optical_flow_vel` | 200 | 光流速度估算 |
| `estimator_rng_hgt_bias` | 500 | 測距儀高度偏差 |
| `estimator_ev_pos_bias` | 500 | 外部視覺位置偏差 |
| `estimator_selector_status` | 連續 | 估算器選擇器狀態 |
| `estimator_sensor_bias` | 連續 | 感測器偏差 |
| `estimator_states` | 1000 | EKF 狀態向量 |
| `estimator_status` | 200 | EKF 狀態 |
| `estimator_status_flags` | 連續 | EKF 狀態旗標 |
| `failure_detector_status` | 100 | 故障偵測狀態 |
| `failsafe_flags` | 連續 | 失效保護旗標 |
| `home_position` | 連續 | 返航點位置 |
| `hover_thrust_estimate` | 100 | 懸停推力估算 |
| `input_rc` | 500 | RC 輸入 |
| `manual_control_setpoint` | 200 | 手動控制設定點 |
| `manual_control_switches` | 連續 | 手動控制開關 |
| `mission_result` | 連續 | 任務執行結果 |
| `navigator_mission_item` | 連續 | 當前任務項目 |
| `npfg_status` | 100 | NPFG 導引狀態（固定翼） |
| `offboard_control_mode` | 100 | Offboard 控制模式 |
| `onboard_computer_status` | 10 | 機載電腦狀態 |
| `parameter_update` | 連續 | 參數更新事件 |
| `position_controller_landing_status` | 100 | 著陸控制器狀態 |
| `position_controller_status` | 500 | 位置控制器狀態 |
| `position_setpoint_triplet` | 200 | 位置設定點三元組 |
| `radio_status` | 連續 | 無線電狀態 |
| `rtl_time_estimate` | 1000 | RTL 時間估算 |
| `sensor_combined` | 連續 | 合併感測器資料 |
| `sensor_selection` | 連續 | 感測器選擇 |
| `sensors_status_imu` | 200 | IMU 感測器狀態 |
| `system_power` | 500 | 系統電源 |
| `trajectory_setpoint` | 200 | 軌跡設定點 |
| `transponder_report` | 連續 | ADS-B 應答機報告 |
| `vehicle_acceleration` | 50 | 載具加速度 |
| `vehicle_air_data` | 200 | 氣壓/溫度資料 |
| `vehicle_angular_velocity` | 20 | 角速度 |
| `vehicle_attitude` | 50 | 姿態四元數 |
| `vehicle_attitude_setpoint` | 50 | 姿態設定點 |
| `vehicle_command` | 連續 | 載具命令 |
| `vehicle_command_ack` | 連續 | 載具命令確認 |
| `vehicle_constraints` | 1000 | 載具約束 |
| `vehicle_control_mode` | 連續 | 控制模式旗標 |
| `vehicle_global_position` | 200 | 全球位置（融合） |
| `vehicle_gps_position` | 500 | GPS 原始位置 |
| `vehicle_land_detected` | 連續 | 著陸偵測 |
| `vehicle_local_position` | 100 | 區域位置（融合） |
| `vehicle_local_position_setpoint` | 100 | 區域位置設定點 |
| `vehicle_magnetometer` | 200 | 磁力計資料 |
| `vehicle_optical_flow` | 500 | 光學流量 |
| `vehicle_rates_setpoint` | 20 | 角速率設定點 |
| `vehicle_roi` | 1000 | 感興趣區域 |
| `vehicle_status` | 連續 | 載具系統狀態 |
| `wind` | 1000 | 風速估算 |
| `yaw_estimator_status` | 1000 | 偏航估算器狀態 |

**多實例 Topic：**

| Topic | 間隔 (ms) | 最大實例 | 說明 |
|---|---|---|---|
| `actuator_outputs` | 100 | 3 | 致動器輸出 |
| `airspeed_wind` | 1000 | 4 | 各空速感測器風速估算 |
| `battery_status` | 200 | 2 | 電池狀態 |
| `control_allocator_status` | 200 | 2 | 控制分配器狀態 |
| `differential_pressure` | 1000 | 2 | 差壓（皮托管） |
| `distance_sensor` | 1000 | 2 | 距離感測器 |
| `estimator_attitude` | 500 | 6 | EKF 姿態（多估算器） |
| `estimator_global_position` | 1000 | 6 | EKF 全球位置 |
| `estimator_local_position` | 500 | 6 | EKF 區域位置 |
| `estimator_wind` | 1000 | 6 | EKF 風速 |
| `rate_ctrl_status` | 200 | 2 | 角速度控制器積分 |
| `rpm` | 200 | 1 | 轉速 |
| `sensor_accel` | 1000 | 4 | 加速度計原始資料 |
| `sensor_baro` | 1000 | 4 | 氣壓計原始資料 |
| `sensor_gps` | 1000 | 2 | GPS 原始資料 |
| `sensor_gnss_relative` | 1000 | 1 | GNSS 相對定位 |
| `sensor_gyro` | 1000 | 4 | 陀螺儀原始資料 |
| `sensor_hygrometer` | 500 | 4 | 濕度計 |
| `sensor_mag` | 1000 | 4 | 磁力計原始資料 |
| `sensor_optical_flow` | 1000 | 2 | 光流感測器 |
| `telemetry_status` | 1000 | 4 | 遙測鏈路狀態 |
| `vehicle_imu` | 500 | 4 | IMU 積分資料 |
| `vehicle_imu_status` | 1000 | 4 | IMU 狀態統計 |
| `vehicle_magnetometer` | 500 | 4 | 多實例磁力計 |
| `vehicle_thrust_setpoint` | 20 | 2 | 推力設定點 |
| `vehicle_torque_setpoint` | 20 | 2 | 力矩設定點 |

---

## 座標系說明

| 縮寫 | 全名 | 說明 |
|---|---|---|
| NED | North-East-Down | 地球固定座標系：北為+X，東為+Y，向下為+Z |
| FRD | Forward-Right-Down | 機體座標系：前為+X，右為+Y，向下為+Z |
| NWU | North-West-Up | 部分 TVMD 計算使用（北為+X，西為+Y，向上為+Z） |
| WGS84 | World Geodetic System 1984 | GPS 緯度/經度/高度基準橢球 |
| AMSL | Above Mean Sea Level | 相對平均海平面高度 |

---

*本文件由自動化腳本從 `/src/modules/logger/logged_topics.cpp` 及 `/msg/*.msg` 生成整理，日期：2026-05-08。*
