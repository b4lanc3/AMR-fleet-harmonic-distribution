# AMR Fleet Harmonic Distribution

Mô phỏng **đội 10–15 AMR (tối đa 30)** vận chuyển nội bộ trong một nhà máy hoàn chỉnh 120 m × 72 m, chạy trên
**ROS 2 Jazzy + Gazebo Harmonic** (tương thích Jetty), **Nav2 + AMCL** trên từng robot và **RViz2**. Hệ thống có:

* **Nhà máy đầy đủ**: tường panel cách nhiệt, dải cửa sổ, cửa cuộn, cột thép, dầm mái, đèn (có nguồn sáng thật),
  sàn epoxy chia khu và vạch làn, kệ đầy thùng/bin linh kiện, kệ pallet thành phẩm, văn phòng 2 tầng, xe tải ở dock,
  bình cứu hoả, biển báo… và **8 dây chuyền chạy liên tục**: 4 dây chuyền sản xuất + 4 dây chuyền đóng gói, sản phẩm
  chạy trên băng tải trong Gazebo và băng tải dừng khi MES báo dây chuyền thiếu đầu vào hoặc nghẽn đầu ra.
* **Nav2 + AMCL trên từng robot**: AMCL định vị trên bản đồ cắt từ chính world Gazebo, `controller_server`
  (Regulated Pure Pursuit + local costmap từ lidar 2D và camera 3D) lái robot theo đoạn route được cấp quyền,
  `collision_monitor` dừng/giảm tốc theo lidar + camera 3D.
* **Fleet manager**: giao task theo MES mô phỏng, điều phối giao thông theo **hàng ưu tiên tính theo task**, robot
  ưu tiên thấp né rồi bám lại route cũ, gỡ deadlock, quản lý trạm sạc 5 dock.
* **Dashboard web** (http://localhost:8080): robot số mấy đang làm gì, đi route nào, ETA, task tiếp theo, hàng ưu
  tiên, pin, đang chờ hoặc nhường ai; bản đồ trực tiếp, 8 dây chuyền, trạm sạc, hàng đợi task, nhật ký điều phối.

![Dashboard: bản đồ trực tiếp, chi tiết AMR-02 (task, lộ trình, ETA, task tiếp theo, lý do hàng ưu tiên)](docs/dashboard.png)

| Gazebo Harmonic: toàn cảnh nhà máy | Gazebo: dây chuyền sản xuất / đóng gói, sản phẩm chạy trên băng tải |
|---|---|
| ![gazebo](docs/gazebo_overview.png) | ![lines](docs/gazebo_lines.png) |

| Gazebo + Nav2: AMR chở thùng SP (khay nâng) rời B3_OUT sang dây chuyền đóng gói | Gazebo + Nav2: AMR-02 (P2) né vào yield bay trên Đường Bắc 2 chiều, AMR-01 (P1) đi qua, sau đó AMR-02 bám lại route cũ |
|---|---|
| ![closeup](docs/gazebo_closeup.png) | ![yield](docs/gazebo_yield.png) |

![RViz2: layout, đồ thị giao thông, robot, lidar, nhóm hiển thị Nav2, ảnh camera 3D](docs/rviz_overview.png)

| Yêu cầu | Cách đáp ứng |
|---|---|
| Fleet ≥ 10–15 AMR | `num_robots:=12` (mặc định), 30 chỗ đỗ nên tối đa 30 robot. Đã kiểm thử 10/12/15 robot (mô phỏng headless 30 phút) và Gazebo + Nav2 với 2–3 robot (container không GPU) |
| 1 trạm sạc tự động cho ≥ 5 robot | Trạm sạc 5 dock (CH1–CH5) giữa nhà máy, có hàng đợi sạc và cơ chế nhường dock |
| Nhà máy rộng, 4 khu | Khu A kho linh kiện (5 trạm cấp) · Khu B **4 dây chuyền sản xuất** · Khu C **4 dây chuyền đóng gói** · Khu D kho thành phẩm (6 trạm) |
| Mỗi khu dây chuyền 3–5 dây hoạt động liên tục, mô phỏng cả dây chuyền | 4 + 4 dây chuyền, mỗi dây có đủ trạm máy (xem mục 2). Sản phẩm di chuyển liên tục trên băng tải trong Gazebo (system `LineAnimator`), dừng khi MES báo dây chuyền dừng |
| Map nhà máy đầy đủ (tường, sàn, hàng hoá, đèn…) | World sinh từ layout: ~5000 chi tiết (gộp thành mesh để render nhanh), 15 đèn điểm + đèn trần phát sáng. Bản đồ AMCL cắt đúng ở độ cao lidar |
| Chu trình sản xuất | A (kit linh kiện) → B_IN → dây chuyền SX → SP chưa đóng gói xếp vào **thùng** ở B_OUT → C_IN → đóng gói từng SP → C_OUT → D |
| Chạy Nav2 và AMCL | Mỗi robot: AMCL + controller_server + collision_monitor (+ planner/behavior/bt_navigator nếu `nav2_full_stack:=true`), một map_server chung |
| 2 bánh chủ động + 4 bánh tự do, lidar 2D, camera 3D | URDF `amr_description`: diff-drive, 4 caster, một `gpu_lidar` **270°** ở mặt trước, `rgbd_camera` (ảnh + depth + point cloud) |
| Xem robot số mấy làm gì, route nào, ETA, task tiếp theo | Dashboard web, nhãn trong RViz, `fleet_cli status`, topic `/fleet/state` |
| Hàng ưu tiên **theo task**; gặp nhau thì robot ưu tiên thấp né rồi bám lại route cũ | `priority_mode: task` (mặc định): mức khẩn của task (theo trạng thái dây chuyền) → đang chở hàng → task chờ lâu. Robot ưu tiên thấp tránh vào yield bay, chờ robot ưu tiên cao đi qua rồi nhập lại **route cũ** |

---

## 1. Kiến trúc

```
              ┌──────────────────────────────── amr_bringup ─────────────────────────────────┐
              │ factory_layout.yaml (nguồn dữ liệu duy nhất) · fleet_params.yaml · nav2_amr.yaml │
              └───────┬───────────────────────────┬──────────────────────────────┬────────────┘
                      │ generate_world             │                              │
   Gazebo Harmonic ◄──┤ world SDF + mesh           │ bản đồ AMCL (cắt ở           │ đồ thị giao thông
   ├ nhà máy + 8 dây chuyền (LineAnimator)          │ độ cao lidar)                │
   ├ N × AMR (URDF: diff-drive, lidar, RGB-D,       ▼                              ▼
   │   khay nâng hàng)                          map_server ──/map──► AMCL     fleet_manager
   └ ros_gz_bridge ──scan, odom, points──►     ┌──────── Nav2 / robot ───────┐ ├ MES mô phỏng → task
          ▲            /tf (odom→base)         │ AMCL: map → amr_XX/odom      │ ├ giao task + "task tiếp theo"
          │ cmd_vel                            │ controller_server (RPP +     │ ├ traffic control: đặt chỗ node,
          │ cargo_lift                         │   local costmap lidar+RGB-D) │ │  ưu tiên theo task, né + bám
          │ /factory/line_state                │ collision_monitor            │ │  lại route cũ, gỡ deadlock
          │                                    │   cmd_vel_nav → cmd_vel ─────┼─┘ ├ pin / trạm sạc 5 dock
          │                                    └──────────▲───────────────────┘   └ /fleet/state, markers …
          │                                               │ FollowPath (đoạn route       │
          │                                               │ đã cấp quyền) + speed_limit  │
          │                                     robot_agent × N ◄──/amr_XX/fleet/route──┘
          │                                     (pose AMCL, pin, khay nâng) ──status──►
          │                                               RViz2 ◄── /fleet/markers, Nav2 topics
          └────────────────────────────────────── dashboard (web :8080) ◄── /fleet/snapshot_json
```

**Phân vai Nav2 ↔ fleet manager.** Fleet manager quyết định robot được đi *đâu* và *tới đâu* (đặt chỗ node trên đồ
thị làn, ai nhường ai). `robot_agent` biến đoạn route đã được cấp quyền thành `nav_msgs/Path` dày (0.1 m) và gửi cho
`controller_server` qua action **FollowPath**; mỗi khi quyền đi được nới thêm thì path được gửi lại (preempt) nên robot
chạy liên tục, không dừng ở từng node. Tốc độ giới hạn của làn (1.0 m/s trên làn, 0.45 m/s trong lay-by/dock) gửi qua
topic `speed_limit` của Nav2. Vị trí robot là ước lượng AMCL. Lidar + camera 3D vào local costmap (RPP tránh va chạm)
và `collision_monitor` (vùng dừng 0.3 m, vùng giảm tốc 0.85 m trước robot).

| Package | Nội dung |
|---|---|
| `amr_fleet_msgs` | Message/service: `RouteCommand`, `RobotStatus`, `RobotState`, `FleetState`, `TaskInfo`, `ProductionState`, `FleetEvent`, `SubmitTask`, `SetRobotPriority` |
| `amr_description` | URDF/xacro của AMR + plugin Gazebo (DiffDrive, OdometryPublisher, JointStatePublisher, JointPositionController cho khay nâng, gpu_lidar, rgbd_camera) |
| `amr_gazebo` | System C++ `LineAnimator` (sản phẩm chạy trên băng tải), launch Gazebo: sinh world + bản đồ từ layout, spawn N robot, một `ros_gz_bridge` chung; world/mesh/bản đồ dựng sẵn |
| `amr_fleet_manager` | Lõi điều phối (Python thuần, có unit test), node ROS: `fleet_manager`, `robot_agent` (driver `nav2` hoặc `simple`), `kinematic_sim`, `dashboard`, công cụ `fleet_cli`, `headless_sim`, `generate_world`, `rviz_config` |
| `amr_bringup` | `bringup.launch.py`, `navigation.launch.py` (Nav2 + AMCL cả đội), `fleet.launch.py`, layout nhà máy, tham số |

Lõi `amr_fleet_manager/core` **không phụ thuộc ROS**: cùng một mã được dùng trong node ROS, trong mô phỏng headless và
trong unit test.

## 2. Nhà máy và chu trình sản xuất

```
 y=72 ┌──────────────── tường panel · cửa sổ · 2 cửa cuộn phía Bắc (xe tải) ──────────────────────────┐ văn
      │P ═ Đường Bắc y=67, 2 CHIỀU (yield bay) ═ A4 ═ A5 ═══ B1_IN ═ B2_IN ════════ B3_IN ═ B4_IN ═══ P│ phòng
      │P   ┌ Khu A: kho linh kiện ┐  ┆  ┌─ Khu B: 4 dây chuyền SẢN XUẤT ─────┆──────────────────┐   P│ 2
      │A3  │ 5 kệ đầy bin/thùng    │  ┆  │ B1      B2                         ┆      B3     B4    │   P│ tầng
      │A2  │                       │  ┆  │ (băng tải, CNC, robot hàn, lắp ráp, QC, xếp thùng)   │   P│ (x>120)
      │A1  └───────────────────────┘  ┆  └ B1_OUT  B2_OUT  [CH5 CH4 CH3 ┆ CH2 CH1]  B3_OUT B4_OUT┘   P│
      │ ═══ Trục chính y=36: 2 làn 1 chiều ═ D5 ═ D6 ══ C1_IN ═ C2_IN ═══════════ C3_IN ═ C4_IN ═════ │
      │P   ┌ Khu D: kho thành phẩm┐  ┆  ┌─ Khu C: 4 dây chuyền ĐÓNG GÓI ─────┆──────────────────┐   P│
      │P   │ 5 kệ pallet           │  ┆  │ C1      C2                         ┆      C3     C4    │   P│
      │P   │                       │  ┆  │ (tháo thùng, máy quấn màng, case packer, robot xếp pallet)│
      │P   └───────────────────────┘  ┆  └ C1_OUT  C2_OUT ────────────────────┆─── C3_OUT C4_OUT ┘   P│
      │ ═══ Đường Nam y=5: 2 làn 1 chiều ═ D4 ═ D3 ═ D2 ═ D1 ═════════════════════════════════════════ │
 y=0  └ x=0 ─ Đường Tây x=5 (2 làn) ─ Trục giữa x=35 (2 chiều) ─ Ngõ x=74 (2 chiều) ─ Đường Đông x=115 ┘ x=120
```

* **Làn 1 chiều theo luật đi bên phải** (Đường Nam, Trục chính, Đường Tây, Đường Đông): hai làn cách nhau 2 m nên
  không có đối đầu. Có mũi tên chỉ chiều trên sàn Gazebo và trong RViz.
* **Làn 2 chiều** (Đường Bắc, Trục giữa x=35, Ngõ x=74) có **yield bay** (ô tránh) dọc làn: đây là nơi robot gặp
  nhau đối đầu và robot ưu tiên thấp phải né.
* **Trạm** là lay-by cạnh làn: `làn → Q (ô chờ) → S (ô phục vụ) → làn`. Robot chờ trạm đứng ở Q, không chặn làn.
  Dock sạc và ô đỗ là ngõ cụt cạnh làn.

**Một dây chuyền sản xuất (B1–B4)**: băng tải chạy suốt từ B_IN (bàn nhận kit) tới B_OUT, trạm CNC, tủ điều khiển,
cell robot hàn (có hàng rào an toàn), bàn lắp ráp, cổng QC, máy xếp thùng, đèn tháp báo trạng thái.
**Một dây chuyền đóng gói (C1–C4)**: tháo thùng, máy quấn màng (flow wrapper), case packer, robot xếp pallet, pallet
thành phẩm ở C_OUT. Sản phẩm (hộp) chạy liên tục trên mọi đoạn băng tải trong Gazebo; khi MES báo dây chuyền
`STARVED`/`BLOCKED` thì băng tải của dây đó dừng (topic `/factory/line_state`).

**Công nhân đi lại trong nhà máy** (Gazebo, system `amr_pedestrians`, mục `people:` trong layout): 8 người mặc áo
phản quang, đội mũ bảo hộ, đi bộ ~1 m/s, tay chân đung đưa theo bước đi, dừng làm việc ở các điểm trên tuyến:
2 người lấy hàng giữa các dãy kệ khu A và D, 4 người vận hành dọc hành lang giữa các dây chuyền B và C, 1 người vận
chuyển vật tư băng qua trục chính, 1 giám sát đi vòng cắt qua ngõ x = 74. Lối đi bộ được sơn vạch xanh, chỗ cắt làn
AMR có vạch qua đường (zebra). Fleet manager không lập kế hoạch quanh họ: lidar 270° thấy chân, camera 3D thấy cả
người, nên `collision_monitor` của Nav2 giảm tốc / dừng AMR khi có người phía trước. Người đứng chờ khi một AMR đang
chạy tới gần và quay lại nếu AMR đứng chắn lối đi quá lâu (không bao giờ kẹt cứng người ↔ robot). Vị trí từng người
có trên `/factory/people` (`geometry_msgs/PoseArray`) và trong RViz. Tắt bằng `people:=false`.

**Chu trình (MES mô phỏng, `process:` trong layout):**

1. `KIT_DELIVERY` A1–A5 → B*_IN: kit linh kiện (12 bộ) cho dây chuyền sản xuất.
2. Mỗi dây chuyền B làm ra 1 SP sau mỗi `cycle_time` (22 s). SP chưa đóng gói được **xếp vào thùng** (12 SP/thùng),
   thùng đầy nằm ở buffer B*_OUT.
3. `BOX_TRANSFER` B*_OUT → C*_IN: chở thùng sang dây chuyền đóng gói (thùng đầy nhất đi trước, chọn dây C ít việc).
4. Dây chuyền C **đóng gói từng sản phẩm** (`pack_time` 21 s/SP) và xếp lên pallet thành phẩm.
5. `FG_DELIVERY` C*_OUT → D1–D6: nhập kho thành phẩm.

Khi khởi động, các dây chuyền đã có sẵn bán thành phẩm (`initial_wip`) nên cả 8 dây chạy ngay, không phải chờ robot
chở chuyến đầu.

## 3. Robot AMR

| Thành phần | Thông số |
|---|---|
| Kích thước | 0.80 × 0.46 m thân (footprint Nav2 0.85 × 0.60 m), 45 kg |
| Truyền động | 2 bánh chủ động Ø160 mm ở giữa thân (vi sai, cách nhau 0.52 m) + **4 bánh tự do** ở 4 góc |
| Lidar 2D | Một `gpu_lidar` **270°** (±135°) giữa mặt trước, cao 0.20 m, 541 tia (0.5°), 0.08–20 m, 10 Hz → `/amr_XX/scan`. Thân xe có **khe quét** ở dải 0.17–0.23 m (lùi vào tới x = 0.18 m) nên tia ±135° đi dọc hai bên hông xe mà không chạm thân, như AMR thật |
| Camera 3D | `rgbd_camera` 320×240, 5 Hz → `/amr_XX/camera/{image,depth_image,points,camera_info}` |
| Odometry | DiffDrive (odometry bánh xe, có trôi như robot thật) → `/amr_XX/odom` + TF `amr_XX/odom → amr_XX/base_footprint`; vị trí thật giữ ở `/amr_XX/ground_truth` để đối chiếu |
| Định vị | AMCL (likelihood field, 300–1500 hạt) trên `/map`, phát TF `map → amr_XX/odom` |
| Khay hàng | Khớp tịnh tiến `cargo_joint`: khi robot chở hàng, kiện hàng nâng lên sàn xe (`/amr_XX/cargo_lift`); khi không chở, kiện hàng hạ xuống dưới sàn để không cắt mặt phẳng lidar |
| Tốc độ | 1.0 m/s trên làn, 0.45 m/s trong lay-by/dock, quay tại chỗ khi lệch hướng lớn |
| An toàn | `collision_monitor`: vùng dừng 0.30 m, vùng giảm tốc 0.85 m trước robot, từ lidar + point cloud camera |

## 4. Điều phối giao thông (cốt lõi)

Mỗi chu kỳ 10 Hz, `core/traffic.py` và `core/fleet.py` làm các bước sau:

1. **Ô đang giữ (held)**: node robot đang đứng, node nó đang đi tới và mọi node robot không kịp phanh trước đó. Đây
   là khoá cứng, robot nào giữ thì robot đó sở hữu.
2. **Cấp quyền đi (grant)** cho từng robot **theo thứ tự ưu tiên giảm dần**: cấp tiếp từng node dọc route (khoảng
   4.5 m phía trước). Một node chỉ trống khi mọi node trong bán kính 1.3 m và mọi node bị cạnh đi quét qua đều không
   thuộc robot khác. **Giao lộ được cấp nguyên khối** gồm cả node ra khỏi giao lộ, nên không robot nào dừng giữa
   giao lộ.
3. **Đồ thị chờ (wait-for)**: robot nào đang bị robot nào chặn.
   * **Đối đầu / deadlock vòng** (A chờ B, B chờ A, hoặc vòng dài hơn): robot **ưu tiên thấp nhất** trong vòng né.
   * **Chặn ưu tiên**: robot ưu tiên cao bị một robot ưu tiên thấp đang đứng yên chặn quá 3 s thì robot thấp nhường.
4. **Né đường**: tìm nơi tránh gần nhất (yield bay, ô đỗ trống hoặc làn khác) nằm **ngoài lộ trình sắp tới** của
   robot ưu tiên cao, còn trống theo đúng luật cấp quyền. Robot thấp vào đó, chờ robot ưu tiên cao đi qua. Nếu
   không né được (hai bên đều kẹt), hai robot **đổi vai**: robot kia lùi vào chỗ tránh.
5. **Bám lại route cũ**: từ chỗ tránh, chọn node trên **route ban đầu** sao cho (đường quay lại + phần route cũ còn
   lại) ngắn nhất, rồi tiếp tục đúng route và nhiệm vụ cũ, ví dụ log:
   `amr_05 bám lại route cũ tại north.16 → tiếp tục đi tới điểm lấy hàng`.
6. **Đổi lộ trình**: robot bị kẹt quá 20 s được lập lại đường đi tránh vùng tắc. A* phạt các node đang có robot
   đứng, nên robot chủ động đi vòng chỗ đông.

### Hàng ưu tiên theo task (`priority_mode: task`, mặc định)

Mức ưu tiên đi theo **task** robot đang làm, không theo số hiệu robot:

```
mức task  = MES tính lại liên tục:  HIGH (2) khi dây chuyền task phục vụ sắp thiếu đầu vào, đầu ra đã đầy
            hoặc máy đóng gói đang đói;  NORMAL (1) bình thường;  task thủ công giữ mức người tạo chọn (0..3)
điểm      = 1000·(mức task + 1)
          + 400                         nếu đang chở hàng của task (đã lấy hàng)
          + 0.5·tuổi task (tối đa 300)  task chờ lâu được ưu tiên hơn
robot không có task: 0 · đi sạc: 300 · đi sạc khi pin < 15 %: 3500 (cao nhất)
hoà điểm: phân xử theo hạng cơ bản (amr_01 = 1, đổi được trên dashboard)
```

Hạng P1..Pn hiển thị trên dashboard/RViz là hạng dùng để quyết định ai nhường. Thời gian chờ chỉ cộng thêm khi cấp
quyền qua giao lộ (chống "chết đói"), không làm lật kèo né đường. Hai chế độ cũ vẫn còn: `priority_mode:=dynamic`
(task + hạng cơ bản + thời gian chờ) và `priority_mode:=static` (chỉ hạng cơ bản).

## 5. Pin và trạm sạc

* Pin được mô phỏng trên robot (`robot_agent`): tiêu hao khi đứng yên và khi chạy, sạc 0.25 %/s trên dock.
* Pin < 30 %: làm xong task hiện tại rồi đi sạc, huỷ "task tiếp theo" nếu có. Pin < 15 % khi đang chạy rỗng: trả
  task về hàng đợi rồi đi sạc ngay.
* Rảnh và pin < 50 % mà có dock trống gần thì **sạc tranh thủ**. Robot sạc tranh thủ nhường dock cho robot pin thấp,
  và được gọi đi làm task khi pin ≥ 60 % (robot vào sạc vì pin thấp thì phải ≥ 80 %).
* Hết dock trống: robot vào **hàng đợi sạc** (ưu tiên pin thấp nhất) và chờ ở bãi đỗ gần trạm sạc.

## 6. Cài đặt

Ubuntu 24.04 + ROS 2 Jazzy (đi kèm Gazebo Harmonic qua `ros_gz`):

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-navigation2 \
     ros-jazzy-nav2-rviz-plugins ros-jazzy-robot-state-publisher python3-colcon-common-extensions
mkdir -p ~/amr_ws && cd ~/amr_ws
git clone <repo này> src/AMR-fleet-harmonic-distribution     # hoặc copy thư mục src/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`amr_gazebo` biên dịch system C++ `LineAnimator` (gz-sim 8); hook môi trường tự thêm
`GZ_SIM_SYSTEM_PLUGIN_PATH` và `GZ_SIM_RESOURCE_PATH` khi `source install/setup.bash`.

**Gazebo Jetty**: các system dùng tên `gz-sim-*-system` và thẻ `gz_frame_id` giống Harmonic, nên chỉ cần build
workspace trên bản ROS 2 đi kèm Jetty (`ros_gz`, `gz_sim_vendor` tương ứng). Không cần sửa mã.

## 7. Chạy

```bash
# Gazebo Harmonic + Nav2/AMCL từng robot + RViz + dashboard, 12 robot
ros2 launch amr_bringup bringup.launch.py num_robots:=12

# 15 robot, không mở GUI Gazebo (vẫn render cảm biến)
ros2 launch amr_bringup bringup.launch.py num_robots:=15 gui:=false

# Nav2 đầy đủ (thêm planner_server, behavior_server, bt_navigator cho mỗi robot)
ros2 launch amr_bringup bringup.launch.py nav2_full_stack:=true

# Không Nav2: bộ bám đường có sẵn trên odometry ground truth (nhẹ hơn)
ros2 launch amr_bringup bringup.launch.py navigation:=simple

# Máy yếu / không GPU: bỏ Gazebo, dùng bộ mô phỏng động học (cùng topic, cùng fleet software)
ros2 launch amr_bringup bringup.launch.py sim:=kinematic num_robots:=15
```

Mở **http://localhost:8080** để xem dashboard.

### Xem robot né đường ngay

Làn 1 chiều và định tuyến tránh ùn tắc khiến đối đầu ít xảy ra (đó là mục tiêu của mạng lưới). Để xem tận mắt cơ
chế né, bấm **"Demo né đường"** trên dashboard hoặc chạy `ros2 run amr_fleet_manager fleet_cli demo`. Hai robot
rảnh được đưa ra hai đầu Đường Bắc (làn 2 chiều) rồi chạy ngược chiều cùng lúc. Robot ưu tiên thấp hơn tránh vào
yield bay, chờ robot kia đi qua rồi **bám lại route cũ**. Mọi bước được ghi vào nhật ký với nhãn "Né đường".

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `sim` | `gazebo` | `gazebo` hoặc `kinematic` |
| `num_robots` | `12` | 1..30 (khuyến nghị 10–15) |
| `navigation` | `nav2` | `nav2`: AMCL + Nav2 mỗi robot, odometry bánh xe · `simple`: bộ bám đường có sẵn, odometry ground truth |
| `nav2_full_stack` | `false` | Thêm planner/behavior/bt_navigator cho mỗi robot (để thử NavigateToPose thủ công) |
| `gui` | `true` | GUI Gazebo |
| `rviz` | `true` | RViz2 với cấu hình sinh tự động cho N robot (thêm hiển thị Nav2 khi `navigation:=nav2`) |
| `camera` | `true` | Mô phỏng camera RGB-D trên mọi robot (point cloud vào costmap + collision monitor) |
| `people` | `true` | Công nhân đi lại trong nhà máy (Gazebo) |
| `bridge_cameras` | `first` | `none` · `first` · `all`: ảnh camera của robot nào được đưa sang ROS |
| `priority_mode` | `task` | `task` · `dynamic` · `static` |
| `production` | `true` | `false`: tắt MES mô phỏng, robot chỉ chạy task thủ công / demo |
| `agents` | `single_process` | `single_process`: mọi agent trong 1 process (nhẹ CPU) · `per_robot`: mỗi robot 1 process như AMR thật |
| `dashboard_port` | `8080` | Cổng web |
| `dashboard_host` | `0.0.0.0` | `127.0.0.1`: chỉ máy này mở được dashboard (dashboard tạo được task, dừng cả đội) |

Chạy từng phần riêng (đúng những gì `bringup.launch.py` làm; các tham số đi theo cặp, trộn sai thì khung toạ độ
không khớp):

```bash
# Nav2: odometry bánh xe + AMCL, fleet manager không phát /map và map -> odom
ros2 launch amr_gazebo gazebo_fleet.launch.py num_robots:=12 odom:=wheel bridge_points:=all
ros2 launch amr_bringup navigation.launch.py num_robots:=12
ros2 launch amr_bringup fleet.launch.py num_robots:=12 driver:=nav2 use_sim_time:=true

# bộ bám đường có sẵn: odometry ground truth, fleet manager phát /map và map -> odom
ros2 launch amr_gazebo gazebo_fleet.launch.py num_robots:=12 odom:=ground_truth bridge_points:=images
ros2 launch amr_bringup fleet.launch.py num_robots:=12 driver:=simple use_sim_time:=true
```

### Công cụ dòng lệnh

```bash
ros2 run amr_fleet_manager fleet_cli status              # bảng trạng thái trực tiếp trong terminal
ros2 run amr_fleet_manager fleet_cli task A2 D1 --priority 3
ros2 run amr_fleet_manager fleet_cli rank amr_07 1       # hạng cơ bản (phân xử khi hoà điểm task)
ros2 run amr_fleet_manager fleet_cli pause | resume
ros2 run amr_fleet_manager fleet_cli demo                # kịch bản đối đầu → né → bám lại route cũ
ros2 run amr_fleet_manager fleet_cli pause_robot amr_03
ros2 run amr_fleet_manager headless_sim --robots 15 --minutes 60   # mô phỏng nhanh không ROS/Gazebo
```

### RViz

Cấu hình RViz được sinh tự động: bản đồ AMCL, layout (khu, máy, làn và chiều đi, trạm, dock, yield bay), model từng
robot, lidar mỗi robot (màu theo robot), **hạt AMCL** của từng robot, **vùng dừng/giảm tốc** của collision monitor,
**local costmap** của `amr_01`, path gửi cho FollowPath (tắt sẵn), point cloud và ảnh camera 3D của `amr_01`. Nhãn
trên mỗi robot có dạng:

```
AMR-05 P2 T0012
TO_DROPOFF -> B1_IN
ETA 34s | 72% | YIELD->amr_02
```

Lộ trình còn lại vẽ nét mảnh, đoạn đã được cấp quyền vẽ nét đậm. Ô vuông màu là các node robot đang giữ, hộp nâu trên
robot nghĩa là đang chở hàng.

## 8. Giao diện ROS

| Topic / service | Kiểu | Mô tả |
|---|---|---|
| `/fleet/state` | `amr_fleet_msgs/FleetState` | Toàn bộ robot (trạng thái, task, route, ETA, task kế tiếp, ưu tiên, chờ/nhường), task, trạm sạc |
| `/fleet/production` | `amr_fleet_msgs/ProductionState` | Trạng thái 8 dây chuyền, buffer, KPI |
| `/fleet/events` | `amr_fleet_msgs/FleetEvent` | Nhật ký: giao task, né đường, deadlock, sạc… |
| `/factory/line_state` | `std_msgs/String` | `"B1=1.0;C3=0.0;…"`: băng tải nào đang chạy (bridge sang Gazebo) |
| `/fleet/markers`, `/fleet/static_markers` | RViz | Nhãn, route, ô đặt chỗ, layout |
| `/map` | `nav_msgs/OccupancyGrid` | Bản đồ AMCL (map_server); chế độ `simple`: fleet manager phát |
| `/amr_XX/fleet/route` | `RouteCommand` | Fleet → robot: waypoint, chỉ số được cấp quyền, giới hạn tốc độ, đang chở hàng |
| `/amr_XX/fleet/status` | `RobotStatus` | Robot → fleet: vị trí AMCL, tiến độ, pin, an toàn |
| `/amr_XX/follow_path` | `nav2_msgs/action/FollowPath` | Agent → Nav2 controller_server |
| `/amr_XX/amcl_pose`, `/particle_cloud` | Nav2 | Định vị AMCL |
| `/amr_XX/cmd_vel`, `/odom`, `/scan`, `/camera/*`, `/battery`, `/cargo_lift`, `/ground_truth` | chuẩn | Giao diện robot |
| `/fleet/submit_task` | `SubmitTask` | Tạo task thủ công |
| `/fleet/set_robot_priority` | `SetRobotPriority` | Đổi hạng cơ bản / tạm dừng robot |
| `/fleet/pause` | `std_srvs/SetBool` | Tạm dừng / tiếp tục cả đội |
| `/fleet/demo_head_on` | `std_srvs/Trigger` | Kịch bản demo: 2 robot đối đầu trên Đường Bắc |

## 9. Tuỳ biến nhà máy

Mọi thứ nằm trong `src/amr_bringup/config/factory_layout.yaml`: kích thước, các aisle (`dual` hoặc `single`), khu,
trạm (lay-by), dock sạc, bãi đỗ, dây chuyền (vị trí, loại, nhịp), vật thể (kệ, pallet, tủ…), trang trí (cửa, xe tải,
văn phòng, cột, đèn) và thông số sản xuất. World Gazebo, mesh và bản đồ AMCL được **sinh lại mỗi lần launch**, nên đồ
thị giao thông, world 3D, bản đồ AMCL, RViz và dashboard luôn khớp nhau. Xuất file riêng:

```bash
ros2 run amr_fleet_manager generate_world --layout src/amr_bringup/config/factory_layout.yaml \
     --out factory.sdf --map factory_map          # + factory_static.obj/.mtl cạnh factory.sdf
```

World dựng sẵn nằm ở `amr_gazebo/worlds/factory.sdf` (mesh `model://amr_factory`, bản đồ `amr_gazebo/maps/`):
`gz sim factory.sdf` chạy được ngay sau khi source workspace.

Mọi chi tiết đục (≈5000 khối) được **gộp vào một mesh OBJ, mỗi vật liệu một sub-mesh**: vài chục draw call thay vì
vài nghìn visual, nên lidar và camera của cả đội render nhanh (trên máy không GPU, RTF tăng từ 0.004 lên ≈0.6 với
3 robot). Đèn, LED, kính, màng quấn vẫn là visual SDF riêng (phát sáng / trong suốt).

## 10. Kiểm thử

```bash
cd src/amr_fleet_manager && python3 -m pytest -q test      # hoặc: colcon test --packages-select amr_fleet_manager
```

Test bao gồm: dựng đồ thị (liên thông, yield bay ngoài làn, conflict đối xứng), **world 3D không chắn làn nào**
(mọi cạnh của đồ thị giao thông cách vật thể ≥ 0.35 m trên bản đồ ở độ cao lidar), bộ bám đường, vùng an toàn, theo
dõi tiến độ route khi Nav2 cắt góc, ghép pose AMCL + odometry, kịch bản **đối đầu trên làn 2 chiều** (robot ưu tiên
thấp vào yield bay, chờ, bám lại route cũ, cả hai về đích không va chạm), và mô phỏng 12–15 robot với tiêu chí 0 va
chạm (kiểm tra va chạm hình chữ nhật xoay) và không robot nào kẹt quá 150 s.

Kết quả mô phỏng headless 30 phút (cấu hình mặc định, `priority_mode: task`):

| Robot | Task xong | SP sản xuất | Thành phẩm nhập kho | 4 dây SX chạy | 4 dây đóng gói chạy | Va chạm | Né đường / đối đầu đã gỡ | Kẹt lâu nhất |
|---|---|---|---|---|---|---|---|---|
| 10 | 66 | 324 | 228 | 100 % | 88–100 % | 0 | 10 / 9 | 26 s |
| 12 | 75 | 324 | 276 | 100 % | 95–100 % | 0 | 15 / 14 | 26 s |
| 15 | 75 | 324 | 276 | 100 % | 95–100 % | 0 | 9 / 6 | 39 s |

Từ 12 robot trở lên, đội xe không còn là nút cổ chai: sản lượng bị giới hạn bởi nhịp dây chuyền (4 dây × 22 s/SP
≈ 650 SP/giờ), thông lượng nhập kho ổn định ≈ 550 thành phẩm/giờ sau giai đoạn khởi động.

Đã chạy thử trên **Gazebo Harmonic (gz-sim 8)** với Nav2 + AMCL cho 3 robot, trong container không GPU (render
phần mềm, RTF ≈ 0.3–0.5): robot spawn ở bãi đỗ, AMCL định vị trên bản đồ sinh từ world (sai số so với vị trí thật
≈ 0.12 m dọc làn, 0.07 m ngang làn, < 1°), `controller_server` bám đoạn route được cấp quyền, robot vào lay-by lấy
kit ở khu A, khay hàng nâng lên khi chở, chở tới B*_IN, trả hàng, rồi chở thùng SP từ B3_OUT sang C2_IN (5 task xong
trong 550 s mô phỏng). Sản phẩm chạy trên băng tải của cả 8 dây chuyền (`LineAnimator`). Kịch bản đối đầu cũng chạy
đúng với Nav2: AMR-02 (P2) né vào `Y.north.16`, AMR-01 (P1) đi qua, rồi AMR-02 bám lại route cũ tại `north.32`.

## 11. Hiệu năng và lưu ý

* 15 robot × (lidar + camera RGB-D) + Nav2 cho cả đội trong Gazebo cần GPU và CPU nhiều nhân. Trên máy yếu: giảm
  `num_robots`, `camera:=false` (costmap/collision monitor chỉ dùng lidar), `navigation:=simple`, hoặc
  `sim:=kinematic`.
* Không có GPU, camera RGB-D render chậm hơn lidar nhiều; nguồn point cloud của collision monitor vì vậy có
  `source_timeout` 6 s (lidar 2 s). Nếu camera vẫn trễ hơn thế, collision monitor dừng robot cho tới khi có dữ liệu
  mới (fail-safe, đúng như trên robot thật); trên máy không GPU nên chạy `camera:=false`.
* `nav2_full_stack:=true` đã thử với NavigateToPose thủ công (planner NavFn + BT mặc định của Nav2). Khi fleet manager
  đang điều khiển robot thì không nên gửi NavigateToPose cho robot đó: hai bên sẽ tranh nhau `controller_server`.
* Để nhẹ CPU, mặc định tất cả `robot_agent` chạy trong 1 process (`fleet_agents`), mọi `robot_state_publisher` trong 1
  component container, Nav2 mỗi robot trong 1 `component_container_isolated`. Agent lấy pose từ `amcl_pose` ghép với
  odometry (đúng như TF) thay vì nghe toàn bộ `/tf` của cả đội trong Python.
* `fleet_agents`, `fleet_manager` và `kinematic_sim` dùng **EventsExecutor** của rclpy (tham số `executor:=events`,
  mặc định): SingleThreadedExecutor dựng lại wait-set của mọi subscription/timer cho từng callback, với 15 robot
  riêng phần đó đã chiếm trọn một nhân. Đo với 15 robot: agent 99 % → 27 %, kinematic_sim ~90 % → 30 %,
  fleet manager 40 % → 11 % CPU. `executor:=single` quay về executor cũ.
* Tránh va chạm giữa các robot do fleet manager đảm bảo bằng đặt chỗ node; Nav2 lo bám đường, định vị và an toàn cục
  bộ (người, vật cản lạ).
