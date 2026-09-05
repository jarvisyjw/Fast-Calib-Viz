# FAST-Calib Online — Web-based LiDAR–Camera Extrinsic Calibration

This repository extends **[FAST-Calib](https://github.com/hku-mars/FAST-Calib)**
with an **online calibration mode** served by a web UI: instead of processing a
recorded rosbag file offline, a persistent ROS node subscribes to **live LiDAR
and camera topics** and captures synchronized data on demand, one click per
scene. After 3 successful captures, the joint multi-scene extrinsic solve runs
automatically and the result is visualized right in the browser (reprojected
depth map, colored point clouds, full pipeline intermediates).

> 🙏 **Credits**: all credit for the calibration algorithm, the target design,
> and the original codebase goes to the authors of
> **[FAST-Calib](https://github.com/hku-mars/FAST-Calib)**
> (*FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second*,
> [paper](https://www.arxiv.org/pdf/2507.17210)) — Chunran Zheng and
> contributors at the HKU MaRS Lab. This repo merely wraps their pipeline in an
> online/web workflow. The target design is itself based on
> [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration).
> If you use this work, please cite the original FAST-Calib paper.

**Highlights of the online mode:**

1. **Live capture** from real drivers or a looping `rosbag play` — click once
   per scene, no bag-file plumbing.
2. **Live sensor streams** in the browser (camera feed + LiDAR cloud) with a
   3D wireframe of the distance-filter crop box.
3. **Robust on sparse clouds**: multi-frame LiDAR accumulation per capture,
   plus a grid-occupancy hole detector that takes over when the classic
   boundary-based circle extraction is starved of points.
4. **Instant visual verification**: project the live LiDAR frame into the
   current camera view as a dense, absolute-scale depth map.
5. **Full pipeline transparency**: every intermediate (filtered cloud, RANSAC
   plane, rim points, circle centers) is a toggleable 3D layer.

## Demo

▶️ **[Demo video: full online calibration in ~90 s](https://youtu.be/VHcaLTlcTU8)** —
set topics, frame the board with the filter-box wireframe, 3 captures,
automatic joint solve, live depth-map verification.

<!-- TIP: for inline playback on GitHub, drag pics/demo_usage.mp4 into any
     issue/PR comment box, copy the https://github.com/user-attachments/assets/...
     URL it produces, and replace the link above with that bare URL. -->

<p align="center">
  <img src="./pics/panel.png" width="100%">
  <font color=#a0a0a0 size=2>The web UI during online calibration — 3D scene with pipeline
  layers (board plane, hole rims in orange, circle centers in red, filter-box
  wireframe) and the control panel: (1) live-capture topics, (2) distance
  filter, Run calibration button, and results with RMSE / T_cam_lidar.</font>
</p>

---

## 1. Quick start (Docker)

```bash
# 1. Build the image (from the repo root)
docker build -t fast-calib:web-online -f docker/Dockerfile .

# 2. Start the container
docker run -d --name fastcalib-online \
  -p 8081:8080 \
  -v $PWD/calib_data:/data:ro \
  -v $PWD/output_online:/output \
  fast-calib:web-online

# 3. Play a recorded bag as a fake live stream (looping)
docker exec -it fastcalib-online bash -c \
  "source /opt/ros/noetic/setup.bash && rosbag play -l /data/<scene>.bag"

# 4. Open the UI
#    http://localhost:8081
```

With real sensors instead of a bag, just make sure the drivers publish the
LiDAR and camera topics to the same ROS master (the container runs its own
`roscore`).

> The first `rosbag play` after a container start spends ~30–60 s indexing the
> bag before topics appear — this is normal.

---

## 2. Calibration procedure

1. **Set the topics** (*Live capture* folder): LiDAR topic
   (e.g. `/livox/lidar`) and camera topic (e.g. `/d435/color/image_raw`).
   Supported message types: `livox_ros_driver/CustomMsg`,
   `livox_ros_driver2/CustomMsg`, `sensor_msgs/PointCloud2` for LiDAR;
   `sensor_msgs/Image` and `sensor_msgs/CompressedImage` for the camera.

2. **Check the live streams** (*Live streams* folder): enable the camera feed
   and the live LiDAR cloud to confirm data is flowing and the board is in
   view.

3. **Adjust the distance filter** (*Distance filter* folder): the crop box
   must isolate the calibration board (and as little background as possible).
   Enable *"Show filter box in 3D scene"* to see the box as a wireframe
   around the live cloud — this is the easiest way to get it right.

4. **Verify intrinsics and target geometry** (prefilled from
   `config/qr_params.yaml`): camera intrinsics `fx fy cx cy k1 k2 p1 p2`,
   marker size, and the board's circle layout (`delta_width_circles`,
   `delta_height_circles`, `circle_radius`).

5. **Click "Run calibration"**. The node accumulates *"LiDAR frames per
   capture"* consecutive LiDAR frames (default 20 ≈ 2 s at 10 Hz) into one
   dense cloud and pairs it with the camera frame nearest to the middle of
   that window.
   **Keep the board perfectly still during the capture window.**

6. Check the result: **RMSE** and `T_cam_lidar` appear under *Results*; the
   reprojected LiDAR depth map (top-left panel) should align with the scene.
   A failed capture is reported in the status line and is **not** recorded.

7. **Repeat at 3 different board poses** (vary distance and tilt). After the
   **3rd successful capture**, the joint multi-scene solve runs automatically
   and the depth panel switches to a side-by-side *single vs multi* comparison.

8. Download `single_calib_result.txt` or `multi_calib_result.txt` from the
   Results section.

### Verifying the calibration live

Once at least one calibration result exists, click **"Verify calibration"**
(top of the left column): the *latest* live LiDAR frame is projected into the
*current* camera frame using the calibrated extrinsic and shown as a dense
depth map over the RGB image, colored on an **absolute metric scale**
(0.3 m = blue → 8 m = red). Move the board or the rig and click again —
there is no accumulation, the map always reflects the scene right now. LiDAR
edges should sit exactly on the RGB edges.

---

## 3. Web UI reference

| Element | Description |
|---|---|
| **Live camera feed** | Real-time camera image (~3 fps). |
| **Live streams** | Toggles for the camera feed and the live LiDAR cloud in the 3D scene. |
| **Live capture** | Topic names + *"LiDAR frames per capture"* (frames merged per click; default 20). |
| **Camera intrinsics** | Pinhole intrinsics + distortion of the camera. |
| **Target geometry** | ArUco marker size, circle grid spacing (0.5 × 0.4 m) and circle radius (0.12 m). |
| **Distance filter** | Axis-aligned crop box in the LiDAR frame; only points inside are used for detection. |
| **Run calibration** | Capture one scene + solve single-scene extrinsic. |
| **Verify calibration** | Live depth-map overlay using the latest result (needs ≥ 1 successful capture). |
| **Layers** | Toggle every intermediate: input / filtered / plane / edge (rim) points / circle centers / colored clouds / camera image / depth panel. |
| **Multi-scene** | Recorded scene counter (n/3), manual joint-solve button, *"Clear records"*, download of the joint result. |

---

## 4. How a capture works (pipeline)

1. **Accumulate** N consecutive LiDAR frames into one cloud (a single 100 ms
   Livox frame is too sparse for hole detection; N=20 works well for Mid-360).
2. **Pair** the camera frame closest (arrival time) to the middle of the
   accumulation window; must be within `sync_tolerance` (default 0.5 s).
3. **Camera side**: detect the 4 ArUco markers → board pose → 4 circle centers
   in the camera frame.
4. **LiDAR side**: crop box → 5 mm voxel downsample → RANSAC board plane →
   align plane to z=0 → find the 4 circular holes. Two detectors run:
   - *boundary-based* edge extraction + clustering + circle RANSAC
     (best on dense clouds), and
   - a *grid hole detector* fallback that rasterizes the board plane to a
     1 cm occupancy grid and finds holes as interior empty regions
     (robust on sparse online clouds).
5. **Solve** the extrinsic by SVD over the 4 LiDAR↔camera center pairs; RMSE
   of the pair distances is reported. A capture needs all 4+4 centers,
   otherwise it fails and nothing is recorded.
6. Successful captures append to `output/circle_center_record.txt`. At 3
   records, the joint multi-scene solve runs automatically.

---

## 5. Output files (mounted at `./output_online`)

| File | Content |
|---|---|
| `single_calib_result.txt` | FAST-LIVO2-format calibration (intrinsics + `Rcl`/`Pcl`). |
| `multi_calib_result.txt` | Joint multi-scene result. |
| `circle_center_record.txt` | One block per recorded scene (4 LiDAR + 4 camera centers). |
| `colored_cloud.pcd` | Point cloud colorized by the camera image. |
| `qr_detect.png` | Camera image with ArUco detections. |
| `input / filtered / plane / aligned / edge_cloud.pcd` | Pipeline intermediates (also viewable in the Layers panel). |
| `fast_calib_online.log` | Node log with per-stage point counts — first place to look on failures. |

---

## 6. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `timed out waiting for topics ...` | Bag not playing / driver down, or wrong topic names. Check with `rostopic list`. |
| `no fresh LiDAR frame within 10 s` | Playback stopped (bag finished without `-l`) or crashed. Restart `rosbag play -l`. |
| `no camera frame within ±0.5 s ...` | Streams lagging (slow machine) — try `rosbag play -r 0.5 -l`. |
| `detection failed (... filtered cloud: N pts)` | N=0: the crop box misses the board — check the wireframe box in the 3D scene. N large: detection issue, inspect the *Layers* (plane/edge) and `fast_calib_online.log`. |
| QR centers < 4 | Board not fully visible in the camera image at that moment, or wrong intrinsics/topic. |
| RMSE jumps between captures | Board moved during a capture window — keep it still for the full ~2 s. |
| Verify map looks stale | It uses only the latest frame; if the stream froze, the status line says the frames are stale — restart playback. |
| Port already in use | Change the host port: `-p 9090:8080`. |

### Tips

- Best capture geometry: board 1.5–3 m from the LiDAR, roughly fronto-parallel,
  fully inside both the camera FOV and the crop box.
- Three poses with clearly different distances/tilts give a much better joint
  solve than three nearly identical ones.
- You can recapture a bad scene: click *"Clear records"* and start over —
  failed captures never enter the record pool.

---

## 7. Offline mode (original FAST-Calib usage)

The original offline pipeline is unchanged. Prerequisites: PCL ≥ 1.8,
OpenCV ≥ 4.0.

1. Prepare the static acquisition data in the `calib_data` folder (see
   [Single-scene Calibration Sample Data](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/Eq_k_4Mf_11Eggg4a5lbRzgBHwd0EivtCJd2ExtcNlu1FA?e=vjm4gH)
   from Mid360, Avia and Ouster, and
   [Multi-scene Calibration Sample Data](https://pan.baidu.com/s/1Mkw7EWfiFT68LEzdkQnxeg?pwd=nyuh)
   from Avia): a rosbag containing point cloud messages + the corresponding
   image.
2. Run the single-scene calibration:
   ```bash
   roslaunch fast_calib calib.launch
   ```
3. After completing step 2 for at least three different scenes, run the
   multi-scene joint calibration:
   ```bash
   roslaunch fast_calib multi_calib.launch
   ```

To run on your own sensor suite: customize the calibration target below (CAD
model [here](https://pan.baidu.com/s/14Q2zmEfY6Z2O5Cq4wgVljQ?pwd=2hhn), PDF
with absolute scale [here](./calib_target/calib_traget.pdf)), collect three
scenes, provide the intrinsics in `qr_params.yaml`, set the distance filter,
and calibrate.

<p align="center">
  <img src="./pics/calibration_target.jpg" width="100%">
  <font color=#a0a0a0 size=2>Left: Actual calibration target | Right: Technical drawing with annotated dimensions.</font>
</p>
<p align="center">
  <img src="./pics/multi-scene.jpg" width="100%">
  <font color=#a0a0a0 size=2>Placement of the calibration target for multi-scene data collection: (a) facing forward, (b) oriented to the right, (c) oriented to the left.</font>
</p>
<p align="center">
  <img src="./pics/calib.jpg" width="100%">
  <font color=#a0a0a0 size=2>Left: Example of circle extraction from Mid360 point cloud | Right: Point cloud colored with calibrated extrinsic.</font>
</p>

For further details on the original algorithm workflow, see
[this document](https://github.com/xuankuzcr/FAST-Calib/blob/main/workflow.md).

## 8. Acknowledgments

This project builds on **[FAST-Calib](https://github.com/hku-mars/FAST-Calib)**
— many thanks to its author Chunran Zheng (zhengcr@connect.hku.hk) and the
HKU MaRS Lab for open-sourcing it. Special thanks to
[Jiaming Xu](https://github.com/Xujiaming1) for his support,
[Haotian Li](https://github.com/luo-xue) for the equipment, and the
[velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration)
algorithm for the target design.
