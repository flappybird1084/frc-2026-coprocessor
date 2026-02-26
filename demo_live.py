import argparse
import os
import sys
import time

import cv2
import numpy as np

import vision_engine
from tag_map_odometry import compute_camera_pose_from_map, load_tag_map


def draw_detections(frame, detections, scale=1.0, show_pose=True):
    for det in detections:
        corners = det["corners"]
        pts = np.array(corners, dtype=np.float32)
        if scale and scale != 1.0:
            pts = pts / scale
        pts = pts.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        center = det["center"]
        if scale and scale != 1.0:
            cx, cy = center[0] / scale, center[1] / scale
        else:
            cx, cy = center[0], center[1]
        cx, cy = int(cx), int(cy)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

        label = f"id={det['id']} h={det['hamming']}"
        for color, thickness in ((0, 0, 0), 3), ((255, 255, 255), 1):
            cv2.putText(
                frame,
                label,
                (cx + 6, cy - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                thickness,
                cv2.LINE_AA,
            )

        if show_pose:
            pose = det.get("pose")
            if pose and "rotation" in pose and "translation" in pose:
                rot = np.array(pose["rotation"], dtype=np.float64)
                tvec = np.array(pose["translation"], dtype=np.float64)
                roll, pitch, yaw = vision_engine.extract_euler_angles_cython(rot)
                rpy_deg = np.degrees([roll, pitch, yaw])
                pose_lines = [
                    f"t=({tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}) m",
                    f"r=({rpy_deg[0]:.1f}, {rpy_deg[1]:.1f}, {rpy_deg[2]:.1f}) deg",
                ]
                ty = cy + 16
                for line in pose_lines:
                    for color, thickness in ((0, 0, 0), 3), ((255, 255, 255), 1):
                        cv2.putText(
                            frame,
                            line,
                            (cx + 6, ty),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            color,
                            thickness,
                            cv2.LINE_AA,
                        )
                    ty += 16


def draw_pose_axes(frame, det, fx, fy, cx, cy, tag_size, dist_coeffs=None, scale=1.0):
    pose = det.get("pose")
    if not pose or "rotation" not in pose or "translation" not in pose:
        return

    rot = np.array(pose["rotation"], dtype=np.float64)
    tvec = np.array(pose["translation"], dtype=np.float64).reshape((3, 1))

    rvec, _ = cv2.Rodrigues(rot)
    cam = np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    if dist_coeffs is None:
        dist = np.zeros((4, 1), dtype=np.float64)
    else:
        dist = dist_coeffs

    axis_len = float(tag_size)
    object_points = np.array(
        [
            [0.0, 0.0, 0.0],
            [axis_len, 0.0, 0.0],
            [0.0, axis_len, 0.0],
            [0.0, 0.0, axis_len],
        ],
        dtype=np.float64,
    )

    image_points, _ = cv2.projectPoints(object_points, rvec, tvec, cam, dist)
    pts = image_points.reshape((-1, 2))
    if scale and scale != 1.0:
        pts = pts / scale

    origin = tuple(pts[0].astype(int))
    x_axis = tuple(pts[1].astype(int))
    y_axis = tuple(pts[2].astype(int))
    z_axis = tuple(pts[3].astype(int))

    cv2.line(frame, origin, x_axis, (0, 0, 255), 2)
    cv2.line(frame, origin, y_axis, (0, 255, 0), 2)
    cv2.line(frame, origin, z_axis, (255, 0, 0), 2)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Live AprilTag demo for vision_engine")
    parser.add_argument("--camera", type=str, default='0', help="Camera index")
    parser.add_argument(
        "--backend",
        choices=("auto", "v4l2"),
        default="auto",
        help="OpenCV capture backend (auto or v4l2)",
    )
    parser.add_argument("--width", type=int, default=640, help="Capture width")
    parser.add_argument("--height", type=int, default=480, help="Capture height")
    parser.add_argument("--camera-fps", type=int, default=30, help="Capture FPS request")
    parser.add_argument(
        "--fourcc",
        default="MJPG",
        help="FourCC for capture (e.g. MJPG, YUYV, H264)",
    )
    parser.add_argument("--tag-size", type=float, default=0.120, help="Tag size in meters")
    parser.add_argument(
        "--family",
        default="tag36h11",
        help="Tag family (e.g. tag36h11, tagStandard41h12)",
    )
    parser.add_argument("--nthreads", type=int, default=None, help="Detector threads")
    parser.add_argument("--quad-decimate", type=float, default=None, help="Quad decimate")
    parser.add_argument("--quad-sigma", type=float, default=None, help="Quad sigma")
    parser.add_argument("--refine-edges", type=int, default=None, help="Refine edges (0/1)")
    parser.add_argument("--decode-sharpening", type=float, default=None, help="Decode sharpening")
    parser.add_argument("--debug-images", action="store_true", help="Write apriltag debug images")
    parser.add_argument("--invert", action="store_true", help="Invert grayscale image")
    parser.add_argument("--scale", type=float, default=1.0, help="Scale factor for detection")
    parser.add_argument("--camera-matrix", default="camera_matrix.npy", help="Path to camera matrix")
    parser.add_argument("--dist-coeffs", default="dist_coeffs.npy", help="Path to distortion coeffs")
    parser.add_argument("--fx", type=float, default=None, help="Focal length x in pixels")
    parser.add_argument("--fy", type=float, default=None, help="Focal length y in pixels")
    parser.add_argument("--cx", type=float, default=None, help="Principal point x in pixels")
    parser.add_argument("--cy", type=float, default=None, help="Principal point y in pixels")
    parser.add_argument("--display", action="store_true", help="Show annotated video window")
    parser.add_argument("--fps", action="store_true", help="Print FPS once per second")
    parser.add_argument(
        "--save-dir",
        default=".",
        help="Directory for saved frames when pressing 's'",
    )
    parser.add_argument(
        "--tag-map",
        default=None,
        help="Optional JSON file describing static tag poses for multitag odometry",
    )

    args = parser.parse_args(argv)

    if args.backend == "v4l2":
        cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("Failed to open camera.", file=sys.stderr)
        return 1

    if args.fourcc:
        fourcc = cv2.VideoWriter_fourcc(*args.fourcc)
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.camera_fps:
        cap.set(cv2.CAP_PROP_FPS, args.camera_fps)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join(
        [
            chr((actual_fourcc >> 0) & 0xFF),
            chr((actual_fourcc >> 8) & 0xFF),
            chr((actual_fourcc >> 16) & 0xFF),
            chr((actual_fourcc >> 24) & 0xFF),
        ]
    )
    print(
        "Capture settings:",
        f"{actual_width}x{actual_height}",
        f"{actual_fps:.2f} FPS",
        f"FOURCC={fourcc_str}",
    )

    fx = args.fx
    fy = args.fy
    cx = args.cx
    cy = args.cy
    dist_coeffs = None

    if args.camera_matrix and os.path.exists(args.camera_matrix):
        cam = np.load(args.camera_matrix)
        if cam.shape[0] >= 3 and cam.shape[1] >= 3:
            fx = float(cam[0, 0])
            fy = float(cam[1, 1])
            cx = float(cam[0, 2])
            cy = float(cam[1, 2])
        else:
            raise ValueError("camera_matrix.npy must be at least 3x3")

    if args.dist_coeffs and os.path.exists(args.dist_coeffs):
        dist = np.load(args.dist_coeffs)
        dist_coeffs = dist.reshape((-1, 1)).astype(np.float64)

    tag_map = None
    if args.tag_map:
        tag_map = load_tag_map(args.tag_map)

    vision_engine.set_tag_family(args.family)
    vision_engine.configure_detector(
        nthreads=args.nthreads,
        quad_decimate=args.quad_decimate,
        quad_sigma=args.quad_sigma,
        refine_edges=args.refine_edges,
        decode_sharpening=args.decode_sharpening,
        debug=args.debug_images,
    )

    frame_count = 0
    last_report = time.time()
    last_cpu_report = time.process_time()
    last_cpu_percent = 0.0
    last_fps = 0.0
    last_latency_ms = 0.0
    last_cap_ms = 0.0
    last_pre_ms = 0.0
    last_det_ms = 0.0
    last_map_pose = None

    while True:
        frame_start = time.time()
        cap_start = time.time()
        ok, frame = cap.read()
        last_cap_ms = (time.time() - cap_start) * 1000.0
        if not ok:
            print("Failed to read camera frame.", file=sys.stderr)
            break

        pre_start = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if args.scale and args.scale != 1.0:
            new_size = (
                max(1, int(gray.shape[1] * args.scale)),
                max(1, int(gray.shape[0] * args.scale)),
            )
            gray = cv2.resize(gray, new_size, interpolation=cv2.INTER_LINEAR)
        if args.invert:
            gray = 255 - gray
        height, width = gray.shape[:2]
        last_pre_ms = (time.time() - pre_start) * 1000.0

        local_fx = fx if fx is not None else float(max(width, height))
        local_fy = fy if fy is not None else local_fx
        local_cx = cx if cx is not None else width / 2.0
        local_cy = cy if cy is not None else height / 2.0

        det_start = time.time()
        detections = vision_engine.detect_tags(
            gray, local_fx, local_fy, local_cx, local_cy, args.tag_size
        )
        last_det_ms = (time.time() - det_start) * 1000.0
        last_latency_ms = (time.time() - frame_start) * 1000.0

        odom_pose = None
        if tag_map:
            odom_pose = compute_camera_pose_from_map(detections, tag_map)
            if odom_pose:
                last_map_pose = odom_pose

        if args.display:
            draw_detections(frame, detections, scale=args.scale, show_pose=True)
            for det in detections:
                draw_pose_axes(
                    frame,
                    det,
                    local_fx,
                    local_fy,
                    local_cx,
                    local_cy,
                    args.tag_size,
                    dist_coeffs=dist_coeffs,
                    scale=args.scale,
                )
            overlay = [
                f"FPS: {last_fps:.1f}  CPU%: {last_cpu_percent:.1f}",
                (
                    f"Detections: {len(detections)}  "
                    f"Latency: {last_latency_ms:.1f} ms"
                ),
                (
                    f"cap={last_cap_ms:.1f}ms pre={last_pre_ms:.1f}ms "
                    f"det={last_det_ms:.1f}ms"
                ),
            ]
            if odom_pose:
                trans = odom_pose["translation"]
                roll, pitch, yaw = vision_engine.extract_euler_angles_cython(
                    odom_pose["rotation"]
                )
                rpy_deg = np.degrees([roll, pitch, yaw])
                overlay.append(
                    f"Map t=({trans[0]:.3f}, {trans[1]:.3f}, {trans[2]:.3f}) m"
                )
                overlay.append(
                    f"Map r=({rpy_deg[0]:.1f}, {rpy_deg[1]:.1f}, {rpy_deg[2]:.1f}) deg"
                )
            y = 20
            for line in overlay:
                cv2.putText(
                    frame,
                    line,
                    (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 0),
                    3,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    line,
                    (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                y += 22
            cv2.imshow("Apriltag Live", frame)

        frame_count += 1
        if time.time() - last_report >= 1.0:
            now = time.time()
            cpu_now = time.process_time()
            elapsed = now - last_report
            cpu_elapsed = cpu_now - last_cpu_report
            last_fps = frame_count / elapsed
            last_cpu_percent = (cpu_elapsed / elapsed) * 100.0 if elapsed > 0 else 0.0
            last_report = now
            last_cpu_report = cpu_now
            frame_count = 0
            map_info = ""
            if last_map_pose:
                trans = last_map_pose["translation"]
                roll, pitch, yaw = vision_engine.extract_euler_angles_cython(
                    last_map_pose["rotation"]
                )
                rpy_deg = np.degrees([roll, pitch, yaw])
                map_info = (
                    f" map t=({trans[0]:.2f}, {trans[1]:.2f}, {trans[2]:.2f}) "
                    f"r=({rpy_deg[0]:.1f}, {rpy_deg[1]:.1f}, {rpy_deg[2]:.1f}) deg"
                )
            if args.fps:
                print(
                    f"FPS: {last_fps:.1f}  detections: {len(detections)}  "
                    f"latency: {last_latency_ms:.1f} ms  cpu: {last_cpu_percent:.1f}%  "
                    f"cap={last_cap_ms:.1f}ms pre={last_pre_ms:.1f}ms "
                    f"det={last_det_ms:.1f}ms"
                    f"{map_info}"
                )

        if args.display:
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                ts = time.strftime("%Y%m%d_%H%M%S")
                color_path = f"{args.save_dir}/frame_{ts}.png"
                gray_path = f"{args.save_dir}/frame_{ts}_gray.png"
                cv2.imwrite(color_path, frame)
                cv2.imwrite(gray_path, gray)
                print(
                    f"Saved {color_path} and {gray_path} "
                    f"(detections: {len(detections)})"
                )

    cap.release()
    if args.display:
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
