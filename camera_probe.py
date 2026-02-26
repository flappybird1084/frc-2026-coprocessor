import os
import subprocess
import cv2


def list_video_devices():
    devs = []
    for name in sorted(os.listdir("/dev")):
        if name.startswith("video") and name[5:].isdigit():
            devs.append(f"/dev/{name}")
    return devs


def v4l2_info(dev):
    try:
        result = subprocess.run(
            ["v4l2-ctl", "--device", dev, "--all"],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return {"error": "v4l2-ctl not found"}

    info = {"returncode": result.returncode}
    if result.stdout:
        for line in result.stdout.splitlines():
            line = line.strip()
            if line.startswith("Driver name"):
                info["driver"] = line.split(":", 1)[1].strip()
            if line.startswith("Card type"):
                info["card"] = line.split(":", 1)[1].strip()
            if line.startswith("Bus info"):
                info["bus"] = line.split(":", 1)[1].strip()
    if result.stderr:
        info["stderr"] = result.stderr.strip()
    return info


def try_open(dev, backend=None):
    cap = cv2.VideoCapture(dev) if backend is None else cv2.VideoCapture(dev, backend)
    ok = cap.isOpened()
    cap.release()
    return ok


def main():
    devs = list_video_devices()
    if not devs:
        print("No /dev/video* devices found.")
        return

    print(f"Found devices: {', '.join(devs)}")
    for dev in devs:
        print(f"\n== {dev} ==")
        info = v4l2_info(dev)
        if "error" in info:
            print(f"v4l2-ctl: {info['error']}")
        else:
            for key in ("card", "driver", "bus"):
                if key in info:
                    print(f"{key}: {info[key]}")
            if "stderr" in info:
                print(f"v4l2-ctl stderr: {info['stderr']}")

        # Try common OpenCV open paths
        ok_default = try_open(dev)
        ok_v4l2 = try_open(dev, cv2.CAP_V4L2)
        ok_any = try_open(dev, cv2.CAP_ANY)
        print(f"OpenCV open default: {ok_default}")
        print(f"OpenCV open V4L2:    {ok_v4l2}")
        print(f"OpenCV open CAP_ANY: {ok_any}")


if __name__ == "__main__":
    main()
