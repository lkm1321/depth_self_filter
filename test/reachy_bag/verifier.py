#!/usr/bin/env python3
"""Save 4-panel comparisons: input depth | rendered robot depth | filtered | removed.
The rendered-robot panel shows exactly what the renderer produced, so we can tell
render errors (missing/mis-posed geometry) from mask/margin errors."""
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

OUT = "/work/out"
MAX_DEPTH_M = 2.5
SAVE_EVERY = 120
MAX_SAVES = 8


def colorize(depth_m):
    d = np.clip(depth_m / MAX_DEPTH_M, 0, 1)
    col = cv2.applyColorMap((d * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    col[depth_m <= 0] = (0, 0, 0)
    return col


class Verifier(Node):
    def __init__(self):
        super().__init__("depth_filter_verifier")
        os.makedirs(OUT, exist_ok=True)
        self.inputs = {}
        self.robot = {}
        self.n_filtered = 0
        self.saves = 0
        self.tot_valid = 0
        self.tot_removed = 0
        self.tot_rob = 0        # rendered-robot px that also have a measured depth
        self.tot_agree = 0      # of those, how many where |meas-render|<tol (aligned)
        # best-shift search accumulators
        self.bs_n = 0
        self.bs_dx = 0.0
        self.bs_dy = 0.0
        self.bs_frac0 = 0.0     # alignment at zero shift
        self.bs_fracbest = 0.0  # alignment at best shift
        self.TOLS = [0.03, 0.05, 0.10, 0.20, 0.50]
        self.tolcount = {t: 0 for t in self.TOLS}
        self.diff_sum = 0.0
        self.diff_absum = 0.0
        self.diff_samples = []
        self.create_subscription(Image, "/teleop_camera/depth/image_raw",
                                 self.on_input, qos_profile_sensor_data)
        self.create_subscription(Image, "/robot_depth",
                                 self.on_robot, qos_profile_sensor_data)
        self.create_subscription(Image, "/depth_self_filtered",
                                 self.on_filtered, qos_profile_sensor_data)
        self.get_logger().info("verifier up")

    @staticmethod
    def key(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    @staticmethod
    def trim(d):
        for k in sorted(d)[:-100]:
            del d[k]

    def on_input(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        self.inputs[self.key(msg)] = arr.astype(np.float32) * 1e-3
        if len(self.inputs) > 200:
            self.trim(self.inputs)

    def on_robot(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        self.robot[self.key(msg)] = arr
        if len(self.robot) > 200:
            self.trim(self.robot)

    def on_filtered(self, msg):
        k = self.key(msg)
        inp = self.inputs.get(k)
        if inp is None:
            return
        out = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        rob = self.robot.get(k)
        valid = inp > 0
        removed = valid & ~np.isfinite(out)
        nv, nr = int(valid.sum()), int(removed.sum())
        self.tot_valid += nv
        self.tot_removed += nr
        self.n_filtered += 1

        # Alignment metric: of rendered-robot pixels that have a measured depth,
        # what fraction has measured ~= rendered (robot surface actually there).
        fa = -1.0
        if rob is not None:
            both = np.isfinite(rob) & (inp > 0)
            nb = int(both.sum())
            if nb > 0:
                diff = inp[both] - rob[both]   # measured - rendered (m)
                agree = int((np.abs(diff) < 0.05).sum())
                self.tot_rob += nb
                self.tot_agree += agree
                fa = 100.0 * agree / nb
                for t in self.TOLS:
                    self.tolcount[t] += int((np.abs(diff) < t).sum())
                self.diff_sum += float(diff.sum())
                self.diff_absum += float(np.abs(diff).sum())
                self.diff_samples.extend(diff[::37].tolist())  # sparse sample for median

            # Periodic best-shift search: what single (dx,dy) pixel translation of
            # the rendered robot best matches the measured surface? Recovers the
            # nature of the misalignment (pure translation => extrinsic/principal).
            if nb > 3000 and self.n_filtered % 15 == 0:
                def frac(dx, dy):
                    rs = np.roll(np.roll(rob, dy, 0), dx, 1)
                    m = np.isfinite(rs) & (inp > 0)
                    c = int(m.sum())
                    if c < 1000:
                        return -1.0, 0
                    return float((np.abs(inp[m] - rs[m]) < 0.05).mean()), c
                best = (-1.0, 0, 0)
                for dy in range(-24, 25, 4):
                    for dx in range(-24, 25, 4):
                        f, _ = frac(dx, dy)
                        if f > best[0]:
                            best = (f, dx, dy)
                f0, _ = frac(0, 0)
                if best[0] >= 0 and f0 >= 0:
                    self.bs_n += 1
                    self.bs_dx += best[1]
                    self.bs_dy += best[2]
                    self.bs_frac0 += f0
                    self.bs_fracbest += best[0]

        if self.n_filtered % SAVE_EVERY == 1 and self.saves < MAX_SAVES:
            self.saves += 1
            left = colorize(inp)
            robot_panel = colorize(np.where(np.isfinite(rob), rob, 0.0)) if rob is not None \
                else np.zeros_like(left)
            # Overlay the rendered-robot OUTLINE (green) on the input so any
            # misalignment between rendered and real robot is directly visible.
            if rob is not None:
                robmask = np.isfinite(rob).astype(np.uint8)
                cnts, _ = cv2.findContours(robmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(left, cnts, -1, (0, 255, 0), 2)
            filt = colorize(np.where(np.isfinite(out), out, 0.0))
            overlay = colorize(inp)
            overlay[removed] = (0, 0, 255)
            nrob = int(np.isfinite(rob).sum()) if rob is not None else -1
            pct = 100.0 * nr / max(nv, 1)
            label = f"f{self.n_filtered}  robot_px={nrob}  removed {nr}/{nv}={pct:.1f}%"
            strip = np.hstack([left, robot_panel, filt, overlay])
            cv2.putText(strip, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(strip, "input | ROBOT-RENDER | filtered | removed(red)",
                        (10, msg.height - 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 1, cv2.LINE_AA)
            path = os.path.join(OUT, f"cmp_{self.saves:02d}.png")
            cv2.imwrite(path, strip)
            self.get_logger().info(f"saved {path}  ({label})")

    def report(self):
        pct = 100.0 * self.tot_removed / max(self.tot_valid, 1)
        align = 100.0 * self.tot_agree / max(self.tot_rob, 1)
        self.get_logger().info(
            f"TOTAL over {self.n_filtered} frames: removed "
            f"{self.tot_removed}/{self.tot_valid} = {pct:.2f}%  |  "
            f"ALIGNMENT {align:.1f}% of rendered-robot px match measured "
            f"(offset={os.environ.get('TOFF','?')})")
        if self.tot_rob:
            curve = "  ".join(f"<{int(t*100)}cm:{100*self.tolcount[t]/self.tot_rob:.0f}%"
                              for t in self.TOLS)
            med = float(np.median(self.diff_samples)) if self.diff_samples else float("nan")
            self.get_logger().info(
                f"DEPTH-DIFF (measured-rendered) over rendered-robot px: "
                f"mean={self.diff_sum/self.tot_rob:+.3f}m mean|.|={self.diff_absum/self.tot_rob:.3f}m "
                f"median={med:+.3f}m | tol curve: {curve}")
        if self.bs_n:
            self.get_logger().info(
                f"BEST-SHIFT over {self.bs_n} frames: mean dx={self.bs_dx/self.bs_n:+.1f}px "
                f"dy={self.bs_dy/self.bs_n:+.1f}px  align@0={100*self.bs_frac0/self.bs_n:.1f}% "
                f"-> align@best={100*self.bs_fracbest/self.bs_n:.1f}%")


def main():
    rclpy.init()
    v = Verifier()
    try:
        rclpy.spin(v)
    except KeyboardInterrupt:
        pass
    finally:
        v.report()
        v.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
