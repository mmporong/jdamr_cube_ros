"""Which sensor measures an in-place rotation correctly: wheel odom or the gyro?

    python3 rotation_truth.py <bag> [<bag> ...]

For every rotation that starts and ends in a stationary window (odom twist 0 for >= 1 s),
or, with --turnarounds, between consecutive reversals of an oscillating in-place search
(single scans nearest each yaw-rate zero crossing, where the base is momentarily still),
the rotation is measured three ways:
  odom  : odom pose yaw change between the two windows
  gyro  : -gz integrated over the same span (z is inverted, see README), minus the gyro
          bias averaged over the two windows
  scan  : scan-to-scan fit of the two windows' median scans (no map needed); the base
          rotates about the laser only approximately, so x/y are searched too
Output: per rotation the three values and the odom/scan and gyro/scan ratios.
"""
import math
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RES = 0.01
TRUNC = 0.10
MIN_SCANS = 3
BIAS_OVERRIDE = 0.0   # --turnarounds has no still window; set from a stationary stretch


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(bag):
    r = rosbag2_py.SequentialReader()
    storage = 'mcap' if any(p.endswith('.mcap') for p in os.listdir(bag)) else 'sqlite3'
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    imu, odom, scans, laser = [], [], [], None
    while r.has_next():
        topic, raw, _t = r.read_next()
        if topic not in ('/imu/data_raw', '/odom', '/scan', '/tf_static'):
            continue
        m = deserialize_message(raw, get_message(types[topic]))
        if topic == '/tf_static':
            for tf in m.transforms:
                if 'laser' in tf.child_frame_id:
                    laser = (tf.transform.translation.x, tf.transform.translation.y,
                             yaw_of(tf.transform.rotation))
            continue
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if topic == '/imu/data_raw':
            imu.append((t, m.angular_velocity.z))
        elif topic == '/odom':
            odom.append((t, m.twist.twist.linear.x, m.twist.twist.angular.z,
                         yaw_of(m.pose.pose.orientation)))
        else:
            a = m.angle_min + m.angle_increment * np.arange(len(m.ranges))
            rr = np.array(m.ranges, dtype=float)
            ok = np.isfinite(rr) & (rr > m.range_min) & (rr < m.range_max)
            scans.append((t, a[ok], rr[ok]))
    return np.array(imu), np.array(odom), scans, laser


def stationary(odom, min_s=1.0):
    still = (np.abs(odom[:, 1]) < 1e-6) & (np.abs(odom[:, 2]) < 1e-6)
    out, i = [], 0
    while i < len(odom):
        if still[i]:
            j = i
            while j + 1 < len(odom) and still[j + 1]:
                j += 1
            if odom[j, 0] - odom[i, 0] >= min_s:
                out.append((odom[i, 0], odom[j, 0]))
            i = j + 1
        else:
            i += 1
    return out


def points(scans, t0, t1, laser):
    """Median scan over a stationary window, in the base frame (1 deg bins)."""
    nb = 720
    rows = []
    for t, a, r in scans:
        if t0 <= t <= t1:
            row = np.full(nb, np.nan)
            b = (np.floor(np.mod(a, 2 * math.pi) / (2 * math.pi) * nb).astype(int)) % nb
            row[b] = r
            rows.append(row)
    if len(rows) < MIN_SCANS:
        return None
    with np.errstate(all='ignore'):
        med = np.nanmedian(np.array(rows), axis=0)
    ang = (np.arange(nb) + 0.5) / nb * 2 * math.pi
    keep = np.isfinite(med) & (med < 4.0)
    xl, yl = med[keep] * np.cos(ang[keep]), med[keep] * np.sin(ang[keep])
    lx, ly, lyaw = laser
    return np.stack([lx + math.cos(lyaw) * xl - math.sin(lyaw) * yl,
                     ly + math.sin(lyaw) * xl + math.cos(lyaw) * yl], axis=1)


def fit(ref, cur, yaw0):
    """Pose of cur in ref's frame: coarse-to-fine search around yaw0."""
    lo = ref.min(axis=0) - 0.5
    hi = ref.max(axis=0) + 0.5
    w, h = (np.ceil((hi - lo) / RES).astype(int) + 1)
    img = np.ones((h, w), np.uint8)
    c = ((ref - lo) / RES).astype(int)
    img[c[:, 1], c[:, 0]] = 0
    dist = np.minimum(cv2.distanceTransform(img, cv2.DIST_L2, 5) * RES, TRUNC)

    def score(x, y, t):
        px = x + math.cos(t) * cur[:, 0] - math.sin(t) * cur[:, 1]
        py = y + math.sin(t) * cur[:, 0] + math.cos(t) * cur[:, 1]
        ix, iy = ((px - lo[0]) / RES).astype(int), ((py - lo[1]) / RES).astype(int)
        inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        v = np.full(len(cur), TRUNC)
        v[inside] = dist[iy[inside], ix[inside]]
        return v.mean()

    best = (score(0, 0, yaw0), 0.0, 0.0, yaw0)
    for span, step_xy, step_t in ((0.06, 0.01, math.radians(0.5)),
                                  (0.012, 0.002, math.radians(0.1)),
                                  (0.003, 0.0005, math.radians(0.02))):
        _, bx, by, bt = best
        for t in np.arange(bt - 5 * step_t, bt + 5 * step_t + 1e-12, step_t):
            for x in np.arange(bx - span, bx + span + 1e-12, step_xy):
                for y in np.arange(by - span, by + span + 1e-12, step_xy):
                    s = score(x, y, t)
                    if s < best[0]:
                        best = (s, x, y, t)
    return best


def turnarounds(odom, scans):
    """Windows of one scan at each yaw-rate sign change while driving straight ~0."""
    out = []
    wz, vx = odom[:, 2], odom[:, 1]
    times = np.array([s[0] for s in scans])
    for i in range(1, len(odom)):
        if wz[i - 1] * wz[i] < 0 and abs(vx[i]) < 1e-3:
            j = int(np.argmin(np.abs(times - odom[i, 0])))
            out.append((times[j] - 1e-3, times[j] + 1e-3))
    return out


def ekf_yaw(bag):
    """(t, yaw) of /odometry/filtered from an ekf_replay recording."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = []
    while r.has_next():
        topic, raw, _t = r.read_next()
        if topic == '/odometry/filtered':
            m = deserialize_message(raw, get_message(types[topic]))
            out.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                        yaw_of(m.pose.pose.orientation)))
    out = np.array(out)
    out[:, 1] = np.unwrap(out[:, 1])
    return out


def main():
    global MIN_SCANS, BIAS_OVERRIDE
    ekf = None
    for arg in sys.argv[1:]:
        if arg.startswith('--ekf='):
            ekf = ekf_yaw(arg.split('=', 1)[1])
    turn_mode = '--turnarounds' in sys.argv
    if turn_mode:
        MIN_SCANS = 1
    for arg in sys.argv[1:]:
        if arg.startswith('--bias='):
            BIAS_OVERRIDE = float(arg.split('=', 1)[1])
    for bag in [a for a in sys.argv[1:] if not a.startswith('--')]:
        imu, odom, scans, laser = read(bag)
        oyaw = np.unwrap(odom[:, 3])
        wins = turnarounds(odom, scans) if turn_mode else stationary(odom)
        print(f'== {os.path.basename(bag)}: {len(wins)} stationary windows')
        rows = []
        for (a0, a1), (b0, b1) in zip(wins, wins[1:]):
            ia, ib = np.searchsorted(odom[:, 0], a1), np.searchsorted(odom[:, 0], b0)
            d_odom = oyaw[ib] - oyaw[ia]
            moved = np.sum(np.abs(odom[ia:ib, 1]) * np.diff(odom[ia:ib + 1, 0]))
            if abs(d_odom) < math.radians(3) or moved > 0.02 or b0 - a1 > 20:
                continue
            if turn_mode:
                bias = BIAS_OVERRIDE
            else:
                bias = np.mean(np.concatenate([imu[(imu[:, 0] >= a0) & (imu[:, 0] <= a1), 1],
                                               imu[(imu[:, 0] >= b0) & (imu[:, 0] <= b1), 1]]))
            k = (imu[:, 0] >= a1) & (imu[:, 0] <= b0)
            d_gyro = -np.sum((imu[k, 1][1:] - bias) * np.diff(imu[k, 0]))
            pa, pb = points(scans, a0, a1, laser), points(scans, b0, b1, laser)
            if pa is None or pb is None:
                continue
            s, x, y, d_scan = fit(pa, pb, d_odom)
            d_ekf = (np.interp(b0, ekf[:, 0], ekf[:, 1]) - np.interp(a1, ekf[:, 0], ekf[:, 1])
                     if ekf is not None else float('nan'))
            rows.append((d_odom, d_gyro, d_scan, s, d_ekf))
            print(f'  odom {math.degrees(d_odom):+7.2f}  gyro {math.degrees(d_gyro):+7.2f}  '
                  f'scan {math.degrees(d_scan):+7.2f} (fit {s * 100:.2f} cm, shift {x * 100:+.1f},'
                  f'{y * 100:+.1f} cm)  odom/scan {d_odom / d_scan:.4f}  gyro/scan '
                  f'{d_gyro / d_scan:.4f}  bias {bias:+.5f}  ekf {math.degrees(d_ekf):+7.2f}')
        if rows:
            r = np.array(rows)
            good = r[:, 3] < 0.02
            cols = (('odom', 0), ('gyro', 1)) + ((('ekf', 4),) if ekf is not None else ())
            for name, col in cols:
                ratio = r[good, col] / r[good, 2]
                err = np.degrees(np.abs(r[good, col] - r[good, 2]))
                print(f'  {name}/scan ratio median {np.median(ratio):.4f} '
                      f'(n={good.sum()}, fit<2 cm), |error| median {np.median(err):.2f} deg '
                      f'max {err.max():.2f} deg')


if __name__ == '__main__':
    main()
