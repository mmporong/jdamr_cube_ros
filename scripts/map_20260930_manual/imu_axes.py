"""Find how the IMU board is mounted from a bag with /odom and /imu/data_raw.

    python3 imu_axes.py <bag> [<bag> ...]      (ROS 2 Jazzy for rosbag2_py)

z: least-squares slope of gyro z against the wheel yaw rate while turning
(+1 means the board's z points up like base_link, -1 down).
x/y: the board's horizontal axes against the wheel forward acceleration dv/dt and
the centripetal acceleration v*w. Both sides are smoothed over SMOOTH_S and have a
BASELINE_S moving average removed, so gravity from a tilted board and slow drift drop
out. The fit gives each board axis as a unit vector in base_link (x forward, y left).
Prints one JSON line per bag and a combined line.
"""
import json
import math
import sys

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

RATE_HZ = 50.0
SMOOTH_S = 0.5
BASELINE_S = 8.0
TURNING_RADPS = 0.05
MOVING_ACCEL_MPS2 = 0.03


def read(bag):
    storage = 'mcap' if any(p.endswith('.mcap') for p in _files(bag)) else 'sqlite3'
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/odom', '/imu/data_raw']))
    odom, imu = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/odom':
            m = deserialize_message(raw, Odometry)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            odom.append((t, m.twist.twist.linear.x, m.twist.twist.angular.z))
        else:
            m = deserialize_message(raw, Imu)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            a, w = m.linear_acceleration, m.angular_velocity
            imu.append((t, a.x, a.y, a.z, w.x, w.y, w.z))
    return np.array(sorted(odom)), np.array(sorted(imu))


def _files(bag):
    from pathlib import Path
    return [str(p) for p in Path(bag).iterdir()]


def smooth(x, seconds):
    n = max(1, int(round(seconds * RATE_HZ)))
    return np.convolve(x, np.ones(n) / n, mode='same')


def high_pass(x):
    return smooth(x, SMOOTH_S) - smooth(x, BASELINE_S)


def analyse(odom, imu):
    t0, t1 = max(odom[0, 0], imu[0, 0]), min(odom[-1, 0], imu[-1, 0])
    t = np.arange(t0, t1, 1.0 / RATE_HZ)
    v = np.interp(t, odom[:, 0], odom[:, 1])
    w = np.interp(t, odom[:, 0], odom[:, 2])
    ax, ay, az, gx, gy, gz = (np.interp(t, imu[:, 0], imu[:, i]) for i in range(1, 7))
    turning = np.abs(w) > TURNING_RADPS
    zs = float(np.dot(gz[turning], w[turning]) / np.dot(w[turning], w[turning])) \
        if turning.sum() > 50 else float('nan')
    vs = smooth(v, SMOOTH_S)
    a_fwd = high_pass(np.gradient(vs, 1.0 / RATE_HZ))
    a_lat = high_pass(v * w)
    hx, hy = high_pass(ax), high_pass(ay)
    use = (np.abs(a_fwd) > MOVING_ACCEL_MPS2) | (np.abs(a_lat) > MOVING_ACCEL_MPS2)
    out = {'seconds': round(float(t1 - t0), 1), 'turning_samples': int(turning.sum()),
           'gyro_z_per_wheel_yaw': round(zs, 3), 'still_accel_z': round(float(np.median(
               az[(np.abs(v) < 1e-6) & (np.abs(w) < 1e-6)])) if np.any(
               (np.abs(v) < 1e-6) & (np.abs(w) < 1e-6)) else float('nan'), 2),
           'accel_samples': int(use.sum())}
    if use.sum() < 100:
        return out, None
    basis = np.column_stack([a_fwd[use], a_lat[use]])
    fit = {}
    for name, h in (('board_x', hx), ('board_y', hy)):
        coef, *_ = np.linalg.lstsq(basis, h[use], rcond=None)
        pred = basis @ coef
        r = float(np.corrcoef(pred, h[use])[0, 1]) if np.std(pred) > 0 else 0.0
        fit[name] = {'base_xy': [round(float(c), 3) for c in coef], 'r': round(r, 3)}
    out['fit'] = fit
    return out, (basis, hx[use], hy[use])


def main():
    stacked = []
    for bag in sys.argv[1:]:
        odom, imu = read(bag)
        if len(odom) < 100 or len(imu) < 100:
            print(json.dumps({'bag': bag, 'skipped': 'no /odom or /imu/data_raw'}))
            continue
        out, rows = analyse(odom, imu)
        print(json.dumps({'bag': bag, **out}))
        if rows is not None:
            stacked.append(rows)
    if not stacked:
        return
    basis = np.vstack([s[0] for s in stacked])
    combined = {}
    for i, name in ((1, 'board_x'), (2, 'board_y')):
        h = np.concatenate([s[i] for s in stacked])
        coef, *_ = np.linalg.lstsq(basis, h, rcond=None)
        combined[name] = {'base_xy': [round(float(c), 3) for c in coef],
                          'r': round(float(np.corrcoef(basis @ coef, h)[0, 1]), 3),
                          'angle_deg': round(math.degrees(math.atan2(coef[1], coef[0])), 1)}
    print(json.dumps({'combined': combined, 'samples': int(len(basis))}))


if __name__ == '__main__':
    main()
