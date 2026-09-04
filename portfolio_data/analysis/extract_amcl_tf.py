#!/usr/bin/env python3
"""corridor_keepout_roundtrip_20260901T150446 bag에서
   (1) /amcl_pose 공분산 시계열, (2) /tf map->odom 지연 분포를 CSV로 뽑는다."""
import csv, math, sys, os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage

BAG = sys.argv[1]
OUT = sys.argv[2]
os.makedirs(OUT, exist_ok=True)

reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id='mcap'),
            ConverterOptions('cdr', 'cdr'))

amcl_rows, tf_rows = [], []
t0 = None
last_mo_recv = None

while reader.has_next():
    topic, data, recv_ns = reader.read_next()
    if t0 is None:
        t0 = recv_ns
    if topic == '/amcl_pose':
        m = deserialize_message(data, PoseWithCovarianceStamped)
        c = m.pose.covariance
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        stamp = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        amcl_rows.append({
            't_rel_s': round((recv_ns - t0)/1e9, 3),
            'stamp_s': round(stamp, 6),
            'x': round(m.pose.pose.position.x, 4),
            'y': round(m.pose.pose.position.y, 4),
            'yaw_deg': round(math.degrees(yaw), 3),
            'cov_xx': f'{c[0]:.6g}',
            'cov_xy': f'{c[1]:.6g}',
            'cov_yy': f'{c[7]:.6g}',
            'cov_yawyaw': f'{c[35]:.6g}',
            'trace_xy': f'{c[0]+c[7]:.6g}',
            'sigma_x_m': f'{math.sqrt(max(c[0],0)):.5f}',
            'sigma_y_m': f'{math.sqrt(max(c[7],0)):.5f}',
            'sigma_yaw_deg': f'{math.degrees(math.sqrt(max(c[35],0))):.4f}',
        })
    elif topic == '/tf':
        m = deserialize_message(data, TFMessage)
        for tr in m.transforms:
            if tr.header.frame_id.lstrip('/') == 'map' and tr.child_frame_id.lstrip('/') == 'odom':
                stamp = tr.header.stamp.sec + tr.header.stamp.nanosec*1e-9
                recv = recv_ns/1e9
                gap = None if last_mo_recv is None else round(recv - last_mo_recv, 6)
                last_mo_recv = recv
                tf_rows.append({
                    't_rel_s': round((recv_ns - t0)/1e9, 3),
                    'stamp_s': round(stamp, 6),
                    'recv_s': round(recv, 6),
                    'latency_ms': round((recv - stamp)*1000, 3),
                    'gap_since_prev_ms': '' if gap is None else round(gap*1000, 3),
                    'tx': round(tr.transform.translation.x, 5),
                    'ty': round(tr.transform.translation.y, 5),
                })

def dump(name, rows):
    p = os.path.join(OUT, name)
    with open(p, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f'{p}: {len(rows)} rows')

dump('amcl_pose_covariance.csv', amcl_rows)
dump('tf_map_odom_latency.csv', tf_rows)

def stats(vals):
    v = sorted(vals); n = len(v)
    def pct(p): return v[min(n-1, int(round(p/100*(n-1))))]
    return dict(n=n, min=round(v[0],3), p50=round(pct(50),3), p90=round(pct(90),3),
                p95=round(pct(95),3), p99=round(pct(99),3), max=round(v[-1],3),
                mean=round(sum(v)/n,3))

lat = [r['latency_ms'] for r in tf_rows]
gaps = [r['gap_since_prev_ms'] for r in tf_rows if r['gap_since_prev_ms'] != '']
tr = [float(r['trace_xy']) for r in amcl_rows]
print('\n[map->odom latency ms]', stats(lat))
print('[map->odom gap ms]', stats(gaps))
print('[amcl trace(xx+yy)]', stats(tr))
print('trace>=2.0 count:', sum(1 for x in tr if x >= 2.0), '/', len(tr))
print('trace>=1.0 count:', sum(1 for x in tr if x >= 1.0))
print('trace>=0.5 count:', sum(1 for x in tr if x >= 0.5))
