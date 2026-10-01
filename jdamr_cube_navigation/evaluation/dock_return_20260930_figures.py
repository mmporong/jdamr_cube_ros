"""
Render the 2026-09-30 dock-return figures from the preserved data package.

Inputs are read from ``--data`` (default ``$HOME/jdamr_data/dock_return_success_20260930``);
nothing is fetched from the robot. Poses, cells and times come from the files named in
``20260930_DOCK_RETURN_PORTFOLIO_HANDOFF.md``; no path or timing is interpolated.
The Nav2 planned path to the staging pose was not recorded, so only its endpoints
are drawn.
"""
import argparse
import json
import math
from pathlib import Path

import matplotlib
from matplotlib import font_manager
from matplotlib.patches import Polygon, Rectangle
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml

matplotlib.use('Agg')

FOOTPRINT = [(-0.295, -0.29), (0.085, -0.29), (0.085, 0.29), (-0.295, 0.29)]
LASER_IN_BASE = (-0.01, 0.0, math.pi)   # base_link -> laser_link, /tf_static at capture
# Event times (KST) from logs/nav2_session_*.log, run/home_events.jsonl and the init output.
TIMELINE = [
    ('Nav2 세션 재시작', '11:56:30', '11:57:08'),
    ('스캔 정합 초기화', '11:57:17', '11:58:42'),
    ('staging까지 주행', '11:59:11', '12:00:57'),
    ('정지 확인', '12:00:57', '12:00:58'),
    ('후진 도킹', '12:01:00', '12:01:52'),
]


def korean_font():
    # The matplotlib cache may predate the Nanum package; register it when present.
    for path in ('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',):
        if Path(path).exists():
            font_manager.fontManager.addfont(path)
    for name in ('NanumGothic', 'Noto Sans CJK KR', 'Noto Sans CJK JP'):
        if any(name == f.name for f in font_manager.fontManager.ttflist):
            plt.rcParams['font.family'] = name
            break
    plt.rcParams['axes.unicode_minus'] = False


def load_grid(yaml_path):
    meta = yaml.safe_load(Path(yaml_path).read_text())
    image = np.array(Image.open(Path(yaml_path).parent / meta['image']))
    res = float(meta['resolution'])
    ox, oy = (float(v) for v in meta['origin'][:2])
    height, width = image.shape
    # imshow order: (left, right, bottom, top)
    return image, res, (ox, ox + width * res, oy, oy + height * res)


def world_footprint(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in FOOTPRINT]


def scan_points(scan_json, pose, n_scans=5):
    data = json.loads(Path(scan_json).read_text())
    lx, ly, lyaw = LASER_IN_BASE
    x, y, yaw = pose
    points = []
    for scan in data['scans'][:n_scans]:
        for i, r in enumerate(scan['ranges']):
            if r is None or r < scan['range_min']:
                continue
            a = scan['angle_min'] + i * scan['angle_increment'] + lyaw
            bx, by = lx + r * math.cos(a), ly + r * math.sin(a)
            points.append((x + math.cos(yaw) * bx - math.sin(yaw) * by,
                           y + math.sin(yaw) * bx + math.cos(yaw) * by))
    return np.array(points)


def draw_map(ax, map_yaml, keepout_yaml, released=None):
    image, res, extent = load_grid(map_yaml)
    keep, _, _ = load_grid(keepout_yaml)
    rgb = np.ones(image.shape + (3,))
    rgb[(image > 100) & (image < 250)] = (0.82, 0.82, 0.82)          # unknown
    rgb[keep < 100] = (1.0, 0.80, 0.80)                               # keepout
    if released is not None:
        rgb[released] = (0.80, 0.93, 0.80)                            # released keepout
    rgb[image < 100] = (0.15, 0.15, 0.15)                             # occupied
    ax.imshow(rgb, extent=extent, origin='upper', interpolation='nearest')
    return res, extent, image.shape


def mark_cells(ax, cells, res, extent, shape, color, label):
    height = shape[0]
    for index, (row, col, *_rest) in enumerate(cells):
        x0 = extent[0] + col * res
        y0 = extent[2] + (height - 1 - row) * res
        ax.add_patch(Rectangle((x0, y0), res, res, fill=False, edgecolor=color, lw=1.6,
                               label=label if index == 0 else None))


def pose_arrow(ax, pose, color, label, length=0.22):
    x, y, yaw = pose
    ax.annotate('', xy=(x + length * math.cos(yaw), y + length * math.sin(yaw)), xytext=(x, y),
                arrowprops={'arrowstyle': '-|>', 'color': color, 'lw': 2})
    ax.plot([x], [y], 'o', color=color, ms=5, label=label)


def figure_overview(data, out):
    assets = data / 'assets'
    registry = yaml.safe_load((assets / 'service_destinations.yaml').read_text())
    home = registry['home']
    dock = (home['x_m'], home['y_m'], home['yaw_rad'])
    offset = home['approach_offset_m']
    stage = (dock[0] + offset * math.cos(dock[2]), dock[1] + offset * math.sin(dock[2]), dock[2])
    init = json.loads((data / 'init/activation.jsonl').read_text().splitlines()[-1])
    start = tuple(init['stationary_pose'])
    final = json.loads((data / 'final_pose/final_pose_vs_dock.json').read_text())
    matched = final['scan_match']['pose']
    matched = (matched[0], matched[1], math.radians(matched[2]))
    provenance = json.loads((assets / 'dockfix_provenance.json').read_text())
    old_keep, _, _ = load_grid(assets / 'keepout.yaml')
    new_keep, _, _ = load_grid(assets / 'keepout_dockfix.yaml')
    released = (old_keep < 100) & (new_keep >= 100)

    fig, ax = plt.subplots(figsize=(10, 6.4), dpi=150)
    res, extent, shape = draw_map(ax, assets / 'map_dockfix.yaml',
                                  assets / 'keepout_dockfix.yaml', released)
    mark_cells(ax, provenance['map_cells_cleared_rc_prev_xy'], res, extent, shape,
               '#e67e00', '지도 흔적 수정 칸')
    points = scan_points(data / 'final_pose/scan.json', matched)
    ax.scatter(points[:, 0], points[:, 1], s=1.2, c='#d62728', label='도크 정지 후 LiDAR')
    ax.add_patch(Polygon(world_footprint(stage), closed=True, fill=False, ls='--',
                         ec='#1f77b4', lw=1.5, label='staging 차체'))
    ax.add_patch(Polygon(world_footprint(dock), closed=True, fill=False,
                         ec='#2ca02c', lw=2.0, label='도크 목표 차체'))
    ax.plot([stage[0], dock[0]], [stage[1], dock[1]], color='#2ca02c', lw=1.2, ls=':',
            label='후진 경로(직선)')
    pose_arrow(ax, start, '#9467bd', '출발 자세(초기화)')
    pose_arrow(ax, stage, '#1f77b4', 'staging 목표')
    pose_arrow(ax, dock, '#2ca02c', '도크 목표')
    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-1.45, 0.75)
    ax.set_aspect('equal')
    ax.set_xlabel('map x (m)')
    ax.set_ylabel('map y (m)')
    ax.set_title('충전소 후진 도킹 (2026-09-30, 지도 0.05 m/칸, 연녹색: 해제한 keepout)')
    ax.legend(loc='upper right', fontsize=7.5, framealpha=0.95)
    fig.tight_layout()
    fig.savefig(out / 'dock_return_20260930_overview.png')
    plt.close(fig)


def figure_dock_detail(data, out):
    assets = data / 'assets'
    registry = yaml.safe_load((assets / 'service_destinations.yaml').read_text())
    home = registry['home']
    dock = (home['x_m'], home['y_m'], home['yaw_rad'])
    offset = home['approach_offset_m']
    stage = (dock[0] + offset * math.cos(dock[2]), dock[1] + offset * math.sin(dock[2]), dock[2])
    provenance = json.loads((assets / 'dockfix_provenance.json').read_text())
    refine = json.loads((data / 'init/dock_init_refine_105836.json').read_text())
    points = scan_points(data / 'init/dock_init_scan_105836.json', refine['pose'], n_scans=20)
    fig, axes = plt.subplots(1, 2, figsize=(10, 5.6), dpi=150, sharey=True)
    panels = (('수정 전: 정적 경로 검사 차단', 'local_updated.yaml', 'keepout.yaml', False),
              ('수정 후: 회전·후진 통로 통과', 'map_dockfix.yaml', 'keepout_dockfix.yaml', True))
    for ax, (title, map_name, keep_name, after) in zip(axes, panels):
        res, extent, shape = draw_map(ax, assets / map_name, assets / keep_name)
        mark_cells(ax, provenance['map_cells_cleared_rc_prev_xy'], res, extent, shape,
                   '#e67e00', '흔적 칸(점유·미확인)')
        for pose, color, label, style in ((stage, '#1f77b4', 'staging 차체', '--'),
                                          (dock, '#2ca02c', '도크 차체', '-')):
            ax.add_patch(Polygon(world_footprint(pose), closed=True, fill=False, ls=style,
                                 ec=color, lw=1.8, label=label))
        circle = plt.Circle(stage[:2], math.hypot(0.295, 0.29), fill=False, ec='#1f77b4',
                            ls=':', lw=1.0, label='staging 회전 원 0.414 m')
        ax.add_patch(circle)
        if after:
            ax.scatter(points[:, 0], points[:, 1], s=1.0, c='#d62728',
                       label='도크 LiDAR(초기화 10:58)')
        ax.set_xlim(-1.35, 0.05)
        ax.set_ylim(-1.3, 0.35)
        ax.set_aspect('equal')
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('map x (m)')
        ax.legend(loc='lower left', fontsize=7, framealpha=0.95)
    axes[0].set_ylabel('map y (m)')
    fig.suptitle('도크 통로의 지도 흔적 3칸과 keepout 경계 (근거: 도크 LiDAR 빔 통과, 차체 내부 위치)',
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(out / 'dock_return_20260930_dock_detail.png')
    plt.close(fig)


def seconds(clock):
    h, m, s = (int(v) for v in clock.split(':'))
    return h * 3600 + m * 60 + s


def figure_timeline(out):
    origin = seconds(TIMELINE[0][1])
    fig, ax = plt.subplots(figsize=(10, 2.6), dpi=150)
    colors = ('#7f7f7f', '#9467bd', '#1f77b4', '#ff7f0e', '#2ca02c')
    for row, ((label, begin, end), color) in enumerate(zip(TIMELINE, colors)):
        left, width = seconds(begin) - origin, max(seconds(end) - seconds(begin), 1)
        ax.barh(len(TIMELINE) - 1 - row, width, left=left, color=color)
        ax.text(left + width + 3, len(TIMELINE) - 1 - row,
                f'{label}  {begin}–{end} ({seconds(end) - seconds(begin)} s)', va='center',
                fontsize=8)
    ax.set_yticks([])
    ax.set_xlim(0, seconds('12:03:10') - origin)
    ax.set_xlabel('11:56:30 기준 경과 시간 (s)')
    ax.set_title('충전소 복귀 실행 시간축 (2026-09-30, KST), 12:01:52 도크 정위치 정지(현장 육안 확인)',
                 fontsize=10)
    for side in ('top', 'right', 'left'):
        ax.spines[side].set_visible(False)
    fig.tight_layout()
    fig.savefig(out / 'dock_return_20260930_timeline.png')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path,
                        default=Path.home() / 'jdamr_data/dock_return_success_20260930')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    korean_font()
    figure_overview(args.data, args.out)
    figure_dock_detail(args.data, args.out)
    figure_timeline(args.out)


if __name__ == '__main__':
    main()
