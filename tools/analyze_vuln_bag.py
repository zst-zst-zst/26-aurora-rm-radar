#!/usr/bin/env python3
"""复盘脚本: 统计回放包中 /radar2sentry 每帧敌车数分布 + /match_info 易伤时间线"""
import sys, glob, os, collections, datetime
from mcap_ros2.reader import read_ros2_messages

def to_sec(t):
    if isinstance(t, datetime.datetime):
        return t.timestamp()
    return t / 1e9

def analyze(bag_dir, sample=1):
    files = sorted(glob.glob(os.path.join(bag_dir, '**', '*.mcap'), recursive=True))
    if not files:
        print(f"[SKIP] no mcap in {bag_dir}")
        return
    topic_counts = collections.Counter()
    enemy_hist = collections.Counter()
    self_hist = collections.Counter()
    radar_frames = 0
    marks_prev = None
    marks_rises = 0
    ult_max_opp = 0
    vuln_active_sec = 0.0
    vuln_last_t = None
    last_t = None
    self_color_votes = collections.Counter()
    t0 = None
    err = collections.Counter()
    for f in files:
        try:
            for m in read_ros2_messages(f):
                try:
                    tn = m.channel.topic
                    ts = to_sec(m.log_time)
                    if t0 is None:
                        t0 = ts
                    last_t = ts
                    topic_counts[tn] += 1
                    t = m.ros_msg
                    if tn == '/radar2sentry':
                        radar_frames += 1
                        if radar_frames % sample:
                            continue
                        n = sum(1 for x in t.radar_enemy_x if x > 0.0)
                        ns = sum(1 for x in t.radar_self_x if x > 0.0)
                        enemy_hist[n] += 1
                        self_hist[ns] += 1
                    elif tn == '/match_info':
                        self_color_votes[t.self_color] += 1
                        cur = tuple(t.marks)
                        if marks_prev is not None:
                            for c, p in zip(cur, marks_prev):
                                if c > 0 and p == 0:
                                    marks_rises += 1
                        marks_prev = cur
                        opp = t.ultimate & 0x03
                        vul = (t.ultimate >> 2) & 0x01
                        ult_max_opp = max(ult_max_opp, opp)
                        if vul and vuln_last_t is None:
                            vuln_last_t = ts
                        if not vul and vuln_last_t is not None:
                            vuln_active_sec += ts - vuln_last_t
                            vuln_last_t = None
                except Exception as e:
                    err[f"{type(e).__name__}: {e}"] += 1
        except Exception as e:
            print(f"  [WARN] {os.path.basename(f)}: {type(e).__name__}: {e}")
    dur = (last_t - t0) if (last_t and t0) else 0
    print(f"== {os.path.basename(bag_dir.rstrip('/'))} dur={dur:.0f}s")
    print("  topics:", dict(topic_counts.most_common(12)))
    if err:
        for k, v in err.most_common(3):
            print(f"  [decode-err] {v}x {k[:120]}")
    if radar_frames:
        tot = sum(enemy_hist.values())
        dist = ' '.join(f"{k}辆:{v*100//tot}%" for k, v in sorted(enemy_hist.items()))
        avg = sum(k*v for k, v in enemy_hist.items())/tot
        print(f"  radar2sentry frames={radar_frames} active_enemy_dist: {dist} avg={avg:.2f}")
        tots = sum(self_hist.values())
        dists = ' '.join(f"{k}:{v*100//tots}%" for k, v in sorted(self_hist.items()))
        print(f"  self_dist: {dists}")
    if self_color_votes:
        print(f"  self_color votes: {dict(self_color_votes)}")
    print(f"  marks rises(标记成功上升沿)={marks_rises} max_opportunities={ult_max_opp} vuln_active_total={vuln_active_sec:.0f}s")

if __name__ == '__main__':
    analyze(sys.argv[1], sample=int(sys.argv[2]) if len(sys.argv) > 2 else 1)
