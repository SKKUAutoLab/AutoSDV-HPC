#!/usr/bin/env python3
"""frame_id_join.py — application CSV ↔ LTTng UST trace event 조인.

목적:
  ros2_tracing이 잡는 trace event(rmw_take, rmw_publish, callback_*)에는
  application layer 필드인 frame_id가 직접 없다. 노드 콜백에서 따로 박은
  CSV(entry_*.csv, exit_*.csv)와 trace event를 (topic, steady_clock) 기준으로
  조인해서 frame_id가 attach된 latency 데이터프레임을 만든다.

산출:
  - per-frame T_HPC = T_HPC_out − T_HPC_in
  - frame_id 단위로 in/out timestamps + 매칭 정확도 리포트

전제:
  - 세션 디렉토리 구조:  <trace_base>/<session>/
      ├── ust/<host>/<session>/  (LTTng CTF)
      └── csv/                    (entry_*.csv, exit_*.csv)
  - CSV 컬럼: frame_id,topic,event_type,steady_clock_ns,wall_clock_ns
  - trace event timestamp는 nanosecond, CLOCK_MONOTONIC.

사용:
  python3 frame_id_join.py /tmp/autosdv_traces/autosdv_hpc_S1_run01
  python3 frame_id_join.py /tmp/autosdv_traces/autosdv_hpc_S1_run01 --out latency.parquet
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd

try:
    import bt2  # python3-bt2 (Babeltrace2 Python bindings)
except ImportError:
    print("[ERROR] python3-bt2 가 필요합니다: apt install python3-bt2", file=sys.stderr)
    sys.exit(1)


def load_application_csv(session_dir: Path) -> pd.DataFrame:
    """entry_*.csv, exit_*.csv 전부 읽어서 한 DataFrame으로."""
    csv_dir = session_dir / "csv"
    if not csv_dir.exists():
        raise FileNotFoundError(f"CSV directory not found: {csv_dir}")

    frames = []
    for p in sorted(csv_dir.glob("*.csv")):
        df = pd.read_csv(p, dtype={"frame_id": str})
        df["source_file"] = p.name
        frames.append(df)

    if not frames:
        raise RuntimeError(f"No CSV files in {csv_dir}")

    return pd.concat(frames, ignore_index=True)


def load_trace_events(session_dir: Path) -> pd.DataFrame:
    """Babeltrace2로 ros2:rmw_take, ros2:rmw_publish 이벤트만 추출."""
    ust_dir = session_dir / "ust"
    if not ust_dir.exists():
        raise FileNotFoundError(f"UST directory not found: {ust_dir}")

    msg_iter = bt2.TraceCollectionMessageIterator(str(ust_dir))

    rows = []
    target_events = {"ros2:rmw_publish", "ros2:rmw_take"}

    for msg in msg_iter:
        if not isinstance(msg, bt2._EventMessageConst):
            continue
        ev = msg.event
        if ev.name not in target_events:
            continue

        # CLOCK_MONOTONIC ns (LTTng default monotonic clock)
        ts_ns = msg.default_clock_snapshot.value

        # 필드 추출 — ROS2 tracetools 스키마에 따라
        row = {
            "ts_ns": ts_ns,
            "event": ev.name,
            "topic": None,
            "msg_addr": None,
        }
        # ros2:rmw_publish payload: rmw_publisher_handle, message
        # ros2:rmw_take    payload: rmw_subscription_handle, message, source_timestamp
        try:
            payload = ev.payload_field
            for fname in payload:
                val = payload[fname]
                if "message" in fname:
                    row["msg_addr"] = int(val) if val is not None else None
                elif "topic" in fname:
                    row["topic"] = str(val)
        except Exception:
            pass

        rows.append(row)

    if not rows:
        raise RuntimeError(
            f"No rmw_publish/rmw_take events in {ust_dir}. "
            "Tracing이 제대로 활성화되었는지 확인하세요 (ros2 run tracetools status)."
        )

    return pd.DataFrame(rows)


def join_frame_id(
    csv_df: pd.DataFrame,
    trace_df: pd.DataFrame,
    max_skew_ns: int = 5_000_000,  # 5 ms — application 로그와 trace event 간 최대 허용 차
) -> pd.DataFrame:
    """각 CSV row(frame_id+topic+steady_clock)에 가장 가까운 trace event ts를 매칭."""

    # event_type → 매칭할 trace event
    type_to_event = {"in": "ros2:rmw_take", "out": "ros2:rmw_publish"}

    csv_df = csv_df.sort_values("steady_clock_ns").reset_index(drop=True)
    trace_df = trace_df.sort_values("ts_ns").reset_index(drop=True)

    out_rows = []
    for _, r in csv_df.iterrows():
        target_event = type_to_event.get(r["event_type"])
        if target_event is None:
            continue
        sub = trace_df[trace_df["event"] == target_event]
        if sub.empty:
            out_rows.append({**r.to_dict(), "trace_ts_ns": None, "skew_ns": None, "matched": False})
            continue

        # binary search 최근접
        idx = sub["ts_ns"].searchsorted(r["steady_clock_ns"])
        candidates = []
        if idx > 0:
            candidates.append(sub.iloc[idx - 1])
        if idx < len(sub):
            candidates.append(sub.iloc[idx])
        best = min(candidates, key=lambda x: abs(x["ts_ns"] - r["steady_clock_ns"]))
        skew = abs(best["ts_ns"] - r["steady_clock_ns"])

        out_rows.append({
            **r.to_dict(),
            "trace_ts_ns": int(best["ts_ns"]),
            "skew_ns": int(skew),
            "matched": skew <= max_skew_ns,
        })

    return pd.DataFrame(out_rows)


def compute_t_hpc(joined: pd.DataFrame) -> pd.DataFrame:
    """frame_id 단위로 in/out 매칭해서 T_HPC 산출."""
    entries = joined[(joined["event_type"] == "in") & joined["matched"]].copy()
    exits = joined[(joined["event_type"] == "out") & joined["matched"]].copy()

    # frame_id가 들어온 image_NN 토픽 5개를 image_publisher가 forward한 뒤
    # motion_planner까지 동일 frame_id가 살아 흐른다고 가정.
    # 같은 frame_id에 대해 (가장 빠른 in, 가장 늦은 out)을 매칭.
    in_per_fid = entries.groupby("frame_id", as_index=False).agg(
        in_ts_ns=("trace_ts_ns", "min"),
        in_topic=("topic", "first"),
    )
    out_per_fid = exits.groupby("frame_id", as_index=False).agg(
        out_ts_ns=("trace_ts_ns", "max"),
        out_topic=("topic", "first"),
    )
    merged = in_per_fid.merge(out_per_fid, on="frame_id", how="inner")
    merged["t_hpc_ns"] = merged["out_ts_ns"] - merged["in_ts_ns"]
    merged["t_hpc_ms"] = merged["t_hpc_ns"] / 1_000_000.0
    return merged.sort_values("in_ts_ns").reset_index(drop=True)


def print_report(csv_df: pd.DataFrame, joined: pd.DataFrame, t_hpc: pd.DataFrame) -> None:
    n_csv = len(csv_df)
    n_matched = int(joined["matched"].sum())
    miss_rate = (n_csv - n_matched) / n_csv if n_csv else 0.0

    print("─" * 60)
    print(f" CSV rows:              {n_csv}")
    print(f" Trace-matched rows:    {n_matched}")
    print(f" Match miss rate:       {miss_rate:.4%}  (목표 < 0.1%)")
    print(f" Unique frame_ids:      {csv_df['frame_id'].nunique()}")
    print(f" T_HPC computed pairs:  {len(t_hpc)}")
    if len(t_hpc):
        print(f" T_HPC mean / median / p99 (ms):  "
              f"{t_hpc['t_hpc_ms'].mean():.2f} / "
              f"{t_hpc['t_hpc_ms'].median():.2f} / "
              f"{t_hpc['t_hpc_ms'].quantile(0.99):.2f}")
    print("─" * 60)


def main() -> int:
    parser = argparse.ArgumentParser(description="Join application CSV ↔ LTTng trace events")
    parser.add_argument("session_dir", type=Path,
                        help="세션 디렉토리 (e.g. /tmp/autosdv_traces/autosdv_hpc_S1_run01)")
    parser.add_argument("--out", type=Path, default=None,
                        help="T_HPC DataFrame을 parquet/csv로 저장 (확장자로 형식 판별)")
    parser.add_argument("--max-skew-ms", type=float, default=5.0,
                        help="application↔trace 매칭 최대 허용 차 (ms, default 5)")
    args = parser.parse_args()

    if not args.session_dir.exists():
        print(f"[ERROR] Session dir not found: {args.session_dir}", file=sys.stderr)
        return 2

    print(f"[1/4] CSV 읽기:  {args.session_dir / 'csv'}")
    csv_df = load_application_csv(args.session_dir)

    print(f"[2/4] Trace 읽기: {args.session_dir / 'ust'}")
    trace_df = load_trace_events(args.session_dir)
    print(f"      rmw_take/publish events: {len(trace_df)}")

    print("[3/4] Join 수행")
    joined = join_frame_id(csv_df, trace_df, max_skew_ns=int(args.max_skew_ms * 1_000_000))

    print("[4/4] T_HPC 산출")
    t_hpc = compute_t_hpc(joined)

    print_report(csv_df, joined, t_hpc)

    if args.out is not None:
        ext = args.out.suffix.lower()
        if ext == ".parquet":
            t_hpc.to_parquet(args.out, index=False)
        else:
            t_hpc.to_csv(args.out, index=False)
        print(f" 저장: {args.out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
