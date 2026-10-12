#!/usr/bin/env python3
"""Summarise a batch CSV per variant: catch rate, hiders caught per round (mean, 95 % CI),
rounds with at least one catch, all-or-nothing rounds, catch times, seeker distance.
Prints a per-seed paired table (hiders caught per variant). Writes <input>_summary.csv,
<input>_paired.csv and <input>_catch_rate.png next to the input."""
import csv
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

MAX_HIDERS = 3  # hider columns per row; matches ResultsCsv.MAX_HIDERS


def main(path: str) -> None:
    src = Path(path)
    groups: dict[str, list[dict]] = defaultdict(list)
    with src.open(newline="") as f:
        for row in csv.DictReader(f):
            groups[row["variant"]].append(row)
    if not groups:
        sys.exit(f"{src}: no data rows (header only or empty); nothing to summarise")

    rows = []
    for variant, rs in groups.items():
        total = sum(int(r["hiders_total"]) for r in rs)
        caught = sum(int(r["hiders_caught"]) for r in rs)
        per_round = [int(r["hiders_caught"]) for r in rs]
        mean_pr = statistics.mean(per_round)
        # 95 % CI of the mean over rounds, normal approximation (needs >= 2 rounds).
        half = 1.96 * statistics.stdev(per_round) / math.sqrt(len(rs)) if len(rs) > 1 else float("nan")
        catch_times = []
        for r in rs:
            for i in range(MAX_HIDERS):
                v = r.get(f"hider{i}_catch_time", "")
                if v not in ("", "-1.000"):
                    catch_times.append(float(v))
        rows.append({
            "variant": variant,
            "rounds": len(rs),
            "catch_rate": caught / total if total else 0.0,
            "mean_caught_per_round": mean_pr,
            "ci95_low": mean_pr - half,
            "ci95_high": mean_pr + half,
            "rounds_with_catch": sum(1 for c in per_round if c > 0) / len(rs),
            "rounds_all_or_nothing": sum(
                1 for r in rs if int(r["hiders_caught"]) in (0, int(r["hiders_total"]))) / len(rs),
            "catches_timed": len(catch_times),
            "mean_catch_time": statistics.mean(catch_times) if catch_times else float("nan"),
            "median_catch_time": statistics.median(catch_times) if catch_times else float("nan"),
            "mean_seeker_distance": statistics.mean(float(r["seeker_distance"]) for r in rs),
            "mean_seeker_collisions": statistics.mean(float(r["seeker_collisions"]) for r in rs),
            "mean_seeker_stuck": statistics.mean(float(r["seeker_stuck"]) for r in rs),
        })

    out = src.with_name(src.stem + "_summary.csv")
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(f"{r['variant']:<24} n={r['rounds']:<4} catch_rate={r['catch_rate']:.3f} "
              f"caught/round={r['mean_caught_per_round']:.2f} "
              f"[{r['ci95_low']:.2f}, {r['ci95_high']:.2f}] "
              f">=1 catch={r['rounds_with_catch']:.2f} 0-or-all={r['rounds_all_or_nothing']:.2f} "
              f"catch_time mean={r['mean_catch_time']:.1f} median={r['median_catch_time']:.1f} "
              f"(n={r['catches_timed']}) dist={r['mean_seeker_distance']:.1f}")

    # Paired by seed: every variant ran the same seeds, so compare them seed by seed.
    variants = list(groups.keys())
    by_seed: dict[int, dict[str, str]] = defaultdict(dict)
    for variant, rs in groups.items():
        for r in rs:
            by_seed[int(r["seed"])][variant] = r["hiders_caught"]
    paired = src.with_name(src.stem + "_paired.csv")
    with paired.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seed"] + variants)
        for seed in sorted(by_seed):
            w.writerow([seed] + [by_seed[seed].get(v, "") for v in variants])
    print("\nhiders caught per seed (paired):")
    print(f"{'seed':>6} " + " ".join(f"{v[:20]:>20}" for v in variants))
    for seed in sorted(by_seed):
        print(f"{seed:>6} " + " ".join(f"{by_seed[seed].get(v, '-'):>20}" for v in variants))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 3.5))
        ax.bar([r["variant"] for r in rows], [r["catch_rate"] for r in rows])
        ax.set_ylabel("catch rate")
        ax.set_ylim(0, 1)
        plt.xticks(rotation=20, ha="right")
        fig.tight_layout()
        fig.savefig(src.with_name(src.stem + "_catch_rate.png"), dpi=150)
    except ImportError:
        print("matplotlib not installed; skipped plot")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: analyze.py <results.csv>")
    main(sys.argv[1])
