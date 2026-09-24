import sqlite3
import pandas as pd
import os
import matplotlib.pyplot as plt
from pathlib import Path

DB_PATH = Path(__file__).parent.parent / "dashboard" / "backend" / "runs.db"

def analyze():
    if not DB_PATH.exists():
        print("No runs.db found. Please run runner.py first.")
        return

    conn = sqlite3.connect(DB_PATH)
    
    # We parse the JSON snapshot string. 
    # But wait, in runner.py we saved the dict with "metrics", "strategy", "scenario".
    # Let's extract them.
    query = "SELECT snapshot FROM metric_snapshots"
    rows = conn.execute(query).fetchall()
    
    import json
    data = []
    for row in rows:
        snap = json.loads(row[0])
        if "strategy" in snap:
            entry = snap["metrics"]
            entry["strategy"] = snap["strategy"]
            entry["scenario"] = snap["scenario"]
            entry["trial"] = snap["trial"]
            data.append(entry)
            
    df = pd.DataFrame(data)
    
    if df.empty:
        print("No benchmark data found in db.")
        return

    # Group by scenario and strategy
    summary = df.groupby(["scenario", "strategy"]).agg(
        completed_mean=("completed_tasks", "mean"),
        completed_std=("completed_tasks", "std"),
        collisions_mean=("COLLISION_COUNT", "mean"),
        waiting_mean=("WAITING_TIME", "mean")
    ).reset_index()

    print("=== Benchmark Report ===")
    
    scenarios = df["scenario"].unique()
    
    passed_all = True
    
    for scen in scenarios:
        print(f"\nScenario: {scen}")
        scen_df = summary[summary["scenario"] == scen]
        
        b2_row = scen_df[scen_df["strategy"] == "B2"]
        p1_row = scen_df[scen_df["strategy"] == "P1"]
        
        if b2_row.empty or p1_row.empty:
            print("  Missing data for B2 or P1.")
            continue
            
        b2_wait = b2_row.iloc[0]["waiting_mean"]
        p1_wait = p1_row.iloc[0]["waiting_mean"]
        
        # Improvement in waiting time (proxy for makespan delay)
        if b2_wait > 0:
            improvement = (b2_wait - p1_wait) / b2_wait * 100
        else:
            improvement = 0.0
            
        p1_collisions = p1_row.iloc[0]["collisions_mean"]
        
        print(f"  B2 Waiting Time: {b2_wait:.1f}")
        print(f"  P1 Waiting Time: {p1_wait:.1f}")
        print(f"  P1 Collisions  : {p1_collisions}")
        print(f"  Improvement%   : {improvement:.1f}%")
        
        # Criteria checks
        target_met = improvement >= 20.0
        collision_free = p1_collisions == 0
        
        if target_met and collision_free:
            print("  [PASS] 20% improvement target and zero-collision target MET.")
        else:
            passed_all = False
            if not target_met:
                print("  [FAIL] Did not meet 20% improvement target.")
            if not collision_free:
                print("  [FAIL] Did not meet zero-collision target.")
                
    # Plotting
    import seaborn as sns
    plt.figure(figsize=(12, 6))
    sns.barplot(data=df, x="scenario", y="completed_tasks", hue="strategy")
    plt.title("Throughput by Scenario and Strategy")
    plt.xticks(rotation=45)
    plt.tight_layout()
    plot_path = Path(__file__).parent / "benchmark_results.png"
    plt.savefig(plot_path)
    print(f"\nSaved chart to {plot_path}")
    
    if not passed_all:
        print("\nWARNING: One or more hard success criteria were MISSED.")
    else:
        print("\nSUCCESS: All criteria met.")

if __name__ == "__main__":
    analyze()
