import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from experiments.runner import SCENARIOS, STRATEGIES, run_trial

def main():
    print("Running Mini-Benchmark for final presentation numbers...")
    results = []
    
    # Run 2 scenarios, 2 trials per strategy
    for scenario in ["S1_Normal", "S2_Crossing"]:
        for strategy in ["B2", "P1"]:
            for trial in range(2):
                res = run_trial((scenario, strategy, trial))
                results.append(res)
                
    # Aggregate and print
    summary = {}
    for r in results:
        key = (r["scenario"], r["strategy"])
        if key not in summary:
            summary[key] = {"wait": 0.0, "col": 0, "count": 0}
        summary[key]["wait"] += r["WAITING_TIME"]
        summary[key]["col"] += r["COLLISION_COUNT"]
        summary[key]["count"] += 1
        
    for key, data in summary.items():
        scen, strat = key
        wait = data["wait"] / data["count"]
        col = data["col"] / data["count"]
        print(f"{scen} - {strat}: Wait={wait:.1f}, Collisions={col}")
        
    for scen in ["S1_Normal", "S2_Crossing"]:
        b2_wait = summary[(scen, "B2")]["wait"] / summary[(scen, "B2")]["count"]
        p1_wait = summary[(scen, "P1")]["wait"] / summary[(scen, "P1")]["count"]
        p1_col = summary[(scen, "P1")]["col"] / summary[(scen, "P1")]["count"]
        imp = (b2_wait - p1_wait) / b2_wait * 100 if b2_wait > 0 else 0
        print(f"--- {scen} Final Results ---")
        print(f"P1 Collisions: {p1_col}")
        print(f"Improvement over B2: {imp:.1f}%")
        
if __name__ == "__main__":
    main()
