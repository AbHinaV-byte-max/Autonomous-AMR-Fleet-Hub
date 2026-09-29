"""Dataset-backed predictive-maintenance profiles for the AMR dashboard.

Source: industrial_robot_isolation_forest_prepared.zip supplied for the project.
The source dataset contains industrial-robot degradation/crack/RUL records, not
mobile-AMR navigation telemetry. These profiles are therefore exposed as
maintenance/degradation context only; simulator position, battery, tasks and
motion remain authoritative.
"""

PROFILES = {
    "item_0": {"samples": 4, "max_time_months": 3, "max_crack": 0.0812808336, "min_rul": 0, "failure_samples": 2},
    "item_1": {"samples": 3, "max_time_months": 2, "max_crack": 0.0378368259, "min_rul": 0, "failure_samples": 2},
    "item_2": {"samples": 3, "max_time_months": 2, "max_crack": 0.0105280729, "min_rul": 0, "failure_samples": 2},
    "item_3": {"samples": 4, "max_time_months": 3, "max_crack": 0.10286045, "min_rul": 0, "failure_samples": 2},
    "item_4": {"samples": 4, "max_time_months": 3, "max_crack": 0.0341796742, "min_rul": 0, "failure_samples": 2},
    "item_5": {"samples": 39, "max_time_months": 38, "max_crack": 0.9178309046, "min_rul": 0, "failure_samples": 1},
    "item_6": {"samples": 37, "max_time_months": 36, "max_crack": 0.9342331451, "min_rul": 0, "failure_samples": 1},
    "item_7": {"samples": 46, "max_time_months": 45, "max_crack": 0.8767052387, "min_rul": 0, "failure_samples": 1},
    "item_8": {"samples": 31, "max_time_months": 30, "max_crack": 0.8589055443, "min_rul": 0, "failure_samples": 1},
    "item_9": {"samples": 43, "max_time_months": 42, "max_crack": 0.8496564785, "min_rul": 0, "failure_samples": 1},
    "item_10": {"samples": 40, "max_time_months": 39, "max_crack": 0.9278444627, "min_rul": 0, "failure_samples": 1},
    "item_11": {"samples": 47, "max_time_months": 46, "max_crack": 0.9575242019, "min_rul": 0, "failure_samples": 1},
    "item_12": {"samples": 29, "max_time_months": 28, "max_crack": 0.4525145103, "min_rul": 0, "failure_samples": 2},
    "item_13": {"samples": 38, "max_time_months": 37, "max_crack": 0.9014665088, "min_rul": 0, "failure_samples": 1},
    "item_14": {"samples": 37, "max_time_months": 36, "max_crack": 0.8530348962, "min_rul": 0, "failure_samples": 1},
    "item_15": {"samples": 37, "max_time_months": 36, "max_crack": 0.8484634026, "min_rul": 0, "failure_samples": 1},
    "item_16": {"samples": 40, "max_time_months": 39, "max_crack": 0.868095351, "min_rul": 0, "failure_samples": 1},
    "item_17": {"samples": 49, "max_time_months": 48, "max_crack": 0.9548011928, "min_rul": 0, "failure_samples": 1},
    "item_18": {"samples": 36, "max_time_months": 35, "max_crack": 0.8783700564, "min_rul": 0, "failure_samples": 1},
    "item_19": {"samples": 61, "max_time_months": 60, "max_crack": 0.9226550378, "min_rul": 0, "failure_samples": 1},
    "item_20": {"samples": 33, "max_time_months": 32, "max_crack": 0.9154041143, "min_rul": 0, "failure_samples": 1},
    "item_21": {"samples": 51, "max_time_months": 50, "max_crack": 0.8815841804, "min_rul": 0, "failure_samples": 1},
    "item_22": {"samples": 40, "max_time_months": 39, "max_crack": 0.8628947718, "min_rul": 0, "failure_samples": 1},
    "item_23": {"samples": 36, "max_time_months": 35, "max_crack": 0.9383059092, "min_rul": 0, "failure_samples": 1},
    "item_24": {"samples": 30, "max_time_months": 29, "max_crack": 0.8588465288, "min_rul": 0, "failure_samples": 1},
    "item_25": {"samples": 14, "max_time_months": 13, "max_crack": 0.1643783959, "min_rul": 0, "failure_samples": 2},
    "item_26": {"samples": 41, "max_time_months": 40, "max_crack": 0.8766168317, "min_rul": 0, "failure_samples": 1},
    "item_27": {"samples": 14, "max_time_months": 13, "max_crack": 0.1301447954, "min_rul": 0, "failure_samples": 2},
    "item_28": {"samples": 35, "max_time_months": 34, "max_crack": 0.8853952095, "min_rul": 0, "failure_samples": 2},
    "item_29": {"samples": 36, "max_time_months": 35, "max_crack": 0.8337734285, "min_rul": 0, "failure_samples": 1},
    "item_30": {"samples": 40, "max_time_months": 39, "max_crack": 0.8850316319, "min_rul": 0, "failure_samples": 1},
    "item_31": {"samples": 43, "max_time_months": 42, "max_crack": 0.8643117928, "min_rul": 0, "failure_samples": 1},
    "item_32": {"samples": 44, "max_time_months": 43, "max_crack": 0.9433352597, "min_rul": 0, "failure_samples": 1},
    "item_33": {"samples": 43, "max_time_months": 42, "max_crack": 0.8969770776, "min_rul": 0, "failure_samples": 1},
    "item_34": {"samples": 40, "max_time_months": 39, "max_crack": 0.8998639106, "min_rul": 0, "failure_samples": 1},
    "item_35": {"samples": 44, "max_time_months": 43, "max_crack": 0.8797985295, "min_rul": 0, "failure_samples": 1},
    "item_36": {"samples": 39, "max_time_months": 38, "max_crack": 0.9067723982, "min_rul": 0, "failure_samples": 1},
    "item_37": {"samples": 37, "max_time_months": 36, "max_crack": 0.8497539788, "min_rul": 0, "failure_samples": 1},
    "item_38": {"samples": 40, "max_time_months": 39, "max_crack": 0.891135924, "min_rul": 0, "failure_samples": 1},
    "item_39": {"samples": 41, "max_time_months": 40, "max_crack": 0.968199742, "min_rul": 0, "failure_samples": 1},
    "item_40": {"samples": 45, "max_time_months": 44, "max_crack": 0.818432892, "min_rul": 0, "failure_samples": 1},
    "item_41": {"samples": 52, "max_time_months": 51, "max_crack": 0.9186999781, "min_rul": 0, "failure_samples": 1},
    "item_42": {"samples": 35, "max_time_months": 34, "max_crack": 0.8084186257, "min_rul": 0, "failure_samples": 1},
    "item_43": {"samples": 52, "max_time_months": 51, "max_crack": 0.9721885738, "min_rul": 0, "failure_samples": 1},
    "item_44": {"samples": 51, "max_time_months": 50, "max_crack": 1.000796564, "min_rul": 0, "failure_samples": 1},
    "item_45": {"samples": 40, "max_time_months": 39, "max_crack": 0.8889336371, "min_rul": 0, "failure_samples": 1},
    "item_46": {"samples": 43, "max_time_months": 42, "max_crack": 0.8984310122, "min_rul": 0, "failure_samples": 1},
    "item_47": {"samples": 26, "max_time_months": 25, "max_crack": 0.3740060946, "min_rul": 0, "failure_samples": 2},
    "item_48": {"samples": 49, "max_time_months": 48, "max_crack": 0.9543252008, "min_rul": 0, "failure_samples": 1},
    "item_49": {"samples": 43, "max_time_months": 42, "max_crack": 0.8421941803, "min_rul": 0, "failure_samples": 1},
}

def maintenance_profile(item_id: str) -> dict:
    p = dict(PROFILES.get(item_id, {}))
    if not p:
        return {"dataset_item_id": item_id, "maintenance_status": "UNMAPPED"}

    crack = float(p["max_crack"])
    if crack >= 0.75:
        status = "HIGH"
    elif crack >= 0.20:
        status = "WATCH"
    else:
        status = "LOW"

    return {
        "dataset_item_id": item_id,
        "maintenance_status": status,
        "degradation_index": round(min(1.0, max(0.0, crack)), 3),
        "max_crack": crack,
        "min_rul_months": int(p["min_rul"]),
        "failure_samples": int(p["failure_samples"]),
        "dataset_samples": int(p["samples"]),
        "dataset_horizon_months": int(p["max_time_months"]),
    }
