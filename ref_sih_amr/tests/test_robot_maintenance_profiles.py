from data.industrial_robot_profiles import PROFILES, maintenance_profile


def test_dataset_profiles_cover_50_items():
    assert len(PROFILES) == 50
    assert set(PROFILES) == {f"item_{i}" for i in range(50)}


def test_maintenance_profile_is_dataset_backed():
    p = maintenance_profile("item_44")
    assert p["dataset_item_id"] == "item_44"
    assert p["max_crack"] == PROFILES["item_44"]["max_crack"]
    assert p["min_rul_months"] == 0
    assert p["maintenance_status"] == "HIGH"


def test_small_crack_profiles_are_not_marked_high():
    assert maintenance_profile("item_2")["maintenance_status"] == "LOW"
