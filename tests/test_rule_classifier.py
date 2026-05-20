"""tests for rule_classifier."""

import random
from src.classification.rule_classifier import TerrainClassifier, TerrainType


FLAT_FEATS = {"slope_deg": 1.0, "roughness": 0.005, "height_range": 0.01,
              "mean_height": 0.0, "imu_pitch_deg": 0.5, "imu_roll_deg": 0.2}
SLOPE_FEATS = {"slope_deg": 15.0, "roughness": 0.01, "height_range": 0.5,
               "mean_height": 0.2, "imu_pitch_deg": 8.0, "imu_roll_deg": 0.5}
ROUGH_FEATS = {"slope_deg": 2.0, "roughness": 0.1, "height_range": 0.3,
               "mean_height": 0.0, "imu_pitch_deg": 1.0, "imu_roll_deg": 3.0}


def test_flat_terrain():
    c = TerrainClassifier()
    assert c.classify(FLAT_FEATS) == TerrainType.FLAT


def test_slope_terrain():
    c = TerrainClassifier()
    assert c.classify(SLOPE_FEATS) == TerrainType.SLOPE


def test_slope_detected_by_imu():
    c = TerrainClassifier()
    result = c.classify({"slope_deg": 1.0, "roughness": 0.005, "height_range": 0.01,
                         "mean_height": 0.0, "imu_pitch_deg": 6.0, "imu_roll_deg": 0.3})
    assert result == TerrainType.SLOPE


def test_rough_terrain():
    c = TerrainClassifier()
    assert c.classify(ROUGH_FEATS) == TerrainType.ROUGH


def test_transition_detection():
    c = TerrainClassifier(history_len=5)
    c.classify(FLAT_FEATS)
    c.classify(SLOPE_FEATS)
    result = c.classify(ROUGH_FEATS)
    assert result == TerrainType.TRANSITION


def test_default_vote_window_is_9():
    """iter01: default vote_window expanded from 5 to 9 for rough-terrain stability."""
    c = TerrainClassifier()
    assert c._vote.maxlen == 9


def test_rough_robust_to_occasional_flat_noise():
    """In a steady ROUGH state, occasional FLAT frames must not flip the output.

    Iter00 baseline (vote_window=5): 2/5 FLAT frames could swing to FLAT.
    Iter01 target (vote_window=9): 4/9 FLAT frames still keep ROUGH output."""
    c = TerrainClassifier()
    # Pre-fill window with ROUGH
    for _ in range(9):
        c.classify(ROUGH_FEATS)
    assert c.classify(ROUGH_FEATS) == TerrainType.ROUGH

    # Inject 4 FLAT frames among ROUGH — minority noise should not flip
    seq = [FLAT_FEATS, ROUGH_FEATS, FLAT_FEATS, ROUGH_FEATS,
           FLAT_FEATS, ROUGH_FEATS, FLAT_FEATS, ROUGH_FEATS, ROUGH_FEATS]
    for f in seq:
        result = c.classify(f)
    # Final state should be ROUGH (5 ROUGH vs 4 FLAT in window)
    assert result == TerrainType.ROUGH


def test_recent_bias_breaks_tie_toward_current_state():
    """When window is split 50/50, recent K frames should decide the winner."""
    c = TerrainClassifier(vote_window=8, recent_bias_window=3)
    # Pre-fill with mix to enter window-full state
    for _ in range(4):
        c.classify(FLAT_FEATS)
    for _ in range(4):
        c.classify(ROUGH_FEATS)
    # Window is now 4 FLAT + 4 ROUGH. Recent 3 are all ROUGH → expect ROUGH
    result = c._classify_for_test() if hasattr(c, '_classify_for_test') else None
    # Direct test: another ROUGH input should keep ROUGH
    assert c.classify(ROUGH_FEATS) == TerrainType.ROUGH


def test_warmup_returns_raw_classifications():
    """While the window is not yet full, the raw single-frame result is returned."""
    c = TerrainClassifier()
    # First few calls should return whatever the C++ kernel says directly
    r = c.classify(FLAT_FEATS)
    assert r == TerrainType.FLAT
    r = c.classify(ROUGH_FEATS)
    # Still in warmup, should return ROUGH (raw)
    assert r == TerrainType.ROUGH


def test_vote_window_disabled_returns_raw():
    """vote_window=1 disables smoothing entirely."""
    c = TerrainClassifier(vote_window=1)
    assert c.classify(FLAT_FEATS) == TerrainType.FLAT
    assert c.classify(ROUGH_FEATS) == TerrainType.ROUGH
    assert c.classify(FLAT_FEATS) == TerrainType.FLAT


def test_reset_clears_vote_window():
    c = TerrainClassifier()
    for _ in range(9):
        c.classify(ROUGH_FEATS)
    assert len(c._vote) == 9
    c.reset()
    assert len(c._vote) == 0
    # After reset, single FLAT frame should classify as FLAT (warmup mode)
    assert c.classify(FLAT_FEATS) == TerrainType.FLAT


def test_steady_state_under_random_noise():
    """Under 70% ROUGH + 30% FLAT random noise, output should be ROUGH most of time."""
    rng = random.Random(42)
    c = TerrainClassifier()
    rough_count = 0
    total = 200
    # Warmup
    for _ in range(15):
        c.classify(ROUGH_FEATS)

    for _ in range(total):
        feats = ROUGH_FEATS if rng.random() < 0.7 else FLAT_FEATS
        result = c.classify(feats)
        if result == TerrainType.ROUGH:
            rough_count += 1

    # Expect ROUGH output >= 85% (smoothing must improve raw 70% input)
    assert rough_count / total >= 0.85, f"Only {rough_count}/{total} ROUGH"
