"""tests for rule_classifier."""

from src.classification.rule_classifier import TerrainClassifier, TerrainType


def test_flat_terrain():
    c = TerrainClassifier()
    assert c.classify({"slope_deg": 1.0, "roughness": 0.005, "height_range": 0.01, "mean_height": 0.0}) == TerrainType.FLAT


def test_slope_terrain():
    c = TerrainClassifier()
    assert c.classify({"slope_deg": 15.0, "roughness": 0.01, "height_range": 0.5, "mean_height": 0.2}) == TerrainType.SLOPE


def test_rough_terrain():
    c = TerrainClassifier()
    assert c.classify({"slope_deg": 2.0, "roughness": 0.1, "height_range": 0.3, "mean_height": 0.0}) == TerrainType.ROUGH


def test_transition_detection():
    c = TerrainClassifier(history_len=5)
    c.classify({"slope_deg": 1.0, "roughness": 0.005, "height_range": 0.01, "mean_height": 0.0})
    c.classify({"slope_deg": 15.0, "roughness": 0.01, "height_range": 0.5, "mean_height": 0.2})
    result = c.classify({"slope_deg": 1.0, "roughness": 0.005, "height_range": 0.01, "mean_height": 0.0})
    assert result == TerrainType.TRANSITION
