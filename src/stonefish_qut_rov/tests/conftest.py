"""Make the source node folders importable without an installed workspace."""
from pathlib import Path
import sys

NODE_ROOT = Path(__file__).resolve().parents[1] / "ros_nodes"
sys.path[:0] = [str(NODE_ROOT / group) for group in ("common", "real", "sim")]
