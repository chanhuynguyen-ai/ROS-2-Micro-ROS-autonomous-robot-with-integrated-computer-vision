"""Plan D (D2): technical-debt guards.

Locks the label-cleanup done in Plan D so the historical turn-lane regression
(`turn-lane == 10` in ipm_transform_node.cpp, `label != 17` in control_node.cpp;
turn-lane is 20 in the current 22-class model)
cannot silently come back. Labels in decision/IPM source must always go through
the generated `LABEL_*` constants, never a bare integer literal.

The `LABEL_TURN_LANE` constant itself is defined in the *generated*
`label_mapping.hpp` (CMake build dir, not the source tree scanned here), so a
digit-comparison scan of the source tree never trips on the definition.
"""

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "ros2_ws/src/avs_perception/src"
INCLUDE_DIR = REPO_ROOT / "ros2_ws/src/avs_perception/include/avs_perception"

# Files that carry label-based decision/IPM logic. ipm_transform_node.cpp is
# included because the documented `label == 10` regression (CLAUDE.md) lived
# there; yolo26_seg.hpp is excluded — it defines class_names, not comparisons.
SCANNED_FILES = [
    SRC_DIR / "control_node.cpp",
    SRC_DIR / "ipm_transform_node.cpp",
]


def _decision_headers() -> list[Path]:
    return [h for h in sorted(INCLUDE_DIR.glob("*.hpp")) if h.name != "yolo26_seg.hpp"]


def _strip_line_comment(line: str) -> str:
    idx = line.find("//")
    return line if idx == -1 else line[:idx]


# A label must be compared against a named LABEL_* constant, never a raw int.
# Matches e.g. `label == 20`, `l.label != 10`, `obj.label==6` (any digits).
LABEL_VS_LITERAL = re.compile(r"\blabel\s*(?:==|!=)\s*\d+")
# The magic literals the historical regressions used (10, 17), plus the
# current turn-lane id (20, after the 22-class model remap), in any comparison
# form, as a belt-and-braces guard.
MAGIC_TURN_LANE = re.compile(r"(?:==|!=)\s*(?:10|17|20)\b")


def _iter_scanned_lines():
    for path in [*SCANNED_FILES, *_decision_headers()]:
        for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            yield path, lineno, _strip_line_comment(raw)


def test_no_magic_turn_lane_literal():
    """No label is compared against a raw integer literal; all go through
    LABEL_* constants. Also asserts the specific 10/17 magic values the old
    turn-lane bugs used never reappear in a comparison."""
    label_offenders = []
    magic_offenders = []
    for path, lineno, code in _iter_scanned_lines():
        if LABEL_VS_LITERAL.search(code):
            label_offenders.append(f"{path.name}:{lineno}: {code.strip()}")
        if MAGIC_TURN_LANE.search(code):
            magic_offenders.append(f"{path.name}:{lineno}: {code.strip()}")

    assert not label_offenders, (
        "label compared against a raw integer literal — use a LABEL_* constant:\n"
        + "\n".join(label_offenders)
    )
    assert not magic_offenders, (
        "magic turn-lane literal (10/17/20) in a comparison — use LABEL_TURN_LANE:\n"
        + "\n".join(magic_offenders)
    )


def test_turn_lane_uses_named_constant():
    """Sanity check the guard is scanning live code: LABEL_TURN_LANE must
    actually be present, otherwise the anti-magic-literal test above could pass
    vacuously against an empty/renamed source set."""
    joined = "".join(p.read_text(encoding="utf-8") for p in SCANNED_FILES)
    assert "LABEL_TURN_LANE" in joined
