"""YOLO class-name and debug-color helpers (no ROS, no Ultralytics).

TensorRT engines often have no name metadata. A custom 2-class engine
(person, shell) must not be silently remapped onto COCO-80, or class 1
becomes 'bicycle' and an allowed_classes=person filter drops shells.
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple, Union

NameMap = Dict[int, str]
Bgr = Tuple[int, int, int]

# OpenCV BGR. person is blue-ish so it matches the old Ultralytics look;
# shell must not share that color or the same label.
BOX_BGR_PERSON: Bgr = (255, 160, 40)
BOX_BGR_SHELL: Bgr = (0, 215, 255)
BOX_BGR_OTHER: Bgr = (0, 165, 255)

_BOX_BGR_BY_NAME = {
    'person': BOX_BGR_PERSON,
    'shell': BOX_BGR_SHELL,
}


def parse_class_csv(raw: str) -> Set[str]:
    """Lowercased unique names from a comma-separated parameter."""
    if not raw:
        return set()
    return {part.strip().lower() for part in raw.split(',') if part.strip()}


def parse_ordered_class_csv(raw: str) -> List[str]:
    """Preserve order and original spelling; skip empty tokens."""
    if not raw:
        return []
    return [part.strip() for part in raw.split(',') if part.strip()]


def names_from_override(override: Sequence[str]) -> Optional[NameMap]:
    cleaned = [str(name).strip() for name in override if str(name).strip()]
    if not cleaned:
        return None
    return {index: name for index, name in enumerate(cleaned)}


def iter_named_classes(
    names: Union[NameMap, Sequence[str]],
) -> List[Tuple[int, str]]:
    if isinstance(names, dict):
        return [(int(key), str(value)) for key, value in names.items()]
    return [(index, str(name)) for index, name in enumerate(names)]


def resolve_predict_class_ids(
    names: Union[NameMap, Sequence[str]],
    allowed: Iterable[str],
    denied: Iterable[str],
) -> Optional[List[int]]:
    """Map allow/deny name filters to Ultralytics class indices.

    None means 'do not pass classes= to predict' (no filter).
    """
    allowed_set = {
        str(name).strip().lower() for name in allowed if str(name).strip()
    }
    denied_set = {
        str(name).strip().lower() for name in denied if str(name).strip()
    }
    if not allowed_set and not denied_set:
        return None

    ids: List[int] = []
    for index, name in iter_named_classes(names):
        key = name.strip().lower()
        if denied_set and key in denied_set:
            continue
        if allowed_set and key not in allowed_set:
            continue
        ids.append(index)
    return ids


def unmatched_allowed_names(
    names: Union[NameMap, Sequence[str]],
    allowed: Iterable[str],
) -> List[str]:
    """allowed names that do not exist in the resolved name map."""
    present = {name.strip().lower() for _, name in iter_named_classes(names)}
    unmatched: List[str] = []
    seen: Set[str] = set()
    for name in allowed:
        key = str(name).strip().lower()
        if not key or key in seen:
            continue
        seen.add(key)
        if key not in present:
            unmatched.append(key)
    return unmatched


def class_name_allowed(
    name: str,
    allowed: Iterable[str],
    denied: Iterable[str],
) -> bool:
    key = (name or '').strip().lower()
    denied_set = {
        str(item).strip().lower() for item in denied if str(item).strip()
    }
    allowed_set = {
        str(item).strip().lower() for item in allowed if str(item).strip()
    }
    if denied_set and key in denied_set:
        return False
    if allowed_set and key not in allowed_set:
        return False
    return True


def parse_class_thresholds(raw: str) -> Dict[str, float]:
    """Parse 'person:0.65,shell:0.3' into lowercased name → cut in [0, 1]."""
    cuts: Dict[str, float] = {}
    if not raw:
        return cuts
    for part in raw.split(','):
        item = part.strip()
        if not item or ':' not in item:
            continue
        name, value = item.split(':', 1)
        key = name.strip().lower()
        try:
            cut = float(value.strip())
        except ValueError:
            continue
        if not key or cut < 0.0 or cut > 1.0:
            continue
        cuts[key] = cut
    return cuts


def predict_conf_floor(
    global_threshold: float,
    class_thresholds: Dict[str, float],
) -> float:
    """Lowest cut used as Ultralytics conf so a high person cut cannot hide shells."""
    cuts = [float(global_threshold)]
    cuts.extend(float(value) for value in class_thresholds.values())
    return max(0.0, min(cuts))


def score_meets_threshold(
    class_name: str,
    score: float,
    global_threshold: float,
    class_thresholds: Dict[str, float],
) -> bool:
    key = (class_name or '').strip().lower()
    cut = class_thresholds.get(key, float(global_threshold))
    return float(score) >= float(cut)


def debug_box_bgr(class_name: str) -> Bgr:
    key = (class_name or '').strip().lower()
    return _BOX_BGR_BY_NAME.get(key, BOX_BGR_OTHER)


def debug_box_label(class_name: str, score: float) -> str:
    return f'{class_name} {score:.2f}'
