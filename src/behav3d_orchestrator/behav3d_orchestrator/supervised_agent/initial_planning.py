"""Review-only initial wall placement and native scalar-field first-batch planning.

Inputs are already verified Agent007 initial-scene data, not robot-frame aliases.
Only ``generate_first_batch`` writes files. Nothing here authorizes fabrication,
checks IK, or establishes substrate material/adhesion/structural capacity.
"""
from __future__ import annotations

import hashlib
import importlib
import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

TASK_FIELDS = (
    "task_id", "width_m", "height_m", "thickness_m", "bead_spacing_m",
    "bead_height_m", "bead_diameter_m", "count", "yaw_degrees", "search_step_m",
    "max_pose_evaluations", "min_confidence", "min_perception_score", "clearance_m",
    "footprint_margin_m", "max_duplicate_layer_separation_m", "max_support_triangle_height_span_m",
    "base_z_offset_m", "support_contact_tolerance_m", "max_frontier_height_span_m",
    "heat_seed_level_m", "heat_t_coef", "iso_level_m", "normal_continuity_rule",
)
ARRAY_FIELDS = (
    "points", "review_class", "perception_score", "confidence", "clearance_m",
    "suitability_region_index", "object_points", "object_owner_index", "object_ids", "surface_cells",
)
MAX_POINTS = 3_000_000
MAX_SAMPLES = 100_000
MAX_ALTERNATIVES = 8
MAX_NATIVE_INITIALIZATIONS = 32
MAX_INITIALIZATION_BYTES = 16 * 1024 * 1024
# Native YAML rounds each world-coordinate origin to 0.01 mm. The 10 um
# reserve exceeds the worst horizontal projection of half-step XY rounding
# (sqrt(2) * 5 um) and is shared by preselection and final validation.
TARGET_ROUNDING_RESERVE_M = .00001
SCHEMA = "behav3d.initial_wall_proposal.v1"


def _json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _digest(value):
    return hashlib.sha256(_json(value).encode()).hexdigest()


def _file_hash(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def validate_task(task):
    """Return a normalized explicit process/search task, rejecting hidden defaults."""
    if not isinstance(task, dict) or set(task) != set(TASK_FIELDS):
        raise ValueError("task must contain exactly TASK_FIELDS")
    task = json.loads(_json(task))
    if not isinstance(task["task_id"], str) or not 1 <= len(task["task_id"]) <= 128:
        raise ValueError("bounded task_id required")
    positive = ("width_m", "height_m", "thickness_m", "bead_spacing_m", "bead_height_m",
                "bead_diameter_m", "search_step_m", "max_duplicate_layer_separation_m",
                "max_support_triangle_height_span_m", "base_z_offset_m", "support_contact_tolerance_m",
                "max_frontier_height_span_m", "heat_seed_level_m", "heat_t_coef")
    nonnegative = ("clearance_m", "footprint_margin_m", "iso_level_m")
    for name in positive + nonnegative + ("min_confidence", "min_perception_score"):
        value = task[name]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("finite task dimension/threshold required: " + name)
        if value < 0 or (name in positive and value == 0):
            raise ValueError("invalid task dimension/threshold: " + name)
        task[name] = float(value)
    if not .01 <= task["width_m"] <= 2 or not .01 <= task["height_m"] <= 2:
        raise ValueError("wall dimensions must be between .01 and 2 metres")
    if not .001 <= task["bead_diameter_m"] <= task["thickness_m"] <= .25:
        raise ValueError("bead diameter must fit the wall thickness")
    if not .001 <= task["bead_height_m"] <= task["height_m"]:
        raise ValueError("bead height must fit the wall height")
    if task["bead_diameter_m"] >= task["width_m"]:
        raise ValueError("bead diameter must be smaller than wall width")
    if task["iso_level_m"] + task["bead_height_m"] > task["height_m"]:
        raise ValueError("initial target elevation exceeds wall height")
    if task["bead_spacing_m"] > 2 or task["search_step_m"] < .001:
        raise ValueError("unbounded spacing or excessive search resolution")
    if any(task[name] > .5 for name in nonnegative + ("max_duplicate_layer_separation_m",
            "max_support_triangle_height_span_m", "base_z_offset_m", "support_contact_tolerance_m",
            "max_frontier_height_span_m", "heat_seed_level_m")):
        raise ValueError("unbounded clearance, margin or base tolerance")
    if task["base_z_offset_m"] >= task["height_m"] or task["heat_seed_level_m"] >= task["height_m"]:
        raise ValueError("base overlap and heat seed level must be below wall height")
    if not .001 <= task["heat_t_coef"] <= 10000:
        raise ValueError("bounded positive heat solver coefficient required")
    if not all(0 <= task[name] <= 1 for name in ("min_confidence", "min_perception_score")):
        raise ValueError("confidence and score must be in [0, 1]")
    if type(task["count"]) is not int or not 1 <= task["count"] <= 1000:
        raise ValueError("count must be an integer between 1 and 1000")
    if type(task["max_pose_evaluations"]) is not int or not 1 <= task["max_pose_evaluations"] <= 25_000:
        raise ValueError("max_pose_evaluations must be between 1 and 25000")
    if type(task["normal_continuity_rule"]) is not bool:
        raise ValueError("normal_continuity_rule must be boolean")
    yaws = task["yaw_degrees"]
    if not isinstance(yaws, list) or not 1 <= len(yaws) <= 36:
        raise ValueError("between 1 and 36 explicit yaw angles required")
    if any(type(yaw) not in (int, float) or not math.isfinite(yaw) or abs(yaw) > 360 for yaw in yaws):
        raise ValueError("bounded finite yaw angles required")
    # Ordering is an explicit preference, not merely an unordered search set.
    # Normalize equivalent wall directions while preserving their first order.
    task["yaw_degrees"] = list(dict.fromkeys(float(yaw) % 180 for yaw in yaws))
    return task


def _inputs(packet, arrays, task):
    task = validate_task(task)
    if not isinstance(packet, dict) or packet.get("schema") != "agent007.initial_scene_packet.v1":
        raise ValueError("unsupported initial scene packet")
    if packet.get("coordinate_units") != "m":
        raise ValueError("scene coordinate_units must be m")
    if not all(isinstance(packet.get(key), str) and packet[key] for key in ("scene_id", "frame_id")):
        raise ValueError("explicit scene and frame identities required")
    cell = packet.get("cell_size_m")
    if type(cell) not in (int, float) or not .001 <= cell <= .1:
        raise ValueError("bounded cell_size_m required")
    indexing = packet.get("surface_cell_indexing", {})
    if (indexing.get("horizontal_axes") != [0, 1] or indexing.get("vertical_axis") != 2
            or indexing.get("origin_m") != [0, 0]):
        raise ValueError("explicit world XY lattice and +Z vertical required; no implicit axis conversion")
    if not isinstance(packet.get("regions"), list) or not isinstance(packet.get("supports"), list):
        raise ValueError("scene regions/supports must be lists")  # noqa: TRY004 -- malformed validation inputs consistently raise ValueError
    if packet.get("class_values", {}).get("CLEAR_CANDIDATE") != 1:
        raise ValueError("explicit CLEAR_CANDIDATE=1 class mapping required")
    _json(packet)
    data = {}
    for name in ARRAY_FIELDS:
        if name not in arrays:
            raise ValueError("missing geometry array: " + name)
        value = np.asarray(arrays[name])
        if value.dtype.hasobject or value.size > MAX_POINTS * 3:
            raise ValueError("unsafe or excessive geometry array: " + name)
        data[name] = value
    n = len(data["points"])
    for name in ("points", "object_points"):
        value = data[name]
        if value.ndim != 2 or value.shape[1] != 3 or len(value) > MAX_POINTS or not np.isfinite(value).all():
            raise ValueError("bounded finite Nx3 points required: " + name)
        data[name] = np.asarray(value, dtype=np.float64)
    if n < 4:
        raise ValueError("at least four surface cells required")
    if (data["surface_cells"].shape != (n, 2)
            or not np.issubdtype(data["surface_cells"].dtype, np.integer)):
        raise ValueError("one explicit integer surface cell per point required")
    expected_cells = np.floor(data["points"][:, :2] / cell + 1e-6).astype(np.int64)
    if not np.array_equal(expected_cells, data["surface_cells"]):
        raise ValueError("surface cells must use explicit world XY / cell_size lattice")
    for name in ("review_class", "suitability_region_index"):
        if data[name].shape != (n,) or not np.issubdtype(data[name].dtype, np.integer):
            raise ValueError("one integer per surface cell required: " + name)
    for name in ("perception_score", "confidence", "clearance_m"):
        value = data[name]
        if value.shape != (n,) or np.isnan(value).any() or np.isneginf(value).any() or np.any(value < 0):
            raise ValueError("one nonnegative value per cell required: " + name)
        if name != "clearance_m" and (not np.isfinite(value).all() or np.any(value > 1)):
            raise ValueError("confidence/score values must be finite in [0,1]")
        data[name] = np.asarray(value, dtype=np.float64)
    owners, ids = data["object_owner_index"], data["object_ids"]
    if (owners.shape != (len(data["object_points"]),) or not np.issubdtype(owners.dtype, np.integer)
            or ids.ndim != 1 or ids.dtype.kind not in "US" or len(set(ids.tolist())) != len(ids)
            or (len(owners) and (owners.min() < 0 or owners.max() >= len(ids)))):
        raise ValueError("invalid object-owner geometry")
    regions = {}
    for record in packet["regions"]:
        if (not isinstance(record, dict) or type(record.get("region_index")) is not int
                or not isinstance(record.get("region_id"), str) or not record["region_id"]
                or not isinstance(record.get("surface_id"), str) or not record["surface_id"]
                or type(record.get("eligible")) is not bool or record["region_index"] in regions):
            raise ValueError("explicit unique region identity/eligibility required")
        regions[record["region_index"]] = record
    # Preserve every source row in the binding. A duplicate cell is usable only
    # when all rows agree on class/region and the full vertical range is bounded.
    # No nearest-row choice can hide an unknown or a second height layer.
    grouped = {}
    for index, key in enumerate(map(tuple, data["surface_cells"])):
        grouped.setdefault(key, []).append(index)
    records = []
    for key, rows in sorted(grouped.items()):
        rows = np.asarray(rows, dtype=np.int64)
        classes = data["review_class"][rows]
        labels = data["suitability_region_index"][rows]
        heights = data["points"][rows, 2]
        records.append({"key": key, "point": np.median(data["points"][rows], axis=0),
                        "class": int(classes[0]) if np.all(classes == classes[0]) else -1,
                        "region": int(labels[0]) if np.all(labels == labels[0]) else -1,
                        "ambiguous": (not np.all(classes == classes[0]) or not np.all(labels == labels[0])
                                      or np.ptp(heights) > task["max_duplicate_layer_separation_m"]),
                        "low": float(np.min(heights)), "high": float(np.max(heights)),
                        "confidence": float(np.min(data["confidence"][rows])),
                        "score": float(np.min(data["perception_score"][rows])),
                        "clearance": float(np.min(data["clearance_m"][rows]))})
    data["_cells"] = records
    data["_cell_lookup"] = {record["key"]: index for index, record in enumerate(records)}
    # Object->represented-cell lookup is fixed by the bound geometry, not pose.
    # Cache it once rather than repeating Python dictionary queries for every
    # full-height volume check. Explicit integer dtype also covers no objects.
    object_keys = np.floor(data["object_points"][:, :2] / cell + 1e-6).astype(np.int64)
    data["_object_cell_index"] = np.asarray(
        [data["_cell_lookup"].get(tuple(key), -1) for key in object_keys], dtype=np.int64)
    for key in ("point", "class", "region", "ambiguous", "low", "high", "confidence", "score", "clearance"):
        data["_cell_" + key] = np.asarray([record[key] for record in records])
    data["_object_tree"] = cKDTree(data["object_points"][:, :2]) if len(owners) else None
    data["_regions"] = regions
    return data, task


def _bindings(packet, data, task):
    geometry = hashlib.sha256()
    for name in ARRAY_FIELDS:
        array = np.ascontiguousarray(data[name])
        geometry.update(name.encode())
        geometry.update(str(array.dtype).encode())
        geometry.update(str(array.shape).encode())
        geometry.update(array.tobytes())
    return {"scene_id": packet["scene_id"], "frame_id": packet["frame_id"], "coordinate_units": "m",
            "scene_sha256": _digest(packet), "geometry_sha256": geometry.hexdigest(),
            "task_sha256": _digest(task)}


def _axes(yaw):
    angle = math.radians(yaw)
    return np.array([math.cos(angle), math.sin(angle)]), np.array([-math.sin(angle), math.cos(angle)])


def _review_owner_record(packet, data, object_id):
    """Require explicit source membership and semantic eligibility, not a label."""
    records = [record for record in packet.get("objects", []) if record.get("object_id") == object_id]
    members = (packet.get("support_grid_provenance", {}).get("augmentations", {})
               .get("semantic_support_augmentation", {}).get("objects", []))
    membership = [record for record in members if record.get("object_id") == object_id]
    matches = np.flatnonzero(data["object_ids"] == object_id)
    if len(records) != 1 or len(membership) != 1 or len(matches) != 1:
        return None
    record, member = records[0], membership[0]
    if _semantic_placement_review(packet, object_id)["recommendation_class"] == "blocked_upstream":
        return None
    for support in packet["supports"]:
        source_id = support.get("source_object_id") or (support.get("owner") or {}).get("object_id")
        if source_id == object_id and support.get("print_candidate_allowed") is False:
            return None
    count = int(np.count_nonzero(data["object_owner_index"] == matches[0]))
    if (record.get("track_status") != "confirmed" or record.get("geometry_eligible") is not True
            or record.get("semantic_role") not in ("support", "printing_plane")
            or member.get("semantic_role") != record["semantic_role"]
            or member.get("input_point_count") != count or count < 4):
        return None
    return record


def _view_metadata(object_id, label, source, surface_id, source_count, cells, reason, intent=False):
    return {"substrate_object_id": object_id, "substrate_label": label,
            "substrate_geometry_source": source, "surface_id": surface_id,
            "review_required": True, "eligibility_reason": reason,
            "original_map_role": "avoidance_and_quality_only", "source_point_count": source_count,
            "represented_cell_count": cells, "task_intent_confirmed": intent,
            "material_capacity_verified": False}


def _record_arrays(data, records):
    """Create an owner-only support view without mutating the bound source."""
    view = dict(data)
    view["_cells"] = records
    view["_cell_lookup"] = {record["key"]: index for index, record in enumerate(records)}
    for key in ("point", "class", "region", "ambiguous", "low", "high", "confidence", "score", "clearance"):
        view["_cell_" + key] = np.asarray([record[key] for record in records])
    view["points"] = view["_cell_point"]
    view["surface_cells"] = np.asarray([record["key"] for record in records], dtype=np.int64)
    for array_name, record_name in (("review_class", "class"), ("suitability_region_index", "region"),
                                    ("confidence", "confidence"), ("perception_score", "score"),
                                    ("clearance_m", "clearance")):
        view[array_name] = view["_cell_" + record_name]
    object_keys = np.floor(data["object_points"][:, :2] / view["_cell_size_m"] + 1e-6).astype(np.int64)
    view["_object_cell_index"] = np.asarray(
        [view["_cell_lookup"].get(tuple(key), -1) for key in object_keys], dtype=np.int64)
    return view


def _substrate_views(packet, data, task):
    """Separate explicitly identified support owners from a mixed aggregate map.

    Each cell is the actual highest observed point of ONE owner, not a fitted
    plane, filled hole, TSDF patch or measurement independent of the scan. The
    original map supplies only unchanged avoidance/quality gates. Semantic
    membership permits a review proposal, never material/fabrication approval.
    """
    primary_owner = None
    for support in packet["supports"]:
        if support.get("surface_id") == packet.get("primary_surface_id"):
            primary_owner = support.get("source_object_id") or (support.get("owner") or {}).get("object_id")
            break
    owners = [str(value) for value in data["object_ids"] if _review_owner_record(packet, data, str(value))]
    views = []
    if primary_owner not in owners and _semantic_placement_review(packet, primary_owner)["recommendation_class"] != "blocked_upstream":
        fallback = dict(data)
        metadata = _view_metadata(primary_owner, next((record.get("label", str(primary_owner)) for record in
            packet.get("objects", []) if record.get("object_id") == primary_owner), "primary_surface"),
            "bound_primary_support_cells", packet.get("primary_surface_id", "primary_surface"),
            len(data["points"]), len(data["_cells"]), "Existing explicitly eligible primary support cells; no additional object selected.")
        fallback["_view_metadata"] = metadata
        fallback["_regions"] = {index: dict(region, _view_metadata=metadata) for index, region in data["_regions"].items()}
        views.append(fallback)
    for object_id in owners:
        owner = _review_owner_record(packet, data, object_id)
        if object_id == primary_owner:
            permitted_regions = {index: region for index, region in data["_regions"].items()
                                 if not _surface_rejection(packet, region)}
            surface_id = packet["primary_surface_id"]
        else:
            # An explicit semantic support member is a task-intent review
            # substrate, not a new primary-table owner or structural support.
            permitted_regions = data["_regions"]
            surface_id = "review_owner:" + object_id
        object_index = int(np.flatnonzero(data["object_ids"] == object_id)[0])
        points = data["object_points"][data["object_owner_index"] == object_index]
        cells = np.floor(points[:, :2] / packet["cell_size_m"] + 1e-6).astype(np.int64)
        highest = {}
        for point, key_array in zip(points, cells, strict=True):
            key = tuple(key_array)
            if key not in highest or point[2] > highest[key][2]:
                highest[key] = point
        records = []
        for key, point in sorted(highest.items()):
            source_index = data["_cell_lookup"].get(key)
            source = data["_cells"][source_index] if source_index is not None else None
            # Height-layer ambiguity may be resolved ONLY by this explicit
            # owner separation. Conflicting/non-clear classes or region labels
            # remain non-clear/unassigned and cannot become placement cells.
            records.append({"key": key, "point": point.copy(),
                "class": source["class"] if source else 6, "region": source["region"] if source else -1,
                "ambiguous": bool(source and source["class"] < 0), "low": float(point[2]), "high": float(point[2]),
                "confidence": source["confidence"] if source else 0.,
                "score": source["score"] if source else 0., "clearance": source["clearance"] if source else 0.})
        metadata = _view_metadata(object_id, owner.get("label", object_id),
            "owner_only_observed_cell_top_envelope", surface_id, len(points), len(records),
            "Confirmed geometry-eligible support explicitly included in the source semantic-support augmentation; task intent and material remain for review.",
            owner.get("intent_confirmed") is True)
        indexed = dict(data, _cell_size_m=packet["cell_size_m"])
        view = _record_arrays(indexed, records)
        view["_view_metadata"] = metadata
        view["_regions"] = {index: dict(region, region_id="review-" + object_id + "-" + region["region_id"],
            surface_id=surface_id, _owner_only_view=True, _view_metadata=metadata)
            for index, region in permitted_regions.items()}
        views.append(view)
    return views


def _placement_policy(packet, data, views):
    primary_id = next((support.get("source_object_id") or (support.get("owner") or {}).get("object_id")
                       for support in packet["supports"] if support.get("surface_id") == packet.get("primary_surface_id")), None)
    matches = np.flatnonzero(data["object_ids"] == primary_id) if primary_id else np.asarray([], dtype=int)
    points = data["object_points"][data["object_owner_index"] == matches[0]] if len(matches) else data["points"]
    center = (np.min(points[:, :2], axis=0) + np.max(points[:, :2], axis=0)) / 2
    return {"ranking": "orientation_then_semantic_review_then_centrality_then_evidence", "reference_object_id": primary_id,
            "reference_center_m": center.tolist(), "eligible_substrate_views": [view["_view_metadata"] for view in views],
            "limitations": ["Centrality is a source-frame task preference, not robot reachability or IK.",
                "Owner-only top envelopes use actual observed maximum-height samples per cell and may retain scan noise/outliers; holes are not filled.",
                "Original aggregate map is avoidance/quality evidence, not owner-specific contact geometry or material permission.",
                "Secondary substrate task intent, adhesion and mechanical capacity need separate review; structural support selection remains abstained."]}


def _selected_view(packet, data, task, selected):
    matches = [view for view in _substrate_views(packet, data, task)
               if view["_view_metadata"]["substrate_object_id"] == selected.get("substrate_object_id")
               and view["_view_metadata"]["substrate_geometry_source"] == selected.get("substrate_geometry_source")]
    if len(matches) != 1:
        raise ValueError("Selected substrate is no longer an explicitly eligible bound review view")
    return matches[0]


def _orientation_policy(packet, task):
    return {"schema": "behav3d.initial_wall_orientation_policy.v1", "frame_id": packet["frame_id"],
            "wall_up_axis": "+Z", "yaw_zero_reference": "wall long edge parallel source-frame +X; wall remains vertical",
            "yaw_order_degrees": task["yaw_degrees"],
            "ranking_order": ["hard_semantic_and_geometry_eligibility", "ordered_yaw_preference",
                              "semantic_review_recommendation", "distance_to_primary_table_xy_centre",
                              "minimum_confidence", "minimum_perception_score", "support_height_span"],
            "native_initialization_limit": MAX_NATIVE_INITIALIZATIONS,
            "native_budget_scope": "shared_across_orientation_stages"}


def _semantic_placement_review(packet, object_id):
    """Categorical task preference from structured evidence, never adhesion proof.

    Free-text evidence is retained upstream for review; it is not interpreted as
    executable rules. An earlier raw model hypothesis cannot override a later
    resolved structured source treatment or operator task intent.
    """
    obj = next((record for record in packet.get("objects", []) if record.get("object_id") == object_id), {})
    review = obj.get("semantic_review") or {}
    material = review.get("material") or {}
    label = material.get("value") if isinstance(material.get("value"), str) else "unknown_material"
    unknown = label.lower() in ("", "unknown", "unknown_material", "unknown_substrate", "unspecified", "unverified", "unclassified", "unresolved", "none")
    confidence = material.get("confidence")
    confidence = float(confidence) if type(confidence) in (int, float) and math.isfinite(confidence) and 0 <= confidence <= 1 else None
    source = material.get("source") if isinstance(material.get("source"), str) else None
    affordances = [value.lower() for value in review.get("perception_affordances", []) if isinstance(value, str)]
    if "semantic_review" in obj:
        treatment = (review.get("operational_treatment") or {}).get("value")
    else:
        treatment = obj.get("operational_treatment")
    treatment = treatment.lower() if isinstance(treatment, str) else ""
    role = str(obj.get("semantic_role", "")).lower()
    candidate_class = str(obj.get("candidate_class", "")).lower()
    vetoes = []
    if obj.get("requires_rescan") is True:
        vetoes.append("Upstream object requires a rescan.")
    if role in ("obstacle", "current_print", "unknown_object"):
        vetoes.append("Resolved source semantic role is not a placement substrate: " + role)
    if treatment in ("avoid", "avoid_object", "avoid_until_reviewed", "requires_rescan", "non_printable", "non_printable_surface"):
        vetoes.append("Explicit resolved upstream treatment: " + treatment)
    if candidate_class in ("avoid_object", "requires_rescan", "non_printable_surface", "non_printable"):
        vetoes.append("Explicit upstream candidate class: " + candidate_class)
    if any(value in ("avoid", "avoid_object", "requires_rescan", "non_printable", "non_printable_surface") for value in affordances):
        vetoes.append("Explicit upstream avoidance/rescan/non-printable affordance.")
    positive = []
    if "potentially_printable_on" in affordances:
        positive.append("Source perception affordance POTENTIALLY_PRINTABLE_ON.")
    for hypothesis in review.get("role_hypotheses", []):
        if isinstance(hypothesis, dict) and str(hypothesis.get("role", "")).lower() == "print_on_candidate":
            value = hypothesis.get("confidence")
            if type(value) in (int, float) and math.isfinite(value) and 0 < value <= 1:
                positive.append("Source role hypothesis print_on_candidate; confidence is model evidence, not physical calibration.")
                break
    if vetoes:
        category, rank = "blocked_upstream", 3
    elif unknown:
        category, rank = "material_unknown_review", 2
    elif positive:
        category, rank = "preferred_review_candidate", 0
    else:
        category, rank = "possible_support_review", 1
    context = packet.get("semantic_context") or {}
    restrictions = list(vetoes)
    restrictions.extend(("Material compatibility, adhesion and structural capacity are not physically verified.",
                         "Semantic preference is categorical task-review evidence, not an invented adhesion score."))
    intent = review.get("task_intent_confirmed", obj.get("intent_confirmed")) is True
    if not intent:
        restrictions.append("Task intent for using this object as a substrate is not confirmed.")
    if review.get("operator_review_requested") is True:
        restrictions.append("The upstream semantic interpretation explicitly requests operator review.")
    if context.get("material_review_required") is True:
        restrictions.append("Upstream scene policy requires material review.")
    for check in context.get("upstream_required_checks", []):
        if isinstance(check, str):
            restrictions.append("Upstream required check: " + check)
    for gap in review.get("evidence_gaps", []):
        if isinstance(gap, str):
            restrictions.append("Source evidence gap: " + gap)
    raw_role = str((review.get("llm_interpretation") or {}).get("recommended_role", "")).lower()
    if raw_role in ("avoid", "avoid_object", "obstacle") and not vetoes:
        restrictions.append("Earlier raw model avoidance hypothesis differs from resolved policy; it remains an operator-review warning, not an override.")
    return {"recommendation_class": category, "preference_rank": rank,
            "material_label": label, "material_confidence": confidence, "material_source": source,
            "positive_basis": positive, "restrictions": restrictions,
            "operator_review_required": True, "task_intent_confirmed": intent,
            "material_compatibility_verified": False, "source_files": review.get("source_files") or {}}


def _semantic_placement_policy(packet, views):
    identities = {view["_view_metadata"]["substrate_object_id"] or "primary_cells" for view in views}
    identities.update(record["object_id"] for record in packet.get("objects", []) if record.get("object_id"))
    return {"schema": "behav3d.initial_wall_semantic_policy.v1",
            "ranking_order": _orientation_policy(packet, {"yaw_degrees": []})["ranking_order"][1:],
            "substrate_reviews": {object_id: _semantic_placement_review(packet, None if object_id == "primary_cells" else object_id)
                                  for object_id in sorted(identities)},
            "limitations": ["Material identity and print-on affordances are source hypotheses, not verified compatibility or load capacity.",
                "Hard upstream avoidance/rescan/non-printable policies veto before material preference.",
                "Ordered yaw preference applies across all acceptable review substrates before semantic recommendation and centrality.",
                "Raw model evidence does not override resolved structured source policy; no new model call occurs."]}


def _surface_rejection(packet, region):
    metadata = region.get("_view_metadata") or {}
    object_id = metadata.get("substrate_object_id")
    if object_id and _semantic_placement_review(packet, object_id)["recommendation_class"] == "blocked_upstream":
        return "substrate_semantically_rejected"
    for support in packet["supports"]:
        if support.get("surface_id") != region["surface_id"]:
            continue
        if (support.get("print_candidate_allowed") is False
                or support.get("semantic_class") in ("REJECTED_OBJECT_SURFACE", "NON_PRINTABLE_SURFACE")
                or (support.get("owner") or {}).get("semantic_role") in ("obstacle", "current_print", "unknown_object")):
            return "surface_explicitly_rejected"
    return None


def _center_rejection(packet, data, task, region, center_xy):
    key = tuple(np.floor(np.asarray(center_xy) / packet["cell_size_m"] + 1e-6).astype(np.int64))
    index = data["_cell_lookup"].get(key)
    if index is None:
        return "unobserved_center"
    if data["_cell_ambiguous"][index]:
        return "ambiguous_center"
    if data["_cell_region"][index] != region["region_index"]:
        return "center_outside_region"
    if data["_cell_class"][index] != 1:
        return "center_not_clear"
    if data["_cell_confidence"][index] < task["min_confidence"]:
        return "center_low_confidence"
    if data["_cell_score"][index] < task["min_perception_score"]:
        return "center_low_perception_score"
    if data["_cell_clearance"][index] < task["clearance_m"]:
        return "center_insufficient_clearance"
    return None


def _rectangle(center, u, v, half_width, half_depth, z):
    return [[float((center + a * half_width * u + b * half_depth * v)[0]),
             float((center + a * half_width * u + b * half_depth * v)[1]), float(z)]
            for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def _sample_rectangle(center, u, v, half_width, half_depth, cell):
    nx = max(2, math.ceil(2 * half_width / (cell / 3)) + 1)
    ny = max(2, math.ceil(2 * half_depth / (cell / 3)) + 1)
    if nx * ny > MAX_SAMPLES:
        raise ValueError("footprint sampling exceeds bounded budget")
    x, y = np.meshgrid(np.linspace(-half_width, half_width, nx), np.linspace(-half_depth, half_depth, ny))
    return center + x.ravel()[:, None] * u + y.ravel()[:, None] * v


def _intersected_cells(center, u, v, half_width, half_depth, cell):
    """All square cells with positive-area overlap, including rotated slivers.

    Separating-axis tests use the rectangle's two axes and the square's X/Y
    axes. Dense samples alone cannot prove coverage of a tiny clipped corner.
    """
    corners = np.asarray([center + a * half_width * u + b * half_depth * v
                          for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    lower = np.floor(corners.min(axis=0) / cell).astype(np.int64)
    upper = np.floor(corners.max(axis=0) / cell).astype(np.int64)
    if int(upper[0] - lower[0] + 1) * int(upper[1] - lower[1] + 1) > MAX_SAMPLES:
        raise ValueError("footprint cell intersection exceeds bounded budget")
    keys = np.asarray([(x, y) for x in range(int(lower[0]), int(upper[0]) + 1)
                       for y in range(int(lower[1]), int(upper[1]) + 1)], dtype=np.int64)
    delta = (keys + .5) * cell - center
    half = cell / 2
    eps = 1e-10
    overlap = ((np.abs(delta[:, 0]) < half_width * abs(u[0]) + half_depth * abs(v[0]) + half - eps)
               & (np.abs(delta[:, 1]) < half_width * abs(u[1]) + half_depth * abs(v[1]) + half - eps)
               & (np.abs(delta @ u) < half_width + half * np.sum(np.abs(u)) - eps)
               & (np.abs(delta @ v) < half_depth + half * np.sum(np.abs(v)) - eps))
    return keys[overlap]


def _owning_substrate_ids(packet, region):
    """Only the explicit represented substrate owner may have intended contact.

    The semantic role alone is insufficient: a board or another table labelled
    support is still an obstacle unless it owns this exact selected surface.
    """
    owners = set()
    if region.get("_owner_only_view"):
        object_id = region["_view_metadata"]["substrate_object_id"]
        return {object_id}
    for support in packet["supports"]:
        owner = support.get("owner") or {}
        source_id = support.get("source_object_id") or owner.get("object_id")
        if (support.get("surface_id") == region["surface_id"]
                and support.get("print_candidate_allowed") is True
                and owner.get("semantic_role") in ("support", "printing_plane")
                and source_id and owner.get("object_id", source_id) == source_id):
            owners.add(source_id)
    return owners


def _pose_obstacle_evidence(packet, data, task, region, center_xy, yaw, base, contact_ceiling):
    """Check the bounded wall volume at its supplied, exact base height."""
    if data["_object_tree"] is None:
        return 0, 0
    u, v = _axes(yaw)
    half_width = task["width_m"] / 2 + task["footprint_margin_m"]
    half_depth = task["thickness_m"] / 2 + task["footprint_margin_m"]
    radius = math.hypot(half_width + task["clearance_m"], half_depth + task["clearance_m"])
    nearby = data["_object_tree"].query_ball_point(center_xy, radius)
    obstacles = data["object_points"][nearby]
    relative = obstacles[:, :2] - center_xy
    collision = ((np.abs(relative @ u) <= half_width + task["clearance_m"])
                 & (np.abs(relative @ v) <= half_depth + task["clearance_m"])
                 & (obstacles[:, 2] >= base)
                 & (obstacles[:, 2] <= base + task["height_m"] + task["clearance_m"]))
    owners = _owning_substrate_ids(packet, region)
    owner_ids = data["object_ids"][data["object_owner_index"][nearby]]
    rows = data["_object_cell_index"][nearby]
    represented = rows >= 0
    safe_rows = np.maximum(rows, 0)
    represented &= ~data["_cell_ambiguous"][safe_rows]
    known_classes = [packet["class_values"][name] for name in
                     ("CLEAR_CANDIDATE", "MATERIAL_REVIEW", "CAUTION_CLEARANCE", "BLOCKED", "NON_PRINTABLE_TOPOGRAPHY")
                     if name in packet["class_values"]]
    represented &= np.isin(data["_cell_class"][safe_rows], known_classes)
    if region["surface_id"] != packet.get("primary_surface_id") and not region.get("_owner_only_view"):
        represented &= data["_cell_region"][safe_rows] == region["region_index"]
    # Contact tolerance belongs only to points from the exact owning substrate
    # and is measured against BOTH its represented local cell and the selected
    # footprint/frontier's global top. A high owner hump cannot authorize itself.
    # Below-surface owner geometry is legitimate buried contact; this is a
    # one-sided upper residual bound, not a symmetric +/- noise interval.
    # A CLEAR region is placement permission, not ground identity: known primary
    # cells outside it may explain the same-table points in the clearance band.
    # Unknown/ambiguous/missing cells and every other object stay non-exempt.
    intended_contact = (np.isin(owner_ids, list(owners)) & represented
                        & (obstacles[:, 2] <= data["_cell_high"][safe_rows]
                           + task["support_contact_tolerance_m"])
                        & (obstacles[:, 2] <= contact_ceiling))
    contact_count = int(np.count_nonzero(collision & intended_contact))
    return int(np.count_nonzero(collision & ~intended_contact)), contact_count


def _evaluate_pose(packet, data, task, region, center_xy, yaw, *, base_z=None):
    """Cheap site gates, separate from exact native mesh placement below scan."""
    semantic_rejection = _surface_rejection(packet, region)
    if semantic_rejection:
        return None, semantic_rejection
    u, v = _axes(yaw)
    half_width = task["width_m"] / 2 + task["footprint_margin_m"]
    half_depth = task["thickness_m"] / 2 + task["footprint_margin_m"]
    samples = _sample_rectangle(center_xy, u, v, half_width, half_depth, packet["cell_size_m"])
    cell_keys = _intersected_cells(center_xy, u, v, half_width, half_depth, packet["cell_size_m"])
    indices = np.asarray([data["_cell_lookup"].get(tuple(key), -1) for key in cell_keys])
    # Explicit square-cell occupancy, not nearest points or bounding-box area.
    if np.any(indices < 0):
        return None, "unobserved_footprint"
    if np.any(data["_cell_ambiguous"][indices]):
        return None, "ambiguous_surface_cell"
    if np.any(data["_cell_region"][indices] != region["region_index"]):
        return None, "footprint_crosses_region"
    if np.any(data["_cell_class"][indices] != 1):
        return None, "footprint_not_clear"
    if np.any(data["_cell_confidence"][indices] < task["min_confidence"]):
        return None, "low_confidence"
    if np.any(data["_cell_score"][indices] < task["min_perception_score"]):
        return None, "low_perception_score"
    if np.any(data["_cell_clearance"][indices] < task["clearance_m"]):
        return None, "insufficient_clearance"
    low, high = data["_cell_low"][indices], data["_cell_high"][indices]
    span = float(np.max(high) - np.min(low))
    inferred_base = float(np.min(low) - task["base_z_offset_m"])
    if base_z is not None and not math.isclose(base_z, inferred_base, abs_tol=1e-9, rel_tol=0):
        return None, "base_height_changed"
    base = inferred_base if base_z is None else base_z
    contact_ceiling = float(np.max(high) + task["support_contact_tolerance_m"])
    obstacle_count, contact_count = _pose_obstacle_evidence(
        packet, data, task, region, center_xy, yaw, base, contact_ceiling)
    if obstacle_count:
        return None, "full_height_obstacle"
    center3 = [float(center_xy[0]), float(center_xy[1]), float(base)]
    left = np.array([*(center_xy - task["width_m"] / 2 * u), base])
    right = np.array([*(center_xy + task["width_m"] / 2 * u), base])
    rise = np.array([0., 0., task["height_m"]])
    pose = {"center_m": center3, "yaw_deg": float(yaw), "region_id": region["region_id"],
            "region_index": region["region_index"], "surface_id": region["surface_id"],
            "support_object_id": None, "wall_corners_m": [left.tolist(), right.tolist(),
                                                         (right + rise).tolist(), (left + rise).tolist()],
            "footprint_m": _rectangle(center_xy, u, v, half_width, half_depth, base),
            "checks": {"sampled_footprint_clear": True, "single_eligible_region": True,
                       "site_global_flatness_required": False, "sampled_full_wall_volume_clear": True,
                       "owning_substrate_contact_bounded": True,
                       "ik_checked": False, "swept_tool_collision_checked": False,
                       "material_adhesion_verified": False, "structural_capacity_verified": False},
            "metrics": {"footprint_sample_count": len(samples), "footprint_checked_cell_count": len(indices),
                        "base_height_span_m": span,
                        "owning_substrate_contact_sample_count": contact_count,
                        "support_contact_tolerance_m": task["support_contact_tolerance_m"],
                        "site_contact_ceiling_world_z_m": contact_ceiling,
                        "minimum_confidence": float(np.min(data["_cell_confidence"][indices])),
                        "minimum_perception_score": float(np.min(data["_cell_score"][indices])),
                        "mean_confidence": float(np.mean(data["_cell_confidence"][indices])),
                        "mean_perception_score": float(np.mean(data["_cell_score"][indices])),
                        "full_height_obstacle_sample_count": obstacle_count}}
    pose["placement_id"] = "placement-" + _digest(pose)[:20]
    metadata = region.get("_view_metadata")
    if metadata:
        pose.update({key: metadata[key] for key in ("substrate_object_id", "substrate_label", "substrate_geometry_source")})
        pose["substrate_review_required"] = True
        pose["semantic_placement_review"] = _semantic_placement_review(packet, metadata["substrate_object_id"])
        pose["placement_id"] = "placement-" + _digest({key: value for key, value in pose.items() if key != "placement_id"})[:20]
    return pose, None


def _limitations(packet):
    return ["Geometric proposal only: no fabrication, robot-frame registration, IK or tool swept-volume approval.",
            "Coverage is a bounded square-cell sampling check; unknown space and hidden objects remain unknown.",
            "Scene cells may include registered-plane augmentation; they are not independent measured support proof.",
            "Existing support intent, material, adhesion and mechanical capacity are not established.",
            "Single centre-plane first-batch beads do not establish complete wall thickness or load capacity.",
            "Configured substrate contact and frontier variation limits are explicit offline review assumptions, not calibrated physical accuracy.",
            "Scene registration status: " + str(packet.get("registration_status", "unverified"))]


def _support_assessment(packet, data, selected, task):
    candidates = []
    if selected:
        center = np.asarray(selected["center_m"])
        u, v = _axes(selected["yaw_deg"])
        for record in packet["supports"]:
            owner = record.get("source_object_id") or record.get("object_id")
            matches = np.flatnonzero(data["object_ids"] == owner) if owner else np.array([], dtype=int)
            points = data["object_points"][np.isin(data["object_owner_index"], matches)]
            if not len(points):
                continue
            delta = points - center
            contact = ((np.abs(delta[:, :2] @ u) <= task["width_m"] / 2)
                       & (np.abs(delta[:, :2] @ v) <= task["thickness_m"] / 2 + task["clearance_m"])
                       & (delta[:, 2] >= 0) & (delta[:, 2] <= task["height_m"]))
            candidates.append({"object_id": str(owner), "near_wall_sample_count": int(np.count_nonzero(contact)),
                               "intent_confirmed": record.get("intent_confirmed") is True,
                               "mechanical_capacity_verified": False, "selected": False})
    return {"decision": "ABSTAIN", "selected_support_object_id": None, "existing_candidates": candidates,
            "reason": "Existing-object proximity alone does not justify using it as a structural or adhesion support."}


def propose_wall(packet, arrays, task):
    """Search bounded deterministic XY/yaw alternatives; never generate candidates."""
    data, task = _inputs(packet, arrays, task)
    bindings = _bindings(packet, data, task)
    reasons, tested, truncated = {}, 0, False
    center_prunes = {}
    views = _substrate_views(packet, data, task)
    policy = _placement_policy(packet, data, views)
    semantic_policy = _semantic_placement_policy(packet, views)
    rejected_semantic_candidates = {entry.get("object_id") for entry in
        packet.get("support_grid_provenance", {}).get("augmentations", {}).get("semantic_support_augmentation", {}).get("objects", [])}
    rejected_semantic_candidates.update(support.get("source_object_id") or (support.get("owner") or {}).get("object_id")
                                       for support in packet["supports"] if support.get("surface_id") == packet.get("primary_surface_id"))
    rejected_count = sum(_semantic_placement_review(packet, object_id)["recommendation_class"] == "blocked_upstream"
                         for object_id in rejected_semantic_candidates if object_id)
    if rejected_count:
        reasons["substrate_semantically_rejected"] = rejected_count
    reference = np.asarray(policy["reference_center_m"])
    search_diagnostics, queue = {}, []
    for view_index, view in enumerate(views):
        view_id = view["_view_metadata"]["substrate_object_id"] or "primary_cells"
        search_diagnostics[view_id] = {"tested_pose_count": 0, "site_accepted_pose_count": 0,
            "native_initialized_pose_count": 0, "rejected_reason_counts": {}}
        for region_index in sorted(view["_regions"]):
            region = view["_regions"][region_index]
            if not region["eligible"]:
                continue
            semantic_rejection = _surface_rejection(packet, region)
            if semantic_rejection:
                reasons[semantic_rejection] = reasons.get(semantic_rejection, 0) + 1
                continue
            points = view["points"][view["suitability_region_index"] == region_index]
            if not len(points):
                continue
            lower, upper = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
            dimensions = np.floor((upper - lower) / task["search_step_m"]).astype(int) + 1
            if int(dimensions[0]) * int(dimensions[1]) * len(task["yaw_degrees"]) > 1_000_000:
                raise ValueError("unbounded search domain; coarser explicit search_step_m required")
            xs = lower[0] + np.arange(dimensions[0]) * task["search_step_m"]
            ys = lower[1] + np.arange(dimensions[1]) * task["search_step_m"]
            midpoint = (lower + upper) / 2
            centres = {(float(midpoint[0]), float(midpoint[1]))}
            if np.all(reference >= lower) and np.all(reference <= upper):
                centres.add(tuple(reference))
            centres.update((float(x), float(y)) for x in xs for y in ys)
            for xy in centres:
                if (len(queue) + 1) * len(task["yaw_degrees"]) > 1_000_000:
                    raise ValueError("unbounded combined substrate/orientation queue; coarser explicit search_step_m required")
                queue.append((float(np.linalg.norm(np.asarray(xy) - reference)), view_id,
                              region_index, xy, view_index, midpoint))
    # Global centrality order gives every substrate an equal opportunity before
    # the shared bounded pose budget is spent; a table search cannot starve the
    # smaller board simply because its object/region ID sorts first.
    queue.sort(key=lambda item: (semantic_policy["substrate_reviews"][item[1]]["preference_rank"], *item[:4]))
    if len(queue) * len(task["yaw_degrees"]) > 1_000_000:
        raise ValueError("unbounded combined substrate/orientation queue; coarser explicit search_step_m required")
    orientations = [{"yaw_deg": yaw, "preference_index": index, "status": "NOT_RUN_PRIOR_STAGE_SELECTED",
        "tested_pose_count": 0, "center_pruned_count": 0, "site_accepted_pose_count": 0,
        "native_attempt_count": 0, "native_valid_pose_count": 0, "full_pose_grid_exhausted": False,
        "full_native_candidates_exhausted": False, "map_rejected_reason_counts": {},
        "native_rejected_reason_counts": {}, "selection_reason": "Not run because a preferred allowed orientation may be selected first."}
        for index, yaw in enumerate(task["yaw_degrees"])]
    initialized, initialization_reasons, initialization_examples = [], {}, {}
    initialization, initialization_attempts, site_accepted_count = None, 0, 0
    unresolved = False
    for stage_index, yaw in enumerate(task["yaw_degrees"]):
        stage = orientations[stage_index]
        accepted = []
        for distance, view_id, region_index, xy, view_index, midpoint in queue:
            view = views[view_index]
            region = view["_regions"][region_index]
            diagnostic = search_diagnostics[view_id]
            center_rejection = _center_rejection(packet, view, task, region, xy)
            if center_rejection:
                center_prunes[center_rejection] = center_prunes.get(center_rejection, 0) + 1
                stage["center_pruned_count"] += 1
                continue
            if tested >= task["max_pose_evaluations"]:
                truncated = True
                break
            tested += 1
            stage["tested_pose_count"] += 1
            diagnostic["tested_pose_count"] += 1
            pose, reason = _evaluate_pose(packet, view, task, region, np.asarray(xy), yaw)
            if reason:
                reasons[reason] = reasons.get(reason, 0) + 1
                stage["map_rejected_reason_counts"][reason] = stage["map_rejected_reason_counts"].get(reason, 0) + 1
                diagnostic["rejected_reason_counts"][reason] = diagnostic["rejected_reason_counts"].get(reason, 0) + 1
            else:
                pose["metrics"]["distance_from_region_center_m"] = float(np.linalg.norm(np.asarray(xy) - midpoint))
                pose["metrics"]["placement_reference_center_m"] = reference.tolist()
                pose["metrics"]["placement_center_distance_m"] = distance
                pose["placement_id"] = "placement-" + _digest({k: v for k, v in pose.items() if k != "placement_id"})[:20]
                accepted.append((pose, view_index))
                diagnostic["site_accepted_pose_count"] += 1
        stage["full_pose_grid_exhausted"] = not truncated
        stage["site_accepted_pose_count"] = len(accepted)
        site_accepted_count += len(accepted)
        accepted.sort(key=lambda item: (item[0]["semantic_placement_review"]["preference_rank"],
                                   item[0]["metrics"]["placement_center_distance_m"],
                                   -item[0]["metrics"]["minimum_confidence"],
                                   -item[0]["metrics"]["minimum_perception_score"],
                                   item[0]["metrics"]["base_height_span_m"],
                                   item[0]["substrate_object_id"] or "", item[0]["region_index"],
                                   item[0]["yaw_deg"], item[0]["center_m"]))
        for candidate, view_index in accepted:
            if initialization_attempts >= MAX_NATIVE_INITIALIZATIONS:
                break
            initialization_attempts += 1
            stage["native_attempt_count"] += 1
            try:
                placed, evidence, _runtime = _initialize_pose(packet, views[view_index], task, candidate)
            except (ValueError, RuntimeError) as error:
                reason = str(error).split(":", 1)[0]
                initialization_reasons[reason] = initialization_reasons.get(reason, 0) + 1
                initialization_examples.setdefault(reason, str(error)[:512])
                stage["native_rejected_reason_counts"][reason] = stage["native_rejected_reason_counts"].get(reason, 0) + 1
                continue
            initialized.append(placed)
            stage["native_valid_pose_count"] += 1
            view_id = placed["substrate_object_id"] or "primary_cells"
            search_diagnostics[view_id]["native_initialized_pose_count"] += 1
            if initialization is None:
                initialization = evidence
            if len(initialized) >= MAX_ALTERNATIVES + 1:
                break
        stage["full_native_candidates_exhausted"] = stage["native_attempt_count"] == len(accepted)
        if initialized:
            stage["status"] = "SELECTED_REVIEW_REQUIRED"
            stage["selection_reason"] = "Selected a valid native placement at this preferred allowed yaw; later orientations were not needed."
            break
        if not stage["full_pose_grid_exhausted"] or not stage["full_native_candidates_exhausted"]:
            unresolved = True
            stage["status"] = "UNRESOLVED_POSE_BUDGET" if truncated else "UNRESOLVED_NATIVE_BUDGET"
            stage["selection_reason"] = "No valid initialized pose found within the current budget; this orientation is unresolved, not proven impossible. Rotation is withheld."
            for later in orientations[stage_index + 1:]:
                later["status"] = "NOT_RUN_PRIOR_STAGE_UNRESOLVED"
                later["selection_reason"] = "Withheld because a preferred orientation has untested map or native candidates."
            break
        stage["status"] = "NO_VALID_POSE_COMPLETE"
        stage["selection_reason"] = "All bounded map positions and accepted native candidates at this yaw were tested without a valid placement; the next allowed yaw may be considered."
    selected = initialized[0] if initialized else None
    proposal = {"schema": SCHEMA, **bindings, "task": task, "status": "PROPOSED_REVIEW_REQUIRED" if selected else "HOLD",
                "selected": selected, "alternatives": initialized[1:MAX_ALTERNATIVES + 1],
                "initialization": initialization, "site_accepted_pose_count": site_accepted_count,
                "placement_policy": policy, "substrate_search_diagnostics": search_diagnostics,
                "semantic_placement_policy": semantic_policy,
                "orientation_policy": _orientation_policy(packet, task), "orientation_diagnostics": orientations,
                "orientation_search_status": "SELECTED_REVIEW_REQUIRED" if selected else "UNRESOLVED_BUDGET" if unresolved else "ALL_ALLOWED_ORIENTATIONS_EXHAUSTED",
                "native_initialization_attempt_count": initialization_attempts,
                "native_initialization_limit": MAX_NATIVE_INITIALIZATIONS,
                "initialization_search_truncated": initialization_attempts < site_accepted_count,
                "initialization_rejected_reason_counts": initialization_reasons,
                "initialization_rejected_examples": initialization_examples,
                "tested_pose_count": tested, "accepted_pose_count": len(initialized), "search_truncated": truncated,
                "center_pruned_count": sum(center_prunes.values()), "center_prune_reason_counts": center_prunes,
                "rejected_reason_counts": reasons, "support_assessment": _support_assessment(packet, data, selected, task),
                "limitations": _limitations(packet), "requires_review": True, "fabrication_authorized": False}
    proposal["selection_reason"] = (
        "Selected the first allowed yaw with a valid native placement; higher-priority yaws must be completely resolved before rotating. "
        "Within this orientation, selected by structured semantic-review recommendation, then distance from the primary table's observed geometric XY centre, "
        "then minimum cell confidence, perception score and support height range. Centrality is a task preference, not IK. The complete footprint "
        "and margin stay in one clear region. The actual requested goal mesh is positioned below the scan by the native fixed-XY/yaw "
        "position_field overlap solve, and its native heat field and phi intersection are reviewed before approval. "
        "Intended contact is permitted only for the represented owning substrate within the explicit local contact band; "
        "other objects, unknowns and protrusions remain obstacles. "
        "No existing object was chosen as an additional support: intent, material compatibility and mechanical capacity are unknown."
        if selected else
        "No initialized placement was selected: inspect per-orientation outcomes to distinguish exhausted geometry from unresolved pose/native budgets. "
        "The checks retain complete footprint coverage, single-region clearance, bounded owning-substrate contact "
        "and native goal/scan intersection checks within the bounded search. "
        "HOLD for inspection or an explicitly revised task; unsupported areas are not filled and obstacles are not discarded.")
    proposal["proposal_id"] = "proposal-" + _digest(proposal)
    return proposal


def validate_proposal(packet, arrays, task, proposal):
    """Recheck the exact selected pose; never search or move an approved placement."""
    data, task = _inputs(packet, arrays, task)
    if not isinstance(proposal, dict) or proposal.get("schema") != SCHEMA:
        raise ValueError("unsupported wall proposal")
    unsealed = {key: value for key, value in proposal.items() if key != "proposal_id"}
    if proposal.get("proposal_id") != "proposal-" + _digest(unsealed):
        raise ValueError("proposal changed since creation")
    if any(proposal.get(key) != value for key, value in _bindings(packet, data, task).items()):
        raise ValueError("scene, geometry, frame or task changed since proposal")
    if proposal.get("fabrication_authorized") is not False or proposal.get("requires_review") is not True:
        raise ValueError("proposal cannot authorize fabrication")
    selected = proposal.get("selected")
    if proposal.get("status") == "HOLD" or selected is None:
        return {"status": "HOLD", "valid": False, "reason": "No selected geometric placement", "fabrication_authorized": False}
    if proposal.get("status") != "PROPOSED_REVIEW_REQUIRED":
        raise ValueError("unsupported selected proposal status")
    expected_policy = _placement_policy(packet, data, _substrate_views(packet, data, task))
    if proposal.get("placement_policy") != expected_policy:
        raise ValueError("Placement substrate eligibility or centrality policy changed since review")
    if proposal.get("orientation_policy") != _orientation_policy(packet, task):
        raise ValueError("Ordered source-frame orientation policy changed since review")
    if proposal.get("semantic_placement_policy") != _semantic_placement_policy(packet, _substrate_views(packet, data, task)):
        raise ValueError("Structured semantic placement-review policy changed since review")
    data = _selected_view(packet, data, task, selected)
    region = data["_regions"].get(selected.get("region_index"))
    center = np.asarray(selected.get("center_m"), dtype=float)
    yaw = selected.get("yaw_deg")
    if (not region or not region["eligible"] or center.shape != (3,) or not np.isfinite(center).all()
            or yaw not in task["yaw_degrees"]):
        raise ValueError("invalid selected pose")
    stages = proposal.get("orientation_diagnostics")
    selected_stage_index = task["yaw_degrees"].index(yaw)
    if (not isinstance(stages, list) or len(stages) != len(task["yaw_degrees"])
            or any(stage.get("yaw_deg") != allowed or stage.get("preference_index") != index
                   for index, (stage, allowed) in enumerate(zip(stages, task["yaw_degrees"], strict=True)))
            or stages[selected_stage_index].get("status") != "SELECTED_REVIEW_REQUIRED"
            or stages[selected_stage_index].get("native_valid_pose_count", 0) < 1):
        raise ValueError("Selected ordered orientation stage is inconsistent with its review evidence")
    if any(stage.get("status") != "NO_VALID_POSE_COMPLETE" or not stage.get("full_pose_grid_exhausted")
           or not stage.get("full_native_candidates_exhausted") or stage.get("native_valid_pose_count") != 0
           for stage in stages[:selected_stage_index]):
        raise ValueError("Cannot approve rotation while a preferred orientation remains unresolved")
    recomputed, reason = _evaluate_pose(packet, data, task, region, center[:2], yaw)
    if reason:
        return {"status": "HOLD", "valid": False, "reason": reason, "fabrication_authorized": False}
    try:
        recomputed, evidence, _runtime = _initialize_pose(packet, data, task, recomputed)
    except (ValueError, RuntimeError) as error:
        return {"status": "HOLD", "valid": False, "reason": str(error), "fabrication_authorized": False}
    if _digest(evidence) != _digest(proposal.get("initialization")):
        raise ValueError("native goal initialization changed since exact review")
    for key in ("center_m", "yaw_deg", "wall_corners_m", "footprint_m", "region_id", "surface_id", "support_object_id", "checks", "initialization_metrics",
                "substrate_object_id", "substrate_label", "substrate_geometry_source", "substrate_review_required", "semantic_placement_review"):
        if recomputed[key] != selected[key]:
            raise ValueError("selected pose geometry/checks were modified")
    if selected["support_object_id"] is not None:
        raise ValueError("automatic existing-support selection is not supported")
    reference = expected_policy["reference_center_m"]
    if (selected["metrics"].get("placement_reference_center_m") != reference
            or not math.isclose(selected["metrics"].get("placement_center_distance_m", -1),
                float(np.linalg.norm(center[:2] - reference)), abs_tol=1e-12, rel_tol=0)):
        raise ValueError("Selected placement centrality metrics changed since review")
    return {"status": "VALIDATED_REVIEW_REQUIRED", "valid": True, "proposal_id": proposal["proposal_id"],
            "selected": selected, "requires_review": True, "fabrication_authorized": False}


def _production():
    # Source checkout and normal installed ROS package layouts are both supported;
    # imports here never load ROS, robot SDKs, hardware clients or model backends.
    root = Path(__file__).resolve().parents[4]
    source = root / "src/behav3d_py/behav3d_py/scalar_field"
    if source.is_dir() and str(source) not in sys.path:
        sys.path.insert(0, str(source))
    try:
        prefix = "lib_scalar"
        importlib.import_module(prefix + ".compute_phi_mask")
    except ModuleNotFoundError as error:
        if error.name != "lib_scalar":
            raise
        prefix = "behav3d_py.scalar_field.lib_scalar"
    return {name: importlib.import_module(prefix + "." + name) for name in
            ("compute_phi_mask", "compute_heat_field", "position_field", "extract_phi_contour",
             "generate_print_points", "print_targets", "target_rules")}


def _substrate_mesh(packet, data, task, region_index, selected=None):
    import open3d as o3d
    eligible = ((data["_cell_region"] == region_index) & (data["_cell_class"] == 1)
                & ~data["_cell_ambiguous"] & (data["_cell_confidence"] >= task["min_confidence"])
                & (data["_cell_score"] >= task["min_perception_score"]))
    if selected is not None:
        u, v = _axes(selected["yaw_deg"])
        delta = data["_cell_point"][:, :2] - np.asarray(selected["center_m"][:2])
        padding = task["footprint_margin_m"] + 2 * packet["cell_size_m"]
        eligible &= ((np.abs(delta @ u) <= task["width_m"] / 2 + padding)
                     & (np.abs(delta @ v) <= task["thickness_m"] / 2 + padding))
    points = data["_cell_point"][eligible]
    grid = np.asarray([record["key"] for record, keep in zip(data["_cells"], eligible) if keep])
    lookup = {}
    for index, xy in enumerate(grid):
        key = tuple(xy)
        lookup[key] = index
    faces = []
    for (x, y), a in sorted(lookup.items()):
        keys = ((x + 1, y), (x, y + 1), (x + 1, y + 1))
        if all(key in lookup for key in keys):
            b, c, d = (lookup[key] for key in keys)
            quad = points[[a, b, c, d]]
            if np.ptp(quad[:, 2]) <= task["max_support_triangle_height_span_m"]:
                faces.extend(((a, b, d), (a, d, c)))
    if not faces:
        raise ValueError("no unambiguous adjacent-cell substrate triangles; holes are not filled")
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(points), o3d.utility.Vector3iVector(np.asarray(faces)))


def _field_geometry(task, selected):
    # One and only one goal/scalar mesh. Native positioning translates the
    # actual lower frontier below the scan; no scalar-only guard extension.
    width, height = task["width_m"], task["height_m"]
    spacing = min(task["bead_spacing_m"] / 4, task["bead_diameter_m"] / 4)
    xs = np.linspace(-width / 2, width / 2, math.ceil(width / spacing) + 1)
    zs = np.linspace(0, height, math.ceil(height / (task["bead_height_m"] / 2)) + 1)
    all_z = zs
    if len(xs) * len(all_z) > 250_000:
        raise ValueError("field tessellation exceeds bounded budget")
    u, _ = _axes(selected["yaw_deg"])
    center = np.asarray(selected["center_m"])
    local = np.asarray([[x * u[0], x * u[1], z] for z in all_z for x in xs])
    faces = []
    nx = len(xs)
    for row in range(len(all_z) - 1):
        for col in range(nx - 1):
            a = row * nx + col
            faces.extend(((a, a + 1, a + nx + 1), (a, a + nx + 1, a + nx)))
    faces = np.asarray(faces, dtype=np.int32)
    return local, faces, center


def _initialize_pose(packet, data, task, site_pose):
    """Run actual native placement at ONE XY/yaw, heat, phi and frontier.

    In-memory proposal evidence has no candidate YAML or approval side effect.
    Translation conditions float32 rays without changing axes or XY/yaw. The
    native position_field receives singleton XY samples, never a second search.
    """
    production = _production()
    local, faces, _unused_center = _field_geometry(task, site_pose)
    heat = production["compute_heat_field"].compute_heat_field(
        local, faces, seed=None, seed_level=task["heat_seed_level_m"], t_coef=task["heat_t_coef"])
    if (not np.isfinite(heat.dist).all() or not np.isfinite(heat.norm).all()
            or np.min(heat.norm) < 0 or np.max(heat.norm) > 1):
        raise ValueError("native_heat_invalid: nonfinite or unnormalized native heat solve")
    substrate = _substrate_mesh(packet, data, task, site_pose["region_index"], site_pose)
    support_world = np.asarray(substrate.vertices).copy()
    support_faces = np.asarray(substrate.triangles).copy()
    origin = np.asarray([*site_pose["center_m"][:2], float(np.median(support_world[:, 2]))])
    substrate.translate(-origin)
    scene, _legacy_far_ray_origin = production["compute_phi_mask"].make_scan_scene(substrate)
    ray_margin = max(task["height_m"], task["bead_height_m"], packet["cell_size_m"])
    z_top = float(np.max(np.asarray(substrate.vertices)[:, 2]) + ray_margin)
    try:
        state = production["position_field"].position_field(
            scene, z_top, local, heat.norm, np.asarray([0.]), np.asarray([0.]),
            clearance=0., iso_level=task["iso_level_m"], base_z_offset=task["base_z_offset_m"],
            require_full_hit=True, verbose=False)
    except RuntimeError as error:
        raise ValueError("native_position_no_full_hit: " + str(error)) from error
    numeric_vertices = state.field_vertices_world
    world = numeric_vertices + origin
    if not state.has_hit.all() or not np.isfinite(state.phi).all():
        raise ValueError("native_position_incomplete: every actual goal vertex must hit the represented support")
    contour, lines = production["extract_phi_contour"].extract_phi_contour(
        numeric_vertices, faces, state.phi, iso=task["iso_level_m"])
    if not len(lines) or not np.any(state.viable) or not np.any(state.phi <= task["iso_level_m"]):
        raise ValueError("native_frontier_missing: actual goal must contain buried and viable vertices and an intersection contour")
    frontier_span = float(np.ptp(contour[:, 2]))
    if frontier_span > task["max_frontier_height_span_m"]:
        raise ValueError("native_frontier_height_span: contour variation exceeds explicit offline review profile")
    center = [float(origin[0]), float(origin[1]), float(origin[2] + state.offset_xyz[2])]
    obstacle_count, contact_count = _pose_obstacle_evidence(
        packet, data, task, data["_regions"][site_pose["region_index"]],
        np.asarray(center[:2]), site_pose["yaw_deg"], center[2],
        float(np.max(contour[:, 2]) + origin[2] + task["support_contact_tolerance_m"]))
    if obstacle_count:
        raise ValueError("native_full_height_obstacle: exact positioned goal volume intersects retained object samples")
    base_mask = local[:, 2] <= min(local[:, 2]) + max(task["base_z_offset_m"], 1e-6 * float(np.linalg.norm(np.ptp(local, axis=0))))
    metrics = {"goal_width_m": task["width_m"], "goal_height_m": task["height_m"],
               "base_z_offset_m": task["base_z_offset_m"], "goal_base_world_z_m": center[2],
               "observed_frontier_min_z_m": float(np.min(contour[:, 2]) + origin[2]),
               "observed_frontier_max_z_m": float(np.max(contour[:, 2]) + origin[2]),
               "frontier_height_span_m": frontier_span,
               "max_frontier_height_span_m": task["max_frontier_height_span_m"],
               "minimum_base_overlap_m": float(np.min(-state.base_dz[base_mask])),
               "maximum_base_overlap_m": float(np.max(-state.base_dz[base_mask])),
               "buried_vertex_count": int(np.count_nonzero(state.phi <= task["iso_level_m"])),
               "viable_vertex_count": int(np.count_nonzero(state.viable)),
               "ray_hit_count": int(np.count_nonzero(state.has_hit)), "goal_vertex_count": len(local),
               "frontier_segment_count": len(lines), "heat_seed_level_m": task["heat_seed_level_m"],
               "exact_pose_obstacle_sample_count": obstacle_count,
               "exact_pose_owning_substrate_contact_sample_count": contact_count,
               "exact_pose_contact_ceiling_world_z_m": float(np.max(contour[:, 2]) + origin[2] + task["support_contact_tolerance_m"]),
               "heat_t_coef": task["heat_t_coef"], "heat_seed_info": heat.seed_info,
               "global_site_height_span_m": site_pose["metrics"]["base_height_span_m"],
               "numeric_origin_world_m": origin.tolist(), "ray_start_height_local_m": z_top,
               "ray_launch_margin_m": ray_margin, "field_guard_row_below_base_m": 0.,
               "offline_profile_assumption": "configured frontier variation and owning-substrate contact limits need physical calibration"}
    placed = json.loads(_json(site_pose))
    delta_z = center[2] - placed["center_m"][2]
    placed["center_m"] = center
    for name in ("wall_corners_m", "footprint_m"):
        for point in placed[name]:
            point[2] += delta_z
    placed["checks"].update(native_goal_scan_intersection=True, native_full_hit=True,
                             native_base_overlap=True, native_heat_field=True,
                             native_exact_pose_obstacle_volume_clear=True,
                             frontier_variation_within_profile=True)
    placed["initialization_metrics"] = metrics
    placed["placement_id"] = "placement-" + _digest({key: value for key, value in placed.items() if key != "placement_id"})[:20]
    evidence = {"schema": "behav3d.initial_goal_initialization.v1",
                "implementation": "native_position_field_fixed_xy_yaw+compute_heat_field",
                "goal_vertices_m": world.tolist(), "goal_faces": faces.tolist(),
                "heat_norm": heat.norm.tolist(), "heat_dist_m": heat.dist.tolist(),
                "phi_m": state.phi.tolist(), "z_scan_world_m": (state.z_scan + origin[2]).tolist(),
                "frontier_points_m": (contour + origin).tolist(), "frontier_lines": lines.tolist(),
                "support_vertices_m": support_world.tolist(), "support_faces": support_faces.tolist(), "metrics": metrics}
    if len(_json(evidence).encode()) > MAX_INITIALIZATION_BYTES:
        raise ValueError("native_initialization_evidence_budget: explicit tessellation exceeds 16MiB review budget")
    runtime = {"production": production, "local": local, "faces": faces, "heat": heat,
               "origin": origin, "numeric_vertices": numeric_vertices, "world": world,
               "state": state, "contour": contour, "lines": lines, "z_top": z_top,
               "support_world": support_world, "support_faces": support_faces}
    return placed, evidence, runtime


def _candidate_width_mask(contour, u, task):
    tangent_coordinates = contour[:, :2] @ u
    return np.abs(tangent_coordinates) <= (task["width_m"] - task["bead_diameter_m"]) / 2 - TARGET_ROUNDING_RESERVE_M


def _write_exact_goal_mesh(path, vertices, faces):
    """ASCII PLY with round-trip float64 vertices, not Open3D's 6-digit output.

    This is only deterministic artifact serialization. Geometry, topology and
    field placement still come from the unchanged native kernels above.
    """
    with Path(path).open("x", encoding="ascii") as stream:
        stream.write("ply\nformat ascii 1.0\nelement vertex " + str(len(vertices))
                     + "\nproperty double x\nproperty double y\nproperty double z\nelement face "
                     + str(len(faces)) + "\nproperty list uchar uint vertex_indices\nend_header\n")
        stream.writelines(" ".join(format(float(value), ".17g") for value in vertex) + "\n" for vertex in vertices)
        stream.writelines("3 " + " ".join(str(int(value)) for value in face) + "\n" for face in faces)


def generate_first_batch(packet, arrays, task, proposal, output):
    """Generate a data-only first batch through the actual production kernels.

    The caller must enforce exact human approval. This function deliberately
    cannot issue a robot command or infer that approval has been granted.
    """
    validation = validate_proposal(packet, arrays, task, proposal)
    if not validation["valid"]:
        return validation
    data, task = _inputs(packet, arrays, task)
    selected = proposal["selected"]
    bound_data = data
    data = _selected_view(packet, data, task, selected)
    region = data["_regions"][selected["region_index"]]
    site, reason = _evaluate_pose(packet, data, task, region, np.asarray(selected["center_m"][:2]), selected["yaw_deg"])
    if reason:
        return {"status": "HOLD", "valid": False, "reason": reason, "fabrication_authorized": False}
    placed, evidence, runtime = _initialize_pose(packet, data, task, site)
    if placed["center_m"] != selected["center_m"] or _digest(evidence) != _digest(proposal["initialization"]):
        raise ValueError("exact approved native goal changed; regeneration cannot reposition it")
    production, local, faces, heat = (runtime[name] for name in ("production", "local", "faces", "heat"))
    origin, numeric_vertices, world, state = (runtime[name] for name in ("origin", "numeric_vertices", "world", "state"))
    contour, lines, z_top = (runtime[name] for name in ("contour", "lines", "z_top"))
    offset = np.asarray(selected["center_m"])
    u, _ = _axes(selected["yaw_deg"])
    mask = _candidate_width_mask(contour, u, task)
    points = production["generate_print_points"].generate_print_points(
        contour, lines, numeric_vertices, heat.norm, count=task["count"], min_spacing=task["bead_spacing_m"],
        point_valid_mask=mask, candidate_mode="gradient_lift", lift_height=task["bead_height_m"],
        field_faces=faces, walk_tangent_sign=1.0)
    targets = production["print_targets"].build_candidate_segment_targets(
        candidate_points=points.points, segment_start_points=points.segment_start_points,
        candidate_mode="gradient_lift", field_vertices_world=numeric_vertices, field_faces=faces, field_scalar=heat.norm,
        tangent_sign=1.0, clamp_to_cone=False, cone_max_tilt_deg=0.0, orient_with_tangent=True)
    targets, rule_stats = production["target_rules"].apply_secondary_target_rules(
        targets, candidate_mode="gradient_lift", bead_height_m=task["bead_height_m"],
        bead_separation_m=task["bead_spacing_m"], normal_continuity_rule=task["normal_continuity_rule"])
    if not targets.count:
        return {"status": "HOLD", "valid": False, "reason": "Native planner produced no viable first-batch targets",
                "fabrication_authorized": False}
    target_points_world = targets.end_points + origin
    relative = target_points_world - offset
    _, v = _axes(selected["yaw_deg"])
    radius = task["bead_diameter_m"] / 2
    # Include YAML's 0.01 mm origin rounding in the approved-footprint check.
    if (np.any(np.abs(relative[:, :2] @ u) + radius + TARGET_ROUNDING_RESERVE_M > task["width_m"] / 2)
            or np.any(np.abs(relative[:, :2] @ v) + radius > task["thickness_m"] / 2 + TARGET_ROUNDING_RESERVE_M)
            or np.any(relative[:, 2] <= 0) or np.any(relative[:, 2] > task["height_m"])):
        return {"status": "HOLD", "valid": False, "reason": "Native targets extend beyond approved wall footprint/height",
                "fabrication_authorized": False}
    output = Path(output).resolve()
    if output.exists():
        raise ValueError("generation output must be a new directory; no overwrite")
    if not output.parent.is_dir():
        raise ValueError("generation output parent must already exist")
    # Reserve the destination exclusively. The final rename replaces only our
    # own empty reservation, never a pre-existing user's directory or files.
    output.mkdir(mode=0o700, exist_ok=False)
    producer_files = {"initial_planning.py": _file_hash(__file__)}
    for name, module in production.items():
        producer_files["lib_scalar/" + name + ".py"] = _file_hash(module.__file__)
    # Capture transitive kernel sources too, not just the five entry points.
    scalar_root = Path(production["target_rules"].__file__).parent
    producer_files.update({"lib_scalar/" + path.name: _file_hash(path) for path in sorted(scalar_root.glob("*.py"))})
    result = {"schema": "behav3d.initial_wall_batch.v1", "status": "GENERATED_REVIEW_REQUIRED", "valid": True,
              **_bindings(packet, bound_data, task), "proposal_id": proposal["proposal_id"], "selected": selected,
              "requested_count": task["count"], "candidate_count": targets.count,
              "partial_batch": targets.count < task["count"], "requires_review": True, "fabrication_authorized": False,
              "planner": {"candidate_mode": "gradient_lift", "heat": "native compute_heat_field multi-source heat-method distance on actual goal mesh",
                          "phi_clearance_m": 0.0, "iso_level_m": task["iso_level_m"],
                          "base_z_offset_m": task["base_z_offset_m"], "heat_seed_level_m": task["heat_seed_level_m"],
                          "heat_t_coef": task["heat_t_coef"], "field_guard_row_below_base_m": 0.,
                          "goal_and_scalar_mesh_identical": True, "position_scale": 1000.0,
                          "base_to_world_yaw_deg": 0.0, "target_origin_units": "mm", "target_frame_id": packet["frame_id"],
                          "numeric_conditioning": {"world_origin_m": origin.tolist(), "rotation": "identity",
                              "ray_start_height_local_m": z_top, "ray_launch_margin_m": evidence["metrics"]["ray_launch_margin_m"],
                              "rounding_reserve_m": TARGET_ROUNDING_RESERVE_M,
                              "z_scan_world_min_m": float(state.z_scan.min() + origin[2]),
                              "z_scan_world_max_m": float(state.z_scan.max() + origin[2])},
                          "normal_continuity_replaced": int(rule_stats["normal_continuity_replaced"]),
                          "endpoint_spacing_removed": int(rule_stats["endpoint_spacing_removed"])},
              "initialization_metrics": evidence["metrics"],
              "candidate_points_m": target_points_world.tolist(), "candidate_z_dirs": targets.end_z_dirs.tolist(),
              "producer": {"sha256": _digest(producer_files), "files": producer_files}, "limitations": _limitations(packet)}
    with tempfile.TemporaryDirectory(prefix=".initial-wall-", dir=output.parent) as temporary:
        folder = Path(temporary)
        np.savez_compressed(folder / "field_state_init.npz", field_vertices_scaled=local, field_faces=faces,
                            heat_dist=heat.dist, heat_norm=heat.norm, offset_xyz=offset, field_vertices_world=world,
                            phi=state.phi, z_scan_world=state.z_scan + origin[2],
                            frontier_points_world_m=contour + origin, frontier_edges=lines,
                            support_vertices_world_m=runtime["support_world"], support_faces=runtime["support_faces"],
                            base_z_offset_m=np.array(task["base_z_offset_m"]),
                            seed_level=np.array(task["heat_seed_level_m"]), t_coef=np.array(task["heat_t_coef"]),
                            numeric_origin_world=origin, ray_start_height_local=np.array(z_top),
                            field_scale=np.array(1.0), coordinate_units=np.array("m"),
                            frame_id=np.array(packet["frame_id"]))
        _write_exact_goal_mesh(folder / "goal_mesh.ply", world, faces)
        production["print_targets"].write_point_targets_yaml(folder / "targets.yaml", target_points_world,
                                                              targets.end_z_dirs, position_scale=1000.0,
                                                              base_to_world_yaw_deg=0.0)
        result["files"] = {name: {"path": str(output / name), "sha256": _file_hash(folder / name)}
                           for name in ("field_state_init.npz", "goal_mesh.ply", "targets.yaml")}
        (folder / "manifest.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        folder.rename(output)
    result["manifest_path"] = str(output / "manifest.json")
    result["manifest_sha256"] = _file_hash(output / "manifest.json")
    return result
