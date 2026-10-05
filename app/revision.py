"""轮廓修订：修复师更正测量有误的陶片轮廓。

修复师选定一片陶片，一次提交：
- 修订后的毫米制闭合轮廓（沿用既有非自交校验）；
- 该片**每条有效拼接关系**的新边缘标记（位于修订轮廓边缘、数量与关系
  另一端一致、每片至少 2 处且同片不重合）；
- 期望装配版本（确认时核对，过期返回 409）。

校验（以**现有位姿**执行，不重新配准、不移动任何碎片）：
- 受影响组（碎片所在连通组）内**全部有效关系**的对应点误差 ≤ 1mm
  —— 涉及本碎片的关系用新标记重算，其余关系用库存标记复核；
- 受影响组成员轮廓（本碎片用修订轮廓）内部不重叠；
- 独立碎片（无有效关系）只校验轮廓。

预览（preview）只演算不写库，返回逐条关系误差与冲突原因；确认（commit）
在锁与事务内重新校验同一输入，全部通过才原子更新轮廓、相关关系的标记与
误差，保持位姿、关系 ID、分组与已撤销历史不变，装配版本只递增一次。
遗漏有效关系或任一校验失败，整次拒绝、不写入任何变化。
"""
from __future__ import annotations

from . import geometry


def _pose(f: dict) -> tuple[float, float, float]:
    return (f["x"], f["y"], f["theta"])


def active_relations_of(state, fid: int) -> list[dict]:
    """该碎片参与的全部有效拼接关系（按 id 排序）。"""
    rels = [r for r in state.all_relations()
            if r["active"] and fid in (r["fragment_a"], r["fragment_b"])]
    return sorted(rels, key=lambda r: r["id"])


def evaluate_revision(state, frag: dict, contour_in,
                      marker_sets: list[dict]) -> dict:
    """演算轮廓修订（只读，不落库），预览与确认共用。

    state 只需实现 get_fragment / group_members / all_relations（DB 满足）。
    marker_sets: [{"relation_id": int, "markers": [[x, y], ...]}, ...]

    返回 dict：
      ok / conflicts           —— 总体结论与全部冲突原因
      relations                —— 本碎片每条有效关系的逐对误差等预览信息
      checked_relation_ids     —— 受影响组内实际受检的全部有效关系 id
      contour / contour_world  —— 规范化后的修订轮廓及其世界坐标（现有位姿）
      updates                  —— 确认落库用：每条关系的新 markers_a/b 与误差
    """
    fid = frag["id"]
    result = {
        "ok": False,
        "fragment_id": fid,
        "group_id": frag["group_id"],
        "conflicts": [],
        "relations": [],
        "checked_relation_ids": [],
        "contour": None,
        "contour_world": None,
        "updates": [],
    }
    conflicts = result["conflicts"]

    # 1) 修订轮廓：沿用既有非自交校验（自交则后续边缘校验无意义，直接返回）
    try:
        contour = geometry.validate_contour(contour_in)
    except geometry.GeometryError as e:
        conflicts.append(str(e))
        return result
    result["contour"] = contour
    pose_f = _pose(frag)
    result["contour_world"] = geometry.contour_world(contour, pose_f)

    # 2) 提交的关系标记必须恰好覆盖该片全部有效关系：遗漏/多余/重复皆拒绝
    rels = active_relations_of(state, fid)
    active_ids = {r["id"] for r in rels}
    provided: dict[int, list] = {}
    for ms in marker_sets:
        rid = ms["relation_id"]
        if rid in provided:
            conflicts.append(f"关系 #{rid} 的新边缘标记重复提交")
            continue
        provided[rid] = ms["markers"]
    for rid in sorted(provided):
        if rid in active_ids:
            continue
        rel = state.get_relation(rid)
        if rel is None:
            conflicts.append(f"关系 #{rid} 不存在")
        elif not rel["active"]:
            conflicts.append(f"关系 #{rid} 已撤销，不属于有效关系，不应提交新标记")
        else:
            conflicts.append(f"关系 #{rid} 不涉及碎片 #{fid}，不应提交新标记")
    for r in rels:
        if r["id"] not in provided:
            other_id = r["fragment_b"] if r["fragment_a"] == fid else r["fragment_a"]
            other = state.get_fragment(other_id)
            name = other["name"] if other else f"#{other_id}"
            conflicts.append(
                f"遗漏有效关系 #{r['id']}（与「{name}」）：该片的每条有效拼接"
                "关系都必须提供新边缘标记")

    # 3) 本碎片每条有效关系：标记合法性 → 位于修订轮廓边缘 → 现有位姿误差
    for r in rels:
        rid = r["id"]
        if rid not in provided:
            continue
        this_is_a = r["fragment_a"] == fid
        other_id = r["fragment_b"] if this_is_a else r["fragment_a"]
        other = state.get_fragment(other_id)
        other_ms = r["markers_b"] if this_is_a else r["markers_a"]
        pair = (provided[rid], other_ms) if this_is_a else (other_ms, provided[rid])
        try:
            va, vb = geometry.validate_markers(*pair)
        except geometry.GeometryError as e:
            conflicts.append(f"关系 #{rid}：{e}")
            continue
        new_ms = va if this_is_a else vb
        try:
            geometry.validate_markers_on_edge(new_ms, contour)
        except geometry.GeometryError as e:
            conflicts.append(f"关系 #{rid}：{e}")
        pose_o = _pose(other)
        errors = geometry.pair_errors(new_ms, other_ms, pose_f, pose_o)
        for i, e in enumerate(errors):
            if e > geometry.MAX_ERROR_MM + 1e-9:
                conflicts.append(
                    f"关系 #{rid} 误差超限：第 {i + 1} 对对应点误差 "
                    f"{e:.3f} mm，超过 {geometry.MAX_ERROR_MM:.0f} mm 限值")
        result["relations"].append({
            "relation_id": rid,
            "other_fragment_id": other_id,
            "other_fragment_name": other["name"],
            "errors": errors,
            "max_error_mm": max(errors),
            "markers_self_world": [geometry.apply_pose(m, pose_f) for m in new_ms],
            "markers_other_world": [geometry.apply_pose(m, pose_o) for m in other_ms],
        })
        result["updates"].append({
            "relation_id": rid,
            "markers_a": new_ms if this_is_a else r["markers_a"],
            "markers_b": r["markers_b"] if this_is_a else new_ms,
            "max_error": max(errors),
        })

    # 4) 受影响组：组内全部有效关系误差复核（不涉及本碎片的关系用库存标记）
    if frag["group_id"] is not None:
        members = state.group_members(frag["group_id"])
    else:
        members = [frag]                      # 独立碎片：只校验轮廓
    member_ids = {m["id"] for m in members}
    by_id = {m["id"]: m for m in members}
    for r in state.all_relations():
        if not r["active"]:
            continue
        if r["fragment_a"] not in member_ids or r["fragment_b"] not in member_ids:
            continue
        result["checked_relation_ids"].append(r["id"])
        if fid in (r["fragment_a"], r["fragment_b"]):
            continue                          # 已在第 3 步用新标记检查
        fa, fb = by_id[r["fragment_a"]], by_id[r["fragment_b"]]
        errs = geometry.pair_errors(r["markers_a"], r["markers_b"],
                                    _pose(fa), _pose(fb))
        for i, e in enumerate(errs):
            if e > geometry.MAX_ERROR_MM + 1e-9:
                conflicts.append(
                    f"关系 #{r['id']}（同组）误差超限：第 {i + 1} 对对应点误差 "
                    f"{e:.3f} mm，超过 {geometry.MAX_ERROR_MM:.0f} mm 限值")

    # 5) 受影响组成员轮廓（本碎片用修订轮廓）内部不重叠
    placed = [
        (m["id"], m["name"],
         geometry.contour_world(contour if m["id"] == fid else m["contour"],
                                _pose(m)))
        for m in members
    ]
    for n1, n2, area in geometry.find_overlaps(placed):
        conflicts.append(
            f"碰撞：碎片「{n1}」与「{n2}」轮廓内部重叠 {area:.2f} mm²")

    result["ok"] = not conflicts
    return result


def apply_revision(d, ev: dict) -> None:
    """在当前事务内落库：只更新轮廓与相关关系的标记/误差。

    位姿、关系 ID、分组与已撤销历史均保持不变（调用方负责版本递增）。
    """
    d.update_contour(ev["fragment_id"], ev["contour"])
    for u in ev["updates"]:
        d.update_relation_geometry(
            u["relation_id"], u["markers_a"], u["markers_b"], u["max_error"])
