"""步道坡线规划台 —— FastAPI 后端。

业务规则
--------
* 测点 3~16 个，相邻测点相距 1 米；
* 地面标高为 0~200 的整数厘米，方案标高同范围；
* 首、尾方案标高必须等于对应地面标高；
* 相邻方案点高差绝对值不超过 8 厘米；
* 任意连续 5 段中至少有 1 段水平段（等价于：非水平段不得连续超过 4 段）；
* 在可行方案中最小化各点 |方案标高 - 地面标高| 之和；
* 最小总调整量并列时，取方案标高序列字典序最小者。

求解方法：按“段”为阶段做动态规划。状态记录当前标高与“结尾连续非水平段
数”(k=0..4)；倒序计算每个状态到终点的最小剩余代价，再从起点按标高从小到
大贪心恢复，即可同时得到字典序最小的最优序列。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from fastapi import FastAPI
from fastapi.exceptions import RequestValidationError
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

MIN_POINTS = 3
MAX_POINTS = 16
MIN_ELEV = 0
MAX_ELEV = 200
MAX_SLOPE = 8          # 相邻方案点高差上限（厘米）
MAX_RUN = 4            # 非水平段最大连续段数（连续 5 段须含水平段）
WINDOW_SEGS = 5        # “任意连续 5 段”的窗口长度

app = FastAPI(title="步道坡线规划台", version="1.0.0")


class PlanRequest(BaseModel):
    ground: list[int] = Field(..., description="各测点地面标高（整数厘米）")


def client_error(message: str, reason: str, status: int = 400, detail: dict | None = None) -> JSONResponse:
    content: dict[str, Any] = {"ok": False, "error": message, "reason": reason}
    if detail is not None:
        content["detail"] = detail
    return JSONResponse(status_code=status, content=content)


def validate_ground(raw: Any) -> tuple[list[int] | None, JSONResponse | None]:
    """校验并归一化输入。"""
    if not isinstance(raw, list):
        return None, client_error("ground 必须是数组。", "bad_type")
    if not raw or any(isinstance(v, bool) or not isinstance(v, int) for v in raw):
        return None, client_error("每个地面标高必须是整数。", "bad_value")

    if not (MIN_POINTS <= len(raw) <= MAX_POINTS):
        return None, client_error(
            f"测点数必须在 {MIN_POINTS}~{MAX_POINTS} 之间，当前为 {len(raw)} 个。",
            "bad_count",
        )

    for idx, value in enumerate(raw):
        if not (MIN_ELEV <= value <= MAX_ELEV):
            return None, client_error(
                f"第 {idx + 1} 个测点的地面标高 {value} 超出 "
                f"{MIN_ELEV}~{MAX_ELEV} 厘米范围。",
                "bad_elevation",
            )
    return raw, None


def plan(ground: list[int]) -> dict[str, Any]:
    """计算最优坡线方案。假定输入已通过校验。"""
    return _solve(
        ground,
        min_elev=MIN_ELEV,
        max_elev=MAX_ELEV,
        max_slope=MAX_SLOPE,
        max_run=MAX_RUN,
        window_segs=WINDOW_SEGS,
    )


def _solve(
    ground: list[int],
    *,
    min_elev: int,
    max_elev: int,
    max_slope: int,
    max_run: int,
    window_segs: int,
) -> dict[str, Any]:
    n = len(ground)
    seg_count = n - 1
    start_v = ground[0]
    end_v = ground[-1]

    # 不可行的快速判定：仅在坡度限制下，终点标高是否可在 seg_count 段内到达。
    reach_lo = max(min_elev, end_v - max_slope * seg_count)
    reach_hi = min(max_elev, end_v + max_slope * seg_count)
    if not (reach_lo <= start_v <= reach_hi):
        return {
            "ok": False,
            "error": (
                f"不存在可行方案：受相邻高差不超过 {max_slope} 厘米的坡度限制，从起点 "
                f"{start_v} 厘米无法在 {seg_count} 段内到达终点 {end_v} 厘米。"
            ),
            "reason": "unreachable_endpoint",
            "detail": {
                "startElevation": start_v,
                "endElevation": end_v,
                "segments": seg_count,
                "reachableStartRange": [reach_lo, reach_hi],
                "maxSlope": max_slope,
            },
        }

    INF = float("inf")

    # F(i, k, v)：第 i 点标高取 v，且“到达第 i 点之前”已连续出现 k 个非水平
    # 段时，从第 i 点（含其偏差代价）到终点的最小总代价。
    #   * 水平段 v->v：连续非水平计数清零，取 F(i+1, 0, v)；
    #   * 非水平段 v->u：要求 k < max_run，取 F(i+1, k+1, u)。
    # 倒序递推；起点 F(0,0,start) 单独求值。
    span = max_elev - min_elev + 1

    def vidx(v: int) -> int:
        return v - min_elev

    f: list[Any] = [[[INF] * span for _ in range(max_run + 1)] for _ in range(n)]
    # 边界：终点只能取 end_v，代价为其自身偏差。
    end_cost = abs(ground[n - 1] - end_v)
    for k in range(max_run + 1):
        f[n - 1][k][vidx(end_v)] = end_cost

    for i in range(n - 2, 0, -1):
        nxt = f[i + 1]
        cur = f[i]
        for v in range(min_elev, max_elev + 1):
            lo = max(min_elev, v - max_slope)
            hi = min(max_elev, v + max_slope)
            vi = vidx(v)
            base = abs(ground[i] - v)

            for k in range(max_run + 1):
                # 本段水平：计数清零。
                best = nxt[0][vi]
                # 本段非水平：k -> k+1（要求 k+1 <= max_run）。
                if k < max_run:
                    layer = nxt[k + 1]
                    for u in range(lo, hi + 1):
                        if u == v:
                            continue
                        val = layer[vidx(u)]
                        if val < best:
                            best = val
                cur[k][vi] = base + best if best != INF else INF

    # 起点：固定标高 start_v，历史连续非水平段数 k=0。
    if n == 1:
        best_total: float = end_cost
    else:
        lo0 = max(min_elev, start_v - max_slope)
        hi0 = min(max_elev, start_v + max_slope)
        best_total = f[1][0][vidx(start_v)]  # 第 1 段水平 -> 下一状态 k=0
        for u in range(lo0, hi0 + 1):
            if u == start_v:
                continue
            val = f[1][1][vidx(u)]  # 第 1 段非水平 -> 下一状态 k=1
            if val < best_total:
                best_total = val
        if best_total != INF:
            best_total += abs(ground[0] - start_v)
    if best_total == INF:
        # 兜底：理论上坡度可达性判定已覆盖全部不可行情形。
        return {
            "ok": False,
            "error": "不存在可行方案：坡度限制与水平段限制无法同时满足。",
            "reason": "infeasible",
            "detail": {},
        }

    # ---- 恢复字典序最小的最优方案 ----
    # 当前状态 (run_k, v) 的合法下一标高 u 中，按 u 从小到大取达到最小剩余
    # 代价者（水平候选 u=v 也参与自然排序），即得到字典序最小序列。
    design = [start_v]
    run_k = 0
    for i in range(seg_count):
        v = design[-1]
        lo = max(min_elev, v - max_slope)
        hi = min(max_elev, v + max_slope)
        chosen_u: int | None = None
        chosen_k = 0
        chosen_cost: float = INF
        for u in range(lo, hi + 1):
            if i + 1 == seg_count and u != end_v:
                continue  # 末点必须等于地面标高
            if u == v:
                nk = 0  # 水平段：计数清零
            else:
                nk = run_k + 1
                if nk > max_run:
                    continue
            val = f[i + 1][nk][vidx(u)]
            if val < chosen_cost:
                chosen_cost = val
                chosen_u = u
                chosen_k = nk
        assert chosen_u is not None
        design.append(chosen_u)
        run_k = chosen_k

    # ---- 汇总 ----
    diffs = [design[j + 1] - design[j] for j in range(seg_count)]
    horizontal = [d == 0 for d in diffs]
    adjustments = [abs(design[j] - ground[j]) for j in range(n)]

    # 标出违反“连续 window_segs 段须含水平段”的窗口（正常结果中恒为空）。
    violated_windows: list[list[int]] = []
    for s in range(seg_count - window_segs + 1):
        window = list(range(s, s + window_segs))
        if not any(horizontal[j] for j in window):
            violated_windows.append([j + 1 for j in window])  # 1 基段号

    points = [
        {
            "index": i + 1,
            "ground": ground[i],
            "design": design[i],
            "adjustment": adjustments[i],
        }
        for i in range(n)
    ]
    segments = [
        {
            "index": j + 1,
            "from": design[j],
            "to": design[j + 1],
            "diff": diffs[j],
            "horizontal": horizontal[j],
        }
        for j in range(seg_count)
    ]

    return {
        "ok": True,
        "points": points,
        "segments": segments,
        "totalAdjustment": sum(adjustments),
        "horizontalCount": sum(horizontal),
        "violatedWindows": violated_windows,
        "constraints": {
            "minPoints": MIN_POINTS,
            "maxPoints": MAX_POINTS,
            "elevationRange": [MIN_ELEV, MAX_ELEV],
            "maxSlope": MAX_SLOPE,
            "windowSegments": WINDOW_SEGS,
        },
    }


@app.get("/")
def index() -> FileResponse:
    return FileResponse(Path(__file__).parent / "static" / "index.html")


@app.post("/api/plan")
def compute(req: PlanRequest) -> JSONResponse:
    ground, err = validate_ground(req.ground)
    if err is not None:
        return err
    assert ground is not None
    result = plan(ground)
    if not result.get("ok"):
        return JSONResponse(status_code=422, content=result)
    return JSONResponse(content=result)


@app.exception_handler(RequestValidationError)
async def validation_handler(_request, exc: RequestValidationError) -> JSONResponse:
    return client_error(f"请求格式不合法：{exc.errors()[0].get('msg', '参数错误')}", "bad_request")


_static_dir = Path(__file__).parent / "static"
if _static_dir.is_dir():
    app.mount("/static", StaticFiles(directory=str(_static_dir)), name="static")


if __name__ == "__main__":  # pragma: no cover
    import uvicorn

    uvicorn.run(
        "main:app",
        host=os.environ.get("HOST", "0.0.0.0"),
        port=int(os.environ.get("PORT", "8000")),
    )
