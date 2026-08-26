"""OpenAI-compatible VLM client for cup center localization."""
from __future__ import annotations

import base64
import json
import logging
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

log = logging.getLogger("nero_grasp.vlm_client")

DEFAULT_USER_INSTRUCTION = "抓取桌上的清洁剂"

DEFAULT_PROMPT = """\
你是一个智能机械臂的视觉控制中枢。你的任务是根据用户的自然语言指令，在图片中**定位一个**机械臂需要抓取的目标物体。

**重要**：每次只返回 **1 个** 目标；若多个物体都符合，请按指令中的空间/距离约束选出最合适的一个（如「最右边」「最近的一个」）；若无此类约束，任选其中一个即可。

你最重要的是给出**准确的中心点**；box_width/box_height 需要**正好将物体包裹在内**。

坐标系定义：
- 图片左上角为原点 (0, 0)。
- 向右为 x 轴 (0 -> 1)。
- 向下为 y 轴 (0 -> 1)。

用户指令可能包含：
- 颜色/形状/类别特征（如：“抓红色的积木”、“抓螺丝”、“抓胶带”）
- 相对距离描述（如：“抓离机械臂最近的”）
- 空间位置描述（如：“抓最右边的”）
- 用户的输入是通过语音转文字得到的，可能存在歧义，请结合图片内容合理推断

请按照以下步骤推理（Thinking Process）：
1. **分析指令**：提取目标物体的关键特征与限制条件。
2. **扫描图像**：列出所有符合特征的候选物体。
3. **逻辑筛选**：从中选出 **唯一一个** 最符合指令的目标。
4. **给出坐标**：输出该目标的中心点 (box_center_x, box_center_y) 及 box_width/box_height。

抓取点说明（水杯/保温杯）：
- box_center 应落在杯身侧面的几何中心（高度约在杯身中间），不要点在杯盖顶部或桌面阴影上。

输出格式要求：
请输出一个纯净的 JSON 对象，不要包含 Markdown 或额外解释。

字段：
- "thinking_process"：简短推理过程。
- "failed"：未找到任何目标时为 true；否则 false。
- "objects"：数组，**最多 1 个元素**，包含：
  - "id"：整数，固定为 1。
  - "class_name"：物体类别描述。
  - "box_center_x" / "box_center_y"：目标中心，0-1，**必须准确**。
  - "box_width" / "box_height"：包围框尺寸，0-1。

示例 1（单个目标）：
用户指令：“抓取最右边的蓝色长条积木”
{{
    "thinking_process": "图中最右侧有一根蓝色长条积木。",
    "failed": false,
    "objects": [
        {{
            "id": 1,
            "class_name": "蓝色长条积木",
            "box_center_x": 0.821,
            "box_center_y": 0.131,
            "box_width": 0.55,
            "box_height": 0.18
        }}
    ]
}}

示例 2（多个候选但只返回一个）：
用户指令：“抓一个红色方块”
{{
    "thinking_process": "图中有 2 个红色方形积木，任选中央偏下的一个。",
    "failed": false,
    "objects": [
        {{
            "id": 1,
            "class_name": "红色方形积木",
            "box_center_x": 0.385,
            "box_center_y": 0.445,
            "box_width": 0.12,
            "box_height": 0.12
        }}
    ]
}}

示例 3（未找到）：
用户指令：“抓最左边的黄色积木”
{{
    "thinking_process": "图中没有黄色积木。",
    "failed": true,
    "objects": []
}}

现在，请根据图片和以下指令执行任务：
用户指令："{user_instruction}"
"""

DETECTION_RESPONSE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "additionalProperties": False,
    "properties": {
        "thinking_process": {"type": "string"},
        "failed": {"type": "boolean"},
        "objects": {
            "type": "array",
            "maxItems": 1,
            "items": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "id": {"type": "integer"},
                    "class_name": {"type": "string"},
                    "box_center_x": {"type": "number"},
                    "box_center_y": {"type": "number"},
                    "box_width": {"type": "number"},
                    "box_height": {"type": "number"},
                },
                "required": [
                    "id",
                    "class_name",
                    "box_center_x",
                    "box_center_y",
                    "box_width",
                    "box_height",
                ],
            },
        },
    },
    "required": ["thinking_process", "failed", "objects"],
}


@dataclass(frozen=True)
class VlmConfig:
    base_url: str
    api_key: str
    model: str
    timeout_s: float = 120.0
    max_tokens: int = 1024
    temperature: float = 0.0
    use_json_schema: bool = True


@dataclass(frozen=True)
class VlmCupPoint:
    found: bool
    u: int
    v: int
    confidence: float
    note: str = ""
    raw_text: str = ""
    class_name: str = ""
    thinking_process: str = ""
    box_center_x: float = 0.0
    box_center_y: float = 0.0
    box_width: float = 0.0
    box_height: float = 0.0


def _env_first(*names: str) -> str:
    for name in names:
        value = os.environ.get(name, "").strip()
        if value:
            return value
    return ""


def _apply_proxy_env() -> None:
    proxy = _env_first("VLM_HTTPS_PROXY", "https_proxy", "HTTPS_PROXY", "http_proxy", "HTTP_PROXY")
    if proxy:
        os.environ.setdefault("http_proxy", proxy)
        os.environ.setdefault("https_proxy", proxy)


def _is_concrete_config_value(value: Any) -> bool:
    if value is None:
        return False
    text = str(value).strip()
    if not text:
        return False
    return not (text.startswith("${") and text.endswith("}"))


def _manifest_from_rbnx_boot_state() -> Path | None:
    candidates: list[Path] = []
    state_env = os.environ.get("RBNX_BOOT_STATE", "").strip()
    if state_env:
        candidates.append(Path(state_env))
    boot_dir = os.environ.get("RBNX_BOOT_DIR", "").strip()
    if boot_dir:
        candidates.append(Path(boot_dir) / "state.json")

    for start in (Path.cwd(), Path(__file__).resolve()):
        current = start
        for _ in range(10):
            candidates.append(current / "rbnx-boot" / "state.json")
            if current.parent == current:
                break
            current = current.parent

    try:
        candidates.extend(Path.home().glob("*/rbnx-boot/state.json"))
    except RuntimeError:
        pass

    seen: set[Path] = set()
    for state_path in candidates:
        resolved = state_path.resolve()
        if resolved in seen or not resolved.is_file():
            continue
        seen.add(resolved)
        try:
            data = json.loads(resolved.read_text(encoding="utf-8"))
            manifest = data.get("manifest_path")
            if not manifest:
                continue
            manifest_path = Path(str(manifest)).resolve()
            if manifest_path.is_file():
                log.debug("resolved deploy manifest from %s", resolved)
                return manifest_path
        except Exception as exc:  # noqa: BLE001
            log.debug("failed to read boot state %s: %s", resolved, exc)
    return None


def _manifest_candidates() -> list[Path]:
    candidates: list[Path] = []
    boot_manifest = _manifest_from_rbnx_boot_state()
    if boot_manifest is not None:
        candidates.append(boot_manifest)
    for env_name in ("ROBONIX_MANIFEST", "RBNX_MANIFEST", "RBNX_DEPLOY_MANIFEST"):
        value = os.environ.get(env_name, "").strip()
        if value:
            candidates.append(Path(value))
    deploy_root = os.environ.get("RBNX_DEPLOY_ROOT", "").strip()
    if deploy_root:
        candidates.append(Path(deploy_root) / "robonix_manifest.yaml")
    cwd = Path.cwd()
    candidates.append(cwd / "robonix_manifest.yaml")
    current = cwd
    for _ in range(6):
        candidates.append(current / "robonix_manifest.yaml")
        if current.parent == current:
            break
        current = current.parent
    seen: set[Path] = set()
    unique: list[Path] = []
    for path in candidates:
        resolved = path.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        unique.append(resolved)
    return unique


def load_pilot_vlm_from_manifest() -> dict[str, Any]:
    """Read ``system.pilot.vlm`` from the deployment robonix_manifest.yaml."""
    try:
        import yaml
    except ImportError:
        log.debug("PyYAML unavailable; skipping manifest pilot.vlm lookup")
        return {}

    for path in _manifest_candidates():
        if not path.is_file():
            continue
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001
            log.debug("failed to read manifest %s: %s", path, exc)
            continue
        if not isinstance(data, dict):
            continue
        pilot = (data.get("system") or {}).get("pilot") or {}
        vlm = pilot.get("vlm") if isinstance(pilot, dict) else None
        if not isinstance(vlm, dict) or not vlm:
            continue
        upstream = vlm.get("upstream", vlm.get("base_url"))
        api_key = vlm.get("api_key")
        model = vlm.get("model")
        if not (
            _is_concrete_config_value(upstream)
            and _is_concrete_config_value(api_key)
            and _is_concrete_config_value(model)
        ):
            log.debug("skipping placeholder pilot.vlm in %s", path)
            continue
        log.info("loaded pilot vlm config from %s", path)
        return dict(vlm)
    return {}


def _pick_str(source: dict[str, Any], *keys: str) -> str:
    for key in keys:
        value = source.get(key)
        if value is None:
            continue
        text = str(value).strip()
        if text:
            return text
    return ""


def _pick_bool(source: dict[str, Any], *keys: str, default: bool = True) -> bool:
    for key in keys:
        if key not in source:
            continue
        value = source[key]
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() not in {"0", "false", "no", "off"}
    return default


def _pick_float(source: dict[str, Any], *keys: str) -> float | None:
    for key in keys:
        if key not in source:
            continue
        try:
            return float(source[key])
        except (TypeError, ValueError):
            continue
    return None


def resolve_vlm_config(*, overrides: dict[str, Any] | None = None) -> VlmConfig:
    """Resolve VLM settings from env, manifest pilot.vlm, and explicit overrides."""
    _apply_proxy_env()
    merged: dict[str, Any] = {}
    merged.update(load_pilot_vlm_from_manifest())
    if overrides:
        merged.update({k: v for k, v in overrides.items() if v is not None})

    base = _pick_str(
        merged,
        "vlm_base_url",
        "base_url",
        "upstream",
    ) or _env_first(
        "VLM_BASE_URL",
        "LLM_BASE_URL",
        "PILOT_VLM_UPSTREAM",
        "VLM_UPSTREAM",
        "OPENAI_BASE_URL",
    )
    key = _pick_str(merged, "vlm_api_key", "api_key") or _env_first(
        "VLM_API_KEY",
        "LLM_API_KEY",
        "PILOT_VLM_API_KEY",
        "OPENAI_API_KEY",
    )
    model = _pick_str(merged, "vlm_model", "model") or _env_first(
        "VLM_MODEL",
        "LLM_MODEL",
        "PILOT_VLM_MODEL",
    )

    if not base:
        raise RuntimeError(
            "VLM base URL is not configured "
            "(set system.pilot.vlm.upstream in robonix_manifest.yaml, "
            "skill config vlm.upstream, detector_settings.vlm_base_url, "
            "or VLM_BASE_URL env)"
        )
    if not key:
        raise RuntimeError(
            "VLM api_key is not configured "
            "(set system.pilot.vlm.api_key in robonix_manifest.yaml, "
            "skill config vlm.api_key, detector_settings.vlm_api_key, "
            "or VLM_API_KEY env)"
        )
    if not model:
        raise RuntimeError(
            "VLM model is not configured "
            "(set system.pilot.vlm.model in robonix_manifest.yaml, "
            "skill config vlm.model, detector_settings.vlm_model, "
            "or VLM_MODEL env)"
        )

    timeout_s = _pick_float(merged, "vlm_timeout_s", "timeout_s")
    temperature = _pick_float(merged, "vlm_temperature", "temperature")
    if temperature is None:
        env_temp = _env_first("VLM_TEMPERATURE", "LLM_TEMPERATURE")
        temperature = float(env_temp) if env_temp else 0.0

    use_json_schema = _pick_bool(
        merged,
        "vlm_use_json_schema",
        "use_json_schema",
        default=_env_first("VLM_USE_JSON_SCHEMA", "LLM_USE_JSON_SCHEMA").lower()
        not in {"0", "false", "no", "off"},
    )

    return VlmConfig(
        base_url=base.rstrip("/"),
        api_key=key,
        model=model,
        timeout_s=timeout_s or VlmConfig.timeout_s,
        temperature=temperature,
        use_json_schema=use_json_schema,
    )


def vlm_config_from_env() -> VlmConfig:
    return resolve_vlm_config()


def encode_bgr_jpeg(bgr: np.ndarray, *, quality: int = 90) -> bytes:
    import cv2

    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("failed to encode RGB image as JPEG")
    return buf.tobytes()


def build_prompt(
    *,
    width: int,
    height: int,
    user_instruction: str | None = None,
    template: str | None = None,
) -> str:
    tpl = template or DEFAULT_PROMPT
    return tpl.format(
        user_instruction=user_instruction or DEFAULT_USER_INSTRUCTION,
        width=width,
        height=height,
    )


def extract_json_from_markdown(text: str) -> str:
    text = text.strip()
    match = re.search(r"```(?:json|JSON)?\s*(.*?)```", text, flags=re.S)
    if match:
        return match.group(1).strip()
    text = re.sub(r"```+\s*$", "", text).strip()
    if text.startswith("```"):
        first_newline = text.find("\n")
        if first_newline != -1:
            text = text[first_newline + 1 :].strip()
        text = re.sub(r"```+\s*$", "", text).strip()
    return text


def _has_detection_payload(raw: str) -> bool:
    try:
        parsed = json.loads(extract_json_from_markdown(raw))
    except json.JSONDecodeError:
        return False
    if not isinstance(parsed, dict):
        return False
    if parsed.get("failed") is True:
        return True
    if parsed.get("objects"):
        return True
    if "box_center_x" in parsed:
        return True
    if "found" in parsed or "u" in parsed:
        return True
    return False


def extract_completion_text(message: Any) -> str | None:
    candidates: list[str] = []
    for attr in ("content", "reasoning_content"):
        value = getattr(message, attr, None)
        if value and str(value).strip():
            candidates.append(str(value).strip())

    for candidate in candidates:
        if _has_detection_payload(candidate):
            return candidate
    return candidates[0] if candidates else None


def _extract_json(text: str) -> dict[str, Any]:
    text = extract_json_from_markdown(text.strip())
    if not text:
        raise ValueError("empty VLM response")
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object, got {type(data).__name__}")
    return data


def _norm_to_pixel(cx: float, cy: float, *, width: int, height: int) -> tuple[int, int]:
    """Map normalized 0-1 center to pixel (u, v)."""
    u = int(round(cx * (width - 1)))
    v = int(round(cy * (height - 1)))
    u = int(np.clip(u, 0, width - 1))
    v = int(np.clip(v, 0, height - 1))
    return u, v


def _looks_normalized(values: list[float]) -> bool:
    return all(0.0 <= v <= 1.0 for v in values)


def _parse_roboarm_format(data: dict[str, Any], *, width: int, height: int, raw_text: str) -> VlmCupPoint:
    thinking = str(data.get("thinking_process", ""))
    failed = bool(data.get("failed", True))
    objects = data.get("objects") or []

    if failed or not objects:
        return VlmCupPoint(
            found=False,
            u=0,
            v=0,
            confidence=0.0,
            note=thinking or "VLM failed to find target",
            raw_text=raw_text,
            thinking_process=thinking,
        )

    obj = objects[0]
    class_name = str(obj.get("class_name", "object"))
    cx = float(obj.get("box_center_x", 0.0))
    cy = float(obj.get("box_center_y", 0.0))
    bw = float(obj.get("box_width", 0.0))
    bh = float(obj.get("box_height", 0.0))

    coords = [cx, cy, bw, bh]
    if not _looks_normalized(coords):
        log.warning("VLM box values look non-normalized: %s", coords)

    u, v = _norm_to_pixel(cx, cy, width=width, height=height)
    note = f"{class_name}"
    if thinking:
        note = f"{class_name}: {thinking}"

    return VlmCupPoint(
        found=True,
        u=u,
        v=v,
        confidence=0.9,
        note=note,
        raw_text=raw_text,
        class_name=class_name,
        thinking_process=thinking,
        box_center_x=cx,
        box_center_y=cy,
        box_width=bw,
        box_height=bh,
    )


def _parse_legacy_pixel_format(data: dict[str, Any], *, width: int, height: int, raw_text: str) -> VlmCupPoint:
    found = bool(data.get("found", False))
    raw_u = int(round(float(data.get("u", 0))))
    raw_v = int(round(float(data.get("v", 0))))
    confidence = float(data.get("confidence", 0.0 if not found else 0.5))
    note = str(data.get("note", ""))

    if found and not (0 <= raw_u < width and 0 <= raw_v < height):
        log.warning(
            "legacy VLM pixel out of bounds (%d, %d) for %dx%d",
            raw_u,
            raw_v,
            width,
            height,
        )
        found = False
        confidence = 0.0
        note = f"invalid pixel ({raw_u},{raw_v}): {note}"
        raw_u, raw_v = 0, 0

    return VlmCupPoint(
        found=found,
        u=int(np.clip(raw_u, 0, width - 1)),
        v=int(np.clip(raw_v, 0, height - 1)),
        confidence=confidence,
        note=note,
        raw_text=raw_text,
    )


def parse_vlm_response(text: str, *, width: int, height: int) -> VlmCupPoint:
    data = _extract_json(text)
    if "objects" in data or "failed" in data:
        return _parse_roboarm_format(data, width=width, height=height, raw_text=text)
    return _parse_legacy_pixel_format(data, width=width, height=height, raw_text=text)


def _build_openai_client(cfg: VlmConfig):
    try:
        from openai import OpenAI
    except ImportError as exc:
        raise RuntimeError("openai package required: pip install openai") from exc

    return OpenAI(
        base_url=cfg.base_url,
        api_key=cfg.api_key,
        timeout=cfg.timeout_s,
        max_retries=1,
    )


def _response_format(cfg: VlmConfig) -> dict[str, Any]:
    if cfg.use_json_schema:
        return {
            "type": "json_schema",
            "json_schema": {
                "name": "detection_output",
                "schema": DETECTION_RESPONSE_SCHEMA,
            },
        }
    return {"type": "json_object"}


def query_cup_center(
    bgr: np.ndarray,
    cfg: VlmConfig,
    *,
    prompt: str | None = None,
    user_instruction: str | None = None,
) -> VlmCupPoint:
    h, w = bgr.shape[:2]
    prompt_text = prompt or build_prompt(width=w, height=h, user_instruction=user_instruction)
    jpeg = encode_bgr_jpeg(bgr)
    b64 = base64.standard_b64encode(jpeg).decode("ascii")

    client = _build_openai_client(cfg)
    log.info("VLM request -> %s model=%s image=%dx%d", cfg.base_url, cfg.model, w, h)

    try:
        completion = client.chat.completions.create(
            model=cfg.model,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt_text},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{b64}"},
                        },
                    ],
                }
            ],
            temperature=cfg.temperature,
            max_tokens=cfg.max_tokens,
            response_format=_response_format(cfg),
        )
    except Exception as exc:
        raise RuntimeError(f"VLM request failed: {exc}") from exc

    if not completion.choices:
        raise RuntimeError("VLM returned no choices")

    message = completion.choices[0].message
    text = extract_completion_text(message)
    if not text:
        raise RuntimeError("VLM returned empty message content")

    result = parse_vlm_response(text, width=w, height=h)
    log.info(
        "VLM cup point: found=%s u=%d v=%d norm=(%.3f,%.3f) class=%s",
        result.found,
        result.u,
        result.v,
        result.box_center_x,
        result.box_center_y,
        result.class_name,
    )
    return result
