#!/usr/bin/env python3
from pathlib import Path

import numpy as np
from PIL import Image

TEAM_COLORS: dict[int, tuple[str, tuple[int, int, int]]] = {
    1: ("red", (210, 25, 25)),
    2: ("blue", (25, 85, 215)),
    3: ("green", (25, 165, 55)),
    4: ("yellow", (240, 200, 15)),
    5: ("orange", (245, 115, 15)),
    6: ("purple", (135, 45, 185)),
}

TEXTURE_DIR = Path(__file__).resolve().parent.parent / "models" / "ztr_car" / "materials" / "textures"
HIGHLIGHT = 0.35


def recolor(source: np.ndarray, rgb: tuple[int, int, int]) -> np.ndarray:
    lum = source[..., :3].astype(np.float32).mean(axis=2, keepdims=True) / 255.0
    color = np.array(rgb, dtype=np.float32) / 255.0
    painted = lum * color
    painted += (lum ** 6) * (1.0 - painted) * HIGHLIGHT
    out = source.astype(np.float32).copy()
    out[..., :3] = np.clip(painted * 255.0, 0, 255)
    return out.astype(np.uint8)


def main() -> None:
    source = np.array(Image.open(TEXTURE_DIR / "Hybrid.png"))
    for team, (name, rgb) in TEAM_COLORS.items():
        path = TEXTURE_DIR / f"Hybrid_{name}.png"
        Image.fromarray(recolor(source, rgb)).save(path, optimize=True)
        print(f"team {team}: {path.name}")


if __name__ == "__main__":
    main()
