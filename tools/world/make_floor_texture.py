"""강당 나무 마루 바닥 그림 생성 (광류 시험용).

방법
  1. 참고 사진(tools/world/ref/floor_ref_topview.png, 학교 체육관 거리뷰 캡처, 바닥을 위에서 본 화면)에서
     선과 로고가 없는 깨끗한 영역만 골라, 판자 1개 폭씩 세로 띠로 잘라 낸다.
  2. 띠를 눕혀(판자 방향 = 강당 긴 변 x 방향) 판자 폭 57 mm(추정)로 크기를 맞추고,
     줄마다 무작위로 이어 붙여 30 × 20 m 마루를 만든다. 이음매는 실제처럼 조금 어둡게 한다.
  3. 코트 선을 표준 규격으로 강당 가운데에 그린다 (위치는 가정).
     파랑 = 농구 (FIBA, 선 폭 50 mm), 빨강 = 배드민턴 3면 (선 폭 40 mm), 흰색 = 배구 (선 폭 50 mm)
     색은 참고 사진에서 뽑은 값.

주의
  - 사진과 "똑같은" 바닥이 아니라, 판자 폭·색·나뭇결·이음매가 비슷한 바닥이다.
  - 참고 사진은 인터넷(학교 홈페이지) 자료라 시험용으로만 쓴다. 발표·논문 화면은 직접 찍은 사진으로 교체.

사용: python3 tools/world/make_floor_texture.py   (numpy, Pillow 필요. seed 고정 → 매번 같은 그림)
결과: models/auditorium_floor/materials/textures/floor_wood.jpg
"""
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # tools/ (auditorium_config.py 위치)
import auditorium_config as C  # noqa: E402

REPO = Path(__file__).resolve().parents[2]                       # 저장소 맨 위
REF = REPO / "tools/world/ref/floor_ref_topview.png"
OUT = REPO / "models/auditorium_floor/materials/textures" / C.FLOOR_TEXTURE

# 참고 사진에서 선·로고·글자가 없는 영역 (x0, x1, y0, y1) 픽셀
CLEAN_REGIONS = [(100, 900, 130, 540), (1660, 2459, 130, 550),
                 (100, 900, 660, 990), (1660, 2459, 660, 1000)]

# 코트 선 색 (참고 사진에서 측정, RGB)
RED = (162, 77, 75)
BLUE = (20, 80, 163)
WHITE = (213, 216, 216)

# 사진은 판자 길이가 짧게 찍혀 있어(거리뷰 왜곡, 영역 제한) 길이 방향으로 늘린다 (추정값)
LENGTH_STRETCH = 1.8
# 사진 3·4번(실내 조명)의 따뜻한 색에 가깝게 보정 (R, G, B 배율)
WARM = np.array([1.06, 0.98, 0.88], np.float32)

rng = np.random.default_rng(1031)


# ── 1. 참고 사진에서 판자 띠 뽑기 ──────────────────────────────
def extract_planks(img):
    """깨끗한 영역에서 판자 사이 이음매(어두운 세로줄)를 찾아 판자 1개씩 잘라 낸다."""
    strips = []
    lum = img.mean(axis=2)
    for x0, x1, y0, y1 in CLEAN_REGIONS:
        col = np.convolve(lum[y0:y1, x0:x1].mean(axis=0), np.ones(5) / 5, "same")
        seams = [i for i in range(3, len(col) - 3)
                 if col[i] == col[i - 3:i + 4].min() and col[i] < np.percentile(col, 40)]
        for a, b in zip(seams, seams[1:]):
            if 38 <= b - a <= 62:                      # 판자 1개 폭(약 49 px)만 사용
                strips.append(img[y0:y1, x0 + a:x0 + b])
    return strips


def normalize(strips):
    """사진 조명 차이를 없애고, 판자마다 밝기를 조금씩 다르게 한다."""
    target = np.mean([s.mean() for s in strips])
    out = []
    for s in strips:
        k = target / s.mean() * rng.normal(1.0, 0.06)   # 판자별 밝기 차이 ±6 %
        out.append(np.clip(s * k * WARM, 0, 255))
    return out


# ── 2. 마루 깔기 ──────────────────────────────────────────────
def build_floor(strips, w_px, h_px, ppm):
    canvas = np.zeros((h_px, w_px, 3), np.float32)
    row_h = C.PLANK_W * ppm                              # 판자 1줄 높이 px
    n_rows = math.ceil(h_px / row_h)
    for r in range(n_rows):
        y0, y1 = round(r * row_h), min(round((r + 1) * row_h), h_px)
        h = y1 - y0
        if h <= 0:
            continue
        x = -int(rng.integers(0, 200))                   # 줄마다 이음매 위치를 엇갈리게
        while x < w_px:
            s = strips[rng.integers(len(strips))]
            seg = np.transpose(s, (1, 0, 2))             # 눕히기: 판자 길이 방향 → x
            scale = h / seg.shape[0]
            full = max(int(seg.shape[1] * scale * LENGTH_STRETCH), 8)
            length = int(full * rng.uniform(0.6, 1.0))   # 판자 길이 무작위
            piece = np.asarray(Image.fromarray(seg.astype(np.uint8))
                               .resize((full, h), Image.BILINEAR), np.float32)[:, :length]
            piece[:, -1] *= 0.75                         # 끝 이음매
            xa, xb = max(x, 0), min(x + length, w_px)
            if xb > xa:
                canvas[y0:y1, xa:xb] = piece[:, xa - x:xb - x]
            x += length
        canvas[y0, :] *= 0.85                            # 판자 사이 긴 이음매
    return canvas


# ── 3. 코트 선 (강당 좌표 m로 그림) ─────────────────────────────
class Painter:
    def __init__(self, img, ppm):
        self.d = ImageDraw.Draw(img)
        self.ppm = ppm

    def p(self, x, y):                                   # 강당 좌표 → 그림 픽셀 (위쪽이 +y)
        return (x * self.ppm, (C.HALL_D - y) * self.ppm)

    def line(self, x0, y0, x1, y1, color, w):
        self.d.line([self.p(x0, y0), self.p(x1, y1)], fill=color, width=max(1, round(w * self.ppm)))

    def rect(self, x0, y0, x1, y1, color, w):
        for a, b in [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]:
            self.line(*a, *b, color, w)

    def arc(self, cx, cy, r, a0, a1, color, w):
        """각도는 강당 좌표 기준 (도, x축에서 반시계)."""
        (x0, y0), (x1, y1) = self.p(cx - r, cy + r), self.p(cx + r, cy - r)
        self.d.arc([x0, y0, x1, y1], start=-a1, end=-a0, fill=color, width=max(1, round(w * self.ppm)))


def draw_courts(img, ppm):
    pt = Painter(img, ppm)
    cx, cy = C.HALL_W / 2, C.HALL_D / 2

    # 농구 (FIBA): 28 × 15 m, 선 폭 50 mm
    w, L, W = 0.05, 28.0, 15.0
    x0, x1, y0, y1 = cx - L / 2, cx + L / 2, cy - W / 2, cy + W / 2
    pt.rect(x0, y0, x1, y1, BLUE, w)
    pt.line(cx, y0, cx, y1, BLUE, w)                     # 중앙선
    pt.arc(cx, cy, 1.8, 0, 360, BLUE, w)                 # 중앙 원
    for side in (-1, 1):
        end = cx + side * L / 2
        bx = end - side * 1.575                          # 골대 중심
        # 제한구역 5.8 × 4.9 m, 자유투 원 r 1.8 m
        ft = end - side * 5.8
        pt.rect(min(end, ft), cy - 2.45, max(end, ft), cy + 2.45, BLUE, w)
        pt.arc(ft, cy, 1.8, 0, 360, BLUE, w)
        # 3점 선: 반지름 6.75 m, 사이드라인에서 0.9 m 직선
        yy = W / 2 - 0.9
        dx = math.sqrt(6.75 ** 2 - yy ** 2)
        for s in (-1, 1):
            pt.line(end, cy + s * yy, bx - side * dx, cy + s * yy, BLUE, w)
        a = math.degrees(math.atan2(yy, dx))
        if side < 0:
            pt.arc(bx, cy, 6.75, -a, a, BLUE, w)
        else:
            pt.arc(bx, cy, 6.75, 180 - a, 180 + a, BLUE, w)

    # 배드민턴 3면: 6.1 × 13.4 m, 선 폭 40 mm, 긴 변이 y 방향, 코트 사이 2 m
    w = 0.04
    for k in (-1, 0, 1):
        bx = cx + k * 8.1
        xa, xb, ya, yb = bx - 3.05, bx + 3.05, cy - 6.7, cy + 6.7
        pt.rect(xa, ya, xb, yb, RED, w)
        for s in (-1, 1):
            pt.line(bx + s * 2.59, ya, bx + s * 2.59, yb, RED, w)            # 단식 사이드라인
            pt.line(xa, cy + s * 1.98, xb, cy + s * 1.98, RED, w)            # 숏 서비스 라인
            pt.line(xa, cy + s * 5.94, xb, cy + s * 5.94, RED, w)            # 복식 롱 서비스 라인
            pt.line(bx, cy + s * 1.98, bx, cy + s * 6.7, RED, w)             # 센터 라인

    # 배구: 18 × 9 m, 선 폭 50 mm
    w = 0.05
    pt.rect(cx - 9, cy - 4.5, cx + 9, cy + 4.5, WHITE, w)
    for xx in (cx, cx - 3, cx + 3):
        pt.line(xx, cy - 4.5, xx, cy + 4.5, WHITE, w)


def main():
    ppm = C.PX_PER_M
    w_px, h_px = round(C.HALL_W * ppm), round(C.HALL_D * ppm)
    ref = np.asarray(Image.open(REF).convert("RGB"), np.float32)
    strips = normalize(extract_planks(ref))
    print(f"판자 띠 {len(strips)}개 추출")
    floor = build_floor(strips, w_px, h_px, ppm)
    img = Image.fromarray(np.clip(floor, 0, 255).astype(np.uint8))
    lines = img.copy()
    draw_courts(lines, ppm)
    img = Image.blend(img, lines, 0.85)                 # 선을 살짝 투명하게 (나뭇결이 비침)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    img.save(OUT, quality=90)
    print("저장:", OUT, img.size, f"(1 px = {1000 / ppm:.1f} mm)")


if __name__ == "__main__":
    main()
