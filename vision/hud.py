"""
Pure-OpenCV HUD rendering for the hand-tracker debug window: translucent panels,
per-finger flexion gauges, serial status pill, and finger-colored skeleton drawing.

Colors are BGR (OpenCV convention). Palette is a dark tech/HUD scheme:
slate background panels, near-white text, a distinct hue per finger, and
green/red/amber status accents.
"""

import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_DUPLEX

# -- palette (BGR) --------------------------------------------------------
PANEL_BG = (59, 41, 30)        # slate #1E293B
PANEL_BG_DEEP = (42, 23, 15)   # slate #0F172A
TEXT = (252, 250, 248)         # near-white #F8FAFC
TEXT_MUTED = (148, 128, 111)   # muted slate-ish
ACCENT_GREEN = (94, 197, 34)   # #22C55E
ACCENT_RED = (68, 68, 239)     # #EF4444
ACCENT_AMBER = (11, 158, 245)  # #F59E0B
TRACK_BG = (85, 65, 51)        # #334155, gauge track background

FINGER_COLORS = {
    "thumb": (248, 189, 56),   # sky #38BDF8
    "index": (94, 197, 34),    # green #22C55E
    "middle": (21, 204, 250),  # yellow #FACC15
    "ring": (182, 114, 244),   # pink #F472B6
    "pinky": (250, 139, 167),  # violet #A78BFA
}
PALM_COLOR = (110, 90, 75)  # muted slate for non-finger (palm) connections
WRIST_COLOR = TEXT

FINGER_LABELS = {"thumb": "T", "index": "I", "middle": "M", "ring": "R", "pinky": "P"}

_LANDMARK_FINGER = {}
for _i in (1, 2, 3, 4):
    _LANDMARK_FINGER[_i] = "thumb"
for _i in (5, 6, 7, 8):
    _LANDMARK_FINGER[_i] = "index"
for _i in (9, 10, 11, 12):
    _LANDMARK_FINGER[_i] = "middle"
for _i in (13, 14, 15, 16):
    _LANDMARK_FINGER[_i] = "ring"
for _i in (17, 18, 19, 20):
    _LANDMARK_FINGER[_i] = "pinky"
_FINGERTIPS = {4, 8, 12, 16, 20}


def _blend_rect(frame, x, y, w, h, color, alpha):
    x2, y2 = x + w, y + h
    roi = frame[y:y2, x:x2]
    overlay = np.full_like(roi, color, dtype=np.uint8)
    cv2.addWeighted(overlay, alpha, roi, 1 - alpha, 0, dst=roi)


def draw_skeleton(frame, points, connections, label=None):
    """points: list of (x, y) pixel coords indexed by landmark id (0-20).
    label: "L"/"R" tag drawn near the wrist point, for dual-hand tracking."""
    for conn in connections:
        fa = _LANDMARK_FINGER.get(conn.start)
        fb = _LANDMARK_FINGER.get(conn.end)
        color = FINGER_COLORS[fa] if fa and fa == fb else PALM_COLOR
        cv2.line(frame, points[conn.start], points[conn.end], color, 2, cv2.LINE_AA)

    for idx, (x, y) in enumerate(points):
        if idx == 0:
            cv2.circle(frame, (x, y), 7, WRIST_COLOR, -1, cv2.LINE_AA)
            cv2.circle(frame, (x, y), 7, PANEL_BG_DEEP, 1, cv2.LINE_AA)
            if label:
                cv2.putText(frame, label, (x + 12, y + 5), FONT, 0.55, TEXT, 1, cv2.LINE_AA)
            continue
        color = FINGER_COLORS[_LANDMARK_FINGER[idx]]
        radius = 6 if idx in _FINGERTIPS else 3
        cv2.circle(frame, (x, y), radius, color, -1, cv2.LINE_AA)
        cv2.circle(frame, (x, y), radius, PANEL_BG_DEEP, 1, cv2.LINE_AA)


def draw_header(frame, fps, title="ROBOTIC HAND CONTROL"):
    w = frame.shape[1]
    _blend_rect(frame, 0, 0, w, 44, PANEL_BG, 0.55)
    cv2.putText(frame, title, (16, 29), FONT, 0.6, TEXT, 1, cv2.LINE_AA)

    fps_color = ACCENT_GREEN if fps >= 20 else ACCENT_AMBER if fps >= 10 else ACCENT_RED
    label = f"FPS {int(round(fps))}"
    (tw, _), _ = cv2.getTextSize(label, FONT, 0.6, 1)
    dot_x = w - tw - 34
    cv2.circle(frame, (dot_x, 22), 5, fps_color, -1, cv2.LINE_AA)
    cv2.putText(frame, label, (dot_x + 14, 29), FONT, 0.6, TEXT, 1, cv2.LINE_AA)


def draw_serial_status(frame, state):
    """state: 'connected' | 'disconnected' | 'disabled'."""
    label, color = {
        "connected": ("SERIAL LINKED", ACCENT_GREEN),
        "disconnected": ("SERIAL LOST", ACCENT_RED),
        "disabled": ("NO SERIAL", TEXT_MUTED),
    }[state]

    pad_x, pad_y = 12, 8
    (tw, th), _ = cv2.getTextSize(label, FONT, 0.5, 1)
    w = frame.shape[1]
    box_w = tw + 2 * pad_x + 16
    box_h = th + 2 * pad_y
    x, y = w - box_w - 12, 54
    _blend_rect(frame, x, y, box_w, box_h, PANEL_BG, 0.55)
    cv2.circle(frame, (x + pad_x + 4, y + box_h // 2), 4, color, -1, cv2.LINE_AA)
    cv2.putText(frame, label, (x + pad_x + 16, y + box_h - pad_y + 1),
                FONT, 0.5, TEXT, 1, cv2.LINE_AA)


def draw_gazebo_status(frame, state):
    """state: 'connected' | 'disconnected' | 'disabled'."""
    label, color = {
        "connected": ("GAZEBO LINKED", ACCENT_GREEN),
        "disconnected": ("GAZEBO LOST", ACCENT_RED),
        "disabled": ("NO GAZEBO", TEXT_MUTED),
    }[state]

    pad_x, pad_y = 12, 8
    (tw, th), _ = cv2.getTextSize(label, FONT, 0.5, 1)
    w = frame.shape[1]
    box_w = tw + 2 * pad_x + 16
    box_h = th + 2 * pad_y
    x, y = w - box_w - 12, 54 + box_h + 6
    _blend_rect(frame, x, y, box_w, box_h, PANEL_BG, 0.55)
    cv2.circle(frame, (x + pad_x + 4, y + box_h // 2), 4, color, -1, cv2.LINE_AA)
    cv2.putText(frame, label, (x + pad_x + 16, y + box_h - pad_y + 1),
                FONT, 0.5, TEXT, 1, cv2.LINE_AA)


def _draw_gauge_row(frame, y0, row_h, hand_label, curls, finger_order, finger_count, tracked):
    """One hand's worth of finger bars, e.g. the top or bottom half of the dual-hand panel."""
    w = frame.shape[1]
    margin = 24
    gap = 14
    n = len(finger_order)
    bar_w = (w - 2 * margin - (n - 1) * gap - 160) // n
    bar_top = y0 + 10
    bar_h = row_h - 34

    tag_color = TEXT if tracked else TEXT_MUTED
    cv2.putText(frame, hand_label, (margin - 18, bar_top + bar_h // 2 + 6),
                FONT, 0.6, tag_color, 1, cv2.LINE_AA)

    for i, name in enumerate(finger_order):
        x = margin + i * (bar_w + gap)
        color = FINGER_COLORS[name] if tracked else TRACK_BG
        cv2.rectangle(frame, (x, bar_top), (x + bar_w, bar_top + bar_h), TRACK_BG, -1)
        t = 0.0 if not tracked else float(np.clip(curls.get(name, 0.0), 0.0, 1.0))
        fill_w = int(bar_w * t)
        if fill_w > 0:
            cv2.rectangle(frame, (x, bar_top), (x + fill_w, bar_top + bar_h), color, -1)
        cv2.rectangle(frame, (x, bar_top), (x + bar_w, bar_top + bar_h), PANEL_BG_DEEP, 1)
        label = FINGER_LABELS[name]
        (tw, _), _ = cv2.getTextSize(label, FONT, 0.45, 1)
        cv2.putText(frame, label, (x + bar_w // 2 - tw // 2, bar_top + bar_h + 15),
                    FONT, 0.45, TEXT_MUTED, 1, cv2.LINE_AA)

    chip_x = w - margin - 110
    chip_label = f"{finger_count}/5" if tracked else "NOT TRACKED"
    cv2.putText(frame, chip_label, (chip_x, bar_top + bar_h // 2 + 6),
                FONT, 0.55, TEXT if tracked else TEXT_MUTED, 1, cv2.LINE_AA)


def draw_finger_gauges(frame, curls_by_hand, finger_order, counts_by_hand, visible_labels=()):
    """curls_by_hand / counts_by_hand: {"L": {...}/int, "R": {...}/int} - may hold a
    hand's last-known values even after it leaves the frame (see hand_tracker.py).
    visible_labels: which hands are actually in view *this* frame - drives the
    dimmed "NOT TRACKED" state, independent of whether curls_by_hand still has
    stale data for a hand that left."""
    h, w = frame.shape[:2]
    row_h = 68
    panel_h = row_h * 2
    y0 = h - panel_h
    _blend_rect(frame, 0, y0, w, panel_h, PANEL_BG, 0.55)
    cv2.line(frame, (0, y0 + row_h), (w, y0 + row_h), PANEL_BG_DEEP, 1)

    for i, label in enumerate(("L", "R")):
        _draw_gauge_row(frame, y0 + i * row_h, row_h, label,
                         curls_by_hand.get(label), finger_order, counts_by_hand.get(label, 0),
                         tracked=label in visible_labels)
