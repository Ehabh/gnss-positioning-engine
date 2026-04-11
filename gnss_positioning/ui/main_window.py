"""
GNSS Positioning Engine — u-center 2 inspired PyQt UI.

Design inspired by u-blox u-center 2:
  · Toolbar: connection controls, mode buttons, recording toggle
  · Left dock: fix badge, position, quality, reference, ephemeris, logging
  · Right dock: satellite table (PRN, system, CNR, used status)
  · Bottom dock: message console
  · Central tabs: Map | Signal Bars | Scatter | DOP/σ
"""
from __future__ import annotations

import sys
import os
import math
import logging
import datetime
import time as _time
import numpy as np
from collections import deque
from typing import Optional, Dict, List

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Qt binding: PySide6 → PyQt6 → PyQt5
# ---------------------------------------------------------------------------
HAS_PYQT = False
QT_VERSION = 0

try:
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QGridLayout, QLabel, QPushButton, QComboBox, QGroupBox,
        QTabWidget, QFrame, QSizePolicy, QFileDialog,
        QTextEdit, QCheckBox, QScrollArea, QLineEdit, QMessageBox,
        QDockWidget, QToolBar, QTableWidget, QTableWidgetItem,
        QHeaderView, QAbstractItemView,
    )
    from PySide6.QtCore import Qt, QTimer, Signal as pyqtSignal, QPointF
    from PySide6.QtGui import (
        QPainter, QColor, QFont, QPen, QBrush,
        QLinearGradient, QPainterPath, QAction, QIcon,
    )
    HAS_PYQT = True
    QT_VERSION = 6
except ImportError:
    pass

if not HAS_PYQT:
    try:
        from PyQt6.QtWidgets import (
            QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
            QGridLayout, QLabel, QPushButton, QComboBox, QGroupBox,
            QTabWidget, QFrame, QSizePolicy, QFileDialog,
            QTextEdit, QCheckBox, QScrollArea, QLineEdit, QMessageBox,
            QDockWidget, QToolBar, QTableWidget, QTableWidgetItem,
            QHeaderView, QAbstractItemView,
        )
        from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QPointF
        from PyQt6.QtGui import (
            QPainter, QColor, QFont, QPen, QBrush,
            QLinearGradient, QPainterPath, QAction, QIcon,
        )
        HAS_PYQT = True
        QT_VERSION = 6
    except ImportError:
        pass

if not HAS_PYQT:
    try:
        from PyQt5.QtWidgets import (
            QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
            QGridLayout, QLabel, QPushButton, QComboBox, QGroupBox,
            QTabWidget, QFrame, QSizePolicy, QFileDialog,
            QTextEdit, QCheckBox, QScrollArea, QLineEdit, QMessageBox,
            QDockWidget, QToolBar, QTableWidget, QTableWidgetItem,
            QHeaderView, QAbstractItemView, QAction,
        )
        from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPointF
        from PyQt5.QtGui import (
            QPainter, QColor, QFont, QPen, QBrush,
            QLinearGradient, QPainterPath, QIcon,
        )
        HAS_PYQT = True
        QT_VERSION = 5
    except ImportError:
        logger.error("No Qt binding found. Install: pip install PySide6")

if not HAS_PYQT:
    class QWidget:
        def __init__(self, *a, **kw): pass
    class QMainWindow:
        def __init__(self, *a, **kw): pass
    class Qt:
        Horizontal = 0
    class QPointF:
        def __init__(self, *a): pass
    class pyqtSignal:
        def __init__(self, *a): pass

# WebEngine (optional — live map)
HAS_WEBENGINE = False
for _pkg in ('PySide6', 'PyQt6', 'PyQt5'):
    try:
        _mod = __import__(f'{_pkg}.QtWebEngineWidgets', fromlist=['QWebEngineView'])
        QWebEngineView = _mod.QWebEngineView
        HAS_WEBENGINE = True
        break
    except ImportError:
        pass

from ..core.pipeline import GNSSPipeline, PositioningMode
from ..core.serial_handler import SerialHandler
from ..core.data_types import PositionSolution, EpochObservations, FixType, Constellation
from ..utils.coordinates import ecef_to_lla, ecef_to_enu, lla_to_ecef

# ---------------------------------------------------------------------------
# Qt enum compatibility shims
# ---------------------------------------------------------------------------
if HAS_PYQT and QT_VERSION == 6:
    _Qt_H         = Qt.Orientation.Horizontal
    _Qt_V         = Qt.Orientation.Vertical
    _Qt_NoPen     = Qt.PenStyle.NoPen
    _Qt_Solid     = Qt.PenStyle.SolidLine
    _Qt_Dash      = Qt.PenStyle.DashLine
    _AA           = QPainter.RenderHint.Antialiasing
    _Bold         = QFont.Weight.Bold
    _HLine        = QFrame.Shape.HLine
    _AC           = Qt.AlignmentFlag.AlignCenter
    _AR           = Qt.AlignmentFlag.AlignRight
    _AVC          = Qt.AlignmentFlag.AlignVCenter
    _LD           = Qt.DockWidgetArea.LeftDockWidgetArea
    _RD           = Qt.DockWidgetArea.RightDockWidgetArea
    _BD           = Qt.DockWidgetArea.BottomDockWidgetArea
    _HV_S         = QHeaderView.ResizeMode.Stretch
    _HV_F         = QHeaderView.ResizeMode.Fixed
    _HV_R         = QHeaderView.ResizeMode.ResizeToContents
    _SEL_ROWS     = QAbstractItemView.SelectionBehavior.SelectRows
    _NO_EDIT      = QAbstractItemView.EditTrigger.NoEditTriggers
    _IE           = Qt.ItemFlag.ItemIsEnabled
    _IS           = Qt.ItemFlag.ItemIsSelectable
elif HAS_PYQT:
    _Qt_H         = Qt.Horizontal
    _Qt_V         = Qt.Vertical
    _Qt_NoPen     = Qt.NoPen
    _Qt_Solid     = Qt.SolidLine
    _Qt_Dash      = Qt.DashLine
    _AA           = QPainter.Antialiasing
    _Bold         = QFont.Bold
    _HLine        = QFrame.HLine
    _AC           = Qt.AlignCenter
    _AR           = Qt.AlignRight
    _AVC          = Qt.AlignVCenter
    _LD           = Qt.LeftDockWidgetArea
    _RD           = Qt.RightDockWidgetArea
    _BD           = Qt.BottomDockWidgetArea
    _HV_S         = QHeaderView.Stretch
    _HV_F         = QHeaderView.Fixed
    _HV_R         = QHeaderView.ResizeToContents
    _SEL_ROWS     = QAbstractItemView.SelectRows
    _NO_EDIT      = QAbstractItemView.NoEditTriggers
    _IE           = Qt.ItemIsEnabled
    _IS           = Qt.ItemIsSelectable
else:
    (_Qt_H, _Qt_V, _Qt_NoPen, _Qt_Solid, _Qt_Dash, _AA, _Bold, _HLine,
     _AC, _AR, _AVC, _LD, _RD, _BD, _HV_S, _HV_F, _HV_R,
     _SEL_ROWS, _NO_EDIT, _IE, _IS) = (0,) * 21

# ---------------------------------------------------------------------------
# Colour palette
# ---------------------------------------------------------------------------
C = {
    'bg':       '#0D1117',
    'panel':    '#161B22',
    'card':     '#1C2128',
    'border':   '#30363D',
    'text':     '#E6EDF3',
    'dim':      '#8B949E',
    'blue':     '#58A6FF',
    'green':    '#3FB950',
    'orange':   '#D29922',
    'red':      '#F85149',
    'purple':   '#BC8CFF',
    'gps':      '#58A6FF',
    'glo':      '#F85149',
    'gal':      '#3FB950',
    'bds':      '#F0A500',
}

CONST_COLOR = {
    Constellation.GPS:     C['gps'],
    Constellation.GLONASS: C['glo'],
    Constellation.GALILEO: C['gal'],
    Constellation.BEIDOU:  C['bds'],
}

CONST_NAME = {
    Constellation.GPS:     'GPS',
    Constellation.GLONASS: 'GLONASS',
    Constellation.GALILEO: 'Galileo',
    Constellation.BEIDOU:  'BeiDou',
}

FIX_INFO = {
    FixType.NO_FIX:    ('NO FIX',    C['red'],    C['text']),
    FixType.SPS:       ('SPS',       C['blue'],   C['bg']),
    FixType.DGNSS:     ('DGNSS',     C['orange'], C['bg']),
    FixType.RTK_FLOAT: ('RTK FLOAT', C['purple'], C['bg']),
    FixType.RTK_FIXED: ('RTK FIXED', C['green'],  C['bg']),
}

# ---------------------------------------------------------------------------
# Stylesheet
# ---------------------------------------------------------------------------
SS = f"""
QMainWindow, QWidget {{
    background-color: {C['bg']};
    color: {C['text']};
    font-family: 'SF Mono', 'Menlo', 'Fira Code', 'Consolas', monospace;
    font-size: 12px;
}}
QGroupBox {{
    background-color: {C['panel']};
    border: 1px solid {C['border']};
    border-radius: 6px;
    margin-top: 14px;
    padding: 12px 10px 10px 10px;
    font-weight: bold;
    font-size: 12px;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
    color: {C['blue']};
}}
QPushButton {{
    background-color: {C['card']};
    color: {C['text']};
    border: 1px solid {C['border']};
    border-radius: 5px;
    padding: 5px 12px;
    font-weight: bold;
    min-height: 26px;
}}
QPushButton:hover {{
    background-color: {C['border']};
    border-color: {C['blue']};
}}
QPushButton:pressed {{
    background-color: {C['blue']};
    color: {C['bg']};
}}
QPushButton:checked {{
    background-color: {C['red']};
    color: {C['text']};
    border-color: {C['red']};
}}
QPushButton:disabled {{
    color: {C['dim']};
    border-color: {C['card']};
}}
QPushButton#primary {{
    background-color: {C['blue']};
    color: {C['bg']};
    border: none;
}}
QPushButton#primary:hover {{
    background-color: #79C0FF;
}}
QPushButton#mode_active {{
    background-color: {C['blue']};
    color: {C['bg']};
    border: none;
}}
QComboBox {{
    background-color: {C['card']};
    border: 1px solid {C['border']};
    border-radius: 5px;
    padding: 4px 10px;
    min-height: 26px;
    min-width: 80px;
}}
QComboBox::drop-down {{ border: none; width: 20px; }}
QComboBox QAbstractItemView {{
    background-color: {C['panel']};
    border: 1px solid {C['border']};
    selection-background-color: {C['blue']};
}}
QLabel {{ background-color: transparent; }}
QTabWidget::pane {{
    background-color: {C['panel']};
    border: 1px solid {C['border']};
}}
QTabBar::tab {{
    background-color: {C['card']};
    color: {C['dim']};
    padding: 6px 16px;
    border: 1px solid {C['border']};
    border-bottom: none;
    border-top-left-radius: 5px;
    border-top-right-radius: 5px;
    margin-right: 2px;
}}
QTabBar::tab:selected {{
    background-color: {C['panel']};
    color: {C['blue']};
    border-bottom: 2px solid {C['blue']};
}}
QStatusBar {{
    background-color: {C['panel']};
    color: {C['dim']};
    border-top: 1px solid {C['border']};
    font-size: 11px;
}}
QStatusBar::item {{ border: none; }}
QToolBar {{
    background-color: {C['panel']};
    border: none;
    border-bottom: 1px solid {C['border']};
    padding: 4px 6px;
    spacing: 3px;
}}
QToolBar QLabel {{
    color: {C['dim']};
    font-size: 11px;
    padding: 0 2px;
}}
QDockWidget {{
    titlebar-close-icon: none;
}}
QDockWidget::title {{
    background-color: {C['card']};
    text-align: left;
    padding: 5px 10px;
    border-bottom: 1px solid {C['border']};
    font-weight: bold;
    color: {C['dim']};
    font-size: 11px;
}}
QScrollArea {{ border: none; }}
QScrollBar:vertical {{
    background: {C['bg']};
    width: 8px;
}}
QScrollBar::handle:vertical {{
    background: {C['border']};
    border-radius: 4px;
    min-height: 20px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QLineEdit {{
    background-color: {C['card']};
    border: 1px solid {C['border']};
    border-radius: 4px;
    padding: 4px 8px;
    color: {C['text']};
}}
QTextEdit {{
    background-color: {C['card']};
    border: none;
    color: {C['text']};
    font-size: 11px;
}}
QTableWidget {{
    background-color: {C['card']};
    border: none;
    gridline-color: {C['border']};
    selection-background-color: #1F3A5F;
    alternate-background-color: {C['panel']};
}}
QTableWidget::item {{ padding: 3px 6px; }}
QHeaderView::section {{
    background-color: {C['bg']};
    border: none;
    border-right: 1px solid {C['border']};
    border-bottom: 1px solid {C['border']};
    padding: 4px 8px;
    color: {C['dim']};
    font-weight: bold;
    font-size: 11px;
}}
QCheckBox {{ color: {C['dim']}; }}
QCheckBox::indicator {{
    width: 14px;
    height: 14px;
    border: 1px solid {C['border']};
    border-radius: 3px;
    background-color: {C['card']};
}}
QCheckBox::indicator:checked {{
    background-color: {C['blue']};
}}
"""


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------
def _tbl_item(text: str, editable: bool = False) -> 'QTableWidgetItem':
    item = QTableWidgetItem(str(text))
    flags = _IE | _IS
    if not editable:
        pass  # flags already non-editable by default (no ItemIsEditable)
    item.setFlags(flags)
    return item


_ASSETS = os.path.join(os.path.dirname(__file__), 'assets')


def _icon(name: str) -> 'QIcon':
    """Load an icon from the assets directory.

    Tries SVG first (sharp at any DPI), falls back to PNG, returns an empty
    QIcon if neither is found (so the button still works without an icon).
    """
    for ext in ('svg', 'png'):
        path = os.path.join(_ASSETS, f'{name}.{ext}')
        if os.path.exists(path):
            return QIcon(path)
    return QIcon()


def _sep(toolbar: 'QToolBar'):
    """Add a visual separator label to the toolbar."""
    lbl = QLabel("│")
    lbl.setStyleSheet(f"color: {C['border']}; font-size: 16px; padding: 0 4px;")
    toolbar.addWidget(lbl)


# ---------------------------------------------------------------------------
# FixBadgeWidget — prominent fix-type display
# ---------------------------------------------------------------------------
class FixBadgeWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self._badge = QLabel("NO FIX")
        self._badge.setAlignment(_AC)
        self._badge.setMinimumHeight(52)
        self._set_badge_style(C['red'], C['text'])

        self._sats = QLabel("—  satellites used")
        self._sats.setAlignment(_AC)
        self._sats.setStyleSheet(f"color:{C['dim']}; font-size:11px;")

        self._tow = QLabel("GPS TOW: —")
        self._tow.setAlignment(_AC)
        self._tow.setStyleSheet(f"color:{C['dim']}; font-size:11px;")

        layout.addWidget(self._badge)
        layout.addWidget(self._sats)
        layout.addWidget(self._tow)

    def _set_badge_style(self, bg: str, fg: str):
        self._badge.setStyleSheet(f"""
            QLabel {{
                background-color: {bg};
                color: {fg};
                border-radius: 8px;
                font-size: 20px;
                font-weight: bold;
                letter-spacing: 2px;
                padding: 10px 20px;
            }}
        """)

    def update_fix(self, fix_type: FixType, num_sats: int, tow: float):
        text, bg, fg = FIX_INFO.get(fix_type, ('UNKNOWN', C['dim'], C['text']))
        self._badge.setText(text)
        self._set_badge_style(bg, fg)
        self._sats.setText(f"{num_sats} satellites used" if num_sats else "— satellites")
        self._tow.setText(f"GPS TOW: {tow:.1f} s" if tow > 0 else "GPS TOW: —")




# ---------------------------------------------------------------------------
# SignalBarsWidget — u-center style vertical CNR bar chart
# ---------------------------------------------------------------------------
class SignalBarsWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._sats: List = []   # (sat_id, constellation, cnr, used)
        self.setMinimumHeight(180)

    def update_satellites(self, sat_list):
        """sat_list: list of (sat_id, constellation, cnr, used)."""
        self._sats = sat_list
        self.update()

    @staticmethod
    def _cnr_color(cnr: float, used: bool) -> QColor:
        if not used:
            return QColor(80, 80, 90)
        if cnr >= 40:
            return QColor(C['green'])
        elif cnr >= 35:
            return QColor(C['bds'])   # yellow-orange
        elif cnr >= 25:
            return QColor(C['orange'])
        else:
            return QColor(C['red'])

    def paintEvent(self, event):
        if not HAS_PYQT:
            return

        p = QPainter(self)
        p.setRenderHint(_AA)

        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor(C['panel']))

        if not self._sats:
            p.setPen(QColor(C['dim']))
            p.setFont(QFont('SF Mono', 11))
            p.drawText(w // 2 - 80, h // 2, "No satellite data")
            p.end()
            return

        # Layout constants
        MARGIN_L = 36    # left margin for Y-axis labels
        MARGIN_R = 8
        MARGIN_T = 12
        MARGIN_B = 36    # bottom margin for PRN labels
        CNR_MAX = 60.0

        plot_w = w - MARGIN_L - MARGIN_R
        plot_h = h - MARGIN_T - MARGIN_B

        # Y-axis grid & labels
        p.setPen(QPen(QColor(C['border']), 1, _Qt_Dash))
        font_ax = QFont('SF Mono', 8)
        p.setFont(font_ax)
        p.setPen(QColor(C['dim']))
        for db in range(0, 61, 10):
            y = MARGIN_T + plot_h - int(plot_h * db / CNR_MAX)
            p.setPen(QPen(QColor(C['border']), 1, _Qt_Dash))
            p.drawLine(MARGIN_L, y, w - MARGIN_R, y)
            p.setPen(QColor(C['dim']))
            p.drawText(2, y + 4, f"{db}")

        # Y-axis label
        p.save()
        p.translate(10, h // 2)
        p.rotate(-90)
        p.drawText(-20, 0, "dB-Hz")
        p.restore()

        # Sort: GPS first, then GLONASS, Galileo, BeiDou
        order = [Constellation.GPS, Constellation.GALILEO,
                 Constellation.GLONASS, Constellation.BEIDOU]
        sats_sorted = sorted(
            self._sats,
            key=lambda s: (order.index(s[1]) if s[1] in order else 99,
                           s[0])
        )

        n = len(sats_sorted)
        if n == 0:
            p.end()
            return

        bar_w = max(6, min(32, (plot_w - n) // n))
        gap = max(1, (plot_w - n * bar_w) // max(n, 1))
        total_needed = n * (bar_w + gap)
        start_x = MARGIN_L + max(0, (plot_w - total_needed) // 2)

        font_prn = QFont('SF Mono', 7)
        font_cnr = QFont('SF Mono', 7)

        prev_const = None
        for i, (sat_id, constellation, cnr, used) in enumerate(sats_sorted):
            bx = start_x + i * (bar_w + gap)
            bar_h = int(plot_h * min(cnr, CNR_MAX) / CNR_MAX)
            by = MARGIN_T + plot_h - bar_h

            # Constellation separator line
            if constellation != prev_const and prev_const is not None:
                p.setPen(QPen(QColor(C['border']), 1))
                p.drawLine(bx - gap // 2, MARGIN_T, bx - gap // 2, MARGIN_T + plot_h)
            prev_const = constellation

            col = self._cnr_color(cnr, used)

            # Bar fill with gradient
            grad = QLinearGradient(0, by, 0, by + bar_h)
            grad.setColorAt(0, col)
            c2 = QColor(col)
            c2.setAlpha(160)
            grad.setColorAt(1, c2)
            p.setPen(_Qt_NoPen)
            p.setBrush(QBrush(grad))
            p.drawRoundedRect(bx, by, bar_w, bar_h, 2, 2)

            # Used border
            if used:
                p.setPen(QPen(QColor(C['text']), 1))
                p.setBrush(Qt.BrushStyle.NoBrush if HAS_PYQT and QT_VERSION == 6
                           else Qt.NoBrush)
                p.drawRoundedRect(bx, by, bar_w, bar_h, 2, 2)

            # CNR value above bar (if bar tall enough)
            if bar_h > 16:
                p.setFont(font_cnr)
                p.setPen(QColor(C['text']))
                val_str = f"{cnr:.0f}"
                p.drawText(bx + bar_w // 2 - 6, by - 2, val_str)

            # PRN label below
            p.setFont(font_prn)
            prn = sat_id[1:] if len(sat_id) > 2 else sat_id
            const_ch = sat_id[0] if sat_id else '?'
            p.setPen(QColor(CONST_COLOR.get(constellation, C['dim'])))
            # Rotate label if bar_w < 18
            if bar_w < 18:
                p.save()
                p.translate(bx + bar_w // 2, MARGIN_T + plot_h + 4)
                p.rotate(45)
                p.drawText(0, 0, sat_id)
                p.restore()
            else:
                p.drawText(bx + 1, MARGIN_T + plot_h + 12, const_ch)
                p.setPen(QColor(C['text']))
                p.drawText(bx + 1, MARGIN_T + plot_h + 24, prn)

        # Legend for used vs tracked
        p.setFont(font_prn)
        p.setPen(_Qt_NoPen)
        p.setBrush(QBrush(QColor(C['green'])))
        p.drawRect(w - MARGIN_R - 90, 4, 8, 8)
        p.setPen(QColor(C['dim']))
        p.drawText(w - MARGIN_R - 80, 12, "Used in fix")
        p.setPen(_Qt_NoPen)
        p.setBrush(QBrush(QColor(70, 70, 80)))
        p.drawRect(w - MARGIN_R - 90, 18, 8, 8)
        p.setPen(QColor(C['dim']))
        p.drawText(w - MARGIN_R - 80, 26, "Tracked only")

        p.end()


# ---------------------------------------------------------------------------
# TimeSeriesWidget — scrolling DOP / accuracy time series
# ---------------------------------------------------------------------------
class TimeSeriesWidget(QWidget):
    WINDOW_S = 300   # seconds of history to display

    def __init__(self, parent=None):
        super().__init__(parent)
        # Deque of (unix_time, hdop, vdop, sigma_h)
        self._data: deque = deque(maxlen=2000)
        self.setMinimumHeight(180)

    def add_point(self, unix_t: float, hdop: float, vdop: float, sigma_h: float):
        self._data.append((unix_t, hdop, vdop, sigma_h))
        self.update()

    def clear(self):
        self._data.clear()
        self.update()

    def paintEvent(self, event):
        if not HAS_PYQT:
            return

        p = QPainter(self)
        p.setRenderHint(_AA)

        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor(C['panel']))

        ML, MR, MT, MB = 42, 12, 14, 28
        pw = w - ML - MR
        ph = h - MT - MB

        if len(self._data) < 2:
            p.setPen(QColor(C['dim']))
            p.setFont(QFont('SF Mono', 11))
            p.drawText(ML + pw // 2 - 80, MT + ph // 2, "Waiting for data…")
            p.end()
            return

        now = self._data[-1][0]
        t0 = now - self.WINDOW_S

        # Visible points
        pts = [(t, hd, vd, sh) for t, hd, vd, sh in self._data if t >= t0]
        if not pts:
            p.end()
            return

        # Y range
        all_vals = [v for _, hd, vd, sh in pts for v in (hd, vd, sh)]
        y_max = max(max(all_vals) * 1.15, 3.0)
        y_min = 0.0

        def tx(t_unix):
            return ML + int(pw * (t_unix - t0) / self.WINDOW_S)

        def ty(val):
            return MT + ph - int(ph * (val - y_min) / (y_max - y_min))

        # Grid
        font_ax = QFont('SF Mono', 8)
        p.setFont(font_ax)
        n_y = 4
        for i in range(n_y + 1):
            val = y_min + (y_max - y_min) * i / n_y
            y = ty(val)
            p.setPen(QPen(QColor(C['border']), 1, _Qt_Dash))
            p.drawLine(ML, y, w - MR, y)
            p.setPen(QColor(C['dim']))
            p.drawText(2, y + 4, f"{val:.1f}")

        # X-axis time labels
        for dt in [0, 60, 120, 180, 240, 300]:
            t_pt = t0 + dt
            if t_pt > now:
                break
            x = tx(t_pt)
            p.setPen(QPen(QColor(C['border']), 1, _Qt_Dash))
            p.drawLine(x, MT, x, MT + ph)
            p.setPen(QColor(C['dim']))
            elapsed = int(now - t_pt)
            p.drawText(x - 10, MT + ph + 16, f"-{elapsed}s")

        # Series: HDOP (blue), VDOP (orange), σH (green)
        series = [
            (1, C['blue'],   'HDOP'),
            (2, C['orange'], 'VDOP'),
            (3, C['green'],  'σH (m)'),
        ]

        for col_idx, col_hex, label in series:
            col = QColor(col_hex)
            p.setPen(QPen(col, 2, _Qt_Solid))
            path = QPainterPath()
            first = True
            for t, hd, vd, sh in pts:
                vals = (None, hd, vd, sh)
                v = vals[col_idx]
                if v is None or math.isnan(v):
                    first = True
                    continue
                x, y = tx(t), ty(max(y_min, min(y_max, v)))
                if first:
                    path.moveTo(x, y)
                    first = False
                else:
                    path.lineTo(x, y)
            p.drawPath(path)

        # Legend
        lx = ML + 6
        ly = MT + 6
        p.setFont(font_ax)
        for i, (_, col_hex, label) in enumerate(series):
            p.setPen(QPen(QColor(col_hex), 2))
            p.drawLine(lx, ly + i * 14 + 6, lx + 16, ly + i * 14 + 6)
            p.setPen(QColor(C['text']))
            p.drawText(lx + 20, ly + i * 14 + 10, label)

        # Y-axis label
        p.save()
        p.setPen(QColor(C['dim']))
        p.translate(10, MT + ph // 2)
        p.rotate(-90)
        p.drawText(-20, 0, "value")
        p.restore()

        p.end()


# ---------------------------------------------------------------------------
# ScatterPlotWidget — ENU position scatter
# ---------------------------------------------------------------------------
class ScatterPlotWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._pts: List = []   # (east, north, fix_type)
        self._ref_ecef = None
        self._ref_lla = None
        self._scale = 10.0
        self._auto = True
        self.setMinimumSize(280, 280)

    def set_reference(self, ecef, lla):
        self._ref_ecef = ecef
        self._ref_lla = lla
        self._pts.clear()

    def add_point(self, ecef, fix_type):
        if self._ref_ecef is None:
            self.set_reference(ecef, ecef_to_lla(*ecef))
            return
        dx = ecef[0] - self._ref_ecef[0]
        dy = ecef[1] - self._ref_ecef[1]
        dz = ecef[2] - self._ref_ecef[2]
        enu = ecef_to_enu(dx, dy, dz, self._ref_lla[0], self._ref_lla[1])
        self._pts.append((enu[0], enu[1], fix_type))
        if self._auto and len(self._pts) > 5:
            self._fit(keep_auto=True)
        self.update()

    def clear_points(self):
        self._pts.clear()
        self._ref_ecef = None
        self._ref_lla = None
        self._scale = 10.0
        self._auto = True
        self.update()

    def zoom_in(self):
        self._auto = False
        self._scale = max(0.5, self._scale * 0.8)
        self.update()

    def zoom_out(self):
        self._auto = False
        self._scale = min(1e7, self._scale * 1.25)
        self.update()

    def _fit(self, keep_auto=False):
        if not self._pts:
            return
        mx = max(max(abs(e) for e, n, _ in self._pts),
                 max(abs(n) for e, n, _ in self._pts), 1.0)
        self._scale = mx * 1.2
        if not keep_auto:
            self._auto = False
        self.update()

    def fit_to_data(self):
        self._fit()

    def set_auto_scale(self, on: bool):
        self._auto = on
        if on:
            self._fit(keep_auto=True)

    def paintEvent(self, event):
        if not HAS_PYQT:
            return
        p = QPainter(self)
        p.setRenderHint(_AA)

        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        M = 36
        p.fillRect(self.rect(), QColor(C['panel']))

        pw, ph = w - 2 * M, h - 2 * M

        # Grid
        p.setPen(QPen(QColor(C['border']), 1, _Qt_Dash))
        for i in range(5):
            x = M + i * pw // 4
            y = M + i * ph // 4
            p.drawLine(x, M, x, h - M)
            p.drawLine(M, y, w - M, y)

        # Axis labels
        font_ax = QFont('SF Mono', 8)
        p.setFont(font_ax)
        p.setPen(QColor(C['dim']))
        for i in range(5):
            v = -self._scale + 2 * self._scale * i / 4
            x = M + i * pw // 4
            y = M + i * ph // 4
            p.drawText(x - 12, h - M + 14, f"{v:.1f}")
            p.drawText(2, h - y + M // 2, f"{v:.1f}")

        p.drawText(cx - 16, h - 4, "East [m]")
        p.drawText(2, M - 4, "North [m]")

        # Cross-hair at center
        p.setPen(QPen(QColor(C['blue']), 1))
        p.drawLine(cx - 10, cy, cx + 10, cy)
        p.drawLine(cx, cy - 10, cx, cy + 10)

        if not self._pts:
            p.end()
            return

        fix_colors = {
            FixType.SPS:       QColor(C['blue']),
            FixType.DGNSS:     QColor(C['orange']),
            FixType.RTK_FLOAT: QColor(C['purple']),
            FixType.RTK_FIXED: QColor(C['green']),
        }

        for i, (e, n, ft) in enumerate(self._pts):
            px = cx + (e / self._scale) * (pw // 2)
            py = cy - (n / self._scale) * (ph // 2)
            if M <= px <= w - M and M <= py <= h - M:
                col = QColor(fix_colors.get(ft, QColor(C['dim'])))
                alpha = int(60 + 195 * i / len(self._pts))
                col.setAlpha(alpha)
                p.setPen(_Qt_NoPen)
                p.setBrush(QBrush(col))
                sz = 3 if i < len(self._pts) - 1 else 6
                p.drawEllipse(QPointF(px, py), sz, sz)

        # CEP50 circle + statistics
        if len(self._pts) > 5:
            es = [pt[0] for pt in self._pts]
            ns = [pt[1] for pt in self._pts]
            me, mn = np.mean(es), np.mean(ns)
            se, sn = np.std(es), np.std(ns)
            cep = 0.62 * sn + 0.56 * se

            # Draw CEP circle
            cep_px = cep / self._scale * (pw // 2)
            p.setPen(QPen(QColor(C['blue']), 1, _Qt_Dash))
            p.setBrush(Qt.BrushStyle.NoBrush if HAS_PYQT and QT_VERSION == 6
                       else Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), cep_px, cep_px)

            # Stats text
            p.setPen(QColor(C['text']))
            p.setFont(font_ax)
            stats = [
                f"N: {len(self._pts)}",
                f"CEP50: {cep:.3f} m",
                f"σE: {se:.3f} m",
                f"σN: {sn:.3f} m",
            ]
            for j, txt in enumerate(stats):
                p.drawText(M + 4, M + 12 + j * 15, txt)

        p.end()


# ---------------------------------------------------------------------------
# SatelliteTableWidget — tabular satellite view (right dock)
# ---------------------------------------------------------------------------
class SatelliteTableWidget(QWidget):
    COLS = ['PRN', 'System', 'CNR (dB)', 'Status', 'Lock (s)']

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        self.table = QTableWidget(0, len(self.COLS))
        self.table.setHorizontalHeaderLabels(self.COLS)
        self.table.setSelectionBehavior(_SEL_ROWS)
        self.table.setEditTriggers(_NO_EDIT)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setShowGrid(True)

        hdr = self.table.horizontalHeader()
        hdr.setSectionResizeMode(0, _HV_F)
        hdr.setSectionResizeMode(1, _HV_F)
        hdr.setSectionResizeMode(2, _HV_R)
        hdr.setSectionResizeMode(3, _HV_R)
        hdr.setSectionResizeMode(4, _HV_S)
        self.table.setColumnWidth(0, 48)
        self.table.setColumnWidth(1, 68)

        lay.addWidget(self.table)

    def update_satellites(self, sat_data: List):
        """sat_data: list of (sat_id, constellation, cnr, used, lock_time)."""
        order = [Constellation.GPS, Constellation.GALILEO,
                 Constellation.GLONASS, Constellation.BEIDOU]
        data = sorted(sat_data, key=lambda s: (
            order.index(s[1]) if s[1] in order else 99, s[0]))

        self.table.setRowCount(0)
        for sat_id, constellation, cnr, used, lock_time in data:
            row = self.table.rowCount()
            self.table.insertRow(row)

            const_col = QColor(CONST_COLOR.get(constellation, C['dim']))

            # PRN
            it = _tbl_item(sat_id)
            it.setForeground(QBrush(const_col))
            if used:
                font = QFont()
                font.setBold(True)
                it.setFont(font)
            self.table.setItem(row, 0, it)

            # System
            it2 = _tbl_item(CONST_NAME.get(constellation, '?'))
            it2.setForeground(QBrush(const_col))
            self.table.setItem(row, 1, it2)

            # CNR
            cnr_str = f"{cnr:.1f}"
            it3 = _tbl_item(cnr_str)
            if cnr >= 40:
                it3.setForeground(QBrush(QColor(C['green'])))
            elif cnr >= 35:
                it3.setForeground(QBrush(QColor(C['bds'])))
            elif cnr >= 25:
                it3.setForeground(QBrush(QColor(C['orange'])))
            else:
                it3.setForeground(QBrush(QColor(C['red'])))
            self.table.setItem(row, 2, it3)

            # Status
            it4 = _tbl_item("Used" if used else "Tracked")
            it4.setForeground(QBrush(
                QColor(C['green']) if used else QColor(C['dim'])))
            self.table.setItem(row, 3, it4)

            # Lock time
            lock_str = f"{lock_time:.0f}" if lock_time > 0 else "—"
            self.table.setItem(row, 4, _tbl_item(lock_str))

            self.table.setRowHeight(row, 22)


# ---------------------------------------------------------------------------
# LiveMapWidget — OpenStreetMap / Google Maps
# ---------------------------------------------------------------------------
class LiveMapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._google_key = ""
        self._loaded = False
        self._rover_track: List = []
        self._ref_track: List = []

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        if HAS_WEBENGINE:
            self.web = QWebEngineView()
            self.web.loadFinished.connect(self._on_loaded)
            lay.addWidget(self.web)
            self._load()
        else:
            self.web = None
            msg = QTextEdit()
            msg.setReadOnly(True)
            msg.setHtml(
                f'<div style="color:{C["dim"]};font-family:monospace;padding:20px;">'
                '<b>Live map unavailable.</b><br><br>'
                'Install a Qt WebEngine package to enable:<br>'
                '&nbsp;&nbsp;pip install PySide6-Addons<br>'
                '&nbsp;&nbsp;pip install PyQt6-WebEngine<br>'
                '&nbsp;&nbsp;pip install PyQtWebEngine</div>'
            )
            lay.addWidget(msg)

    def set_google_api_key(self, key: str):
        self._google_key = key.strip()
        self._load()

    def add_rover_point(self, lla: tuple):
        self._append(self._rover_track, lla)
        self._sync()

    def add_reference_point(self, lla: tuple):
        self._append(self._ref_track, lla)
        self._sync()

    def clear_tracks(self):
        self._rover_track.clear()
        self._ref_track.clear()
        self._sync()

    def _append(self, track, lla):
        if lla is None:
            return
        pt = (float(lla[0]), float(lla[1]))
        if track and abs(track[-1][0] - pt[0]) < 1e-12:
            return
        track.append(pt)
        if len(track) > 8000:
            del track[0:len(track) - 8000]

    def _sync(self):
        if not HAS_WEBENGINE or not self.web or not self._loaded:
            return
        def js_arr(t):
            if not t:
                return "[]"
            return "[" + ",".join(f"[{a:.10f},{b:.10f}]" for a, b in t) + "]"
        rover_now = "null" if not self._rover_track else f"[{self._rover_track[-1][0]:.10f},{self._rover_track[-1][1]:.10f}]"
        ref_now   = "null" if not self._ref_track   else f"[{self._ref_track[-1][0]:.10f},{self._ref_track[-1][1]:.10f}]"
        js = f"window.setTracks({js_arr(self._rover_track)},{js_arr(self._ref_track)},{rover_now},{ref_now});"
        self.web.page().runJavaScript(js)

    def _load(self):
        if not HAS_WEBENGINE or not self.web:
            return
        self._loaded = False
        html = self._google_html(self._google_key) if self._google_key else self._leaflet_html()
        self.web.setHtml(html)

    def _on_loaded(self, ok):
        if not ok:
            return
        self._loaded = True
        self._sync()

    @staticmethod
    def _leaflet_html() -> str:
        return f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    html,body,#map{{height:100%;margin:0;background:{C['bg']};}}
    .badge{{background:rgba(13,17,23,.88);color:{C['text']};padding:5px 8px;
            border-radius:5px;font-family:monospace;font-size:11px;}}
  </style>
</head>
<body>
<div id="map"></div>
<script>
  const map = L.map('map').setView([0,0],2);
  L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',
    {{maxZoom:19,attribution:'&copy; OpenStreetMap'}}).addTo(map);
  const rIcon = L.circleMarker([0,0],{{radius:8,color:'{C["blue"]}',fillColor:'{C["blue"]}',fillOpacity:0.9,weight:2}});
  const fIcon = L.circleMarker([0,0],{{radius:7,color:'{C["green"]}',fillColor:'{C["green"]}',fillOpacity:0.7,weight:2}});
  const rLine = L.polyline([],{{color:'{C["blue"]}',weight:2,opacity:0.85}});
  const fLine = L.polyline([],{{color:'{C["green"]}',weight:2,opacity:0.7,dashArray:'5,5'}});
  let hasR=false,hasF=false,hasRL=false,hasFL=false;
  window.setTracks = function(rTrack,fTrack,rNow,fNow){{
    const bounds=[];
    if(rTrack&&rTrack.length){{rLine.setLatLngs(rTrack);if(!hasRL){{rLine.addTo(map);hasRL=true;}}rTrack.forEach(p=>bounds.push(p));}}
    if(fTrack&&fTrack.length){{fLine.setLatLngs(fTrack);if(!hasFL){{fLine.addTo(map);hasFL=true;}}fTrack.forEach(p=>bounds.push(p));}}
    if(rNow){{rIcon.setLatLng(rNow);if(!hasR){{rIcon.addTo(map);hasR=true;}}rIcon.bindPopup('<div class="badge">Computed position</div>');bounds.push(rNow);}}
    if(fNow){{fIcon.setLatLng(fNow);if(!hasF){{fIcon.addTo(map);hasF=true;}}fIcon.bindPopup('<div class="badge">Reference (NMEA)</div>');bounds.push(fNow);}}
    if(bounds.length===1)map.setView(bounds[0],17);
    else if(bounds.length>1)map.fitBounds(bounds,{{padding:[40,40],maxZoom:19}});
  }};
</script>
</body></html>"""

    @staticmethod
    def _google_html(key: str) -> str:
        safe = key.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        return f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <style>html,body,#map{{height:100%;margin:0;background:{C['bg']};}}</style>
  <script src="https://maps.googleapis.com/maps/api/js?key={safe}"></script>
</head>
<body><div id="map"></div>
<script>
  const map=new google.maps.Map(document.getElementById('map'),{{zoom:3,center:{{lat:0,lng:0}}}});
  let rM=null,fM=null,rP=null,fP=null;
  window.setTracks=function(rT,fT,rN,fN){{
    const bounds=new google.maps.LatLngBounds();let c=0;
    if(rT&&rT.length){{const path=rT.map(p=>({{lat:p[0],lng:p[1]}}));if(!rP){{rP=new google.maps.Polyline({{path,geodesic:true,strokeColor:'{C["blue"]}',strokeWeight:2,map}});}}else{{rP.setPath(path);}}rT.forEach(p=>bounds.extend({{lat:p[0],lng:p[1]}}));c+=rT.length;}}
    if(fT&&fT.length){{const path=fT.map(p=>({{lat:p[0],lng:p[1]}}));if(!fP){{fP=new google.maps.Polyline({{path,geodesic:true,strokeColor:'{C["green"]}',strokeWeight:2,map}});}}else{{fP.setPath(path);}}fT.forEach(p=>bounds.extend({{lat:p[0],lng:p[1]}}));c+=fT.length;}}
    if(rN){{const pt={{lat:rN[0],lng:rN[1]}};if(!rM){{rM=new google.maps.Marker({{position:pt,map,title:'Computed'}});}}else{{rM.setPosition(pt);}}bounds.extend(pt);c++;}}
    if(fN){{const pt={{lat:fN[0],lng:fN[1]}};if(!fM){{fM=new google.maps.Marker({{position:pt,map,title:'Reference'}});}}else{{fM.setPosition(pt);}}bounds.extend(pt);c++;}}
    if(c===1){{map.setZoom(17);map.setCenter(bounds.getCenter());}}else if(c>1){{map.fitBounds(bounds,60);}}
  }};
</script>
</body></html>"""


# ===========================================================================
# Main Window
# ===========================================================================
class MainWindow(QMainWindow):
    solution_received  = pyqtSignal(object)
    satellites_received = pyqtSignal(object)
    ephemeris_updated  = pyqtSignal(dict)
    status_message     = pyqtSignal(str)
    reference_updated  = pyqtSignal(dict)

    def __init__(self):
        super().__init__()

        self.pipeline = GNSSPipeline()
        self._err2d_hist: deque = deque(maxlen=5000)
        self._err3d_hist: deque = deque(maxlen=5000)
        self._last_sol: Optional[PositionSolution] = None
        # sat_id → (constellation, cnr, used, lock_time)
        self._sat_cache: Dict[str, tuple] = {}

        self._setup_signals()
        self._build_toolbar()
        self._build_central()
        self._build_fix_dock()
        self._build_sat_dock()
        self._build_console_dock()
        self.setStyleSheet(SS)
        self.setWindowTitle("GNSS Positioning Engine")
        self.setWindowIcon(_icon('app_icon'))
        self.setMinimumSize(1280, 820)

        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(1000)

    # ------------------------------------------------------------------
    # Signal wiring
    # ------------------------------------------------------------------
    def _setup_signals(self):
        self.pipeline.on_solution         = lambda s: self.solution_received.emit(s)
        self.pipeline.on_satellites       = lambda s: self.satellites_received.emit(s)
        self.pipeline.on_ephemeris_update = lambda e: self.ephemeris_updated.emit(e)
        self.pipeline.on_status           = lambda s: self.status_message.emit(s)
        self.pipeline.on_reference        = lambda r: self.reference_updated.emit(r)

        self.solution_received.connect(self._on_solution)
        self.satellites_received.connect(self._on_satellites)
        self.ephemeris_updated.connect(self._on_ephemeris)
        self.status_message.connect(self._on_status_msg)
        self.reference_updated.connect(self._on_reference)

    # ------------------------------------------------------------------
    # Toolbar
    # ------------------------------------------------------------------
    def _build_toolbar(self):
        tb = QToolBar("Main", self)
        tb.setMovable(False)
        self.addToolBar(tb)

        # --- Rover connection ---
        tb.addWidget(QLabel("Rover:"))
        self.rover_port = QComboBox()
        self.rover_port.setMinimumWidth(140)
        tb.addWidget(self.rover_port)

        self.rover_baud = QComboBox()
        self.rover_baud.addItems(['115200', '921600', '460800', '230400', '57600', '9600'])
        self.rover_baud.setMinimumWidth(80)
        tb.addWidget(self.rover_baud)

        self.rover_btn = QPushButton(_icon('icon_connect'), "Connect")
        self.rover_btn.setObjectName("primary")
        self.rover_btn.clicked.connect(self._toggle_rover)
        tb.addWidget(self.rover_btn)

        self.cfg_rtcm_btn = QPushButton(_icon('icon_satellite'), "RTCM Only")
        self.cfg_rtcm_btn.setEnabled(False)
        self.cfg_rtcm_btn.clicked.connect(self._cfg_rtcm)
        tb.addWidget(self.cfg_rtcm_btn)

        self.cfg_ref_btn = QPushButton(_icon('icon_satellite'), "RTCM+NMEA")
        self.cfg_ref_btn.setEnabled(False)
        self.cfg_ref_btn.clicked.connect(self._cfg_rtcm_nmea)
        tb.addWidget(self.cfg_ref_btn)

        _sep(tb)

        # --- Base connection ---
        tb.addWidget(QLabel("Base:"))
        self.base_port = QComboBox()
        self.base_port.setMinimumWidth(120)
        tb.addWidget(self.base_port)

        self.base_baud = QComboBox()
        self.base_baud.addItems(['115200', '921600', '460800', '230400'])
        self.base_baud.setMinimumWidth(80)
        tb.addWidget(self.base_baud)

        self.base_btn = QPushButton(_icon('icon_connect'), "Connect Base")
        self.base_btn.clicked.connect(self._toggle_base)
        tb.addWidget(self.base_btn)

        _sep(tb)

        # --- Mode ---
        tb.addWidget(QLabel("Mode:"))
        self.sps_btn   = QPushButton(_icon('icon_sps'),   "SPS")
        self.dgnss_btn = QPushButton(_icon('icon_dgnss'), "DGNSS")
        self.rtk_btn   = QPushButton(_icon('icon_rtk'),   "RTK")
        self.sps_btn.setObjectName("mode_active")
        for btn, mode in [(self.sps_btn, PositioningMode.SPS),
                          (self.dgnss_btn, PositioningMode.DGNSS),
                          (self.rtk_btn, PositioningMode.RTK)]:
            _m = mode
            btn.clicked.connect(lambda checked, m=_m: self._set_mode(m))
            tb.addWidget(btn)

        _sep(tb)

        # --- Record ---
        self.rec_btn = QPushButton(_icon('icon_record'), "Record")
        self.rec_btn.setCheckable(True)
        self.rec_btn.toggled.connect(self._toggle_recording)
        tb.addWidget(self.rec_btn)

        _sep(tb)

        # --- Refresh ---
        refresh_btn = QPushButton(_icon('icon_refresh'), "Refresh Ports")
        refresh_btn.clicked.connect(self._refresh_ports)
        tb.addWidget(refresh_btn)

    # ------------------------------------------------------------------
    # Central tab widget
    # ------------------------------------------------------------------
    def _build_central(self):
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.map_view     = LiveMapWidget()
        self.sig_bars     = SignalBarsWidget()
        self.scatter      = ScatterPlotWidget()
        self.time_series  = TimeSeriesWidget()

        self.tabs.addTab(self.map_view,    "🗺  Map")
        self.tabs.addTab(self.sig_bars,    "📶  Signal")
        self.tabs.addTab(self.scatter,     "·  Scatter")
        self.tabs.addTab(self.time_series, "📈  DOP / σ")

    # ------------------------------------------------------------------
    # Left dock — Fix & Position
    # ------------------------------------------------------------------
    def _build_fix_dock(self):
        dock = QDockWidget("Fix & Position", self)
        dock.setAllowedAreas(_LD)

        content = QWidget()
        lay = QVBoxLayout(content)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(8)

        # Fix badge
        self.fix_badge = FixBadgeWidget()
        lay.addWidget(self.fix_badge)

        # Position
        pos_grp = QGroupBox("Position")
        pg = QGridLayout(pos_grp)
        pg.setSpacing(4)
        self._pos_labels = {}
        for i, (k, lbl) in enumerate([
            ('lat',  'Latitude'),
            ('lon',  'Longitude'),
            ('alt',  'Altitude'),
        ]):
            dim = QLabel(lbl + ":")
            dim.setStyleSheet(f"color:{C['dim']};")
            val = QLabel("—")
            val.setStyleSheet("font-weight:bold;font-size:13px;")
            self._pos_labels[k] = val
            pg.addWidget(dim, i, 0)
            pg.addWidget(val, i, 1)
        lay.addWidget(pos_grp)

        # Quality
        qual_grp = QGroupBox("Quality")
        qg = QGridLayout(qual_grp)
        qg.setSpacing(4)
        self._qual_labels = {}
        for i, (k, lbl) in enumerate([
            ('hdop',    'HDOP'),
            ('vdop',    'VDOP'),
            ('pdop',    'PDOP'),
            ('sigma_h', 'Sigma H'),
            ('sigma_v', 'Sigma V'),
        ]):
            dim = QLabel(lbl + ":")
            dim.setStyleSheet(f"color:{C['dim']};")
            val = QLabel("—")
            val.setStyleSheet("font-weight:bold;")
            self._qual_labels[k] = val
            qg.addWidget(dim, i, 0)
            qg.addWidget(val, i, 1)
        lay.addWidget(qual_grp)

        # Reference error
        ref_grp = QGroupBox("Reference Error")
        rg = QGridLayout(ref_grp)
        rg.setSpacing(4)
        self._ref_labels = {}
        for i, (k, lbl) in enumerate([
            ('src',   'Source'),
            ('lat',   'Ref Lat'),
            ('lon',   'Ref Lon'),
            ('alt',   'Ref Alt'),
            ('e2d',   'Error 2D'),
            ('e3d',   'Error 3D'),
            ('r2d',   'RMS 2D'),
            ('r3d',   'RMS 3D'),
        ]):
            dim = QLabel(lbl + ":")
            dim.setStyleSheet(f"color:{C['dim']};")
            val = QLabel("—" if k != 'src' else "Auto (GGA/RMC)")
            val.setStyleSheet("font-weight:bold;")
            self._ref_labels[k] = val
            rg.addWidget(dim, i, 0)
            rg.addWidget(val, i, 1)
        lay.addWidget(ref_grp)

        # Ephemeris
        eph_grp = QGroupBox("Ephemeris")
        eg = QGridLayout(eph_grp)
        eg.setSpacing(4)
        self._eph_labels = {}
        for i, (k, col) in enumerate([
            ('GPS',     C['gps']),
            ('GLONASS', C['glo']),
            ('Galileo', C['gal']),
            ('BeiDou',  C['bds']),
        ]):
            lbl = QLabel(k + ":")
            lbl.setStyleSheet(f"color:{col};")
            val = QLabel("0 SVs")
            self._eph_labels[k] = val
            eg.addWidget(lbl, i, 0)
            eg.addWidget(val, i, 1)
        lay.addWidget(eph_grp)

        # Session logging
        log_grp = QGroupBox("Session Logging")
        lg = QVBoxLayout(log_grp)
        lg.setSpacing(4)

        log_path_row = QHBoxLayout()
        self.log_path = QLineEdit(self._default_log_path())
        browse_btn = QPushButton("…")
        browse_btn.setMaximumWidth(28)
        browse_btn.clicked.connect(self._browse_log)
        log_path_row.addWidget(self.log_path)
        log_path_row.addWidget(browse_btn)
        lg.addLayout(log_path_row)

        self.log_status_lbl = QLabel("Idle")
        self.log_status_lbl.setStyleSheet(f"color:{C['dim']};font-size:11px;")
        lg.addWidget(self.log_status_lbl)
        lay.addWidget(log_grp)

        # Map settings
        map_grp = QGroupBox("Map Settings")
        mg = QHBoxLayout(map_grp)
        self.google_key_edit = QLineEdit()
        self.google_key_edit.setPlaceholderText("Google Maps API key (optional)")
        apply_key_btn = QPushButton("Apply")
        apply_key_btn.setMaximumWidth(48)
        apply_key_btn.clicked.connect(self._apply_map_key)
        mg.addWidget(self.google_key_edit)
        mg.addWidget(apply_key_btn)
        lay.addWidget(map_grp)

        # Scatter controls
        scatter_grp = QGroupBox("Scatter Controls")
        sg = QHBoxLayout(scatter_grp)
        zi = QPushButton("＋")
        zo = QPushButton("－")
        ft = QPushButton("Fit")
        cl = QPushButton("Clear")
        self.auto_cb = QCheckBox("Auto")
        self.auto_cb.setChecked(True)
        self.auto_cb.toggled.connect(self.scatter.set_auto_scale)
        zi.clicked.connect(self.scatter.zoom_in)
        zo.clicked.connect(self.scatter.zoom_out)
        ft.clicked.connect(self.scatter.fit_to_data)
        cl.clicked.connect(self._clear_history)
        for w in [zi, zo, ft, cl, self.auto_cb]:
            sg.addWidget(w)
        lay.addWidget(scatter_grp)

        lay.addStretch()

        scroll = QScrollArea()
        scroll.setWidget(content)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(270)
        dock.setWidget(scroll)
        self.addDockWidget(_LD, dock)

    # ------------------------------------------------------------------
    # Right dock — Satellite Table
    # ------------------------------------------------------------------
    def _build_sat_dock(self):
        dock = QDockWidget("Satellites", self)
        dock.setAllowedAreas(_RD | _LD)
        self.sat_table = SatelliteTableWidget()
        dock.setWidget(self.sat_table)
        self.addDockWidget(_RD, dock)

    # ------------------------------------------------------------------
    # Bottom dock — Console
    # ------------------------------------------------------------------
    def _build_console_dock(self):
        dock = QDockWidget("Console", self)
        dock.setAllowedAreas(_BD)

        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)

        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumHeight(160)

        hdr = QHBoxLayout()
        clear_btn = QPushButton(_icon('icon_clear'), "Clear")
        clear_btn.setMaximumWidth(80)
        clear_btn.clicked.connect(self.console.clear)
        hdr.addStretch()
        hdr.addWidget(clear_btn)

        lay.addLayout(hdr)
        lay.addWidget(self.console)
        dock.setWidget(w)
        self.addDockWidget(_BD, dock)

    # ------------------------------------------------------------------
    # Port management
    # ------------------------------------------------------------------
    def _refresh_ports(self):
        ports = SerialHandler.list_ports()
        self.rover_port.clear()
        self.base_port.clear()
        if not ports:
            self.rover_port.addItem("No ports found")
            self.base_port.addItem("No ports found")
            self._log("No serial ports detected")
            return
        for port in ports:
            desc = f"{port['device']} — {port['description']}"
            self.rover_port.addItem(desc, port['device'])
            self.base_port.addItem(desc, port['device'])
        self._log(f"Found {len(ports)} port(s)")

    def _toggle_rover(self):
        if self.pipeline.rover_serial.is_connected:
            self.pipeline.rover_serial.disconnect()
            self.rover_btn.setIcon(_icon('icon_connect'))
            self.rover_btn.setText("Connect")
            self.rover_btn.setObjectName("primary")
            self.rover_btn.style().unpolish(self.rover_btn)
            self.rover_btn.style().polish(self.rover_btn)
            self.cfg_rtcm_btn.setEnabled(False)
            self.cfg_ref_btn.setEnabled(False)
            self._log("Rover disconnected")
            return

        port = self.rover_port.currentData()
        if not port:
            return
        baud = int(self.rover_baud.currentText())
        if self.pipeline.connect_rover(port, baud):
            self.rover_btn.setIcon(_icon('icon_disconnect'))
            self.rover_btn.setText("Disconnect")
            self.rover_btn.setObjectName("")
            self.rover_btn.style().unpolish(self.rover_btn)
            self.rover_btn.style().polish(self.rover_btn)
            self.cfg_rtcm_btn.setEnabled(True)
            self.cfg_ref_btn.setEnabled(True)
            self._log(f"Rover connected: {port} @ {baud}")
        else:
            self._log("Rover connection failed!", error=True)

    def _toggle_base(self):
        if self.pipeline.base_serial.is_connected:
            self.pipeline.base_serial.disconnect()
            self.base_btn.setIcon(_icon('icon_connect'))
            self.base_btn.setText("Connect Base")
            self._log("Base disconnected")
            return
        port = self.base_port.currentData()
        if not port:
            return
        baud = int(self.base_baud.currentText())
        if self.pipeline.connect_base(port, baud):
            self.base_btn.setIcon(_icon('icon_disconnect'))
            self.base_btn.setText("Disconnect Base")
            self._log(f"Base connected: {port} @ {baud}")
        else:
            self._log("Base connection failed!", error=True)

    def _cfg_rtcm(self):
        if self.pipeline.configure_rover_rtcm():
            self._log("Rover configured: RTCM-only output")
        else:
            self._log("RTCM configuration failed!", error=True)

    def _cfg_rtcm_nmea(self):
        if self.pipeline.configure_rover_rtcm_with_reference():
            self._log("Rover configured: RTCM + GGA/RMC reference")
        else:
            self._log("RTCM+NMEA configuration failed!", error=True)

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------
    def _set_mode(self, mode: PositioningMode):
        self.pipeline.set_mode(mode)
        for btn, m in [(self.sps_btn, PositioningMode.SPS),
                       (self.dgnss_btn, PositioningMode.DGNSS),
                       (self.rtk_btn, PositioningMode.RTK)]:
            btn.setObjectName("mode_active" if m == mode else "")
            btn.style().unpolish(btn)
            btn.style().polish(btn)
        self._log(f"Mode: {mode.name}")

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------
    def _toggle_recording(self, checked: bool):
        if checked:
            path = self.log_path.text().strip() or self._default_log_path()
            self.log_path.setText(path)
            if self.pipeline.start_session_logging(path):
                self.rec_btn.setIcon(_icon('icon_record_stop'))
                self.rec_btn.setText("Stop")
                self.log_status_lbl.setText(f"● REC  {os.path.basename(path)}")
                self.log_status_lbl.setStyleSheet(f"color:{C['red']};font-size:11px;font-weight:bold;")
                self._log(f"Recording started: {path}")
            else:
                self.rec_btn.setChecked(False)
                self._log("Recording already active or failed", error=True)
        else:
            self.pipeline.stop_session_logging()
            self.rec_btn.setIcon(_icon('icon_record'))
            self.rec_btn.setText("Record")
            self.log_status_lbl.setText("Idle")
            self.log_status_lbl.setStyleSheet(f"color:{C['dim']};font-size:11px;")
            self.log_path.setText(self._default_log_path())
            self._log("Recording stopped")

    def _default_log_path(self) -> str:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(os.getcwd(), "logs", f"gnss_session_{ts}.jsonl")

    def _browse_log(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Log File", self.log_path.text(),
            "JSON Lines (*.jsonl);;All Files (*)"
        )
        if path:
            self.log_path.setText(path)

    # ------------------------------------------------------------------
    # Map settings
    # ------------------------------------------------------------------
    def _apply_map_key(self):
        key = self.google_key_edit.text().strip()
        self.map_view.set_google_api_key(key)
        self._log("Google Maps key applied" if key else "Using OpenStreetMap (key cleared)")

    # ------------------------------------------------------------------
    # History clear
    # ------------------------------------------------------------------
    def _clear_history(self):
        self.scatter.clear_points()
        self.map_view.clear_tracks()
        self.time_series.clear()
        self._err2d_hist.clear()
        self._err3d_hist.clear()
        for k in ('e2d', 'e3d', 'r2d', 'r3d'):
            self._ref_labels[k].setText("—")
        self._log("History cleared")

    # ------------------------------------------------------------------
    # Pipeline callbacks
    # ------------------------------------------------------------------
    def _on_solution(self, sol: PositionSolution):
        self._last_sol = sol

        # Fix badge
        self.fix_badge.update_fix(sol.fix_type, sol.num_satellites, sol.timestamp_gps)

        if sol.is_valid:
            # Position
            self._pos_labels['lat'].setText(f"{sol.latitude:.8f}°")
            self._pos_labels['lon'].setText(f"{sol.longitude:.8f}°")
            self._pos_labels['alt'].setText(f"{sol.altitude:.3f} m")

            # Quality
            self._qual_labels['hdop'].setText(f"{sol.dop.hdop:.2f}")
            self._qual_labels['vdop'].setText(f"{sol.dop.vdop:.2f}")
            self._qual_labels['pdop'].setText(f"{sol.dop.pdop:.2f}")
            self._qual_labels['sigma_h'].setText(f"{sol.sigma_horizontal:.3f} m")
            self._qual_labels['sigma_v'].setText(f"{sol.sigma_vertical:.3f} m")

            # Time series
            self.time_series.add_point(
                _time.time(), sol.dop.hdop, sol.dop.vdop, sol.sigma_horizontal)

            # Scatter + map
            self.scatter.add_point(sol.position_ecef, sol.fix_type)
            self.map_view.add_rover_point(sol.position_lla)

            # Update 'used' flags in sat cache
            for sat_id in sol.satellites_used:
                if sat_id in self._sat_cache:
                    c, cnr, _, lt = self._sat_cache[sat_id]
                    self._sat_cache[sat_id] = (c, cnr, True, lt)

            # Reference error
            ref = self.pipeline.reference_position
            err = self._update_ref_error(sol, ref)

        self._update_sat_views()

        # Log epoch
        self._log_epoch(sol)

    def _on_satellites(self, epoch: EpochObservations):
        seen = set()
        # Mark all existing as not-used first (solution callback sets used=True)
        for sat_id in list(self._sat_cache):
            c, cnr, _, lt = self._sat_cache[sat_id]
            self._sat_cache[sat_id] = (c, cnr, False, lt)

        for obs in epoch.observations:
            if obs.sat_id in seen:
                continue
            seen.add(obs.sat_id)
            prev = self._sat_cache.get(obs.sat_id)
            used = prev[2] if prev else False
            self._sat_cache[obs.sat_id] = (obs.constellation, obs.cnr, used, obs.lock_time)

        self._update_sat_views()

    def _on_ephemeris(self, summary: dict):
        for name, lbl in self._eph_labels.items():
            lbl.setText(f"{summary.get(name, 0)} SVs")

    def _on_reference(self, ref: dict):
        self._ref_labels['src'].setText(ref.get('source', '?'))
        self._ref_labels['lat'].setText(f"{ref.get('latitude', 0.0):.8f}°")
        self._ref_labels['lon'].setText(f"{ref.get('longitude', 0.0):.8f}°")
        self._ref_labels['alt'].setText(f"{ref.get('altitude', 0.0):.3f} m")
        self.map_view.add_reference_point((
            float(ref.get('latitude', 0.0)),
            float(ref.get('longitude', 0.0)),
            float(ref.get('altitude', 0.0)),
        ))

    def _on_status_msg(self, msg: str):
        self.statusBar().showMessage(msg)

    # ------------------------------------------------------------------
    # Satellite view update (sky + signal bars + table)
    # ------------------------------------------------------------------
    def _update_sat_views(self):
        sat_list_bars  = []
        sat_list_table = []

        for sat_id, (constellation, cnr, used, lock_time) in self._sat_cache.items():
            sat_list_bars.append((sat_id, constellation, cnr, used))
            sat_list_table.append((sat_id, constellation, cnr, used, lock_time))

        self.sig_bars.update_satellites(sat_list_bars)
        self.sat_table.update_satellites(sat_list_table)

    # ------------------------------------------------------------------
    # Reference error
    # ------------------------------------------------------------------
    def _update_ref_error(self, sol: PositionSolution, ref: Optional[dict]):
        if not ref:
            return None
        try:
            rlat = float(ref['latitude'])
            rlon = float(ref['longitude'])
            ralt = float(ref.get('altitude', 0.0))
            ref_ecef = lla_to_ecef(rlat, rlon, ralt)
            dx = float(sol.x - ref_ecef[0])
            dy = float(sol.y - ref_ecef[1])
            dz = float(sol.z - ref_ecef[2])
            enu = ecef_to_enu(dx, dy, dz, rlat, rlon)
            e2d = float(np.hypot(enu[0], enu[1]))
            e3d = float(np.linalg.norm([dx, dy, dz]))
        except Exception:
            return None

        self._err2d_hist.append(e2d)
        self._err3d_hist.append(e3d)
        r2d = float(np.sqrt(np.mean(np.square(self._err2d_hist))))
        r3d = float(np.sqrt(np.mean(np.square(self._err3d_hist))))

        self._ref_labels['e2d'].setText(f"{e2d:.3f} m")
        self._ref_labels['e3d'].setText(f"{e3d:.3f} m")
        self._ref_labels['r2d'].setText(f"{r2d:.3f} m")
        self._ref_labels['r3d'].setText(f"{r3d:.3f} m")
        return e2d, e3d

    # ------------------------------------------------------------------
    # Status bar 1-Hz tick
    # ------------------------------------------------------------------
    def _tick(self):
        stats = self.pipeline.stats
        rover = stats.get('rover_serial', {})
        if not rover.get('connected'):
            self.statusBar().showMessage("Not connected — select port and click Connect")
            return
        rate = rover.get('data_rate_bps', 0)
        ref  = stats.get('reference')
        log  = stats.get('logging', {})
        fix  = self._last_sol.fix_type.name if self._last_sol else 'NO_FIX'
        nsats = self._last_sol.num_satellites if self._last_sol else 0
        ref_txt = f"Ref:{ref.get('source','?')}" if ref else "Ref:none"
        log_txt = "● REC" if log.get('active') else "LOG:idle"
        self.statusBar().showMessage(
            f"Mode:{stats['mode']}  |  Fix:{fix}  |  Sats:{nsats}  |  "
            f"Epochs:{stats['epochs_processed']}  |  "
            f"Sol:{stats['solutions_computed']}  |  "
            f"{ref_txt}  |  {log_txt}  |  "
            f"Rate:{rate/1000:.1f} kbps"
        )

    # ------------------------------------------------------------------
    # Console log
    # ------------------------------------------------------------------
    def _log(self, msg: str, error: bool = False):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        col = C['red'] if error else C['dim']
        self.console.append(
            f'<span style="color:{C["dim"]}">[{ts}]</span> '
            f'<span style="color:{col}">{msg}</span>'
        )

    def _log_epoch(self, sol: PositionSolution):
        ref = self.pipeline.reference_position
        if not sol.is_valid:
            ref_str = (f"ref=({ref['latitude']:.7f},{ref['longitude']:.7f})"
                       if ref else "ref=none")
            self._log(f"TOW {sol.timestamp_gps:.1f}: NO_FIX | {ref_str}")
            return
        sol_str = f"({sol.latitude:.7f},{sol.longitude:.7f},{sol.altitude:.2f}m)"
        if ref:
            ref_str = (f"({ref['latitude']:.7f},{ref['longitude']:.7f},"
                       f"{ref.get('altitude',0.):.2f}m)")
            # Compute 2D error directly from the ref coordinates being logged.
            # Reading from the label is unreliable: the NMEA reference can be
            # updated by the serial thread between _update_ref_error and here.
            try:
                rlat = float(ref['latitude'])
                rlon = float(ref['longitude'])
                ralt = float(ref.get('altitude', 0.0))
                ref_ecef = lla_to_ecef(rlat, rlon, ralt)
                dx = float(sol.x - ref_ecef[0])
                dy = float(sol.y - ref_ecef[1])
                dz = float(sol.z - ref_ecef[2])
                enu = ecef_to_enu(dx, dy, dz, rlat, rlon)
                e2d_str = f"{float(np.hypot(enu[0], enu[1])):.3f} m"
            except Exception:
                e2d_str = "?"
            self._log(f"TOW {sol.timestamp_gps:.1f}: {sol.fix_type.name} "
                      f"pos={sol_str} ref={ref_str} 2D={e2d_str}")
        else:
            self._log(f"TOW {sol.timestamp_gps:.1f}: {sol.fix_type.name} pos={sol_str}")

    # ------------------------------------------------------------------
    # Window events
    # ------------------------------------------------------------------
    def showEvent(self, event):
        super().showEvent(event)
        self._refresh_ports()

    def closeEvent(self, event):
        self.pipeline.disconnect_all()
        event.accept()


# ===========================================================================
# Entry point
# ===========================================================================
def run_app():
    if not HAS_PYQT:
        print("ERROR: No Qt binding found. Install one of:")
        print("  pip install PySide6")
        print("  pip install PyQt6")
        sys.exit(1)

    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
        datefmt='%H:%M:%S',
    )

    # On macOS, setting the application name here controls the menu bar and
    # Dock entry.  Must be done before QApplication is created so that the
    # CFBundleName in the process info is overridden.
    if sys.platform == 'darwin':
        try:
            from Foundation import NSBundle          # type: ignore
            info = NSBundle.mainBundle().infoDictionary()
            info['CFBundleName']         = 'GNSS Engine'
            info['CFBundleDisplayName']  = 'GNSS Engine'
        except Exception:
            pass  # pyobjc not installed — name will be set via Qt below

    app = QApplication(sys.argv)
    app.setApplicationName("GNSS Engine")
    app.setApplicationDisplayName("GNSS Engine")
    app.setOrganizationName("GNSS Engine")
    app.setOrganizationDomain("gnss-engine.local")

    # App icon (used in Dock, taskbar, alt-tab, window chrome)
    app_icon = _icon('app_icon')
    app.setWindowIcon(app_icon)

    window = MainWindow()
    window.show()

    sys.exit(app.exec() if QT_VERSION == 6 else app.exec_())
