from PyQt6.QtGui import QPalette, QColor
from PyQt6.QtCore import Qt

# ── Color Palettes ──────────────────────────────────────────────────

DARK = {
    "bg_main":          "#2b2b2b",
    "bg_panel":         "#1e1e1e",
    "bg_button":        "#3e3e3e",
    "bg_button_hover":  "#4e4e4e",
    "bg_button_press":  "#222222",
    "bg_header":        "#333333",
    "bg_header_border": "#444444",
    "border":           "#3e3e3e",
    "border_light":     "#555555",
    "border_hover":     "#777777",
    "text":             "#e0e0e0",
    "text_secondary":   "#aaaaaa",
    "text_button":      "white",
    "text_header":      "white",
    "accent_gold":      "#ffd700",
    "console_text":     "#00ff00",
    "grid":             "#333333",
    "gl_bg":            [0.17, 0.17, 0.17, 1.0],
    "gl_text":          [1, 1, 1, 1],
    "gl_grid":          [0.3, 0.3, 0.3, 1.0],
    "plot_bg":          "#1e1e1e",
    "plot_fg":          "#e0e0e0",
    "plot_grid":        "#333333",
}

LIGHT = {
    "bg_main":          "#f0f0f0",
    "bg_panel":         "#ffffff",
    "bg_button":        "#dcdcdc",
    "bg_button_hover":  "#c8c8c8",
    "bg_button_press":  "#b0b0b0",
    "bg_header":        "#e0e0e0",
    "bg_header_border": "#cccccc",
    "border":           "#cccccc",
    "border_light":     "#999999",
    "border_hover":     "#888888",
    "text":             "#222222",
    "text_secondary":   "#555555",
    "text_button":      "#222222",
    "text_header":      "#222222",
    "accent_gold":      "#996515",
    "console_text":     "#006600",
    "grid":             "#cccccc",
    "gl_bg":            [0.88, 0.88, 0.88, 1.0],
    "gl_text":          [0, 0, 0, 1],
    "gl_grid":          [0.7, 0.7, 0.7, 1.0],
    "plot_bg":          "#ffffff",
    "plot_fg":          "#222222",
    "plot_grid":        "#cccccc",
}

# ── Stylesheet Generator ────────────────────────────────────────────

def get_stylesheet(p: dict) -> str:
    return f"""
QMainWindow, QWidget {{
    background-color: {p['bg_main']};
    color: {p['text']};
    font-family: 'Segoe UI', sans-serif;
}}
QGroupBox {{
    border: 2px solid {p['border']};
    border-radius: 6px;
    margin-top: 1.5em;
    font-weight: bold;
    color: {p['accent_gold']};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
}}
QPushButton {{
    background-color: {p['bg_button']};
    border: 1px solid {p['border_light']};
    border-radius: 4px;
    padding: 5px;
    color: {p['text_button']};
}}
QPushButton:hover {{
    background-color: {p['bg_button_hover']};
    border: 1px solid {p['border_hover']};
}}
QPushButton:pressed {{
    background-color: {p['bg_button_press']};
}}
QTableWidget {{
    background-color: {p['bg_panel']};
    gridline-color: {p['grid']};
    color: {p['text']};
    border: 1px solid {p['border']};
}}
QHeaderView::section {{
    background-color: {p['bg_header']};
    color: {p['text_header']};
    padding: 4px;
    border: 1px solid {p['bg_header_border']};
}}
QTextEdit {{
    background-color: {p['bg_panel']};
    color: {p['console_text']};
    border: 1px solid {p['border']};
    font-family: 'Consolas', monospace;
}}
QLabel {{
    color: {p['text']};
}}
"""

# ── QPalette for native widgets / plot integration ──────────────────

def get_qpalette(p: dict) -> QPalette:
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(p['bg_main']))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(p['text']))
    palette.setColor(QPalette.ColorRole.Base, QColor(p['bg_panel']))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(p['bg_main']))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(p['bg_panel']))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(p['text']))
    palette.setColor(QPalette.ColorRole.Text, QColor(p['text']))
    palette.setColor(QPalette.ColorRole.Button, QColor(p['bg_button']))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(p['text_button']))
    palette.setColor(QPalette.ColorRole.BrightText, QColor('red'))
    palette.setColor(QPalette.ColorRole.Highlight, QColor('#0088ff'))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor('white'))
    return palette

# ── Active theme state ──────────────────────────────────────────────

_active_theme = "dark"

def get_active_palette() -> dict:
    return DARK if _active_theme == "dark" else LIGHT

def set_active_theme(name: str):
    global _active_theme
    _active_theme = name

def toggle_theme() -> str:
    global _active_theme
    _active_theme = "light" if _active_theme == "dark" else "dark"
    return _active_theme

def is_dark() -> bool:
    return _active_theme == "dark"
