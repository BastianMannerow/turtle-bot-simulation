from typing import Any, List, Optional, Tuple, Dict
import tkinter as tk
from tkinter import font as tkfont

from simulation.AgentConstruct import AgentConstruct
from simulation.environment.Wall import Wall
from simulation.environment.FakeWall import FakeWall
from simulation.environment.DefinitelyAWall import DefinitelyAWall


class ExampleGUI:
    """
    Compact, responsive grid renderer for an Environment with `level_matrix`.

    Contract
    --------
    - Expects `env.level_matrix` as `List[List[List[Any]]]`.
    - `update()` fully repaints. Safe to call frequently.
    - Runs headless if no Tk root is attached.

    Responsibilities
    ----------------
    - Paint a rectilinear grid with semantic cell colors.
    - Draw agents as labeled discs. Same color for all agents, distinct text labels.
    - Keep rendering adaptive to widget resize.

    Accessibility
    -------------
    - High-contrast palette.
    - Auto-fitting label font to avoid overflow.

    State of the Art (comments)
    ---------------------------
    - UI responsiveness via <Configure> debounced resize and cell size recompute.
    - Text legibility using dynamic font measurement and truncation.
    - Predictable theming with a cohesive dark palette and semantic roles.
    - Full redraw simplicity favored; upstream dirty-rects can be added if needed.
    """

    # Theming: WCAG-friendly dark palette (semantic roles).
    _COLORS: Dict[str, str] = {
        "app_bg": "#0F1117",
        "grid_bg": "#0F1117",
        "grid_line": "#1B2130",
        "cell_outline": "#232A36",
        "agent_fill": "#94D3A2",          # all agents share this hue
        "agent_stroke": "#E5E9F0",
        "agent_text": "#0B0C10",
        "wall_fill": "#3B4252",           # regular wall
        "fake_wall_fill": "#5E81AC",      # fake wall
        "def_wall_fill": "#BF616A",       # definitely-a-wall
        "empty_fill": "#10131C",
        "text_fallback": "#D8DEE9",
    }

    def __init__(self, env: Any, root: Optional[tk.Misc] = None, *, cell_px: int = 40) -> None:
        self.env = env
        self.root: Optional[tk.Misc] = root
        self.cell_px = max(8, int(cell_px))

        self.canvas: Optional[tk.Canvas] = None
        self._font_cache: Dict[Tuple[int, int], tkfont.Font] = {}
        self._pending_resize = False

        if self.root is not None:
            self._ensure_canvas()
            self.update()

    # ---------- Public API ----------
    def update(self) -> None:
        """Full repaint of the matrix."""
        if self.canvas is None:
            return

        matrix: List[List[List[Any]]] = self.env.level_matrix
        rows = len(matrix)
        cols = len(matrix[0]) if rows else 0

        self._resize_canvas(cols, rows)
        cv = self.canvas
        cv.delete("all")

        # Grid cells with semantic fills
        for r in range(rows):
            for c in range(cols):
                x1, y1, x2, y2 = self._cell_bounds(c, r)
                fill, outline = self._cell_style(matrix[r][c])
                cv.create_rectangle(x1, y1, x2, y2, outline=outline, fill=fill)

        # Agents on top
        for r in range(rows):
            for c in range(cols):
                for obj in matrix[r][c]:
                    if obj is None:
                        continue
                    if isinstance(obj, AgentConstruct):
                        label = getattr(obj, "name", "") or "A"
                        self._draw_agent(c, r, label)

        cv.update_idletasks()

    def set_root(self, root: tk.Misc) -> None:
        """Attach a Tk root after construction."""
        self.root = root
        self._ensure_canvas()
        self.update()

    # ---------- Internals ----------
    def _ensure_canvas(self) -> None:
        if self.canvas is not None or self.root is None:
            return
        rows = len(self.env.level_matrix)
        cols = len(self.env.level_matrix[0]) if rows else 0
        w = max(1, cols * self.cell_px)
        h = max(1, rows * self.cell_px)
        self.canvas = tk.Canvas(
            self.root, width=w, height=h,
            bg=self._COLORS["app_bg"], highlightthickness=0
        )
        # Resizable container
        self.canvas.pack(fill="both", expand=True)
        # Debounced resize handling for responsiveness
        self.canvas.bind("<Configure>", self._on_configure)

    def _on_configure(self, event: tk.Event) -> None:
        """Debounce resize to avoid redundant repaints on continuous drag."""
        if self._pending_resize:
            return
        self._pending_resize = True
        # Finish after idle so geometry has settled
        assert self.canvas is not None
        self.canvas.after_idle(self._finish_resize)

    def _finish_resize(self) -> None:
        self._pending_resize = False
        if self.canvas is None:
            return
        matrix: List[List[List[Any]]] = self.env.level_matrix
        rows = len(matrix)
        cols = len(matrix[0]) if rows else 0
        if rows == 0 or cols == 0:
            return

        # Compute new cell size from available canvas pixel area
        w = max(1, int(self.canvas.winfo_width()))
        h = max(1, int(self.canvas.winfo_height()))
        cell_w = max(8, w // cols)
        cell_h = max(8, h // rows)
        self.cell_px = max(8, min(cell_w, cell_h))

        # Snap canvas size to the grid so no partial cells appear
        self.canvas.config(width=cols * self.cell_px, height=rows * self.cell_px)
        self._font_cache.clear()  # font depends on cell_px
        self.update()

    def _resize_canvas(self, cols: int, rows: int) -> None:
        if self.canvas is None:
            return
        self.canvas.config(
            width=max(1, cols * self.cell_px),
            height=max(1, rows * self.cell_px)
        )

    def _cell_bounds(self, col: int, row: int) -> Tuple[int, int, int, int]:
        x1 = col * self.cell_px
        y1 = row * self.cell_px
        x2 = x1 + self.cell_px
        y2 = y1 + self.cell_px
        return x1, y1, x2, y2

    def _cell_style(self, cell_objs: List[Any]) -> Tuple[str, str]:
        """
        Return (fill, outline) for a cell based on semantic content.

        Precedence
        ----------
        - DefinitelyAWall
        - Wall
        - FakeWall
        - Empty
        """
        C = self._COLORS
        fill = C["empty_fill"]
        outline = C["grid_line"]

        # Precedence checks
        has_def = any(isinstance(o, DefinitelyAWall) for o in cell_objs if o is not None)
        if has_def:
            return (C["def_wall_fill"], C["cell_outline"])

        has_wall = any(isinstance(o, Wall) for o in cell_objs if o is not None)
        if has_wall:
            return (C["wall_fill"], C["cell_outline"])

        has_fake = any(isinstance(o, FakeWall) for o in cell_objs if o is not None)
        if has_fake:
            return (C["fake_wall_fill"], C["cell_outline"])

        return (fill, outline)

    def _draw_agent(self, col: int, row: int, label: str) -> None:
        """
        Render a filled circle with a compact, auto-fitting label.
        """
        x1, y1, x2, y2 = self._cell_bounds(col, row)
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        r = self.cell_px * 0.38  # slight increase for readability

        C = self._COLORS
        assert self.canvas is not None
        self.canvas.create_oval(
            cx - r, cy - r, cx + r, cy + r,
            fill=C["agent_fill"], outline=C["agent_stroke"], width=max(1, int(self.cell_px * 0.03))
        )

        short = self._short_label(label)
        font = self._fit_font(short)
        self.canvas.create_text(
            cx, cy, text=short, fill=C["agent_text"], font=font
        )

    def _short_label(self, label: str) -> str:
        """
        Keep labels short and clear. Prefer up to 4 glyphs. Fall back to initials.
        """
        txt = (label or "A").strip()
        if len(txt) <= 4:
            return txt
        # Try initials if name contains separators
        parts = [p for p in txt.replace("_", " ").replace("-", " ").split() if p]
        if len(parts) >= 2:
            initials = "".join(p[0] for p in parts)[:4]
            if initials:
                return initials
        return txt[:4]

    def _fit_font(self, text: str) -> tkfont.Font:
        """
        Choose a bold Tk font that fits within the agent disc without overflow.
        Uses caching keyed by (len(text), cell_px).
        """
        key = (len(text), self.cell_px)
        hit = self._font_cache.get(key)
        if hit:
            return hit

        # Max width budget ~80% of the cell to avoid touching the stroke
        max_w = int(self.cell_px * 0.80)
        size = max(6, int(self.cell_px * 0.42))
        f = tkfont.Font(family="TkDefaultFont", size=size, weight="bold")
        # Decrease until it fits
        while size > 6 and f.measure(text) > max_w:
            size -= 1
            f = tkfont.Font(family="TkDefaultFont", size=size, weight="bold")
        self._font_cache[key] = f
        return f
