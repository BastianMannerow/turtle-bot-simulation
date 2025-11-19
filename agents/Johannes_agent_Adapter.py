# ExampleAdapter.py
import pyactr as actr
import heapq
from simulation.environment.Wall import Wall
from simulation.environment.FakeWall import FakeWall
from simulation.environment.DefinitelyAWall import DefinitelyAWall


class ExampleAdapter:

    def __init__(self, agent_construct):
        self.agent = agent_construct
        self.actr_agent = agent_construct.actr_agent
        self.environ = agent_construct.environ
        self.middleman = agent_construct.middleman

        self.current_path = []
        self.destination = None
        self.last_intended = None

    # ------------------------------------------------------------------
    # Called from production via:  +manual> cmd call_adapter action request_step
    # ------------------------------------------------------------------
    def request_step(self):
        agent_pos = self._get_agent_pos()

        # If no destination stored yet → do nothing
        if self.destination is None:
            return

        # Compute new path if needed
        if not self.current_path:
            self.current_path = self._astar(agent_pos, self.destination)

        # If no path → do nothing
        if not self.current_path:
            return

        # Skip current position if included
        if self.current_path[0] == agent_pos:
            self.current_path.pop(0)

        if not self.current_path:
            return

        next_cell = self.current_path.pop(0)
        self.last_intended = next_cell

        direction = self._step_to_direction(agent_pos, next_cell)

        # Write this step into ACT-R movement buffer
        chunk = actr.chunkstring(f"""
            isa movement
            state ready
            direction {direction}
        """)

        self.actr_agent.goal["movement"].add(chunk)

    # ------------------------------------------------------------------
    # Called when ACT-R production sees bump → action bump
    # ------------------------------------------------------------------
    def bump(self):
        if not self.last_intended:
            return

        # Mark the cell as blocked
        blocked = self.last_intended

        # Reset path so a new one will be computed
        self.current_path = []
        self.last_intended = None

    # ------------------------------------------------------------------
    # Utility: convert coordinates to movement direction
    # ------------------------------------------------------------------
    def _step_to_direction(self, cur, nxt):
        r, c = cur
        nr, nc = nxt
        if nr < r:
            return "up"
        if nr > r:
            return "down"
        if nc < c:
            return "left"
        return "right"

    # ------------------------------------------------------------------
    def _get_agent_pos(self):
        for r, row in enumerate(self.environ.level_matrix):
            for c, cell in enumerate(row):
                if self.agent in cell:
                    return (r, c)
        return None

    # ------------------------------------------------------------------
    # A* Implementation
    # ------------------------------------------------------------------
    def _astar(self, start, goal):
        rows = len(self.environ.level_matrix)
        cols = len(self.environ.level_matrix[0])

        def in_bounds(p):
            return 0 <= p[0] < rows and 0 <= p[1] < cols

        def passable(p):
            r, c = p
            for obj in self.environ.level_matrix[r][c]:
                if isinstance(obj, (Wall, DefinitelyAWall)):
                    return False
            return True

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        frontier = [(0, start)]
        came = {start: None}
        g = {start: 0}

        while frontier:
            _, cur = heapq.heappop(frontier)
            if cur == goal:
                break

            r, c = cur
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nxt = (r+dr, c+dc)
                if not in_bounds(nxt):
                    continue
                if not passable(nxt):
                    continue

                cost = g[cur] + 1
                if nxt not in g or cost < g[nxt]:
                    g[nxt] = cost
                    priority = cost + heuristic(nxt, goal)
                    heapq.heappush(frontier, (priority, nxt))
                    came[nxt] = cur

        if goal not in came:
            return []

        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = came[cur]

        return list(reversed(path))
