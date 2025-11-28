import random
from typing import Any, List, Optional, Sequence, Tuple
from simulation.environment.Wall import Wall
from simulation.environment.FakeWall import FakeWall
from simulation.environment.DefinitelyAWall import DefinitelyAWall

def build_level(
    level_type: str,
    height: int,
    width: int,
    agents: Sequence[Any],
    rng: Optional[random.Random] = None
) -> list[list[Any | None]] | None:
    """
    Minimal grid builder: randomly place agents on an empty matrix.

    Intent
    ------
    - Provide a minimal example.
    - Fail fast if the grid cannot accommodate all agents.
    - Keep the API surface small and explicit.

    Parameters
    ----------
    level_type : str
        The level type, which should be created.
    height : int
        Number of rows in the matrix (must be > 0).
    width : int
        Number of columns in the matrix (must be > 0).
    agents : Sequence[Any]
        Agent objects to place. Each agent occupies exactly one cell.
    rng : Optional[random.Random]
        Optional RNG for deterministic placements in tests. Defaults to `random`.

    Returns
    -------
    List[List[Optional[Any]]]
        A `height × width` matrix. Cells contain either `None` or a single agent.

    Raises
    ------
    ValueError
        - If `height` or `width` is non-positive.
        - If `len(agents) > height * width` (not enough free cells).
    """
    if height <= 0 or width <= 0:
        raise ValueError("height and width must be positive integers")

    if level_type == "Exercise 1":
        return exercise_one(height, width, agents, rng)
    elif level_type in ("Exercise 2", "Level 2"):
        return exercise_two(height, width, agents, rng)
    elif level_type == "Real":
        return real_world_environment(agents)

    raise ValueError(f"Unknown level_type: {level_type!r}")


def exercise_one(height: int, width: int, agents: Sequence[Any],
                 rng: Optional[random.Random]) -> List[List[Optional[Any]]]:
    total_cells = height * width
    num_agents = len(agents)
    if num_agents > total_cells:
        raise ValueError(
            f"Not enough space: {num_agents} agents for {total_cells} cells "
            f"({height}×{width})"
        )

    # Create an empty matrix
    matrix: List[List[Optional[Any]]] = [[None for _ in range(width)] for _ in range(height)]

    # Generate a shuffled list of all cell coordinates; take the first N for agents
    rng = rng or random
    coords: List[Tuple[int, int]] = [(r, c) for r in range(height) for c in range(width)]
    rng.shuffle(coords)

    for agent, (r, c) in zip(agents, coords[:num_agents]):
        matrix[r][c] = agent

    return matrix


def exercise_two(
    height: int,
    width: int,
    agents: Sequence[Any],
    rng: Optional[random.Random] = None
) -> List[List[Optional[Any]]]:

    # Feste Größe unabhängig von übergebenen Parametern
    height = 9
    width = 5

    rng = rng or random

    # Matrix erstellen
    matrix: List[List[Optional[Any]]] = [[None for _ in range(width)] for _ in range(height)]

    # Außenrahmen: DefinitelyAWall
    for r in range(height):
        for c in range(width):
            if r == 0 or r == height - 1 or c == 0 or c == width - 1:
                matrix[r][c] = DefinitelyAWall()

    # Agent - exakt 1, falls mehr übergeben wurden → nur den ersten nehmen
    agent = agents[0] if len(agents) > 0 else None

    center_r = height // 2
    center_c = width // 2

    matrix[center_r][center_c] = agent

    # Wandzeilen
    upper_row = center_r - 2   # Zeile 2
    lower_row = center_r + 2   # Zeile 6

    # 75 % Wahrscheinlichkeit: oben FakeWall
    if rng.random() < 0.75:
        upper_cls, lower_cls = FakeWall, Wall
    else:
        upper_cls, lower_cls = Wall, FakeWall

    # Wand füllen
    for c in range(1, width - 1):
        matrix[upper_row][c] = upper_cls()
        matrix[lower_row][c] = lower_cls()

    return matrix



# TODO Johannes
def real_world_environment(agents: Sequence[Any]) -> None:
    return None
