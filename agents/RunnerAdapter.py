from simulation import pyactrFunctionalityExtension
import pyactr as actr

from simulation.pyactrFunctionalityExtension import production_fired


class RunnerAdapter:
    """
    Adapter supervising the agent's internal movement state.
    Tracks position changes between ACT-R production firings
    and reports vertical movement events.
    """

    def __init__(self, agent_construct):
        """
        Initialize adapter state.

        Parameters
        ----------
        agent_construct : Any
            Reference to the pyACT-R agent object.
        """
        self.agent_construct = agent_construct

        # Initial position of the agent.
        # The task definition ensures A starts in the center (3,3) of a 7×7 grid.
        self.last_position = (4, 4)

        # Precomputed boundaries relative to the initial center.
        # Movement is restricted to rows 2–4.
        self.top_row = self.last_position[0] - 3     # row 2
        self.bottom_row = self.last_position[0] + 3  # row 4

    def _find_agent_position(self, stimuli, symbol="A"):
        """
        Locate the agent's position in the current visual stimulus.

        Parameters
        ----------
        stimuli : list[list[str]]
            Two-dimensional representation of the current environment.
        symbol : str
            Identifier symbol for the agent within the matrix.

        Returns
        -------
        (row, col) tuple or None
        """
        for r, row in enumerate(stimuli):
            for c, cell in enumerate(row):
                if cell == symbol:
                    return (r, c)
        return None

    def extending_actr(self):
        """
        Supervises ACT-R during production firings.
        Detects vertical movement, boundary events,
        and updates internal state accordingly.
        """

        actr_agent = self.agent_construct.actr_agent
        prod = pyactrFunctionalityExtension.production_fired(self.agent_construct)

        # Only process relevant productions
        if prod not in ("evalUp", "evalDown"):
            return

        # Acquire current visual stimuli from the agent
        stimuli = self.agent_construct.visual_stimuli
        print(stimuli)

        # Determine agent's new position
        current_pos = self._find_agent_position(stimuli, "A")
        if current_pos is None:
            print("Warning: agent symbol 'A' not found in stimuli.")
            return

        old_row, old_col = self.last_position
        new_row, new_col = current_pos

        # Detect vertical displacement
        if new_row != old_row:
            new = {"state": "decideUpOrDown"}
            pyactrFunctionalityExtension.set_goal(actr_agent, actr.makechunk(typename="decision", **new))
        else:
            if prod == "evalDown":
                new = {"state": "failDown"}
                pyactrFunctionalityExtension.set_goal(actr_agent, actr.makechunk(typename="decision", **new))
            elif prod == "evalUp":
                new = {"state": "failDown"}
                pyactrFunctionalityExtension.set_goal(actr_agent, actr.makechunk(typename="fail", **new))

        # Boundary detection
        if new_row == self.top_row:
            new = {"state": "successHappy"}
            pyactrFunctionalityExtension.set_goal(actr_agent, actr.makechunk(typename="success", **new))
        if new_row == self.bottom_row:
            new = {"state": "successHappy"}
            pyactrFunctionalityExtension.set_goal(actr_agent, actr.makechunk(typename="success", **new))

        # Commit position update
        self.last_position = current_pos

    def on_bump_detected(self):
        """
        Triggered when the environment signals an impact or blocked movement.
        Extend as needed for environment-specific responses.
        """
        pass
