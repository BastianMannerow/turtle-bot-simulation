from simulation import pyactrFunctionalityExtension
import pyactr as actr

from simulation.pyactrFunctionalityExtension import production_fired


class JohannesAgentAdapter:
    """
    Adapter supervising the agent's internal movement state.
    Tracks position changes between ACT-R production firings
    and reports vertical movement events.
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
        Initialize adapter state.

        Parameters
        ----------
        agent_construct : Any
            Reference to the pyACT-R agent object.
        """
        self.agent_construct = agent_construct

        # Initial position of the agent.
        self.last_position = (20, 1)

        # Precomputed boundaries relative to the initial center.
        self.top_row = self.last_position[0] - 19
        self.bottom_row = self.last_position[0]
        self.left_column = self.last_position[1]
        self.right_column = self.last_position[1] + 23

    def _find_symbol_position(self, stimuli, symbol):
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

        self.actr_agent = self.agent_construct.actr_agent
        self.prod = pyactrFunctionalityExtension.production_fired(self.agent_construct)

        # Only process relevant productions
        if self.prod not in ("locate_self", "locate_obstacles", "locate_goal", "start_pathfinding", "fast_path", "safe_path", "moveToGoal", "evalUp", "evalDown", "evalRight", "evalLeft", "check_position_if_fake_obstacle", "goal_reached"):
            return

        # Acquire current visual stimuli from the agent
        self.stimuli = self.agent_construct.visual_stimuli
        print(self.stimuli)
    
    def locate_self(self):
        if self.prod == "locate_self":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="locate", state="locateObstacles"))
            # Determine agent's position
            current_pos = self._find_symbol_position(self.stimuli, "A")
            if current_pos is None:
                print("Warning: agent symbol 'A' not found in stimuli.")
                return
            pyactrFunctionalityExtension.set_imaginal(self.actr_agent, actr.makechunk(typename="position", row=current_pos[0], column=current_pos[1]))
            
    def locate_obstacles(self):
        obstacles = []
        if self.prod == "locate_obstacles":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="locate", state="locateGoal"))
            # Determine obstacles' positions
            obstacles.append(self._find_symbol_position(self.stimuli, "Z"))
            if obstacles is None:
                print("Warning: obstacle symbol 'Z' not found in stimuli.")
                return
            i=0
            for obs in obstacles:
                pyactrFunctionalityExtension.add_to_declarative_memory(self.actr_agent, actr.makechunk(typename="obstacle", row=obs[0], column=obs[1], state="unknown"))
                production_name = f"remember_obstacle_{i}_request_solid"

                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     {phase}
                        state   {phase}DecideToContribute
                        ==>
                        =g>
                        isa     {next_phase}
                        state   {next_phase}start
                        +retrieval>
                        isa     obstacle
                        row     {obs[0]}
                        column  {obs[1]}
                        status  solid
                        """
                    
                    actr_agent.productionstring(name=production_name, string=production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                    if agent_construct.print_actr_construct_trace:
                        print(Fore.BLUE + f"{agent_construct.name} Learned a new production: {production_name}" + Style.RESET_ALL)

                production_name = f"remember_obstacle_{i}_request_positive"

                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     {phase}
                        state   {phase}DecideToContribute
                        =retrieval>
                        isa     obstacle
                        row     {obs[0]}
                        column  {obs[1]}
                        status  solid
                        ==>
                        =g>
                        isa     {next_phase}
                        state   {next_phase}start
                        """
                    
                    actr_agent.productionstring(name=production_name, string=production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                    if agent_construct.print_actr_construct_trace:
                        print(Fore.BLUE + f"{agent_construct.name} Learned a new production: {production_name}" + Style.RESET_ALL)

                production_name = f"remember_obstacle_{i}_request_negative"

                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     {phase}
                        state   {phase}DecideToContribute
                        ?retrieval>
                        state   error
                        ==>
                        =g>
                        isa     {next_phase}
                        state   {next_phase}start
                        """
                    
                    actr_agent.productionstring(name=production_name, string=production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                    if agent_construct.print_actr_construct_trace:
                        print(Fore.BLUE + f"{agent_construct.name} Learned a new production: {production_name}" + Style.RESET_ALL)

                production_name = f"remember_obstacle_{i}_request_failed"

                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     {phase}
                        state   {phase}DecideToContribute
                        ==>
                        =g>
                        isa     {next_phase}
                        state   {next_phase}start
                        """
                    
                    actr_agent.productionstring(name=production_name, string=production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                    if agent_construct.print_actr_construct_trace:
                        print(Fore.BLUE + f"{agent_construct.name} Learned a new production: {production_name}" + Style.RESET_ALL)
                i += 1
            
            
    def locate_goal(self):
        if self.prod == "locate_goal":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="locate", state="startPathFinding"))
            # Determine goal's position
            goal_pos = self._find_symbol_position(self.stimuli, "T")
            if goal_pos is None:
                print("Warning: goal symbol 'T' not found in stimuli.")
                return
            
    def start_pathfinding(self):
        if self.prod == "start_pathfinding":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="path_finding", state="fastPath"))
            # Start pathfinding process
            # A* algorithm for fastest path
            pass

    def a_star_fast_path(self):
        if self.prod == "fast_path":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="moving", state="startMoving"))
            # Implement A* algorithm for fastest path
            pass

    def a_star_safe_path(self):
        if self.prod == "safe_path":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="moving", state="startMoving"))
            # Implement A* algorithm for safest path
            pass

    def moveToGoal(self):
        if self.prod == "moveToGoal":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="decision", state="decideUpOrDownOrRightOrLeft"))
            # Move towards goal based on pathfinding
            pass

    def evalUp(self):
        if self.prod == "evalUp":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="pending", state="pendingDecision"))
            # Evaluate upward movement
            pass
    
    def evalDown(self):
        if self.prod == "evalDown":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="pending", state="pendingDecision"))
            # Evaluate downward movement
            pass

    def evalRight(self):
        if self.prod == "evalRight":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="pending", state="pendingDecision"))
            # Evaluate rightward movement
            pass

    def evalLeft(self):
        if self.prod == "evalLeft":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="pending", state="pendingDecision"))
            # Evaluate leftward movement
            pass

    def check_position_in_front_if_obstacle(self):
        if self.prod == "check_position_in_front_if_obstacle":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="moving", state="continueMoving"))
            # Check if the position is a fake obstacle
            pass

    def goal_reached(self):
        if self.prod == "goal_reached":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename="locate_goal", state="locateGoal"))
            # Handle goal reached event
            pass
        
    def on_bump_detected(self):
        """
        Triggered when the environment signals an impact or blocked movement.
        Extend as needed for environment-specific responses.
        """
        """
        Triggered when the environment signals an impact or blocked movement.
        Extend as needed for environment-specific responses.
        """
        pass
        # when bump is detected, update obstacle chunk on this position to state "solid"
        # also search for new fastes path to goal with updated obstacle information
        # after 10 times bump change path finding strategy to safe path (avoid all obstacles without testing them)
        # naturally forgetting strategy after some time to allow testing obstacles again
        # then the number of bumps allowed should be reduced since the agent should have learned about this circumstance













    def __init__(self, agent_construct):
        """
        Initialize adapter state.

        Parameters
        ----------
        agent_construct : Any
            Reference to the pyACT-R agent object.
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

'''
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
                new = {"state": "failUp"}
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
        """Supervises ACT-R during production firings.
        Detects vertical movement, boundary events,
        and updates internal state accordingly.
        """

        actr_agent = self.agent_construct.actr_agent
        prod = pyactrFunctionalityExtension.production_fired(self.agent_construct)

    def on_bump_detected(self):
        """
        Triggered when the environment signals an impact or blocked movement.
        Extend as needed for environment-specific responses.
        """
        """
        Triggered when the environment signals an impact or blocked movement.
        Extend as needed for environment-specific responses.
        """
        pass
'''