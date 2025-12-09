from simulation import pyactrFunctionalityExtension
import pyactr as actr

from simulation.pyactrFunctionalityExtension import request_if_production_fired


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
        self.agent_start_position = (20, 1)

        # Precomputed boundaries relative to the initial center.
        self.top_row = self.agent_start_position[0] - 19
        self.bottom_row = self.agent_start_position[0]
        self.left_column = self.agent_start_position[1]
        self.right_column = self.agent_start_position[1] + 23

        self.dynamic_productions = {}

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
        self.goal_phases = ["locate", "pathfinding", "moving", "eval", "goal", "retrieval"]
        self.prod = request_if_production_fired(self.agent_construct)

        # Only process relevant productions
        if self.prod not in (f"{self.goal_phases[0]}_self", f"{self.goal_phases[0]}_obstacles", f"{self.goal_phases[0]}_goal", f"{self.goal_phases[1]}_fast_path", f"{self.goal_phases[1]}_safe_path", f"{self.goal_phases[3]}_evalUp", f"{self.goal_phases[3]}_evalDown", f"{self.goal_phases[3]}_evalRight", f"{self.goal_phases[3]}_evalLeft", f"{self.goal_phases[4]}_reached"):
            return

        # Acquire current visual stimuli from the agent
        self.stimuli = self.agent_construct.visual_stimuli
        print(self.stimuli)
    
    def locate_self(self):
        if self.prod == f"{self.goal_phases[0]}_self":
            # Determine agent's position
            self.current_pos = self._find_symbol_position(self.stimuli, "A")
            if self.current_pos is None:
                print("Warning: agent symbol 'A' not found in stimuli.")
                return
            pyactrFunctionalityExtension.set_imaginal(self.actr_agent, actr.makechunk(typename="Agent", current_pos_x=self.current_pos[0], current_pos_y=self.current_pos[1]), "Agent")
            
    def locate_obstacles(self):
        obstacles = []
        if self.prod == f"{self.goal_phases[0]}_obstacles":
            # Determine obstacles' positions
            obstacles.append(self._find_symbol_position(self.stimuli, "Z"))
            if obstacles is None:
                print("Warning: obstacle symbol 'Z' not found in stimuli.")
                return
            i=0
            for obs in obstacles:
                pyactrFunctionalityExtension.add_to_declarative_memory(self.actr_agent, actr.makechunk(typename="obstacle", pos_x=obs[0], pos_y=obs[1], state="unknown"))
                
                production_name = f"{self.goal_phases[5]}_obstacle_{i}_request_solid"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}SearchForSolidObstacle
                        ==>
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveSolidObstacle
                        +retrieval>
                        isa     obstacle
                        pos_x     {obs[0]}
                        pos_y     {obs[1]}
                        status  solid
                        """
                    
                    self.actr_agent.productionstring(name=production_name, string=production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                    
                production_name = f"{self.goal_phases[5]}_obstacle_{i}_request_positive"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveSolidObstacle
                        =retrieval>
                        isa     obstacle
                        pos_x     {obs[0]}
                        pos_y     {obs[1]}
                        status  solid
                        ==>
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}SolidObstacleRetrieved
                        """
                    
                    self.actr_agent.productionstring(name=production_name, string=production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                    
                production_name = f"{self.goal_phases[5]}_obstacle_{i}_request_negative"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveSolidObstacle
                        ?retrieval>
                        state   error
                        ==>
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}RetrievalFailed
                        """
                    
                    self.actr_agent.productionstring(name=production_name, string=production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                i += 1
            
    def locate_goal(self):
        if self.prod == f"{self.goal_phases[0]}_goal":
            # Determine goal's position
            goal_pos = self._find_symbol_position(self.stimuli, "T")
            if goal_pos is None:
                print("Warning: goal symbol 'T' not found in stimuli.")
                return
            pyactrFunctionalityExtension.set_imaginal(self.actr_agent, actr.makechunk(typename="Agent", goal_pos_x=goal_pos[0], goal_pos_y=goal_pos[1]), "Agent")

    def a_star_fast_path(self):
        if self.prod == f"{self.goal_phases[1]}_fast_path":
            '''Implement A* algorithm for fastest path - resulting in a list of coordinates for the shortest path
            for every coordinate retrieve if an solid obstacle is there, if so, safe this coordinate in a list
            and start pathfinding without this coordinate again till no solid obstacle on path is found.
            Since no solid obstacles are on the path, set the goal to moving phase (phase: f"{self.goal_phases[2]}", state: f"{self.goal_phases[2]}PathDecisionFinished")'''
            goal_pos = pyactrFunctionalityExtension.get_imaginal(self.actr_agent, f"{self.goal_phases[0]}_goal")
            pass

    def a_star_safe_path(self):
        if self.prod == f"{self.goal_phases[1]}_safe_path":
            # Implement A* algorithm for safest path
            
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}PathDecisionFinished"))
            pass

    def decideDirection(self):
        if self.prod == f"{self.goal_phases[2]}_decide_direction":
            '''With the list of coordinates without solid obstacles from pathfinding, calculate a queue of directionsteps (up, down, left, right) and set goal accordingly for each step.
            (phase: f"{self.goal_phases[3]}", state: f"{self.goal_phases[3]}MoveUp/Down/Left/Right")'''
            pass

    def evalUp(self):
        if self.prod == "evalUp":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}NextStep"))
            # Evaluate upward movement
    
    def evalDown(self):
        if self.prod == "evalDown":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}NextStep"))
            # Evaluate downward movement

    def evalRight(self):
        if self.prod == "evalRight":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}NextStep"))
            # Evaluate rightward movement

    def evalLeft(self):
        if self.prod == "evalLeft":
            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}NextStep"))
            # Evaluate leftward movement

    def goal_reached(self):
        if self.prod == "goal_reached":
            '''Update current goal position to new start position and current start position to goal position. Than start over again, such that the agent can show that it has learned.'''

            pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[0]}", state=f"{self.goal_phases[0]}Goal"))
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

        '''If Bump is detected update obstacle chunk on this position to state "solid".
        After this go back do pathfinding an search for new fastest path with updated obstacle information.
        After 5 times bump change path finding strategy to safe path (avoid all obstacles without testing them).
        Naturally forgetting strategy after some time to allow testing obstacles again.
        Then the number of bumps allowed should be reduced since the agent should have learned about this circumstance'''
        pass
