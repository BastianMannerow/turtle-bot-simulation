from simulation import pyactrFunctionalityExtension
import pyactr as actr

from simulation.pyactrFunctionalityExtension import get_production_fired


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
        self.states = ["unknown", "solid", "passable"]
        self.goal_phases = ["locate", "pathfinding", "moving", "eval", "goal", "retrieval"]
        self.dynamic_productions = {}


    # def generate_path():
    #     '''Tupel path generieren, filtern des Paths von leeren Feldern (self.agent_construnct.visual_stimuli = "(-)", agent und goal)
    #         jetzt bleiben nur noch Felder mit Obstacle-koordinaten. Hier ACT-R aufrufen, erinnern ob obstacle passable?
    #         Nach der Aktion muss die geprüfte kooridnate removed werden. Solange repeaten bis Liste leer, oder Abbruch durch z.B. bump, unbreakable'''

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
                    return (c, r)
        return None


    def extending_actr(self):
        """
        Supervises ACT-R during production firings.
        Detects vertical movement, boundary events,
        and updates internal state accordingly.
        """

        actr_agent = self.agent_construct
        prod = get_production_fired(self.agent_construct)
        if (
            len(pyactrFunctionalityExtension.get_goal(actr_agent)) == 0
            or len(pyactrFunctionalityExtension.get_imaginal(actr_agent, "imaginal_agent")) == 0
            or len(pyactrFunctionalityExtension.get_imaginal(actr_agent, "path_and_obs_imaginal")) == 0
        ):
            print("alles leer")
            return

        # Only process relevant productions
        if self.prod not in (f"{self.goal_phases[0]}_self", f"{self.goal_phases[0]}_obstacles", f"{self.goal_phases[0]}_goal", f"{self.goal_phases[1]}_fast_path", f"{self.goal_phases[1]}_safe_path", f"{self.goal_phases[3]}_evalUp", f"{self.goal_phases[3]}_evalDown", f"{self.goal_phases[3]}_evalRight", f"{self.goal_phases[3]}_evalLeft", f"{self.goal_phases[4]}_reached"):
            return
        
        # Acquire current visual stimuli from the agent
        self.stimuli = self.agent_construct.visual_stimuli
        #print(self.stimuli)
        self.obstacles = []

        goal = get_goal_chunk(actr_agent)
        imaginal_agent = get_imaginal_agent_chunk(actr_agent)
        path_and_obs_imaginal = get_path_and_obs_imaginal_chunk(actr_agent)

        print("locate_self should be fired now")
        self.locate_self(actr_agent, prod, goal, imaginal_agent, path_and_obs_imaginal)
        self.locate_obstacles()
        self.locate_goal()
        self.a_star_fast_path()
        # self.a_star_safe_path()
        # self.decideDirection()
        # self.evalUp()
        # self.evalDown()
        # self.evalRight()
        # self.evalLeft()
        # self.goal_reached()

        # Putting the modified chunks back into the buffers
        set_imaginal_agent_chunk(actr_agent, "agent", imaginal_agent)
        set_path_and_obs_imaginal_chunk(actr_agent, "path_and_obs_imaginal", path_and_obs_imaginal)
        set_goal_chunk(actr_agent, "goal", goal)

        
    
    def locate_self(self, prod, goal, imaginal_agent, path_and_obs_imaginal):
        if prod == f"{self.goal_phases[0]}_self":
            print("locate_self called")
            # Determine agent's position
            self.current_pos = self._find_symbol_position(self.stimuli, "A")
            print("current_pos: ", self.current_pos)
            if self.current_pos is None:
                print("Warning: agent symbol 'A' not found in stimuli.")
                return
            imaginal_agent.agent_pos_x = self.current_pos[0]
            imaginal_agent.agent_pos_y = self.current_pos[1]
            print("imaginal_agent after locating self: ", imaginal_agent)

    def locate_obstacles(self):
        if self.prod == f"{self.goal_phases[0]}_obstacles":
            # Determine obstacles' positions
            for r, row in enumerate(self.stimuli):
                for c, cell in enumerate(row):
                    if cell == "Z":
                        self.obstacles.append((c, r))
            if self.obstacles is None:
                print("Warning: obstacle symbol 'Z' not found in stimuli.")
                return
            self.i=0
            # # print("obstacles: ", self.obstacles)
            for obs in self.obstacles:
                pyactrFunctionalityExtension.add_to_declarative_memory(self.agent_construct, actr.chunkstring("obstacle", f"obstacle_pos_x={obs[0]} obstacle_pos_y={obs[1]} status={self.states[0]}"))
                production_name = f"{self.goal_phases[5]}_obstacle_{self.i}_request_solid"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}SearchForSolidObstacle
                        =path_and_obs_imaginal>
                        coordinate_x   {obs[0]}
                        coordinate_y   {obs[1]}
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
                    
                production_name = f"{self.goal_phases[5]}_obstacle_{self.i}_request_solid_positive"
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
                    
                production_name = f"{self.goal_phases[5]}_obstacle_{self.i}_request_solid_negative"
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
                        state   {self.goal_phases[5]}SolidObstacleRetrievalFailed
                        """
                    
                    self.actr_agent.productionstring(name=production_name, string=production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.

                production_name = f"{self.goal_phases[5]}_obstacle_{self.i}_request_unknown"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}SearchForUnknownObstacle
                        =path_and_obs_imaginal>
                        current_pos_x     {obs[0]}
                        current_pos_y     {obs[1]}
                        ==>
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveUnknownObstacle
                        +retrieval>
                        isa     obstacle
                        pos_x     {obs[0]}
                        pos_y     {obs[1]}
                        status  unknown
                        """
                    
                    self.actr_agent.productionstring(name=production_name, string=production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.

                production_name = f"{self.goal_phases[5]}_obstacle_{self.i}_request_unknown_positive"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveUnknownObstacle
                        =retrieval>
                        isa     obstacle
                        pos_x     {obs[0]}
                        pos_y     {obs[1]}
                        status  unknown
                        ==>
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}UnknownObstacleRetrieved
                        """
                    
                    self.actr_agent.productionstring(name=production_name, string=production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                    
                production_name = f"{self.goal_phases[5]}_obstacle_{self.i}_request_unknown_negative"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveUnknownObstacle
                        ?retrieval>
                        state   error
                        ==>
                        =g>
                        isa     {self.goal_phases[5]}
                        state   {self.goal_phases[5]}UnknownObstacleRetrievalFailed
                        """
                    
                    self.actr_agent.productionstring(name=production_name, string=production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                self.i += 1
            # # decmem = pyactrFunctionalityExtension.get_declarative_memory(self.agent_construct)
            # # print("decmem after adding obstacle productions: ", decmem)

    def locate_goal(self):
        if self.prod == f"{self.goal_phases[0]}_goal":
            # Determine goal's position
            goal_pos = self._find_symbol_position(self.stimuli, "T")
            if goal_pos is None:
                print("Warning: goal symbol 'T' not found in stimuli.")
                return
            imaginal_agent = pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "imaginal_agent")
            print("imaginal agent chunk: ", imaginal_agent)
            #pyactrFunctionalityExtension.set_imaginal(self.agent_construct, actr.makechunk("agent", goal_pos_x=goal_pos[0], goal_pos_y=goal_pos[1]), "imaginal_agent")
            #imaginal_agent = pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "imaginal_agent")
            #print("imaginal_agent chunk: ", imaginal_agent)


    def a_star_fast_path(self):
        if self.prod == f"{self.goal_phases[1]}_fast_path":
            '''Implement A* algorithm for fastest path - resulting in a list of coordinates for the shortest path.
            Taking into account obstacles with state "solid" as unpassable. (temp_list of coordinates of solid obstacles)
            Set goal to retrieval phase: (phase: f"{self.goal_phases[5]}", state: f"{self.goal_phases[5]}SearchForSolidObstacle")
            to retrieve for every coordinate if an solid obstacle is there.'''
            grid = self.stimuli
            rows = len(grid)
            cols = len(grid[0])
            start = pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "imaginal_agent")
            self.temp_list_of_solid_obstacles = []  # Temporary list to store solid obstacles found during retrieval
            self.list_of_coordinates = []  # Result from A* algorithm: List of coordinates (tuple) for the fastest path

            '''here A*-algorithm'''

            if len(self.temp_list_of_solid_obstacles) == 0:
                # No solid obstacles found, proceed to decide direction
                pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}PathDecisionFinished"))
            else:
                # Solid obstacles found, restart pathfinding with updated obstacle information
                pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[1]}", state=f"{self.goal_phases[1]}fast_path"))
            pass

    # def a_star_safe_path(self):
    #     if self.prod == f"{self.goal_phases[1]}_safe_path":
    #         # Implement A* algorithm for safest path
    #         chunk = pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "imaginal")
    #         agent_current_pos_x = chunk["current_pos_x"]
            
    #         pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}PathDecisionFinished"))
    #         pass

    # def check_for_obstacles(self):
    #     for i in range(len(self.dynamic_productions)//6):
    #         if self.prod == f"{self.goal_phases[5]}_obstacle_{i}_request_solid_positive":
    #             '''add this position to the list of temp_solid obstacles'''
    #             self.temp_list_of_solid_obstacles.append((obs[0], obs[1]))
    #             return

    #         if self.prod == f"{self.goal_phases[5]}_obstacle_{i}_request_solid_negative":
    #             pass

    #         if self.prod == f"{self.goal_phases[5]}_obstacle_{i}_request_unknown_positive":
    #             '''Update obstacle chunk on this position to state "unknown".
    #             After this go back do pathfinding an search for new fastest path with updated obstacle information.'''
    #             pass

    #         if self.prod == f"{self.goal_phases[5]}_obstacle_{i}_request_unknown_negative":
    #             '''Update obstacle chunk on this position to state "unknown".
    #             After this go back do pathfinding an search for new fastest path with updated obstacle information.'''
    #     pass

    # def decideDirection(self):
    #     if self.prod == f"{self.goal_phases[2]}_decide_direction":

    #         '''current_pos and goal_pos from imaginal, movement_directions from manual if possible or new variable, self.list_of_coordinates.
    #         Calculate movement directions from currentpos and list_of_coordinates and add to a queue'''
    #         if move_direction == UP:
    #             pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}MoveUp"))
    #         elif move_direction == DOWN:
    #             pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}MoveDown"))
    #         elif move_direction == RIGHT:
    #             pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}MoveRight"))
    #         elif move_direction == LEFT:
    #             pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}MoveLeft"))
    #         if current_pos == goal_pos:
    #             pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[4]}", state=f"{self.goal_phases[4]}Reached"))

    # '''get current pos from imaginal and test for obstacle on this position, if so update state to "passable" and continue.'''
    # '''(maybe safe obstacle positions on path and just compare currentpos with this positions, if equal then update state to "passable")'''
    # '''remove coordinate of this obstacle from self.obstacles, if nothing appears just continue with steps'''
    

    # def evalUp(self):
    #     if self.prod == "evalUp":
    #         pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}NextStep"))
    #         # Evaluate upward movement
    
    # def evalDown(self):
    #     if self.prod == "evalDown":
    #         pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}NextStep"))
    #         # Evaluate downward movement

    # def evalRight(self):
    #     if self.prod == "evalRight":
    #         pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}NextStep"))
    #         # Evaluate rightward movement

    # def evalLeft(self):
    #     if self.prod == "evalLeft":
    #         pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}NextStep"))
    #         # Evaluate leftward movement

    # def goal_reached(self):
    #     if self.prod == "goal_reached":
    #         '''Update current goal position to new start position and current start position to goal position. Than start over again, such that the agent can show that it has learned.'''
    #         '''reset variables: self.list_of_coords, self.temp_list_of_solid_obstacles (should be empty), self.obstacles (DO NOT TOUCH - empty at the start of the programm), self.queue_of_movement_commands (should be empty at the goal - FIFO)'''

    #         pass
        
    # def on_bump_detected(self):
    #     """
    #     Triggered when the environment signals an impact or blocked movement.
    #     Extend as needed for environment-specific responses.
    #     """

    #     '''If Bump is detected update obstacle chunk on this position to state "solid".
    #     After this go back, do pathfinding and search for new fastest path with updated obstacle information.
    #     After 5 times bump change path finding strategy to safe path (avoid all obstacles without testing them).
    #     Naturally forgetting strategy after some time to allow testing obstacles again.
    #     Then the number of bumps allowed should be reduced since the agent should have learned about this circumstance'''
    #     self.bumpi = True

class dotdict(dict):
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

def mutablechunk(chunk):
    """Converts a chunk into a mutable dotdict"""
    return dotdict(dict(chunk))

def get_goal_chunk(actr_agent):
    return mutablechunk(pyactrFunctionalityExtension.get_goal(actr_agent).pop())

def get_imaginal_agent_chunk(actr_agent):
    return mutablechunk(pyactrFunctionalityExtension.get_imaginal(actr_agent, "imaginal_agent").pop())

def get_path_and_obs_imaginal_chunk(actr_agent):
    return mutablechunk(pyactrFunctionalityExtension.get_imaginal(actr_agent, "path_and_obs_imaginal").pop())

def set_goal_chunk(actr_agent, type, chunk):
    pyactrFunctionalityExtension.set_goal(actr_agent, actr.makechunk(typename=type, **chunk))

def set_imaginal_agent_chunk(actr_agent, type, chunk):
    pyactrFunctionalityExtension.set_imaginal(actr_agent, actr.makechunk(typename=type, **chunk), "imaginal_agent")

def set_path_and_obs_imaginal_chunk(actr_agent, type, chunk):
    pyactrFunctionalityExtension.set_imaginal(actr_agent, actr.makechunk(typename=type, **chunk), "path_and_obs_imaginal")