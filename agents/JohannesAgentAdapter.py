from simulation import pyactrFunctionalityExtension
import pyactr as actr
import heapq

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

        # Initial position of the agent.
        #self.agent_start_position = (20, 1)
        #pyactrFunctionalityExtension.set_imaginal(self.actr_agent, actr.makechunk(typename="Agent", start_pos_x=self.agent_start_position[0], start_pos_y=self.agent_start_position[1]), "imaginal")
        #self.temporary_path = [] # Tupel which have to be checked
        '''# Precomputed boundaries relative to the initial center.
        self.top_row = self.agent_start_position[0] - 19
        self.bottom_row = self.agent_start_position[0]
        self.left_column = self.agent_start_position[1]
        self.right_column = self.agent_start_position[1] + 23'''

        self.dynamic_productions = {}
        self.obstacles = []  # List of obstacle coordinates (tuples)
        self.goal_phases = ["locate", "pathfinding", "moving", "eval", "goal", "retrieval"]
        self.states = ["unknown", "passable", "solid"]
        self.number_of_bumps = 0
        self.bumped = False
        self.move_counter = 0

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
                    return (r, c)
        return None


    def extending_actr(self):
        """
        Supervises ACT-R during production firings.
        Detects vertical movement, boundary events,
        and updates internal state accordingly.
        """

        self.actr_agent = self.agent_construct.actr_agent
        self.prod = get_production_fired(self.agent_construct)
        self.decMem = {}

        if (
            len(pyactrFunctionalityExtension.get_goal(self.agent_construct)) == 0
            or len(pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "imaginal_agent")) == 0
            or len(pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "path_and_obs_imaginal")) == 0
        ):
            print(pyactrFunctionalityExtension.get_goal(self.agent_construct))
            print(pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "imaginal_agent"))
            print(pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "path_and_obs_imaginal"))
            return
        print(pyactrFunctionalityExtension.get_goal(self.agent_construct))
        print(pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "imaginal_agent"))
        print(pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "path_and_obs_imaginal"))

        # Only process relevant productions
        if self.prod not in (f"{self.goal_phases[0]}_self", f"{self.goal_phases[0]}_obstacles", f"{self.goal_phases[0]}_goal", f"{self.goal_phases[1]}_fast_path", f"{self.goal_phases[1]}_safe_path", f"{self.goal_phases[3]}_evalUp", f"{self.goal_phases[3]}_evalDown", f"{self.goal_phases[3]}_evalRight", f"{self.goal_phases[3]}_evalLeft", f"{self.goal_phases[4]}_reached"):
            return

        # Acquire current visual stimuli from the agent
        self.stimuli = self.agent_construct.visual_stimuli
        #print(self.stimuli)

        goal = get_goal_chunk(self.agent_construct)
        imaginal_agent = get_imaginal_agent_chunk(self.agent_construct)
        path_and_obs_imaginal = get_path_and_obs_imaginal_chunk(self.agent_construct)

        self.locate_self(imaginal_agent)
        self.locate_obstacles()
        self.locate_goal(imaginal_agent)
        self.a_star_fast_path(goal, imaginal_agent)

        # Putting the modified chunks back into the buffers
        set_imaginal_agent_chunk(self.agent_construct, "agent", imaginal_agent)
        set_path_and_obs_imaginal_chunk(self.agent_construct, "path_and_obs", path_and_obs_imaginal)
        set_goal_chunk(self.agent_construct, "goal", goal)
    
    def locate_self(self, imaginal_agent):
        if self.prod == f"{self.goal_phases[0]}_self":
            print("Adapter locate self + self.prod: ", self.prod)
            # Determine agent's position
            self.current_pos = self._find_symbol_position(self.stimuli, "A")
            print(self.current_pos)
            if self.current_pos is None:
                print("Warning: agent symbol 'A' not found in stimuli.")
                return
            imaginal_agent.current_pos_x = self.current_pos[0]
            imaginal_agent.current_pos_y = self.current_pos[1]

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
            i=0
            for obs in self.obstacles:
                pyactrFunctionalityExtension.add_to_declarative_memory(self.agent_construct, actr.chunkstring(f"obstacle_{i}", f"isa obstacle obstacle_pos_x {obs[0]} obstacle_pos_y {obs[1]} status {self.states[0]}"))
                production_name = f"{self.goal_phases[5]}_obstacle_request_solid_{i}"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}SearchForSolidObstacle
                        =path_and_obs_imaginal>
                        check_obstacle_pos_x   {obs[0]}
                        check_obstacle_pos_y   {obs[1]}
                        ==>
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveSolidObstacle
                        +retrieval>
                        isa     obstacle
                        obstacle_pos_x     {obs[0]}
                        obstacle_pos_y     {obs[1]}
                        status  solid
                        """
                    pyactrFunctionalityExtension.add_production(self.agent_construct, production_name, production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.

                production_name = f"{self.goal_phases[5]}_obstacle_request_solid_positive_{i}"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveSolidObstacle
                        =retrieval>
                        isa     obstacle
                        obstacle_pos_x     {obs[0]}
                        obstacle_pos_y     {obs[1]}
                        status  solid
                        ==>
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}SolidObstacleRetrieved
                        """
                    
                    pyactrFunctionalityExtension.add_production(self.agent_construct, production_name, production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                    
                production_name = f"{self.goal_phases[5]}_obstacle_request_solid_negative_{i}"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveSolidObstacle
                        ?retrieval>
                        state   error
                        ==>
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}SolidObstacleRetrievalFailed
                        """
                    
                    pyactrFunctionalityExtension.add_production(self.agent_construct, production_name, production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.

                production_name = f"{self.goal_phases[5]}_obstacle_request_unknown_{i}"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}SearchForUnknownObstacle
                        =path_and_obs_imaginal>
                        check_obstacle_pos_x     {obs[0]}
                        check_obstacle_pos_y     {obs[1]}
                        ==>
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveUnknownObstacle
                        +retrieval>
                        isa     obstacle
                        obstacle_pos_x     {obs[0]}
                        obstacle_pos_y     {obs[1]}
                        status  unknown
                        """
                    
                    pyactrFunctionalityExtension.add_production(self.agent_construct, production_name, production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.

                production_name = f"{self.goal_phases[5]}_obstacle_request_unknown_positive_{i}"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveUnknownObstacle
                        =retrieval>
                        isa     obstacle
                        obstacle_pos_x     {obs[0]}
                        obstacle_pos_y     {obs[1]}
                        status  unknown
                        ==>
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}UnknownObstacleRetrieved
                        """
                    
                    pyactrFunctionalityExtension.add_production(self.agent_construct, production_name, production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                    
                production_name = f"{self.goal_phases[5]}_obstacle_request_unknown_negative_{i}"
                # CRUCIAL! Skip if the production already exists. Otherwise, the utility will be overwritten!
                if production_name not in self.actr_agent.productions:
                    production_string = f"""
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}StartToRetrieveUnknownObstacle
                        ?retrieval>
                        state   error
                        ==>
                        =g>
                        isa     goal
                        phase   {self.goal_phases[5]}
                        state   {self.goal_phases[5]}UnknownObstacleRetrievalFailed
                        """
                    
                    pyactrFunctionalityExtension.add_production(self.agent_construct, production_name, production_string, utility=1.0)
                    self.dynamic_productions[production_name] = 0.0 # Initially 0, because no utility was learned.
                i += 1
            # print("after adding all productions: ", pyactrFunctionalityExtension.get_all_productions(self.agent_construct))
            # decmem = pyactrFunctionalityExtension.get_declarative_memory(self.agent_construct)
            # print("decmem: ", decmem)
            
    def locate_goal(self, imaginal_agent):
        if self.prod == f"{self.goal_phases[0]}_goal":
            # Determine goal's position
            goal_pos = self._find_symbol_position(self.stimuli, "T")
            if goal_pos is None:
                print("Warning: goal symbol 'T' not found in stimuli.")
                return
            imaginal_agent.goal_pos_x = goal_pos[0]
            imaginal_agent.goal_pos_y = goal_pos[1]
            

    def a_star_fast_path(self, goal, imaginal_agent):
        if self.prod == f"{self.goal_phases[1]}_fast_path":
            grid = self.stimuli
            rows = len(grid)
            cols = len(grid[0])

            BLOCKED = {"X"}

            start_pos = (int(f"{imaginal_agent.current_pos_y}"), int(f"{imaginal_agent.current_pos_x}"))
            goal_pos = (int(f"{imaginal_agent.goal_pos_y}"), int(f"{imaginal_agent.goal_pos_x}"))
            
            def heuristic(a, b):
                return abs(a[0] - b[0]) + abs(a[1] - b[1])
            
            directions = [(1,0), (-1,0), (0,1), (0,-1)]

            open_set = []
            heapq.heappush(open_set, (0, start_pos))

            came_from = {}
            g_score = {start_pos: 0}

            while open_set:
                _, current = heapq.heappop(open_set)

                # Ziel gefunden → Pfad aufbauen
                if current == goal_pos:
                    path = []
                    node = current
                    while node in came_from:
                        path.append(node)
                        node = came_from[node]
                    path.reverse()

                    self.planned_path = path
                    print("Planned Path: ", self.planned_path)
                    goal.phase = f"{self.goal_phases[2]}"
                    goal.state = f"{self.goal_phases[2]}AdapterPathDecisionFinished"
                    return

                cx, cy = current

                for dx, dy in directions:
                    nx, ny = cx + dx, cy + dy

                    if not (0 <= nx < cols and 0 <= ny < rows):
                        continue

                    # Blockierte Felder (X & Z)
                    if grid[ny][nx] in BLOCKED:
                        continue

                    tentative_g = g_score[current] + 1
                    neighbor = (nx, ny)

                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        priority = tentative_g + heuristic(neighbor, goal_pos)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

            self.planned_path = []
            print("No path found")

            '''Implement A* algorithm for fastest path - resulting in a list of coordinates for the shortest path.
            Taking into account obstacles with state "solid" as unpassable. (temp_list of coordinates of solid obstacles)
            Set goal to retrieval phase: (phase: f"{self.goal_phases[5]}", state: f"{self.goal_phases[5]}SearchForSolidObstacle")
            to retrieve for every coordinate if an solid obstacle is there.'''
            # self.temp_list_of_solid_obstacles = []  # Temporary list to store solid obstacles found during retrieval

            # '''here A*-algorithm'''

            # self.list_of_coordinates = []  # Result from A* algorithm: List of coordinates (tuple) for the fastest path
            # for coordinate in self.list_of_coordinates:
            #     pyactrFunctionalityExtension.set_imaginal(self.actr_agent, actr.makechunk(typename="pathCoordinate", path_coordinate_x=coordinate[0], path_coordinate_y=coordinate[1]), "imaginal")
            #     pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[5]}", state=f"{self.goal_phases[5]}SearchForSolidObstacle"))
            # if len(self.temp_list_of_solid_obstacles) == 0:
            #     # No solid obstacles found, proceed to decide direction
            #     pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}PathDecisionFinished"))
            # else:
            #     # Solid obstacles found, restart pathfinding with updated obstacle information
            #     pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[1]}", state=f"{self.goal_phases[1]}fast_path"))
            # pass
    
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

    def decideDirection(self):
        if self.prod == f"{self.goal_phases[2]}_decide_direction":
            if self.move_counter <= len(self.planned_path)-1:
                current_step = self.planned_path[self.move_counter]
                '''current_pos and goal_pos from imaginal, movement_directions from manual if possible or new variable, self.list_of_coordinates.
                Calculate movement directions from currentpos and list_of_coordinates and add to a queue'''
                if move_direction == UP:
                    pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}MoveUp"))
                    self.move_counter += 1
                elif move_direction == DOWN:
                    pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}MoveDown"))
                elif move_direction == RIGHT:
                    pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}MoveRight"))
                elif move_direction == LEFT:
                    pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[2]}", state=f"{self.goal_phases[2]}MoveLeft"))
                if current_pos == goal_pos:
                    pyactrFunctionalityExtension.set_goal(self.actr_agent, actr.makechunk(typename=f"{self.goal_phases[4]}", state=f"{self.goal_phases[4]}Reached"))

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
        
    def on_bump_detected(self):
        """
        Triggered when the environment signals an impact or blocked movement.
        Extend as needed for environment-specific responses.
        """

        '''If Bump is detected update obstacle chunk in decmem on position where the robot should be to state "solid".
        After this go back, do pathfinding and search for new fastest path with updated obstacle information.
        After 5 times bump change path finding strategy to safe path (avoid all obstacles without testing them).
        Naturally forgetting strategy after some time to allow testing obstacles again.
        Then the number of bumps allowed should be reduced since the agent should have learned about this circumstance'''
        self.bumped = True
        self.number_of_bumps += 1


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