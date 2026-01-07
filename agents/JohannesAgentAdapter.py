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
        self.move_directions = [(1,0), (-1,0), (0,1), (0,-1)]
        self.number_of_bumps = 0
        self.bumped = False
        self.move_counter = 0
        self.path_coord_counter = 0
        self.temp_solid_obstacle_coords = []
        self.current_temp_solid_obstacle_coords = []

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
            or len(pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "obstacle_update_imaginal")) == 0
        ):
            return
        print(pyactrFunctionalityExtension.get_goal(self.agent_construct))
        print(pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "imaginal_agent"))
        print(pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "path_and_obs_imaginal"))
        print(pyactrFunctionalityExtension.get_imaginal(self.agent_construct, "obstacle_update_imaginal"))

        # Only process relevant productions
        allowed_prods = {
            f"{self.goal_phases[0]}_self",
            f"{self.goal_phases[0]}_obstacles",
            f"{self.goal_phases[0]}_goal",
            f"{self.goal_phases[1]}_start",
            f"{self.goal_phases[1]}_fast_path",
            f"{self.goal_phases[1]}_check_obstacles_on_path",
            f"{self.goal_phases[5]}_obstacle_request_solid_positive",
            f"{self.goal_phases[5]}_obstacle_request_unknown_positive",
            f"{self.goal_phases[5]}_obstacle_request_passable_positive",
            f"{self.goal_phases[5]}_decide_that_obstacle_is_solid",
            f"{self.goal_phases[1]}_safe_path",
            f"{self.goal_phases[2]}_decide_direction",
            f"{self.goal_phases[3]}_evalUp",
            f"{self.goal_phases[3]}_evalDown",
            f"{self.goal_phases[3]}_evalRight",
            f"{self.goal_phases[3]}_evalLeft",
            f"{self.goal_phases[4]}_reached",
        }

        if self.prod not in allowed_prods:
            return
        
        # Acquire current visual stimuli from the agent
        self.stimuli = self.agent_construct.visual_stimuli
        #print(self.stimuli)

        goal = get_goal_chunk(self.agent_construct)
        imaginal_agent = get_imaginal_agent_chunk(self.agent_construct)
        path_and_obs_imaginal = get_path_and_obs_imaginal_chunk(self.agent_construct)
        obstacle_update_imaginal = get_obstacle_update_imaginal_chunk(self.agent_construct)

        self.locate_self(imaginal_agent)
        self.locate_obstacles()
        self.locate_goal(imaginal_agent)
        self.decide_pathfinding_strategy(goal)
        self.a_star_fast_path(goal, imaginal_agent)
        self.a_star_safe_path(goal, imaginal_agent)
        self.check_for_obstacles(goal, path_and_obs_imaginal)
        self.solid_obstacle_retrieved(goal, path_and_obs_imaginal)
        self.unknown_obstacle_retrieved(goal, path_and_obs_imaginal)
        self.passable_obstacle_retrieved(goal)
        # self.add_to_list_of_solid_obstacles(goal, path_and_obs_imaginal)
        self.decideDirection(goal, imaginal_agent, path_and_obs_imaginal)
        self.evalUp(goal, imaginal_agent, path_and_obs_imaginal)
        self.evalDown(goal, imaginal_agent, path_and_obs_imaginal)
        self.evalRight(goal, imaginal_agent, path_and_obs_imaginal)
        self.evalLeft(goal, imaginal_agent, path_and_obs_imaginal)
        self.goal_reached()

        # Putting the modified chunks back into the buffers
        set_imaginal_agent_chunk(self.agent_construct, "agent", imaginal_agent)
        set_path_and_obs_imaginal_chunk(self.agent_construct, "path_and_obs", path_and_obs_imaginal)
        set_goal_chunk(self.agent_construct, "goal", goal)
        set_obstacle_update_imaginal_chunk(self.agent_construct, "obstacle", obstacle_update_imaginal)
    
    def locate_self(self, imaginal_agent):
        if self.prod == f"{self.goal_phases[0]}_self":
            # print("Adapter locate self + self.prod: ", self.prod)
            # Determine agent's position
            self.current_pos = self._find_symbol_position(self.stimuli, "A")
            # print(self.current_pos)
            if self.current_pos is None:
                print("Warning: agent symbol 'A' not found in stimuli.")
                return
            imaginal_agent.current_pos_x = self.current_pos[1]
            imaginal_agent.current_pos_y = self.current_pos[0]
            imaginal_agent.start_pos_x = self.current_pos[1]
            imaginal_agent.start_pos_y = self.current_pos[0]

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
            #print(self.obstacles)
            for obs in self.obstacles:
                pyactrFunctionalityExtension.add_to_declarative_memory(self.agent_construct, actr.chunkstring(f"obstacle_{i}", f"isa obstacle obstacle_pos_x {obs[0]} obstacle_pos_y {obs[1]} status {self.states[0]}"))
            
    def locate_goal(self, imaginal_agent):
        if self.prod == f"{self.goal_phases[0]}_goal":
            # Determine goal's position
            goal_pos = self._find_symbol_position(self.stimuli, "T")
            if goal_pos is None:
                print("Warning: goal symbol 'T' not found in stimuli.")
                return
            imaginal_agent.goal_pos_x = goal_pos[1]
            imaginal_agent.goal_pos_y = goal_pos[0]

    def decide_pathfinding_strategy(self, goal):
        if self.prod == f"{self.goal_phases[1]}_start":
            if self.number_of_bumps >= 3:
                goal.phase = f"{self.goal_phases[1]}"
                goal.state = f"{self.goal_phases[1]}SafePath"
            else:
                goal.phase = f"{self.goal_phases[1]}"
                goal.state = f"{self.goal_phases[1]}FastPath"


    def a_star_fast_path(self, goal, imaginal_agent):
        if self.prod == f"{self.goal_phases[1]}_fast_path":

            self.move_counter = 0
            self.path_coord_counter = 0
            
            grid = self.stimuli
            rows = len(grid)
            cols = len(grid[0])

            BLOCKED = {"X"}
            print("self.temp_solid_obstacle_coords: ", self.temp_solid_obstacle_coords)

            start_pos = (int(f"{imaginal_agent.current_pos_x}"), int(f"{imaginal_agent.current_pos_y}"))
            print("start_pos: ", start_pos)
            goal_pos = (int(f"{imaginal_agent.goal_pos_x}"), int(f"{imaginal_agent.goal_pos_y}"))
            #print(start_pos)
            #print(goal_pos)
            
            def heuristic(a, b):
                return abs(a[0] - b[0]) + abs(a[1] - b[1])

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
                    goal.phase = f"{self.goal_phases[1]}"
                    goal.state = f"{self.goal_phases[1]}CheckObstaclesOnPath"
                    self.current_temp_solid_obstacle_coords = []
                    return

                cx, cy = current

                for dx, dy in self.move_directions:
                    nx, ny = cx + dx, cy + dy

                    if not (0 <= nx < cols and 0 <= ny < rows):
                        continue

                    # Blockierte Felder (X & Z)
                    if grid[ny][nx] in BLOCKED:
                        continue

                    neighbor = (nx, ny)
                    
                    if neighbor in self.temp_solid_obstacle_coords:
                        continue

                    tentative_g = g_score[current] + 1
                    

                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        priority = tentative_g + heuristic(neighbor, goal_pos)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

            self.planned_path = []
            print("No path found")
    
    def a_star_safe_path(self, goal, imaginal_agent):
        if self.prod == f"{self.goal_phases[1]}_safe_path":
            
            self.move_counter = 0
            self.path_coord_counter = 0
            
            grid = self.stimuli
            rows = len(grid)
            cols = len(grid[0])

            BLOCKED = {"X", "Z"}
            print("self.temp_solid_obstacle_coords: ", self.temp_solid_obstacle_coords)

            start_pos = (int(f"{imaginal_agent.current_pos_x}"), int(f"{imaginal_agent.current_pos_y}"))
            print("start_pos: ", start_pos)
            goal_pos = (int(f"{imaginal_agent.goal_pos_x}"), int(f"{imaginal_agent.goal_pos_y}"))
            #print(start_pos)
            #print(goal_pos)
            
            def heuristic(a, b):
                return abs(a[0] - b[0]) + abs(a[1] - b[1])

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
                    self.current_temp_solid_obstacle_coords = []
                    return

                cx, cy = current

                for dx, dy in self.move_directions:
                    nx, ny = cx + dx, cy + dy

                    if not (0 <= nx < cols and 0 <= ny < rows):
                        continue

                    # Blockierte Felder (X & Z)
                    if grid[ny][nx] in BLOCKED:
                        continue

                    neighbor = (nx, ny)
                    
                    if neighbor in self.temp_solid_obstacle_coords:
                        continue

                    tentative_g = g_score[current] + 1
                    

                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        priority = tentative_g + heuristic(neighbor, goal_pos)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

            self.planned_path = []
            print("No path found")

    def check_for_obstacles(self, goal, path_and_obs_imaginal):
        if self.prod == f"{self.goal_phases[1]}_check_obstacles_on_path":
            #print("adapter check for obstacles started")
            
            # clean up path to get only coordinates with obstacles for retrieval
            obstacles = {'Z', 'X'}

            self.path_with_obstacles = [
                (x, y)
                for (x, y) in self.planned_path
                if self.stimuli[y][x] in obstacles
            ]
            #print("decmem: ", pyactrFunctionalityExtension.get_declarative_memory(self.agent_construct))
            if self.path_coord_counter <= len(self.path_with_obstacles)-1:
                check_obstacle_path_coord = self.path_with_obstacles[self.path_coord_counter]
                #print("current planned path step: ", check_obstacle_path_coord)
                goal.phase = f"{self.goal_phases[5]}"
                goal.state = f"{self.goal_phases[5]}SearchForObstacles"
                goal.prev_phase = f"{self.goal_phases[1]}"
                path_and_obs_imaginal.check_obstacle_pos_x = check_obstacle_path_coord[0]
                path_and_obs_imaginal.check_obstacle_pos_y = check_obstacle_path_coord[1]
                #print("after adding all productions: ", pyactrFunctionalityExtension.get_all_productions(self.agent_construct))
            elif self.current_temp_solid_obstacle_coords == None or len(self.current_temp_solid_obstacle_coords) == 0:
                #print("set goal to move to goal")
                goal.phase = f"{self.goal_phases[2]}"
                goal.state = f"{self.goal_phases[2]}AdapterPathDecisionFinished"
                goal.prev_phase = f"{self.goal_phases[1]}"
                self.path_coord_counter = 0
            else:
                goal.phase = f"{self.goal_phases[1]}"
                goal.state = f"{self.goal_phases[1]}Start"
                goal.prev_phase = f"{self.goal_phases[1]}"
                self.path_coord_counter = 0


    def solid_obstacle_retrieved(self, goal, path_and_obs_imaginal):
        if self.prod == f"{self.goal_phases[5]}_obstacle_request_solid_positive":
            if f"{goal.prev_phase}" == f"{self.goal_phases[1]}":
                solid_obstacle_coord = (int(f"{path_and_obs_imaginal.check_obstacle_pos_x}"), int(f"{path_and_obs_imaginal.check_obstacle_pos_y}"))
                self.temp_solid_obstacle_coords.append(solid_obstacle_coord)
                self.current_temp_solid_obstacle_coords.append(solid_obstacle_coord)
                self.path_coord_counter += 1
                goal.phase = f"{self.goal_phases[1]}"
                goal.state = f"{self.goal_phases[1]}CheckObstaclesOnPath"
                goal.prev_phase = f"{self.goal_phases[5]}"
            elif f"{goal.prev_phase}" == f"{self.goal_phases[3]}":
                goal.phase = f"{self.goal_phases[1]}"
                goal.state = f"{self.goal_phases[1]}Start"

    def unknown_obstacle_retrieved(self, goal, path_and_obs_imaginal):
        if self.prod == f"{self.goal_phases[5]}_obstacle_request_unknown_positive":
            if f"{goal.prev_phase}" == self.goal_phases[1]:
                print("pathfinding")
                self.path_coord_counter += 1
                goal.phase = f"{self.goal_phases[1]}"
                goal.state = f"{self.goal_phases[1]}CheckObstaclesOnPath"
                goal.prev_phase = f"{self.goal_phases[5]}"
            elif f"{goal.prev_phase}" == self.goal_phases[3]:
                print("eval")
                if f"{path_and_obs_imaginal.bumped}" == "true":
                    goal.phase = f"{self.goal_phases[5]}"
                    goal.state = f"{self.goal_phases[5]}DecisionSolidObstacle"
                else:
                    goal.phase = f"{self.goal_phases[5]}"
                    goal.state = f"{self.goal_phases[5]}DecisionPassableObstacle"
            

    def passable_obstacle_retrieved(self, goal):
        if self.prod == f"{self.goal_phases[5]}_obstacle_request_passable_positive":
            if f"{goal.prev_phase}" == f"{self.goal_phases[1]}":
                self.path_coord_counter += 1
                goal.phase = f"{self.goal_phases[1]}"
                goal.state = f"{self.goal_phases[1]}CheckObstaclesOnPath"
                goal.prev_phase = f"{self.goal_phases[5]}"
            elif f"{goal.prev_phase}" == f"{self.goal_phases[3]}":
                goal.phase = f"{self.goal_phases[2]}"
                goal.state = f"{self.goal_phases[2]}NextStep"
                
    # def add_to_list_of_solid_obstacles(self, goal, path_and_obs_imaginal):
    #     if self.prod == f"{self.goal_phases[5]}_decide_that_obstacle_is_solid":
    #         #solid_obstacle_coord = (int(f"{path_and_obs_imaginal.check_obstacle_pos_x}"), int(f"{path_and_obs_imaginal.check_obstacle_pos_y}"))
    #         #self.temp_solid_obstacle_coords.append(solid_obstacle_coord)
    #         goal.state = f"{self.goal_phases[5]}ClearObstacleUpdateImaginalSolid"


    def decideDirection(self, goal, imaginal_agent, path_and_obs_imaginal):
        if self.prod == f"{self.goal_phases[2]}_decide_direction":
            self.temp_solid_obstacle_coords = []
            self.current_temp_solid_obstacle_coords = []
            #print("adapter decide direction started")
            self.current_agent_pos = (int(f"{imaginal_agent.current_pos_x}"), int(f"{imaginal_agent.current_pos_y}")) # y, x
            self.goal_agent_pos = (int(f"{imaginal_agent.goal_pos_x}"), int(f"{imaginal_agent.goal_pos_y}")) # y, x
            if self.move_counter <= len(self.planned_path) - 1:
                #print("move counter: ", self.move_counter)
                
                #print("current pos: ", self.current_agent_pos)
                #print("goal_pos: ", self.goal_agent_pos)
                #print("next step: ", self.planned_path[self.move_counter])
                current_step = self.planned_path[self.move_counter] # y, x
                print("current_step: ", current_step)
                print("path_with_obstacles: ", self.path_with_obstacles)
                if current_step in self.path_with_obstacles:
                    print("set next_pos_might_be_obstacle to true")
                    path_and_obs_imaginal.next_pos_might_be_obstacle = "true"
                else:
                    path_and_obs_imaginal.next_pos_might_be_obstacle = "false"

                direction_map = {
                    (1, 0): "RIGHT",
                    (-1, 0): "LEFT",
                    (0, 1): "UP",
                    (0, -1): "DOWN"
                }

                direction = (current_step[0] - self.current_agent_pos[0], self.current_agent_pos[1] - current_step[1])
                #print("calculated direction: ", direction)

                move_direction = direction_map.get(direction)
                # print("decided move direction: ", move_direction)
                # print("current_pos[0]: ", self.current_agent_pos[0])
                # print("current_pos[1]: ", self.current_agent_pos[1])
                # print("imaginal_agent.current_pos_x: ", imaginal_agent.current_pos_x)
                # print("imaginal_agent.current_pos_y: ", imaginal_agent.current_pos_y)
                
                '''current_pos and goal_pos from imaginal, movement_directions from manual if possible or new variable, self.list_of_coordinates.
                Calculate movement directions from currentpos and list_of_coordinates and add to a queue'''
                if move_direction == "UP":
                    goal.phase = f"{self.goal_phases[2]}"
                    goal.state = f"{self.goal_phases[2]}MoveUp"
                    temp_current_pos = (self.current_agent_pos[0], self.current_agent_pos[1] - 1)
                    imaginal_agent.current_pos_y = str(temp_current_pos[1])
                    self.move_counter += 1
                elif move_direction == "DOWN":
                    goal.phase = f"{self.goal_phases[2]}"
                    goal.state = f"{self.goal_phases[2]}MoveDown"
                    temp_current_pos = (self.current_agent_pos[0], self.current_agent_pos[1] + 1)
                    imaginal_agent.current_pos_y = str(temp_current_pos[1])
                    self.move_counter += 1
                elif move_direction == "RIGHT":
                    goal.phase = f"{self.goal_phases[2]}"
                    goal.state = f"{self.goal_phases[2]}MoveRight"
                    temp_current_pos = (self.current_agent_pos[0] + 1, self.current_agent_pos[1])
                    imaginal_agent.current_pos_x = str(temp_current_pos[0])
                    self.move_counter += 1
                elif move_direction == "LEFT":
                    goal.phase = f"{self.goal_phases[2]}"
                    goal.state = f"{self.goal_phases[2]}MoveLeft"
                    temp_current_pos = (self.current_agent_pos[0] - 1, self.current_agent_pos[1])
                    imaginal_agent.current_pos_x = str(temp_current_pos[0])
                    self.move_counter += 1
                path_and_obs_imaginal.check_obstacle_pos_x = str(temp_current_pos[0])
                path_and_obs_imaginal.check_obstacle_pos_y = str(temp_current_pos[1])

            else:
                if self.current_agent_pos == self.goal_agent_pos:
                    print("GOAL REACHED")
                    goal.phase = f"{self.goal_phases[4]}"
                    goal.state = f"{self.goal_phases[4]}Reached"
                    goal.prev_phase = f"{self.goal_phases[3]}"
                else:
                    #print("current pos: ", self.current_agent_pos)
                    #print("goal_pos: ", self.goal_agent_pos)
                    print("current_pos != goal_pos")

    def evalUp(self, goal, imaginal_agent, path_and_obs_imaginal):
        if self.prod == f"{self.goal_phases[3]}_evalUp":
            current_agent_pos = (int(f"{imaginal_agent.current_pos_x}"), int(f"{imaginal_agent.current_pos_y}"))
            if self.bumped == False:
                path_and_obs_imaginal.bumped = "false"
                goal.phase = f"{self.goal_phases[5]}"
                goal.state = f"{self.goal_phases[5]}SearchForObstacles"
            else:
                path_and_obs_imaginal.bumped = "true"
                temp_current_pos = (current_agent_pos[0], current_agent_pos[1] + 1)
                imaginal_agent.current_pos_y = str(temp_current_pos[1])
                goal.phase = f"{self.goal_phases[5]}"
                goal.state = f"{self.goal_phases[5]}SearchForObstacles"
                self.bumped = False
    
    def evalDown(self, goal, imaginal_agent, path_and_obs_imaginal):
        current_agent_pos = (int(f"{imaginal_agent.current_pos_x}"), int(f"{imaginal_agent.current_pos_y}"))
        if self.prod == f"{self.goal_phases[3]}_evalDown":
            if self.bumped == False:
                path_and_obs_imaginal.bumped = "false"
                goal.phase = f"{self.goal_phases[5]}"
                goal.state = f"{self.goal_phases[5]}SearchForObstacles"
            else:
                path_and_obs_imaginal.bumped = "true"
                temp_current_pos = (current_agent_pos[0], current_agent_pos[1] - 1)
                imaginal_agent.current_pos_y = str(temp_current_pos[1])
                goal.phase = f"{self.goal_phases[5]}"
                goal.state = f"{self.goal_phases[5]}SearchForObstacles"
                self.bumped = False

    def evalRight(self, goal, imaginal_agent, path_and_obs_imaginal):
        current_agent_pos = (int(f"{imaginal_agent.current_pos_x}"), int(f"{imaginal_agent.current_pos_y}"))
        if self.prod == f"{self.goal_phases[3]}_evalRight":
            print("self.bumped: ", self.bumped)
            if self.bumped == False:
                path_and_obs_imaginal.bumped = "false"
                goal.phase = f"{self.goal_phases[5]}"
                goal.state = f"{self.goal_phases[5]}SearchForObstacles"
            else:
                path_and_obs_imaginal.bumped = "true"
                temp_current_pos = (current_agent_pos[0] - 1, current_agent_pos[1])
                imaginal_agent.current_pos_x = str(temp_current_pos[0])
                goal.phase = f"{self.goal_phases[5]}"
                goal.state = f"{self.goal_phases[5]}SearchForObstacles"
                self.bumped = False

    def evalLeft(self, goal, imaginal_agent, path_and_obs_imaginal):
        current_agent_pos = (int(f"{imaginal_agent.current_pos_x}"), int(f"{imaginal_agent.current_pos_y}"))
        if self.prod == f"{self.goal_phases[3]}_evalLeft":
            if self.bumped == False:
                path_and_obs_imaginal.bumped = "false"
                goal.phase = f"{self.goal_phases[5]}"
                goal.state = f"{self.goal_phases[5]}SearchForObstacles"
            else:
                path_and_obs_imaginal.bumped = "true"
                temp_current_pos = (current_agent_pos[0] + 1, current_agent_pos[1])
                imaginal_agent.current_pos_x = str(temp_current_pos[0])
                goal.phase = f"{self.goal_phases[5]}"
                goal.state = f"{self.goal_phases[5]}SearchForObstacles"
                self.bumped = False

    def goal_reached(self):
        if self.prod == f"{self.goal_phases[4]}_reached":
            self.move_counter = 0
            self.number_of_bumps = 0
        
    def on_bump_detected(self):
        """
        Triggered when the environment signals an impact or blocked movement.
        Extend as needed for environment-specific responses.
        """
        self.bumped = True
        self.number_of_bumps += 1
        print("bump detected: ", self.bumped)
        
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

def get_obstacle_update_imaginal_chunk(actr_agent):
    return mutablechunk(pyactrFunctionalityExtension.get_imaginal(actr_agent, "obstacle_update_imaginal").pop())

def set_goal_chunk(actr_agent, type, chunk):
    pyactrFunctionalityExtension.set_goal(actr_agent, actr.makechunk(typename=type, **chunk))

def set_imaginal_agent_chunk(actr_agent, type, chunk):
    pyactrFunctionalityExtension.set_imaginal(actr_agent, actr.makechunk(typename=type, **chunk), "imaginal_agent")

def set_path_and_obs_imaginal_chunk(actr_agent, type, chunk):
    pyactrFunctionalityExtension.set_imaginal(actr_agent, actr.makechunk(typename=type, **chunk), "path_and_obs_imaginal")
    
def set_obstacle_update_imaginal_chunk(actr_agent, type, chunk):
    pyactrFunctionalityExtension.set_imaginal(actr_agent, actr.makechunk(typename=type, **chunk), "obstacle_update_imaginal")

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
