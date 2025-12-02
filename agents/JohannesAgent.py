import pyactr as actr


class JohannesAgent:
    """
    Minimal demonstrator agent for the ACT-R architecture.

    Purpose
    -------
    - Serves as a reference implementation for agent structure and initialization.
    - Demonstrates how to define goal states, chunk types, and productions.
    - Can be extended to create domain-specific cognitive models.

    Design
    ------
    - Each agent instance holds its own ACT-R model (`actr_agent`).
    - `goal_phases` define independent reasoning states for clarity.
    - A single phase (`"test"`) is provided as a structural example.

    Attributes
    ----------
    this_agent_key : str | None
        Symbol identifying this agent in the shared simulation dictionary.
    other_agents_key_list : list[str] | None
        Symbols identifying all other agents in the environment.
    environ : Environment
        The pyACT-R environment; currently retained for backward compatibility.
    actr_agent : actr.ACTRModel
        The ACT-R cognitive model controlling this agent.
    goal_phases : list[str]
        Logical task phases used to segment the agent’s goals.
    initial_goal : actr.Chunk
        Default goal chunk that defines the agent’s starting state.
    """

    def __init__(self, environ):
        """
        Initialize the agent with a pyACT-R environment.

        Parameters
        ----------
        environ : actr.Environment
            pyACT-R environment. Deprecated; will be removed in future versions.
        """
        self.this_agent_key = None
        self.other_agents_key_list = None
        self.environ = environ

        # Base ACT-R model setup
        self.actr_agent = actr.ACTRModel(
            environment=self.environ,
            motor_prepared=True,
            automatic_visual_search=False,
            subsymbolic=True
        )

        self.decmem = self.actr_agent.decmem

        # Define reasoning phases
        self.goal_phases = ["init", "search", "move", "decide", "avoid", "walk_through"]

        # Initial goal chunk (starting cognitive state)
        self.initial_goal = actr.chunkstring(string=f"""
            isa     goal
            phase   {self.goal_phases[0]}
            state   {self.goal_phases[0]}
        """)

    # ----------------------------------------------------------------------
    # Agent construction
    # ----------------------------------------------------------------------
    def build_agent(self, agent_list):
        """
        Construct and return a fully initialized ACT-R agent.

        Parameters
        ----------
        agent_list : list[str]
            The symbolic identifiers from `AgentConstruct` used to reference
            self and other agents in the simulation.

        Returns
        -------
        actr.ACTRModel
            A configured ACT-R agent instance ready for simulation.
        """
        self.this_agent_key = agent_list[0]
        self.other_agents_key_list = agent_list[1:]

        actr_agent = self.actr_agent

        # Configure ACT-R parameters
        actr_agent.model_parameters["utility_noise"] = 0  # stochastic exploration
        actr_agent.model_parameters["baselevel_learning"] = False  # disable base-level activation

        # Define goal chunk types (optional, but clarifies agent state structure)
        for phase in self.goal_phases:
            actr.chunktype(phase, "state")
        actr.chunktype("goal", "phase state pos_x pos_y path monitor_agent_movement monitor_obstacles")
        
        # Define decmem chunk types for environment interaction
        actr.chunktype("obstacle", "pos_x pos_y state")

        # Define imaginal chunk type for navigation
        actr.chunktype("self", "start_pos_x start_pos_y current_pos_x current_pos_y reached_goal")


        STATES = ["unknown", "obstacle_fake", "obstacle_solid", "goal_reached", "goal_not_reached"]
        actr_agent.set_goal(name="imaginal", delay=0)
        actr_agent.decmem.add(actr.chunkstring(string=f"isa goal pos_x {3} pos_y {2} state {STATES[4]}"))

        """obstacles = [
            (0,0), (1,0), (2,0), (3,0), (4,0), (5,0), (6,0), (7,0), (8,0), (9,0), (10,0), (11,0), (12,0), (13,0), (14,0), (15,0), (16,0), (17,0), (18,0), (19,0), (20,0), (21,0), (22,0), (23,0), (24,0),
            (0,1), (12,1), (13,1), (14,1), (15,1), (16,1), (17,1), (18,1), (19,1), (20,1), (21,1), (22,1), (23,1), (24,1),
            (0,2), (12,2), (13,2), (14,2), (15,2), (16,2), (17,2), (18,2), (19,2), (20,2), (21,2), (22,2), (23,2), (24,2),
            (0,3), (12,3), (13,3), (14,3), (15,3), (16,3), (17,3), (18,3), (19,3), (20,3), (21,3), (22,3), (23,3), (24,3),
            (0,4), (1,4), (2,4), (3,4), (4,4), (5,4), (6,4), (7,4), (8,4), (12,4), (13,4), (14,4), (15,4), (16,4), (17,4), (18,4), (19,4), (20,4), (21,4), (22,4), (23,4), (24,4),
            (0,5), (1,5), (2,5), (3,5), (4,5), (5,5), (6,5), (7,5), (8,5), (12,5), (13,5), (14,5), (15,5), (16,5), (17,5), (18,5), (19,5), (20,5), (21,5), (22,5), (23,5), (24,5),
            (0,6), (1,6), (2,6), (3,6) (4,6), (5,6), (6,6), (7,6), (8,6), (18,6), (19,6), (20,6),
            (0,7), (1,7), (2,7), (3,7),
            (0,8), (1,8), (2,8), (3,8),
            (0,9), (1,9), (2,9), (3,9),
            (0,10), (1,10), (2,10), (3,10),
            (0,11), (1,11), (2,11), (3,11), (11,11), (12,11), (13,11), (14,11),
            (0,12), (1,12), (2,12), (3,12), (11,12), (12,12), (13,12), (14,12),
            (0,13), (1,13), (2,13), (3,13), (11,13), (12,13), (13,13), (14,13),
            (0,14), (1,14), (2,14), (3,14), (11,14), (12,14), (13,14), (14,14), (17,14), (18,14), (19,14), (20,14), (21,14), (22,14), (23,14), (24,14),
            (0,15), (1,15), (2,15), (3,15), (4,15), (5,15), (6,15), (7,15), (8,15), (17,15), (18,15), (19,15), (20,15), (21,15), (22,15), (23,15), (24,15),
            (0,16), (1,16), (2,16), (3,16), (4,16), (5,16), (6,16), (7,16), (8,16), (17,16), (18,16), (19,16), (20,16), (21,16), (22,16), (23,16), (24,16),
            (0,17), (1,17), (2,17), (3,17), (4,17), (5,17), (6,17), (7,17), (8,17), (21,17), (22,17), (23,17), (24,17),
            (0,18), (21,18), (22,18), (23,18), (24,18),
            (0,19), (10,19), (11,19), (12,19), (13,19), (14,19), (15,19), (16,19), (21,19), (22,19), (23,19), (24,19),
            (0,20), (10,20), (11,20), (12,20), (13,20), (14,20), (15,20), (16,20), (21,20), (22,20), (23,20), (24,20),
            (0,21), (1,21), (2,21), (3,21), (4,21), (5,21), (6,21), (7,21), (8,21), (9,21), (10,21), (11,21), (12,21), (13,21), (14,21), (15,21), (16,21), (17,21), (18,21), (19,21), (20,21), (21,21), (22,21), (23,21), (24,21)
        ]
        # in (x,y) format

        for obstacle in obstacles:
            actr_agent.decmem.add(actr.chunkstring(string=f"isa obstacle pos_x {obstacle[0]} pos_y {obstacle[1]} state unknown"))"""
        
            

        # Add productions corresponding to the first goal phase
        self.add_init_productions(actr_agent)
        self.add_pathfinding_productions(actr_agent)
        self.add_movement_productions(actr_agent)
        self.add_decision_productions(actr_agent)
        self.add_avoidance_productiona(actr_agent)
        self.add_walkthrough_productions(actr_agent)
        return actr_agent

    # ----------------------------------------------------------------------
    # Cognitive model definition
    # ----------------------------------------------------------------------
    def add_init_productions(self, actr_agent):
        """Adds productions that initialize the buffers and the imaginal state"""
        actr_agent.productionstring(
            name="init",
            string=f"""
            =g>
            isa goal
            phase init
            state init
            ==>
            =g>
            isa goal
            phase init
            state locate_self
            monitor_obstacles True
            +imaginal>
            isa self
            start_pos_x 1
            start_pos_y 20
            current_pos_x 1
            current_pos_y 20
            reached_goal False
            """
        )
        actr_agent.productionstring(
            name="locate_self",
            string="""
            =g>
            isa goal
            phase init
            state locate_self
            ==>
            =g>
            isa goal
            phase search
            state path_finding
            """
        )

    def add_pathfinding_productions(self, actr_agent):
        """Adds productions that handle pathfinding to the goal"""
        actr_agent.productionstring(
            name="find_path",
            string="""
            =g>
            isa goal
            phase search
            state path_finding
            ==>
            =g>
            isa goal
            phase search
            state decide_path
            """
        )
        actr_agent.productionstring(
            name="decide_path",
            string="""
            =g>
            isa goal
            phase search
            state decide_path
            ==>
            =g>
            isa goal
            phase search
            state 
            """
        )





















    def add_productions(self, actr_agent, phase):
        """
        Define production rules for the given cognitive phase.

        Parameters
        ----------
        actr_agent : actr.ACTRModel
            The agent’s ACT-R model.
        phase : str
            Label of the goal phase (e.g., "test").

        Notes
        -----
        - The example rule provided here does not change state; it is a placeholder
          to demonstrate syntax and execution.
        - Extend this method to encode actual task behavior.
        """
        actr_agent.productionstring(
            name=f"move right",
            string=f"""
                        =g>
                        isa movement
                        state right
                        ?manual>
                        state free
                        ==>
                        =g>
                        isa movement
                        state up
                        +manual>
                        isa _manual
                        cmd press_key
                        key D
                        """)
        
        actr_agent.productionstring(
            name=f"move up",
            string=f"""
                        =g>
                        isa movement
                        state up
                        ?manual>
                        state free
                        ==>
                        =g>
                        isa movement
                        state right
                        +manual>
                        isa _manual
                        cmd press_key
                        key W
                        """)
        
'''
import pyactr as actr


class Runner:
    """
    Minimal demonstrator agent for the ACT-R architecture.

    Purpose
    -------
    - Serves as a reference implementation for agent structure and initialization.
    - Demonstrates how to define goal states, chunk types, and productions.
    - Can be extended to create domain-specific cognitive models.

    Design
    ------
    - Each agent instance holds its own ACT-R model (`actr_agent`).
    - `goal_phases` define independent reasoning states for clarity.
    - A single phase (`"test"`) is provided as a structural example.

    Attributes
    ----------
    this_agent_key : str | None
        Symbol identifying this agent in the shared simulation dictionary.
    other_agents_key_list : list[str] | None
        Symbols identifying all other agents in the environment.
    environ : Environment
        The pyACT-R environment; currently retained for backward compatibility.
    actr_agent : actr.ACTRModel
        The ACT-R cognitive model controlling this agent.
    goal_phases : list[str]
        Logical task phases used to segment the agent’s goals.
    initial_goal : actr.Chunk
        Default goal chunk that defines the agent’s starting state.
    """

    def __init__(self, environ):
        """
        Initialize the agent with a pyACT-R environment.

        Parameters
        ----------
        environ : actr.Environment
            pyACT-R environment. Deprecated; will be removed in future versions.
        """
        self.this_agent_key = None
        self.other_agents_key_list = None
        self.environ = environ

        # Base ACT-R model setup
        self.actr_agent = actr.ACTRModel(
            environment=self.environ,
            motor_prepared=True,
            automatic_visual_search=False,
            subsymbolic=True
        )

        # Define reasoning phases
        self.goal_phases = ["test"]

        # Initial goal chunk (starting cognitive state)
        self.initial_goal = actr.chunkstring(string=f"""
            isa     {self.goal_phases[0]}
            state   {self.goal_phases[0]}Start
        """)

    # ----------------------------------------------------------------------
    # Agent construction
    # ----------------------------------------------------------------------
    def build_agent(self, agent_list):
        """
        Construct and return a fully initialized ACT-R agent.

        Parameters
        ----------
        agent_list : list[str]
            The symbolic identifiers from `AgentConstruct` used to reference
            self and other agents in the simulation.

        Returns
        -------
        actr.ACTRModel
            A configured ACT-R agent instance ready for simulation.
        """
        self.this_agent_key = agent_list[0]
        self.other_agents_key_list = agent_list[1:]

        actr_agent = self.actr_agent

        # Configure ACT-R parameters
        actr_agent.model_parameters["utility_noise"] = 5  # stochastic exploration
        actr_agent.model_parameters["baselevel_learning"] = False  # disable base-level activation

        # Define goal chunk types (optional, but clarifies agent state structure)
        for phase in self.goal_phases:
            actr.chunktype(phase, "state")

        # Add productions corresponding to the first goal phase
        self.add_productions(actr_agent, self.goal_phases[0])
        return actr_agent

    # ----------------------------------------------------------------------
    # Cognitive model definition
    # ----------------------------------------------------------------------
    def add_productions(self, actr_agent, phase):
        """
        Define production rules for the given cognitive phase.

        Parameters
        ----------
        actr_agent : actr.ACTRModel
            The agent’s ACT-R model.
        phase : str
            Label of the goal phase (e.g., "test").

        Notes
        -----
        - The example rule provided here does not change state; it is a placeholder
          to demonstrate syntax and execution.
        - Extend this method to encode actual task behavior.
        """
        actr_agent.productionstring(
            name="initialDecision",
            string=f"""
                =g>
                isa     {phase}
                state   {phase}Start
                ==>
                =g>
                isa     decision
                state   decideUpOrDown
            """
        )

        # Either up or down, hold in imaginal which direction
        actr_agent.productionstring(
            name="moveUp",
            string=f"""
                =g>
                isa     decision
                state   decideUpOrDown
                ==>
                =g>
                isa     eval
                state   evalUpNow
                +manual>
                isa _manual
                cmd press_key
                key W
            """
        )

        actr_agent.productionstring(
            name="moveDown",
            string=f"""
                =g>
                isa     decision
                state   decideUpOrDown
                ==>
                =g>
                isa     eval
                state   evalDownNow
                +manual>
                isa _manual
                cmd press_key
                key S
            """
        )

        # Eval
        actr_agent.productionstring(
            name="evalUp",
            string=f"""
                =g>
                isa     eval
                state   evalUpNow
                ?manual>
                state free
                ==>
                =g>
                isa     pending
                state   pendingDecision
            """
        )

        actr_agent.productionstring(
            name="evalDown",
            string=f"""
                =g>
                isa     eval
                state   evalDownNow
                ?manual>
                state free
                ==>
                =g>
                isa     pending
                state   pendingDecision
            """
        )

        # Punish
        actr_agent.productionstring(
            name="punishUp",
            string=f"""
                =g>
                isa     fail
                state   failUp
                ==>
                =g>
                isa     {phase}
                state   {phase}Start
            """
        )

        actr_agent.productionstring(
            name="punishDown",
            string=f"""
                =g>
                isa     fail
                state   failDown
                ==>
                =g>
                isa     {phase}
                state   {phase}Start
            """
        )

        # Success
        actr_agent.productionstring(
            name="happy",
            string=f"""
                =g>
                isa     success
                state   successHappy
                ==>
                =g>
                isa     success
                state   successHappy
            """
        )
'''