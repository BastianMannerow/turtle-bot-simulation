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

        # Define reasoning phases
        self.goal_phases = ["locate", "pathfinding", "moving", "eval", "goal"]

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
        actr.chunktype("goal", "phase, state, goal_pos_x, goal_pos_y")

        # Imaginal Chunk Type for imaginal_agent
        actr_agent.chunktype("agent", "current_pos_x, current_pos_y, goal_pos_x, goal_pos_y")

        # Imaginal Chunk Type for path and obstacles
        actr_agent.chunktype("path_and_obs", "obstacle_pos_x, obstacle_pos_y, status, next_pos_x, next_pos_y")

        # DecMem Chunk Type
        actr_agent.chunktype("obstacle", "obstacle_pos_x, obstacle_pos_y, status")

        actr_agent.chunktype("retrieval", "pos_x, pos_y, status, state")

        # Initial goal chunk (starting cognitive state)
        self.initial_goal = actr.chunkstring(string=f"""
            isa     goal
            phase   {self.goal_phases[0]}
            state   {self.goal_phases[0]}Self
        """)

        # Imaginal
        actr_agent.set_goal(name="imaginal_agent", delay=0)
        actr_agent.set_goal(name="path_and_obs_imaginal", delay=0)

        # Add productions corresponding to the first goal phase
        self.add_init_productions(actr_agent, self.goal_phases[0])
        self.add_pathfinding_productions(actr_agent, self.goal_phases[1])
        # self.add_moving_productions(actr_agent, self.goal_phases[2])
        # self.add_eval_productions(actr_agent, self.goal_phases[3])
        return actr_agent

    # ----------------------------------------------------------------------
    # Cognitive model definition
    # ----------------------------------------------------------------------
    def add_init_productions(self, actr_agent, phase):
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
            name=f"{phase}_self",
            string=f"""
                =g>
                isa     goal
                phase   {phase}
                state   {phase}Self
                ==>
                =g>
                isa     goal
                phase   {phase}
                state   {phase}Obstacles
            """
        )

        actr_agent.productionstring(
            name=f"{phase}_obstacles",
            string=f"""
                =g>
                isa     goal
                phase   {phase}
                state   {phase}Obstacles
                ==>
                =g>
                isa     goal
                phase   {phase}
                state   {phase}Goal
            """
        )

        actr_agent.productionstring(
            name=f"{phase}_goal",
            string=f"""
                =g>
                isa     goal
                phase   {phase}
                state   {phase}Goal
                ==>
                =g>
                isa     {phase}
                state   {phase}Finished
            """
        )

        actr_agent.productionstring(
            name=f"{phase}_finished",
            string=f"""
                =g>
                isa     goal
                phase   {phase}
                state   {phase}Finished
                ==>
                =g>
                isa     goal
                phase   {self.goal_phases[1]}
                state   {self.goal_phases[1]}Start
            """
        )

    def add_pathfinding_productions(self, actr_agent, phase):
        actr_agent.productionstring(
            name=f"{phase}_start",
            string=f"""
                =g>
                isa     goal
                phase   {phase}
                state   {phase}Start
                ==>
                =g>
                isa     goal
                phase   {phase}
                state   {phase}FastPath
            """
        )

        actr_agent.productionstring(
            name=f"{phase}_fast_path",
            string=f"""
                =g>
                isa     goal
                phase   {phase}
                state   {phase}FastPath
                ==>
                =g>
                isa     goal
                phase   {phase}
                state   {phase}FastPathAdapterStart
            """
        )

    # #     actr_agent.productionstring(
    # #         name=f"{phase}_safe_path",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}SafePath
    # #             ==>
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}SafePathAdapterStart
    # #         """
    # #     )

    # # def add_moving_productions(self, actr_agent, phase):

    # #     actr_agent.productionstring(
    # #         name=f"{phase}_move_to_goal",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}AdapterPathDecisionFinished
    # #             ==>
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}NextStep
    # #         """
    # #     )

    # #     actr_agent.productionstring(
    # #         name=f"{phase}_decide_direction",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}NextStep
    # #             ==>
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}NextStepFromAdapterQueue
    # #         """
    # #     )

    # #     # Either up or down, hold in imaginal which direction
    # #     actr_agent.productionstring(
    # #         name=f"{phase}_moveUp",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}MoveUp
    # #             ==>
    # #             =g>
    # #             isa     {self.goal_phases[3]}
    # #             state   {self.goal_phases[3]}EvalUp
    # #             +manual>
    # #             isa _manual
    # #             cmd press_key
    # #             key W
    # #         """
    # #     )

    # #     actr_agent.productionstring(
    # #         name=f"{phase}_moveDown",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}MoveDown
    # #             ==>
    # #             =g>
    # #             isa     {self.goal_phases[3]}
    # #             state   {self.goal_phases[3]}EvalDown
    # #             +manual>
    # #             isa _manual
    # #             cmd press_key
    # #             key S
    # #         """
    # #     )

    # #     actr_agent.productionstring(
    # #         name=f"{phase}_moveRight",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}MoveRight
    # #             ==>
    # #             =g>
    # #             isa     {self.goal_phases[3]}
    # #             state   {self.goal_phases[3]}EvalRight
    # #             +manual>
    # #             isa _manual
    # #             cmd press_key
    # #             key D
    # #         """
    # #     )

    # #     actr_agent.productionstring(
    # #         name=f"{phase}_moveLeft",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}MoveLeft
    # #             ==>
    # #             =g>
    # #             isa     {self.goal_phases[3]}
    # #             state   {self.goal_phases[3]}EvalLeft
    # #             +manual>
    # #             isa _manual
    # #             cmd press_key
    # #             key A
    # #         """
    # #     )

    # # def add_eval_productions(self, actr_agent, phase):
    # #     # Eval
    # #     actr_agent.productionstring(
    # #         name=f"{phase}_evalUp",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}EvalUp
    # #             ?manual>
    # #             state free
    # #             ==>
    # #             =g>
    # #             isa     {phase}Pending
    # #             state   {phase}PendingEvaluation
    # #         """
    # #     )

    # #     actr_agent.productionstring(
    # #         name=f"{phase}_evalDown",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}EvalDown
    # #             ?manual>
    # #             state free
    # #             ==>
    # #             =g>
    # #             isa     {phase}Pending
    # #             state   {phase}PendingDecision
    # #         """
    # #     )

    # #     actr_agent.productionstring(
    # #         name=f"{phase}_evalRight",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}EvalRight
    # #             ?manual>
    # #             state free
    # #             ==>
    # #             =g>
    # #             isa     {phase}Pending
    # #             state   {phase}PendingDecision
    # #         """
    # #     )

    # #     actr_agent.productionstring(
    # #         name=f"{phase}_evalLeft",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}EvalLeft
    # #             ?manual>
    # #             state free
    # #             ==>
    # #             =g>
    # #             isa     {phase}Pending
    # #             state   {phase}PendingDecision
    # #         """
    # #     )


    # # def add_goal_productions(self, actr_agent, phase):
    # #     actr_agent.productionstring(
    # #         name=f"{phase}_reached",
    # #         string=f"""
    # #             =g>
    # #             isa     {phase}
    # #             state   {phase}Reached
    # #             ?manual>
    # #             state free
    # #             =Imaginal>
    # #             isa     Imaginal
    # #             goal_pos_x =goal_pos_x
    # #             goal_pos_y =goal_pos_y
    # #             start_pos_x =start_pos_x
    # #             start_pos_y =start_pos_y
    # #             ==>
    # #             =g>
    # #             isa     {self.goal_phases[1]}
    # #             state   {self.goal_phases[1]}Start
    # #             +Imaginal>
    # #             goal_pos_x =start_pos_x
    # #             goal_pos_y =start_pos_y
    # #             start_pos_x =goal_pos_x
    # #             start_pos_y =goal_pos_y
    # #         """
    # #     )