# Example.py
import pyactr as actr


class Example:

    def __init__(self, environ):
        self.environ = environ

        # Main ACT-R agent
        self.actr_agent = actr.ACTRModel(
            environment=self.environ,
            subsymbolic=True,
            motor_prepared=True
        )

        # Goal buffer for phase
        self.actr_agent.set_goal("phase")
        # Movement buffer
        self.actr_agent.set_goal("movement")

        # Adapter reference (set from simulation)
        self.actr_adapter = None


    # ----------------------------------------------------------------------
    # Cognitive model definition (your production style preserved)
    # ----------------------------------------------------------------------
    def add_productions(self, actr_agent, phase):
        """
        Define ACT-R productions using productionstring syntax
        exactly like your provided structure.
        """

        # ------------------------------------------------------------------
        # Initial phase → movement init
        # ------------------------------------------------------------------
        actr_agent.productionstring(
            name="phase_start_to_movement",
            string=f"""
                =g>
                isa     {phase}
                state   {phase}Start
                ?manual>
                state free
            ==>
                =g>
                isa     movement
                state   request_step
            """
        )

        # ------------------------------------------------------------------
        # Request a step from the adapter
        # (Adapter will compute next move and set movement buffer)
        # ------------------------------------------------------------------
        actr_agent.productionstring(
            name="request-step",
            string=f"""
                =g>
                isa movement
                state request_step
                ?manual>
                state free
            ==>
                +manual>
                isa _manual
                cmd call_adapter
                action request_step
                =g>
                isa movement
                state wait_step
            """
        )

        # ------------------------------------------------------------------
        # Adapter has inserted the next step into the movement buffer
        # movement: state ready direction <dir>
        # ------------------------------------------------------------------
        directions = {
            "up": "W",
            "down": "S",
            "left": "A",
            "right": "D",
        }

        # A production for each direction
        for dir_name, key in directions.items():
            actr_agent.productionstring(
                name=f"move_{dir_name}",
                string=f"""
                    =g>
                    isa movement
                    state ready
                    direction {dir_name}
                    ?manual>
                    state free
                ==>
                    +manual>
                    isa _manual
                    cmd press_key
                    key {key}
                    =g>
                    isa movement
                    state request_step
                """
            )

        # ------------------------------------------------------------------
        # Handle bump (adapter marks bump buffer, we react here)
        # ------------------------------------------------------------------
        actr_agent.productionstring(
            name="handle_bump",
            string=f"""
                =bump>
                isa bump
                fired yes
                ?manual>
                state free
            ==>
                +manual>
                isa _manual
                cmd call_adapter
                action bump
                =g>
                isa movement
                state request_step
            """
        )


    # ----------------------------------------------------------------------
    # Mandatory by your simulation: returns the ACT-R model
    # ----------------------------------------------------------------------
    def build_agent(self, agent_list):
        """
        Simulation calls this to finalize the agent and install productions.
        """
        phase_name = "phase"
        self.add_productions(self.actr_agent, phase_name)
        return self.actr_agent
