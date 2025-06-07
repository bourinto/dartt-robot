"""Finite State Machine implementation used by various robot controllers."""

import sys


class FiniteStateMachine:
    """Simple finite state machine engine."""

    def __init__(self):
        self.transitions = {}
        self.states = []
        self.events = []
        self.curState = None
        self.curEvent = None
        self.prevState = None
        self.endState = None

    def str2fun(self, astr):
        """Return the function object from a module-qualified string."""
        module, _, function = astr.rpartition('.')
        if module:
            __import__(module)
            mod = sys.modules[module]
        else:
            mod = sys.modules['__main__']  # or whatever's the "default module"
        return getattr(mod, function)

    def load_fsm_from_file(self, file_fsm):
        """Load FSM configuration from a text file."""
        with open(file_fsm) as ffsm:
            mode = None
            for l in ffsm.readlines():
                l = l.strip()  # remove newline
                if l.startswith("----- States"):
                    mode = "st"
                elif l.startswith("----- Events"):
                    mode = "ev"
                elif l.startswith("----- Transitions"):
                    mode = "tr"
                elif l.startswith("---- Start State"):
                    mode = "ss"
                elif l.startswith("---- Start Event"):
                    mode = "se"
                elif l.startswith("---- End State"):
                    mode = "es"
                else:
                    if mode == "ss":
                        self.curState = l
                    if mode == "es":
                        self.endState = l
                    if mode == "se":
                        self.curEvent = l
                    elif mode == "tr":
                        sl = l.split(" ")
                        func = self.str2fun(sl[3])
                        self.add_transition(sl[0], sl[1], sl[2], func)
                    elif mode == "ev":
                        self.add_event(l)
                    elif mode == "st":
                        self.add_state(l)

    def add_transition(self, state1, state2, event, funct):
        """Register a transition between two states."""
        key = state1 + '.' + event
        self.transitions[key] = (state2, funct)

    def add_state(self, state):
        """Register a state."""
        self.states.append(state)

    def add_event(self, event):
        """Register an event."""
        self.events.append(event)

    def set_state(self, state):
        """Set the current state."""
        self.curState = state

    def set_end_state(self, state):
        """Define the end state of the FSM."""
        self.endState = state

    def set_event(self, event):
        """Set the current event."""
        self.curEvent = event

    def run(self):
        """Perform one transition and return the associated action."""
        event = self.curEvent
        state = self.curState
        key = state + '.' + event
        self.prevState = state
        self.curState = self.transitions[key][0]
        if self.prevState != self.curState:
            st = "Transition - Old State : " + state + "; Event : " + event + "; New state : " + self.curState
            st = st + "; Action : " + self.transitions[key][1].__name__ + "()"
            print(st)
        return self.transitions[key][1]

    def exe(self):  # fsm loop
        """Execute the FSM until the end state is reached."""
        run = True
        while run:
            funct = self.run()  # function to be executed in the new state
            print("\nCurrent State : ", self.curState)
            if self.curState != self.endState:
                newEvent = funct()  # new event when state action is finished
                print("New Event : ", newEvent)
                if newEvent is None:
                    break
                else:
                    self.set_event(newEvent)  # set new event for next transition
            else:
                funct()
                run = False
