from transitions import Machine
import time

class Robot(object):
    states = [
        'initial_state', 'found_gate', 'reached_gate', 'arrow_state', 
        'found_arrow', 'reached_end_of_arrow', 'reached_buoy', 
        'frequency_state', 'found_frequency', 'reached_frequency', 'end'
    ]

    transitions = [
        ['search_gate', 'initial_state', 'found_gate'],
        ['move_to_gate', 'found_gate', 'reached_gate'],
        ['detect_image', 'reached_gate', 'arrow_state'],
        ['search_arrow', 'arrow_state', 'found_arrow'],
        ['follow_arrow', 'found_arrow', 'reached_end_of_arrow'],
        ['find_buoy', 'reached_end_of_arrow', 'reached_buoy'],
        ['touch_buoy', 'reached_buoy', 'frequency_state'],
        ['search_frequency', 'frequency_state', 'found_frequency'],
        ['move_to_frequency', 'found_frequency', 'reached_frequency'],
        ['surface', 'reached_frequency', 'end'],
    ]

    def __init__(self):
        self.machine = Machine(
            model=self,
            states=Robot.states,
            transitions=Robot.transitions,
            initial='initial_state',
            after_state_change='on_enter_state'
        )
        self.perform_behavior_for_state()

    
    def end(self):
        print("Process ended.")

    def on_enter_state(self):
        print(f"Entered {self.state}")
        time.sleep(1)
        
        self.perform_behavior_for_state()

        if self.state == 'end':
            return
        
        self.perform_action()
        
    def perform_behavior_for_state(self):
        behaviors = {
            'initial_state': self.moving_side_to_side,
            'found_gate': self.move_towards_gate,
            'reached_gate': self.detect_and_go_through_gate,
            'arrow_state': self.looking_for_arrow,
            'found_arrow': self.following_arrow,
            'reached_end_of_arrow': self.find_and_move_towards_buoy,
            'reached_buoy': self.detect_and_touch_buoy,
            'frequency_state': self.looking_for_frequency,
            'found_frequency': self.move_towards_frequency,
            'reached_frequency': self.prepare_to_surface,
        }
        behavior = behaviors.get(self.state)
        if behavior:
            behavior()

    def perform_action(self):
        if self.state == 'end':
            return
        
        actions = {
            'initial_state': self.search_gate,
            'found_gate': self.move_to_gate,
            'reached_gate': self.detect_image,
            'arrow_state': self.search_arrow,
            'found_arrow': self.follow_arrow,
            'reached_end_of_arrow': self.find_buoy,
            'reached_buoy': self.touch_buoy,
            'frequency_state': self.search_frequency,
            'found_frequency': self.move_to_frequency,
        }
        action = actions.get(self.state)
        if action:
            action()
        else:
            print("Action not found for the current state.")


    def moving_side_to_side(self):
        print("Moving side to side and looking for gate")

    def move_towards_gate(self):
        print("Moving towards gate")

    def detect_and_go_through_gate(self):
        print("Detecting image and going through the correct gate")

    def looking_for_arrow(self):
        print("Looking for an arrow")

    def following_arrow(self):
        print("Following arrow")

    def find_and_move_towards_buoy(self):
        print("Finding buoy and moving towards it")

    def detect_and_touch_buoy(self):
        print("Detecting correct image and touching buoy")

    def looking_for_frequency(self):
        print("Looking for frequency")

    def move_towards_frequency(self):
        print("Moving towards frequency")

    def prepare_to_surface(self):
        print("Preparing to surface")
        self.to_end()

    def surface(self):
        print("Surfacing and ending")
        self.to_end() 

    def to_end(self):
        self.trigger('surface')


robot = Robot()

robot.perform_action()