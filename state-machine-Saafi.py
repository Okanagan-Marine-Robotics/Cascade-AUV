from transitions import Machine


class Student(object):
    # Defines all the states the student can be in
    states = ["asleep", "in_class", "studying", "at_home", "oiled_up"]

    # Defines the actions that allow a student to change states
    transitions = [
        ["alarm_rings", "asleep", "at_home"],
        ["catch_the_bus", "at_home", "in_class"],
        ["go_home", "in_class", "at_home"],
        ["sleep", ["at_home", "oiled_up"], "asleep"],
        ["oil_up", "at_home", "oiled_up"],
    ]

    def __init__(self, name):
        self.name = name
        self.hours_studied = 0

        machine = Machine(
            model=self,
            states=Student.states,
            transitions=Student.transitions,
            initial="asleep",
        )

        machine.add_transition(
            "study", ["in_class", "at_home"], "studying", after="studiedHours"
        )
        machine.add_transition("keep_studying", "studying", "=", after="studiedHours")

    def studiedHours(self):
        self.hours_studied += 1
        print(self.name + " has studied for " + str(self.hours_studied) + " hours")


Gokul = Student("Gokul")
print(Gokul.state)

Gokul.alarm_rings()
print(Gokul.state)

Gokul.catch_the_bus()
print(Gokul.state)

Gokul.study()
print(Gokul.state)

Gokul.keep_studying()
print(Gokul.state)
