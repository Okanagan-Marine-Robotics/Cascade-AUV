"""
Notes about the current version of automated planning

1. For this version we are commiting to the blue path. If there is an issue with this please tell 
Raph or someone else on the software team.

2. This will only be trying for the gate and buoy task as discussed before. If there are any changes please tell Raph.

3. Currently this build assumes that we are not able to use the paths provided by the competition.
I have implemented a search pattern to go from the gate to the buoy to the finish line. If we
become able to recognize the paths before the competition please tell Raph.

4. This build will try for style points AFTER going through the gate. This means 8 full rotations.
If we need to change this please tell Raph.

5. This build will assume we take the coinflip at the beginning of the course.

6. This build does not have intructions for what to do after completing the buoy task.
"""










import rclpy
from rclpy.node import Node
from cascade_msgs.msg import MovementCommand
from cascade_msgs.srv import FindObject
from cascade_msgs.srv import Status as CascadeStatus 
from cascade_msgs.msg import Classes
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Selector
from py_trees.decorators import Retry
from py_trees.decorators import Repeat
from py_trees import display  
from geometry_msgs.msg import Vector3


#Defining the action behaviours for locomotion

class MovementBehaviour(Behaviour):
    def __init__(self, name, command):
        super(MovementBehaviour, self).__init__(name)
        self.node = rclpy.create_node(f'_{name}_node')
        self.publisher = self.node.create_publisher(MovementCommand, 'movement_command', 10)
        self.client = self.node.create_client(CascadeStatus, '/navigator_status')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'Service /navigator_status not available, waiting again for {name}...')
        self.message_sent = False
        self.command = command

    def setup(self):
        self.logger.debug(f"{self.name}::setup")

    def initialise(self):
        self.logger.debug(f"{self.name}::initialise")

    def update(self):
        self.logger.debug(f"  {self.name} [update()]")
        if not self.message_sent:
            self.publish_movement_command()
            self.message_sent = True

        request = CascadeStatus.Request()
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)

        if self.future.result() is not None:
            response = self.future.result()
            if response.ongoing:
                return Status.RUNNING
            else:
                if response.success:
                    self.message_sent = False
                    return Status.SUCCESS
                else:
                    return Status.FAILURE
        else:
            self.logger.debug(f"  {self.name} [update()] - Service call failed")
            return Status.FAILURE

    def publish_movement_command(self):
        msg = MovementCommand()
        msg.command = self.command
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.set_command_data(msg)
        self.publisher.publish(msg)
        self.logger.debug(f"Published MovementCommand with command: {self.command}")

    def set_command_data(self, msg):
        raise NotImplementedError

    def terminate(self, new_status):
        self.logger.debug(f"  {self.name} [terminate()][{self.status}->{new_status}]")

    def shutdown(self):
        self.node.destroy_node()
        
        
        

class move_1m(MovementBehaviour):
    def __init__(self, name):
        super(move_1m, self).__init__(name, MovementCommand.MOVE_RELATIVE)
    
    def set_command_data(self, msg):
        msg.data0 = Vector3(x=1.0, y=0.0, z=0.0)  # Move forward by 1 meter
        msg.data1 = Vector3(x=0.0, y=0.0, z=0.0)  # No rotation
        
        
class rise(MovementBehaviour):
    def __init__(self, name):
        super(rise, self).__init__(name, MovementCommand.MOVE_RELATIVE)
    
    def set_command_data(self, msg):
        msg.data0 = Vector3(x=0.0, y=0.0, z=0.5)  # Rise by 1 meter
        msg.data1 = Vector3(x=0.0, y=0.0, z=0.0)  # No rotation


class fall(MovementBehaviour):
    def __init__(self, name):
        super(fall, self).__init__(name, MovementCommand.MOVE_RELATIVE)
    
    def set_command_data(self, msg):
        msg.data0 = Vector3(x=0.0, y=0.0, z=-0.5)  # Fall by 1 meter
        msg.data1 = Vector3(x=0.0, y=0.0, z=0.0)  # No rotation


class turncw(MovementBehaviour):
    def __init__(self, name):
        super(turncw, self).__init__(name, MovementCommand.MOVE_RELATIVE)
    
    def set_command_data(self, msg):
        msg.data0 = Vector3(x=0.0, y=0.0, z=0.0)  # No linear movement
        msg.data1 = Vector3(x=0.0, y=0.0, z=-45.0)  # Turn 45 degrees clockwise

class turnccw(MovementBehaviour):
    def __init__(self, name):
        super(turnccw, self).__init__(name, MovementCommand.MOVE_RELATIVE)
    
    def set_command_data(self, msg):
        msg.data0 = Vector3(x=0.0, y=0.0, z=0.0)  # No linear movement
        msg.data1 = Vector3(x=0.0, y=0.0, z=45.0)  # Turn 45 degrees counterclockwise
        
class turn_around(MovementBehaviour):
    def __init__(self, name):
        super(turn_around, self).__init__(name, MovementCommand.MOVE_RELATIVE)
    
    def set_command_data(self, msg):
        msg.data0 = Vector3(x=0.0, y=0.0, z=0.0)  # No linear movement
        msg.data1 = Vector3(x=0.0, y=0.0, z=180.0)  # Turn 180 degrees



class move_to_gate(MovementBehaviour):
    def __init__(self, name):
        super(move_to_gate, self).__init__(name, MovementCommand.GO_TO_GATE)
    
    def set_command_data(self, msg):
        msg.data0 = Vector3(x=1.0, y=0.0, z=0.0)  # Example data for stopping distance
        
class move_to_buoy(MovementBehaviour):
    def __init__(self,name):
        super(move_to_buoy, self).__init__(name, MovementCommand.GO_TO_BUOY)
    
    def set_command_data(self, msg):
        msg.data0 = Vector3(x=1.0, y=0.0, z=0.0)  # Example data for stopping distance
        
        
#This next movement behaviour is for stalling at the start in case we get faced upwards from the coinflip. Insert recentering command.

class stall(MovementBehaviour):
    def __init__(self,name):
        super(stall, self).__init__(name, MovementCommand.MOVE_RELATIVE)
        
    def set_command_data(self, msg): "fill in stall/ recentering command here"

#Defining a condition

class found_gate(Behaviour):
    def __init__(self, name):
        super(found_gate, self).__init__(name)
        self.node = rclpy.create_node('_found_gate_node')  # Create a ROS2 node
        self.client = self.node.create_client(FindObject, '/find_object')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service /find_object not available, waiting again...')

    def setup(self):
        self.logger.debug(f"found_gate::setup{self.name}")

    def initialise(self):
        self.logger.debug(f"found_gate::initialise{self.name}")

    def update(self):
        self.logger.debug("  %s [found_gate::update()]" % self.name)
        
        request = FindObject.Request()
        request.object_type = Classes.GATE  # Specify the object to find

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)

        if self.future.result() is not None:
            response = self.future.result()
            if response.exists:
                self.logger.debug(f"  %s [found_gate::update()] - Found gate" % self.name)
                return Status.SUCCESS
            else:
                self.logger.debug(f"  %s [found_gate::update()] - Gate not found" % self.name)
                return Status.FAILURE
        else:
            self.logger.debug(f"  %s [found_gate::update()] - Service call failed" % self.name)
            return Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s found_gate::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    def shutdown(self):
        self.node.destroy_node()
        
        
class found_buoy(Behaviour):
    def __init__(self, name):
        super(found_buoy, self).__init__(name)
        self.node = rclpy.create_node('_found_buoy_node')  # Create a ROS2 node
        self.client = self.node.create_client(FindObject, '/find_object')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service /find_object not available, waiting again...')

    def setup(self):
        self.logger.debug(f"found_buoy::setup{self.name}")

    def initialise(self):
        self.logger.debug(f"found_buoy::initialise{self.name}")

    def update(self):
        self.logger.debug("  %s [found_buoy::update()]" % self.name)
        
        request = FindObject.Request()
        request.object_type = Classes.BUOY  # Specify the object to find

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, self.future)

        if self.future.result() is not None:
            response = self.future.result()
            if response.exists:
                self.logger.debug(f"  %s [found_buoy::update()] - Found buoy" % self.name)
                return Status.SUCCESS
            else:
                self.logger.debug(f"  %s [found_buoy::update()] - Buoy not found" % self.name)
                return Status.FAILURE
        else:
            self.logger.debug(f"  %s [found_buoy::update()] - Service call failed" % self.name)
            return Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s found_buoy::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    def shutdown(self):
        self.node.destroy_node()
        



#This is where the bahaviour tree is created

class PlannerNode(Node):
    def __init__(self):
        super().__init__ ("Automated_Planner")
    
    def publish(self, msg):
        self.publisher_.publish(msg)

    def subscription_callback(self, msg):
        pass
    
def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()

    root = py_trees.composites.Sequence(name = "root", memory = True)
    
    turnccwsq6 = turnccw("turnccwsq6")
    decorator5 = Repeat(name = "decorator5", child = turnccwsq6, num_success = 2)
    move_1msq6 = move_1m("move_1msq6")
    
    sequence6= Sequence(name = "sequence6", memory = True)
    sequence6.add_child(move_1msq6)
    sequence6.add_child(decorator5)
    
    decorator4 = Repeat(name = "decorator4", child = sequence6, num_success = 4)
    
    
    move_to_buoysq5 = move_to_buoy("move_to_buoysq5")
    turncwsq5 = turncw("turncwsq5")
    
    sequence5 = Sequence(name = "sequence5", memory = True)
    sequence5.add_child(move_to_buoysq5)
    sequence5.add_child(turncwsq5)
    sequence5.add_child(decorator4)
    
    turncwsq4 = turncw("turncwsq4")
    found_buoysq4 = found_buoy("found_buoysq4")
    
    sequence4 = Sequence(name = "sequence4", memory = True)
    sequence4.add_child(turncwsq4)
    sequence4.add_child(found_buoysq4)
    
    decorator3 = Retry(name = "decorator3", child = sequence4, num_failures= 8)
    
    move_1msq3 = move_1m("move_1msq3")
    
    sequence3 = Sequence(name = "sequence3", memory = True)
    sequence3.add_child(move_1msq3)
    sequence3.add_child(decorator3)
    
    decorator2 = Retry(name = "decorator2", child = sequence3, num_failures = 6)
    
    found_buoysq1 = found_buoy("found_buoysq1")
    
    turnccwdc1 = turnccw("turnccwdc1")
    
    decorator1 = Repeat(name = "decorator1", child = turnccwdc1, num_success = 64)
    
    turn_aroundsq2 = turn_around("turnaroundsq2")
    found_gatesq2 = found_gate("foundgatesq2")
    move_to_gatesq2 = move_to_gate("move_to_gatesq2")
    move_1msq2 = move_1m("move_1msq2")
    
    sequence2 = Sequence(name = "sequence2", memory = True)
    sequence2.add_child(turn_aroundsq2)
    sequence2.add_child(found_gatesq2)
    sequence2.add_child(move_to_gatesq2)
    sequence2.add_child(move_1msq2)
    
    found_gatesl1 = found_gate("found_gatesl1")
    
    selector1 = Selector(name = "selector1", memory = True)
    selector1.add_child(found_gatesl1)
    selector1.add_child(sequence2)
    
    stallsq1 = stall("stallsq1")
    
    sequence1 = Sequence(name = "sequence1", memory = True)
    sequence1.add_child(stallsq1)
    sequence1.add_child(selector1)
    sequence1.add_child(decorator1)
    sequence1.add_child(found_buoysq1)
    sequence1.add_child(decorator2)
    sequence1.add_child(sequence5)
    
    root.add_child([sequence1])
    
    behaviour_tree = py_trees.trees.BehaviourTree(
        root=root
    )
    node.get_logger().info(py_trees.display.unicode_tree(root=root))
    behaviour_tree.setup(timeout=15)

    def print_tree(tree):
        #node.get_logger().info(py_trees.display.unicode_tree(root=tree.root, show_status=True))
        pass

    while(rclpy.ok()):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            behaviour_tree.tick_tock(
                period_ms=250,
                number_of_iterations=1,
                pre_tick_handler=print_tree,
                post_tick_handler=None
            )
        except KeyboardInterrupt:
            behaviour_tree.interrupt()
 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
    
