import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from solution_interfaces.action import HeldItem
from assessment_interfaces.msg import ItemHolder, ItemHolders
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class HeldColourActionServer(Node):

    def __init__(self):
        super().__init__('held_colour_action_server')

        self.already_holding_item = False
        self.is_running_action = False
        self.is_executing_item_id_logic = False
        self.current_held_items = []
        self.current_item_held = ''

        # Fetching current robot_name
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value

        self.action_server = ActionServer(
            self,
            HeldItem,
            'heldItem',
            self.execute_callback
        )

        self.item__holder_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.item_holder_callback,
            10
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing current item colour held request goal')

        # Setting action server execution status to running to prevent unneccessary logic flow runs of item_holder_callback method

        self.is_running_action = True
        while self.current_item_held == '':
            print(f"Awaiting determining colour of item held by {self.robot_name} if holding")
        if (self.current_item_held != 'No Items Found Held'):
            print(f'Currently holding item of colour type: {self.current_item_held}')

            goal_handle.succeed()
            
            result = HeldItem.Result()
            result.data = self.current_item_held


            # Resetting to defaults after succesful execution of goal server node in determing item colour held 
            self.current_held_items = []
            self.current_item_held = ''
            self.is_running_action = False
        return result
    

    def item_holder_callback(self, msg):
        
        if (len(self.current_held_items) == 0) & self.is_running_action:
            
            self.is_executing_item_id_logic = True
            self.current_held_items = msg.data


            for held_item in self.current_held_items:

                if (held_item.robot_id == self.robot_name) & (held_item.holding_item) & (not(self.already_holding_item)):

                    self.already_holding_item = True

                    current_item_held = held_item.item_colour
                    parsed_current_item_held = current_item_held.lower()
                    self.current_item_held = parsed_current_item_held

                    print(f"currently held item colour for {self.robot_name} is: {self.current_item_held}")
                    return

                if held_item.holding_item == False:
                    self.already_holding_item = False

            self.current_item_held = 'No Items Found Held'
            self.is_executing_item_id_logic = False

            
        
def main(args=None):
    rclpy.init(args=args)

    held_colour_action_server = HeldColourActionServer()

    rclpy.spin(held_colour_action_server)

if __name__ == '__main__':
    main()