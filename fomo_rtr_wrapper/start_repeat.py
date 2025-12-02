import sys
import time
import rclpy
import os
from rclpy.node import Node
from vtr_navigation_msgs.msg import MissionCommand, GoalHandle
from vtr_navigation_msgs.srv import GraphState

repeat_msg = MissionCommand()     
repeat_msg.type = MissionCommand.ADD_GOAL
repeat_msg.goal_handle = GoalHandle()
repeat_msg.goal_handle.type = GoalHandle.REPEAT


teach_msg = MissionCommand()     
teach_msg.type = MissionCommand.ADD_GOAL
teach_msg.goal_handle = GoalHandle()
teach_msg.goal_handle.type = GoalHandle.TEACH

begin_msg = MissionCommand()
begin_msg.type = MissionCommand.BEGIN_GOALS


set_root_vertex = MissionCommand()
set_root_vertex.type = MissionCommand.LOCALIZE

class VTR_CLI(Node):
    def __init__(self):
        super().__init__('start_up_vtr')

        self.command_pub = self.create_publisher(MissionCommand, 'mission_command', 10)
        self.graph_srv = self.create_client(GraphState, 'graph_state_srv')
        self.req = GraphState.Request()
    
    def run(self):
        route = self.request_route()
        if len(route) > 0:
            teach_path = sorted(route[0].ids)
            repeat_msg.goal_handle.waypoints = [teach_path[-1]]
            set_root_vertex.vertex = teach_path[0]

        
        self.get_logger().info(f'Starting VTR. IS_MAPPING={os.getenv("IS_MAPPING")}')
        
        self.command_pub.publish(set_root_vertex)
        time.sleep(0.1)
        self.command_pub.publish(teach_msg if int(os.getenv("IS_MAPPING")) == 1 else repeat_msg)
        time.sleep(0.1)
        self.command_pub.publish(begin_msg)


    def request_route(self):
        if not self.graph_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('VTR not available after 10 seconds.')
            return []

        self.future = self.graph_srv.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().graph_state.fixed_routes
    

def main(args=None):
    rclpy.init(args=args)
    node = VTR_CLI()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
