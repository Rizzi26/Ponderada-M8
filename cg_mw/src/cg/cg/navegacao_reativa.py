import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from cg_interfaces.srv import MoveCmd
from collections import deque

class NavReativa(Node):
    def __init__(self):
        super().__init__('nav_reativa')
        
        # Cliente para o serviço 'move_command'
        self.cli = self.create_client(MoveCmd, 'move_command')
        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Service move_command not available, waiting again...')
        
        # Cria uma requisição para o serviço 'MoveCmd'
        self.req = MoveCmd.Request()
        
        # Inicializa posições visitadas e posição atual
        self.visited_positions = set()
        self.current_position = (0, 0)
        
        # Variável para armazenar a resposta do serviço
        self.response = None

    def move(self, direction):
        self.req.direction = direction
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.done_callback)

    def done_callback(self, future):
        try:
            self.response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.response = None

    def nav(self):
        directions_map = {
            'up': (0, 1),
            'down': (0, -1),
            'left': (-1, 0),
            'right': (1, 0)
        }
        
        queue = deque([(self.current_position, None)])  

        while queue:
            current_position, direction_to_get_here = queue.popleft()
            self.visited_positions.add(current_position)
            
            if direction_to_get_here:
                self.move(direction_to_get_here)
                
                while self.response is None:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                response = self.response
                self.response = None
                
                if response and all(a == b for a, b in zip(response.target_pos, response.robot_pos)):
                    self.get_logger().info("Target reached!")
                    return
                
                if not response or not response.success:
                    continue

                self.get_logger().info(f"Move successful to position {current_position}")
                self.get_logger().info(f"Robot sees: left={response.left}, down={response.down}, up={response.up}, right={response.right}")
            
            for direction, (dx, dy) in directions_map.items():
                new_position = (current_position[0] + dx, current_position[1] + dy)
                if new_position not in self.visited_positions:
                    queue.append((new_position, direction))
        
        self.get_logger().info("Exploration complete, target not found.")

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    
    # Cria um executor dedicado
    executor = SingleThreadedExecutor()
    
    # Cria o nó e adiciona ao executor
    navigator = NavReativa()
    executor.add_node(navigator)
    
    try:
        # Inicia a navegação
        navigator.get_logger().info("Starting navigation...")
        navigator.nav()
    finally:
        navigator.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

# Ponto de entrada do programa
if __name__ == '__main__':
    main()
