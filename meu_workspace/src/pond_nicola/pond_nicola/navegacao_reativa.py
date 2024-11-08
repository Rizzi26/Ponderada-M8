import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd


class NavegacaoReativa(Node):
    def __init__(self):
        super().__init__('navegacao_reativa')
        self.get_logger().info("Nó de Navegação Reativa iniciado")

        # Configuração de assinantes e clientes de serviço
        # Exemplo: cliente para o serviço /move_command
        self.cliente_move = self.create_client(MoveCmd, '/move_command')

    def navegar(self):
        # Lógica de navegação reativa (a ser implementada)
        self.get_logger().info("Executando navegação reativa")

def main(args=None):
    rclpy.init(args=args)
    nodo_navegacao_reativa = NavegacaoReativa()

    # Execute a lógica de navegação
    nodo_navegacao_reativa.navegar()

    rclpy.spin(nodo_navegacao_reativa)
    nodo_navegacao_reativa.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
