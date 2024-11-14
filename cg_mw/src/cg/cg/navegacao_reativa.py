import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd


class NavegacaoReativa(Node):
    def __init__(self):
        super().__init__('navegacao_reativa')
        self.get_logger().info("Nó de Navegação Reativa iniciado")

        # Configuração do cliente de serviço
        self.cliente_move = self.create_client(MoveCmd, '/move_command')

        # Certifique-se de que o serviço está disponível antes de continuar
        while not self.cliente_move.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando pelo serviço /move_command...')

    def navegar(self):
        # Lógica de navegação reativa
        self.get_logger().info("Executando navegação reativa")

        while rclpy.ok():
            # Cria uma requisição para o serviço de movimento
            req = MoveCmd.Request()
            req.direction = "right"  # Temporário, apenas para inicializar a variável

            # Chama o serviço e aguarda a resposta
            future = self.cliente_move.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                response = future.result()

                # Verifica os sensores
                left = response.left
                down = response.down
                up = response.up
                right = response.right

                # Lógica para decidir a direção com base nos sensores
                if right == 't':
                    req.direction = "right"
                elif down == 't':
                    req.direction = "down"
                elif left == 't':
                    req.direction = "left"
                elif up == 't':
                    req.direction = "up"
                elif right == 'f':
                    req.direction = "right"
                elif down == 'f':
                    req.direction = "down"
                elif left == 'f':
                    req.direction = "left"
                elif up == 'f':
                    req.direction = "up"
                else:
                    self.get_logger().info("Nenhuma direção disponível para se mover.")
                    break

                # Envia o comando de movimento
                future_move = self.cliente_move.call_async(req)
                rclpy.spin_until_future_complete(self, future_move)

                # Verifica se o movimento foi bem-sucedido
                if future_move.result().success:
                    self.get_logger().info(f"Movendo para {req.direction}")
                else:
                    self.get_logger().warning(f"Movimento {req.direction} falhou.")
            else:
                self.get_logger().error("Falha na resposta do serviço de movimento.")
                break


def main(args=None):
    rclpy.init(args=args)
    nodo_navegacao_reativa = NavegacaoReativa()

    # Execute a lógica de navegação
    nodo_navegacao_reativa.navegar()

    nodo_navegacao_reativa.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
