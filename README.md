# Navegação

- [Configuração do ambiente virtual para rodar a atividade](#configuração-do-ambiente-virtual-para-rodar-a-atividade)
- [Configuração do pacote python](#configuração-do-pacote-python)
- [Rodando meu pacote em seu computador](#rodando-meu-pacote-em-seu-computador)
- [Explicação do código e suas principais funções](#explicação-do-código-e-suas-principais-funções)
  - [Método de Navegação Reativa](#método-de-navegação-reativa)
  - [Método de Navegação com Mapa](#método-de-navegação-com-mapa)
  - [Função Inicializadora](#função-inicializadora)
- [Vídeo que comprova plenamente o funcionamento do sistema criado](#vídeo-que-comprova-plenamente-o-funcionamento-do-sistema-criado)

# Atividade ponderada - Navegação no Labirinto

&emsp;**Objetivo:** Implementar dois métodos distintos de navegação - **Navegação Reativa** e **Navegação com Mapa** - para um robô autônomo. O robô deve navegar até o alvo representado no mapa enquanto utiliza diferentes estratégias de movimento.

#### Aviso:

&emsp;Parte do código desenvolvido nesta atividade foi reaproveitado de implementações colaborativas feitas com colegas.

---

# Configuração do ambiente virtual para rodar a atividade:

&emsp;Primeiro, clone o repositório com o seguinte comando:

```bash
git clone git@github.com:Rizzi26/Ponderada-M8.git
```

&emsp;Depois de clonar o repositório, navegue até a pasta raiz do projeto no terminal, ative o ambiente virtual do Python com os comandos abaixo e instale as dependências necessárias:

```python
python3 -m venv venv

source venv/bin/activate

python3 -m pip install -r requirements.txt
```

# Configuração do pacote python:

&emsp;Para configurar o pacote e integrá-lo ao ROS2, navegue até o diretório cg_mw dentro do projeto e execute os comandos abaixo:

```python
colcon build

source install/local_setup.bash
```

# Rodando meu pacote em seu computador:

&emsp;Esta atividade utiliza um simulador baseado no pygame para representar o labirinto e o robô. O simulador utiliza ROS2 para integração.

1. Execute o simulador com o comando:

```python
ros2 run cg maze
```

2. Após o simulador ser carregado, você verá o robô azul e o alvo vermelho no mapa.

3. Escolha no terminal qual método você deseja utilizar: Nav reativa ou Nav por mapa

4. Agora é só observar a mágica.

# Explicação do código e suas principais funções:

## Método de Navegação Reativa:

&emsp;O método de navegação reativa permite ao robô navegar pelo labirinto utilizando exclusivamente informações capturadas pelos sensores exteroceptivos, ou seja, dados do ambiente ao redor do robô. Este método não tem acesso ao mapa completo do labirinto, e suas decisões são tomadas com base em informações pontuais fornecidas em tempo real.

&emsp;Os principais dados utilizados são:

&emsp;Posição atual do robô e do alvo.

&emsp;Estado dos quadrados adjacentes ao robô (esquerda, direita, cima e baixo), que indicam:

- `t`: Alvo.
- `f`: Espaço livre.
- `b`: Espaço bloqueado.

&emsp;Com base nesses dados, o algoritmo utiliza uma abordagem reativa para ajustar a trajetória e navegar em direção ao alvo.

### Explicação do Código:

1. Inicialização do Nó e Serviços ROS2:

- A classe NavReativa herda da classe Node do ROS2 e é responsável por gerenciar a navegação.

- O serviço /move_command é utilizado para enviar comandos de movimento ao robô. A comunicação é configurada por meio de um cliente ROS (create_client).

- A posição inicial do robô é configurada como (0, 0) e uma estrutura de dados (set) é usada para rastrear posições já visitadas, evitando ciclos na navegação.

2. Função de Movimento (move):

- Recebe a direção de movimento como entrada (up, down, left, right) e envia a requisição ao serviço ROS.

- Usa um callback (done_callback) para processar a resposta do serviço, armazenando-a em self.response.

3. Callback para Resposta do Serviço (done_callback):

- Recebe os dados retornados pelo serviço, incluindo:
    - success: Indica se o movimento foi bem-sucedido.
    - robot_pos: Nova posição do robô.
    - left, down, up, right: Estado dos quadrados adjacentes.

4. Navegação Principal (navigate):

- Implementa um algoritmo de busca em largura (BFS) usando uma fila (deque) para explorar posições no labirinto.
- Para cada posição, verifica:
    - Se é possível mover para a próxima posição com base no estado dos quadrados adjacentes.
    - Se o alvo foi alcançado comparando as posições do robô e do alvo.
- Marca as posições visitadas para evitar repetições.
- Continua a navegação até que o alvo seja encontrado ou todas as posições acessíveis sejam exploradas.

5. Estratégia de Navegação:

- Um dicionário mapeia direções (up, down, left, right) para deslocamentos de coordenadas no grid.
- Para cada movimento, a função calcula a próxima posição e verifica se ela já foi visitada antes de adicioná-la à fila.

6. Execução Geral (main):

- A função main inicializa o sistema ROS2 e cria o nó NavReativa.
- Um executor dedicado (SingleThreadedExecutor) gerencia o nó, permitindo chamadas controladas ao método de navegação.
- Ao final, o executor e o nó são encerrados para garantir que os recursos sejam liberados adequadamente.

### Code:

```python
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
```

## Método de Navegação com Mapa:

&emsp;O método de navegação com mapa permite ao robô planejar sua rota com antecedência, utilizando informações completas do labirinto fornecidas pelo serviço /get_map. Esse método implementa o algoritmo A* para encontrar um trajeto otimizado do ponto inicial ao alvo, garantindo uma navegação eficiente e evitando decisões reativas baseadas apenas no ambiente imediato.

&emsp;Os passos principais incluem:

    Obtenção do Mapa:
        O robô solicita o mapa ao serviço /get_map, que retorna um grid representando o labirinto.
        O mapa é reconstruído a partir dos dados recebidos para facilitar o planejamento.

    Planejamento da Rota:
        Com o mapa reconstruído, o algoritmo A* é usado para calcular a melhor rota até o alvo. Isso inclui:
            Avaliação do custo total do caminho.
            Utilização de uma heurística (distância Manhattan) para priorizar movimentos.

    Execução da Rota:
        Após o planejamento, o robô segue os passos do trajeto calculado, enviando comandos ao serviço /move_command para cada movimento.



# Função Inicializadora

&emsp;A função principal (main()) gerencia a execução do sistema e integra os métodos implementados. Ela:

- Inicializa o ROS2.
- Configura o nó ROS.
- Seleciona o método de navegação (reativa ou com mapa).
- Executa o movimento do robô no labirinto.

# Vídeo que comprova plenamente o funcionamento do sistema criado:

&emsp;No vídeo, demonstro o funcionamento de ambos os métodos de navegação:

- Navegação Reativa: O robô utiliza apenas informações dos sensores para navegar até o alvo.
- Navegação com Mapa: O robô obtém o mapa, planeja a rota e segue um caminho otimizado.

&emsp;Clique na imagem abaixo para assistir ao vídeo: