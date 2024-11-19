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

&emsp;O método de navegação com mapa permite ao robô planejar sua rota com antecedência, utilizando informações completas do labirinto fornecidas pelo serviço /get_map. Este método implementa o algoritmo A* para calcular o caminho otimizado do ponto inicial ao alvo, garantindo eficiência e evitando decisões baseadas apenas no ambiente imediato.

### Explicação do Código:

1. Inicialização do Nó e Serviços ROS2:

- A classe `NavMapa` herda da classe `Node` do ROS2 e gerencia a navegação por mapa.
- Dois serviços ROS são utilizados:
    - `/move_command`: para enviar comandos de movimento ao robô.
    - `/get_map`: para obter o mapa completo do labirinto.
- O mapa é reconstruído a partir de dados recebidos e usado para identificar as posições inicial (`start_pos`) e alvo (`target_pos`).

2. Obtenção do Mapa (`get_map`):

- O mapa é solicitado ao serviço /get_map, que retorna o grid do labirinto.
- O mapa é convertido de uma representação linear para uma matriz 2D (map_2d) para facilitar a manipulação.

3. Identificação de Posições (`find_pose`):

- A função percorre o mapa 2D para localizar a posição inicial do robô (r) e o alvo (t).

4. Planejamento do Caminho (`plan_path`):

- O algoritmo A* é utilizado para calcular a rota otimizada entre o ponto inicial e o alvo.
- Para cada posição, são avaliados:
    - O custo acumulado do caminho (g_score).
    - A distância estimada até o alvo usando a heurística Manhattan (f_score).
- Um heap de prioridades (open_list) é usado para processar os nós com menor custo estimado primeiro.
- Se nenhum caminho é encontrado, o método retorna uma lista vazia.

5. Execução do Caminho Planejado (navigate):

- Após o planejamento, o robô segue a rota calculada enviando comandos de movimento ao serviço /move_command.
- Cada comando é enviado e confirmado individualmente.

6. Funções Auxiliares:

- heuristic: Calcula a distância Manhattan entre duas posições.
- get_neighbors: Retorna os vizinhos válidos (dentro dos limites do mapa) para uma posição, incluindo suas direções (up, down, left, right).
- reconstruct_path: Reconstrói a rota completa a partir do ponto final até o inicial usando o dicionário came_from.

7. Execução Geral (main):

- Inicializa o sistema ROS2 e cria um executor dedicado.
- O nó NavMapa é adicionado ao executor, que gerencia sua execução.
- Após o planejamento, o robô navega pelo labirinto seguindo o caminho planejado.

### Code:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from cg_interfaces.srv import MoveCmd, GetMap
import heapq


class NavMapa(Node):
    def __init__(self):
        super().__init__('nav_mapa')
        
        # Clientes para os serviços de movimento e obtenção de mapa
        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.map_client = self.create_client(GetMap, '/get_map')
        
        # Aguarda até que os serviços estejam disponíveis
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /move_command...')
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /get_map...')
        
        # Obtém o mapa e define as posições inicial e alvo
        self.map = self.get_map()
        self.start_pos, self.target_pos = self.find_pose(self.map)
        
        if self.start_pos is None or self.target_pos is None:
            self.get_logger().error("Posições inicial ou alvo não encontradas no mapa.")
            raise ValueError("Posições inválidas no mapa.")
        
        # Planeja o caminho usando A*
        self.path = self.plan_path(self.start_pos, self.target_pos)

    def get_map(self):
        request = GetMap.Request()
        future = self.map_client.call_async(request)
        
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        response = future.result()
        if not response:
            self.get_logger().error("Falha ao obter o mapa.")
            raise RuntimeError("Erro ao requisitar o mapa.")
        
        map_data = response.occupancy_grid_flattened
        map_shape = response.occupancy_grid_shape
        
        # Reconstrói o mapa 2D a partir do grid planificado
        map_2d = [map_data[i:i + map_shape[1]] for i in range(0, len(map_data), map_shape[1])]
        return map_2d

    def find_pose(self, map_2d):
        start_pos = None
        target_pos = None
        for i, row in enumerate(map_2d):
            for j, cell in enumerate(row):
                if cell == 'r':
                    start_pos = (i, j)
                elif cell == 't':
                    target_pos = (i, j)
        return start_pos, target_pos

    def plan_path(self, start, target):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, target)}
        
        while open_list:
            _, current = heapq.heappop(open_list)
            
            # Verifica se chegou ao alvo
            if current == target:
                return self.reconstruct_path(came_from, current)
            
            # Expande nós vizinhos
            for direction, neighbor in self.get_neighbors(current):
                if self.map[neighbor[0]][neighbor[1]] == 'b':  # Ignora posições bloqueadas
                    continue
                
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = (current, direction)
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, target)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
        
        # Retorna uma lista vazia se não encontrar caminho
        self.get_logger().warning("Caminho não encontrado.")
        return []

    def heuristic(self, pos, target):
        """Calcula a heurística de Manhattan para A*."""
        return abs(pos[0] - target[0]) + abs(pos[1] - target[1])

    def get_neighbors(self, pos):
        directions = {
            'up': (-1, 0),
            'down': (1, 0),
            'left': (0, -1),
            'right': (0, 1)
        }
        neighbors = []
        for direction, (di, dj) in directions.items():
            neighbor = (pos[0] + di, pos[1] + dj)
            if 0 <= neighbor[0] < len(self.map) and 0 <= neighbor[1] < len(self.map[0]):
                neighbors.append((direction, neighbor))
        return neighbors

    def reconstruct_path(self, came_from, current):
        """Reconstrói o caminho de volta usando o dicionário came_from."""
        path = []
        while current in came_from:
            current, direction = came_from[current]
            path.append(direction)
        path.reverse()
        return path

    def navigate(self):
        """Navega pelo caminho planejado."""
        if not self.path:
            self.get_logger().error("Nenhum caminho planejado. Abortando navegação.")
            return
        
        for direction in self.path:
            result = self.send_move_request(direction)
            if result and result.success:
                self.get_logger().info(f"Movendo para {direction}. Posição atual: {result.robot_pos}")
            else:
                self.get_logger().warning("Falha ao mover.")

    def send_move_request(self, direction):
        """Envia o comando de movimento para o robô."""
        request = MoveCmd.Request()
        request.direction = direction
        future = self.move_client.call_async(request)
        
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return future.result()


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    
    executor = SingleThreadedExecutor()
    navigator = NavMapa()
    executor.add_node(navigator)
    
    try:
        navigator.get_logger().info("Iniciando navegação por mapa...")
        navigator.navigate()
    finally:
        navigator.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

# Função Inicializadora

&emsp;A função principal (main()) gerencia a execução do sistema e integra os métodos implementados. Ela utiliza um sistema baseado em CLI (Command Line Interface) para permitir que o usuário escolha entre os métodos de navegação disponíveis - Navegação Reativa ou Navegação com Mapa. O fluxo de execução é organizado da seguinte forma:

1. Interface CLI para Escolha de Navegação (main_cli):
        - A interface CLI é implementada usando as bibliotecas typer e inquirer.
        - O comando principal da CLI (navegacao) apresenta uma lista de opções ao usuário:
            - Reativa: Seleciona a navegação reativa.
            - Mapa: Seleciona a navegação com mapa.
        - Após o usuário escolher, a função processar_respostas é chamada para iniciar o método correspondente:
            - Chama main_reativa() para executar a navegação reativa.
            - Chama main_mapa() para executar a navegação por mapa.

    - Execução Paralela com ROS2:
        - A função principal game inicializa o sistema ROS2 e utiliza threads para executar tanto o nó ROS quanto o CLI simultaneamente:
            - Uma thread executa a interface CLI (main_cli) para capturar a escolha do usuário.
            - Outra thread executa o rclpy.spin para rodar o nó ROS associado ao método escolhido.
        - Essa separação garante que o sistema CLI e o ROS2 possam funcionar de maneira independente sem bloquear um ao outro.

    - Integração com o Sistema de Navegação:
        - Com base na escolha feita pelo usuário na CLI, a função correspondente (main_reativa ou main_mapa) é chamada.
        - Ambas as funções inicializam o nó ROS e executam o método de navegação especificado até que o robô atinja o alvo ou que a execução seja interrompida.

    - Manutenção de Fluxo Contínuo:
        - O programa principal (game) roda continuamente em um loop até que o sistema seja encerrado (ex.: por KeyboardInterrupt).
        - Ao finalizar, as threads são cuidadosamente encerradas para liberar os recursos e encerrar o sistema ROS de maneira limpa.

### Code:

&emsp;O corpo principal do pacote é estruturado de forma que o `main` seja chamado nos entry points do pacote, integrando o jogo e o sistema ROS com a CLI. A partir da CLI, o usuário escolhe se deseja executar a navegação reativa ou por mapa. Com base nessa escolha, os códigos explorados anteriormente são acionados pela CLI, executando os arquivos `navegacao_mapa` ou `navegacao_reativa`. Nessas execuções, a função `main` de cada arquivo é chamada, iniciando o programa de navegação correspondente.

#### Code main_cli:

```python
import typer
import inquirer
import logging
from cg.navegacao_reativa import main as main_reativa
from cg.navegacao_mapa import main as main_mapa

logging.basicConfig(level=logging.INFO)

app = typer.Typer()

# Função para processar a escolha do usuário
def processar_respostas(respostas):

    if respostas["opcao"] == "Reativa":
        logging.info("Iniciando navegação reativa...")
        typer.echo("Iniciando navegação reativa...")
        main_reativa() 
    elif respostas["opcao"] == "Mapa":
        logging.info("Iniciando navegação por mapa...")
        typer.echo("Iniciando navegação por mapa...")
        main_mapa()  

# Comando para escolher a navegação
@app.command()
def navegacao():
    perguntas = [
        inquirer.List("opcao", message="Escolha a navegação", choices=["Reativa", "Mapa"]),
    ]
    respostas = inquirer.prompt(perguntas)
    
    if respostas:
        processar_respostas(respostas)

def main():
    app()
```

#### Code main:

```python
import rclpy
import threading
from cg.main_cli import main as main_cli
from .Utils.Csv import load_from_csv
from .Game import Game
from .Editor import Editor

def game():
    rclpy.init()
    game = Game(load_from_csv("maps/default.csv"))
    
    # Executa main_cli em uma thread separada para evitar bloqueio
    cli_thread = threading.Thread(target=main_cli)
    cli_thread.start()

    game_thread = threading.Thread(target=rclpy.spin, args=(game,))
    game_thread.start()

    try:
        while rclpy.ok():
            game.run()
    except KeyboardInterrupt:
        pass
    finally:
        game.destroy_node()
        rclpy.shutdown()
        game_thread.join()
        cli_thread.join()  # Aguarda a finalização do CLI

def editor():
    editor = Editor(load_from_csv("maps/default.csv"))
    editor.run()
```


# Vídeo que comprova plenamente o funcionamento do sistema criado:

&emsp;No vídeo, demonstro o funcionamento de ambos os métodos de navegação:

- Navegação Reativa: O robô utiliza apenas informações dos sensores para navegar até o alvo.
- Navegação com Mapa: O robô obtém o mapa, planeja a rota e segue um caminho otimizado.

&emsp;Clique na imagem abaixo para assistir ao vídeo:

[![Vídeo que comprova plenamente o funcionamento do sistema criado](https://db0dce98.rocketcdn.me/en/files/2023/12/pygame-datascientest.webp)](https://youtu.be/srfY_ng86pg)
