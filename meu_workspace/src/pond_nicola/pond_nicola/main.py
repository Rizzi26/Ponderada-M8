import typer
import inquirer
from pond_nicola.navegacao_reativa import main as main_reativa
from pond_nicola.navegacao_mapa import main as main_mapa

# Cria uma instância da aplicação
app = typer.Typer()

# Função para processar a escolha do usuário
def processar_respostas(respostas):
    # Exibe a escolha do usuário e inicia o modo correspondente
    if respostas["opcao"] == "Reativa":
        typer.echo("Iniciando navegação reativa...")
        main_reativa()
    elif respostas["opcao"] == "Mapa":
        typer.echo("Iniciando navegação por mapa...")
        main_mapa()

# Comando para escolher a navegação
@app.command()
def navegacao():
    perguntas = [
        inquirer.List("opcao", message="Escolha a navegação", choices=["Reativa", "Mapa"]),
    ]
    # Realiza a leitura das respostas
    respostas = inquirer.prompt(perguntas)
    
    # Chama a função para processar a escolha do usuário
    if respostas:
        processar_respostas(respostas)

# Executa a aplicação
if __name__ == "__main__":
    app()
