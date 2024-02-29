# Configuração para o ROS NOETIC - Lisa
[ [Back to README.md](../README.md) ]

## Etapas anteriores
As etapas anteriores estão disponíveis no arquivo [README.md](../README.md).
NOTA: Temos apenas o espaço de trabalho Go1 que é totalmente compatível com ROS Noetic.

## Passo 1 - ROS Noetic Workspace da Lisa

O comando a seguir preparará todos os repositórios necessários e construirá a imagem docker necessária com imagem noetic-lisa.

```bash
./scripts/setup_lisa.sh noetic-lisa
```

## Passo 2 - Versão completa "LISA" do ROS Noetic (Ubuntu 20.04)
O comando baixo, via o script **run.sh**, construirá a imagem desejada, no caso da Lisa, e iniciará o container com o ROS Noetic funcional, pronto para ser utilizado. 

```bash
./docker/run.sh noetic-lisa
```

## Step 3 - Construindo os Pacotes 

Agora, com o container executando, construiremos os pacotes desejados, os quais foram passados pelo script **setup_lisa**. Este são pacotes externos que só precisam estar disponíveis dentro do container, como o projeto não possui pacotes externos, utilizar este comando não funcioná. 

```Se você não sabe se está dentro de um contêiner, verifique se sua pasta e usuário atuais se parecem com:``` **user@computer:~/catkin_ws$**

```bash
./scripts/build_generico_ws.sh
```

# FAQ - Perguntas frequentes❓
[Clica aqui: docs/FAQ.md file.](docs/FAQ.md)
