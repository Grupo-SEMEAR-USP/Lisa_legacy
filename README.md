# SEMEADA
Projeto realizado em parceria com o grupo SEMEAR com o objetivo de construir um robô de mesa que se comunique com as pessoas, respondendo a perguntas, contando piadas e se movimentando

## Tecnologias Usadas

## Máquina de Estados
Para realizar o projeto, fizemos algumas máquinas de estados modificadas que indicam o estado do robô após realizar os comandos.
Um exemplo disso é a maquina de estado desenvolvida para vários comandos diversos, mostrada abaixo. Nela, após realizar um comando como dançar, ele fica automaticamente no estado de "feliz".

![Máquina de Estados - Diversos](https://github.com/ADA-EC/SEMEADA/blob/main/Maquina_Estados/Diversos.png)

Além disso, nota-se na imagem que alguns comandos não serão obedecidos se o robô estiver em determinado estado emocional, como o comando "dançar" também, que só pode ser realizado quando este está "feliz", "neutro" ou "triste".

Alguns comentários foram adicionados aos desenhos, para facilitar compreensão e auxiliar no entendimento na hora de construir o código em Python. Um exemplo de comentário pode ser visto na imagem abaixo, referente à máquina de estados "Tocar música".

![Máquina de Estados - Tocar Música](https://github.com/ADA-EC/SEMEADA/blob/main/Maquina_Estados/Tocar_musica.png)

## Membros
- Érika Hortência
- Breno Seixas
- Emanuela Porto
- Gabriela Barion Vidal
- Rodrigo Bragato Piva
- Tsuyoshi Sonobe

## Agradecimentos
- Victor Amaral
