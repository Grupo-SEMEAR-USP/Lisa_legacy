# Lisa
A Lisa é uma robô humanóide construída do zero pelo grupo SEMEAR EESC/USP, com o objetivo de ser um robô capaz de interagir com pessoas por meio de voz, movimentos de braços e torso, uma tela que representa seus olhos, entre outros. A Lisa pode ser utilizada no contexto de extensão, como em um robô de companhia, por exemplo, além de ser um projeto ideal para o estudo de robótica e programação por estudantes, isso pela extensa documentação disponível.

## Tecnologias Usadas

## Máquina de Estados
Para realizar o projeto, fizemos algumas máquinas de estados modificadas que indicam o estado do robô após realizar os comandos.
Um exemplo disso é a maquina de estado desenvolvida para vários comandos diversos, mostrada abaixo. Nela, após realizar um comando como dançar, ele fica automaticamente no estado de "feliz".

![Máquina de Estados - Diversos](Arquivos_readme/Diversos.png)

Além disso, nota-se na imagem que alguns comandos não serão obedecidos se o robô estiver em determinado estado emocional, como o comando "dançar" também, que só pode ser realizado quando este está "feliz", "neutro" ou "triste".

Alguns comentários foram adicionados aos desenhos, para facilitar compreensão e auxiliar no entendimento na hora de construir o código em Python. Um exemplo de comentário pode ser visto na imagem abaixo, referente à máquina de estados "Tocar música".

![Máquina de Estados - Tocar Música](Arquivos_readme/Tocar_musica.png)


