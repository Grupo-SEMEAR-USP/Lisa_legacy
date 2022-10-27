# Lisa
## Introdução
A Lisa é uma robô humanóide construída do zero pelo grupo SEMEAR EESC/USP, com o objetivo de ser um robô capaz de interagir com pessoas por meio de voz, movimentos de braços e torso, uma tela que representa seus olhos, entre outros. A Lisa pode ser utilizada no contexto de extensão, como em um robô de companhia, por exemplo, além de ser um projeto ideal para o estudo de robótica e programação por estudantes, isso pela extensa documentação disponível.


## Como testar o código
Depois de instalar uma versão recente de python 3, utilize em uma linha de comando: `pip install -r requirements.txt`, esse programa instalará todas as dependências necessárias para rodar a Lisa. Recomendamos que isso seja feito em um virtual environment, mas isso não é uma obrigação.

Depois disso, utilize `python3 ./src/Servidor/main.py` em um terminal para iniciar o servidor da Lisa e `python3 ./src/Cliente/cliente.py` em outro para iniciar o cliente, utilizando a função "responder" para testar as funcionalidades da Lisa


## Tecnologias Usadas
Há diversas tecnologias de áreas diferentes utilizadas para o desenvolvimento do projeto, desde robótica e programação de baixo nível em C até processamento de linguagem natual em python!

Essa diversidade de tecnologias para resolver o problema é possível por conta da infraestrutura de código da Lisa: temos de um lado a robô em si, que tem microfone, speaker, a tela com os olhinhos, etc. De outro um serviço de núvem da IBM, que é capaz de entender a conversa e controlar a lógica do diálogo, e entre os dois um servidor de python que comunica os dois e faz umas coisas legais no caminho!

Aqui temos um flowchart que mostra as capacidades da Lisa e como elas se comunicam:
![imagem do flowchart](./Arquivos_readme/lisa.svg)

### Programação do microcontrolador
Com o microprocessador ESP32, usamos a linguagem de programação C++ para dar vida à Lisa movendo motores, controlando a tela, microfone e alto-falantes, além de se comunicar com o servidor. Todo o código envolvendo essa área está em [Micro/](./src/Micro/)

### Cliente e servidor HTTP
Por meio da bibloteca flask de python e a biblioteca de wifi da ESP32, é possível criar uma forma de comunicação que conecta ambas as partes da robô. O código dessa área está tanto em [Micro/](./src/Micro/) quanto em [Servidor/servidor.py](./src/Servidor/servidor.py)

### Reconhecimento de Áudio
Para compreender o que a pessoa está falando, usamos a biblioteca speech_recognition de python, que é capaz de gerar um texto a partir do áudio de alguém falando, além de utilizar alguns métodos para limpeza do áudio, já que em ambientes barulhentos é difícil identificar a fala. O código relacionado a isso está em [Servidor/reconhecimentoAudio.py](./src/Servidor/reconhecimentoAudio.py)

### Reconhecimento de Sentido
Depois que a Lisa souber o que você falou, ela precisa saber como responder! Utilizamos um sistema baseado no IBM Watson Studio, um serviço de nuvem capaz de criar chatbots complexos, além de controlar os passos da conversa com o usuário, feito de forma simples para que qualquer um possa entender.

O código relacionado a isso está em [Servidor/Lisa.py](./src/Servidor/Lisa.py)
 e na documentação do projeto, na parte da cloud da IBM.

### Text To Speech
Para as respostas que envolvem a fala da Lisa, utilizamos o sistema de text to speech do Google (gTTS) e modificações próprias de intonação via a biblioteca pyrubberband para que a Lisa tenha uma voz adorável! Todo o código relacionado está em [Servidor/TTSLisa.py](./src/Servidor/TTSLisa.py)

### Cliente loopback
Mesmo o robô da Lisa sendo pensado para ser acessível, nem sempre é possível que todos tenham uma Lisa para testar suas funcionalidades. Pensando nisso, e também para facilitar o desenvolvimento e aumentar as funcionalidades da Lisa no futuro (por exemplo, tornar a Lisa não só uma robô mas um "bot" da internet) criamos uma implementação simples de cliente da Lisa em [Cliente/](./src/Cliente/) que pode ser utilizada a partir de qualquer computador com acesso à um servidor, seja na rede local ou pela internet


## Documentação
A documentação do projeto ainda está por vir


## Como ajudar a Lisa?
Há diversas formas de contibuir para o projeto, veja em [CONTRIBUTING.md](./CONTRIBUTING.md)


## Licença de Código
Todo o código da Lisa (exceto quando explicitado) está disponibilizado pela licença Apache 2.0, disponível em [LICENSE](./LICENSE), caso você queira utilizar o código da Lisa no seu projeto você é plenamente permitido, apenas, caso você redistribua códigos relacionados à Lisa, não deixe de avisar suas modificações e incluir o arquivo [NOTICE](./NOTICE), como obrigado pela licença de código. Também agradecemos se incluir um link para esse repositório ou para o [site do SEMEAR](https://semear.eesc.usp.br/)
