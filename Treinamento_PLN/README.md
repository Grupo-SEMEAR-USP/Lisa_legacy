# Treinamento do PLN
Para atualizar os dados e rodar o treinamento, é necessário seguir os seguintes passos:

1- Modificar/atualizar a base de dados ("DataBaseFinal.csv") dentro da pasta "assets";
2- Rodar o documento "convert.py" dentro da pasta scripts;
3- Digitar o comando "python -m spacy project run convert", que converte o .json em .spacy;
4- Digitar o comando "python -m spacy project run train", que roda o treinamento da máquina.

## Teste
Para testar o projeto, basta entrar no documento "teste.py" e escrever a frase desejada na variável "text", na linha 29.

## Comandos possíveis
Para rodar os comandos, basta digitar [`spacy project run [comando]`](https://spacy.io/api/cli#project-run). Os comandos só funcionam quando os inputs são modificados.

| Comando | Descrição |
| --- | --- |
| `convert` | Convert the data to spaCy's binary format |
| `train` | Train the textcat model |
| `evaluate` | Evaluate the model and export metrics |
| `package` | Package the trained model as a pip package |
| `visualize-model` | Visualize the model's output interactively using Streamlit |
| `all` | `convert` &rarr; `train` &rarr; `evaluate` &rarr; `package` |