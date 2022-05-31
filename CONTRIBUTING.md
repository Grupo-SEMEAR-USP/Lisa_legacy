# Contribuindo no projeto da Lisa
## O que contribuir
Há diversas formas de ajudar um projeto open source como a Lisa, não só incluindo contribuições de código, mas também de material, tutoriais, e muitos outros! Aqui estão algumas formas de melhorar o projeto para todos:
* Ideias de adição de estados e funcionalidades
* Tutoriais e documentação do projeto, tanto dentro quanto fora do repositório
* Divulgação de bugs
* Contribuições de código para resolver bugs ou adicionar funcionalidades

## Como contribuir
Para todas as formas de contribuição, é necessário para todos os interessados externos ao grupo SEMEAR que abram uma issue (por meio do github) descrevendo o problema ou ideia de contribuição, seguindo o nosso Modelo de Issue (a ser definido)

### Contribuições de código
Para as contribuições de código, é necessário para os não membros do grupo SEMEAR que criem um fork do repositório, realizem commits na branch main no fork, e abram uma pull request da sua branch main para a branch main do código da Lisa

Além disso, para as contribuições de código, todos devem seguir um padrão de programação em python:
* Todas as variáveis ou devem ter um nome em português descritivo ou um nome padrão utilizados externamente (df para pandas.DataFrame, por exemplo) ou outro nome padrão explicitado no repositório
    * além disso, todas as variáveis devem ser compostas de letras minúsculas ou "\_", excluindo qualquer acento gráfico
* Todas as funções devem seguir o padrão lowerCamelCase (a primeira palavra começa com letras minúsculas, qualquer outra palavra começa com letras maiúsculas) e estar em português
* Todas as classes devem seguir o padrão UpperCamelCase (todas as palavras, incluindo a primeira, começa com letras maiúsculas) e estar em português
* Todas as indentações de código devem ser compostas de quatro espaços, sem a utilização de tabs (configure seu IDE para fazer isso por padrão)
* Todas as linhas de código devem estar contidas em 80 caracteres, caso uma linha passe disso, utilize variáveis auxiliares ou separe a linha em duas utilizando "\\" e indente a segunda linha com um nível a mais de intentação
* Todos os módulos devem ser importados no início do arquivo e com o nome completo, exceto caso o nome do módulo seja grande ou haja um nome padrão utilizado (np para numpy, plt para matplotlib.pyplot)
* Todos os módulos que importem no máximo três funções ou objetos em um módulo podem utilizar `from import`
* Logo depois de toda a declaração de função ou classe, utilize uma string de múltiplas linhas para descrever o que a função ou classe implementa, quais seus argumentos, etc.
    * a única excessão é caso uma classe contenha uma função `__init__` que apenas execute linhas como `self.propriedade = argumento_propriedade` e validação de input
* Todas as funções que podem ser executadas diretamente devem conter toda a execução de código contida no final, indentado por `if __name__ == "__main__":`
#### exemplo de código bem formatado
```
from functools import reduce

class Objeto:
    '''
    A classe objeto representa um ser não vivo genérico.
    Essa classe contém apenas duas propriedades: nome e dimensoes,
    no qual o nome é uma string com o nome e dimensoes é uma lista contendo o
    tamanho em X, Y, e Z em metros, respectivamente.
    '''

    def __init__(self, nome, dimensoes):
        if type(nome) is not str:
            raise ValueError("nome deve ser uma string")
        self.nome = nome

        if type(dimensoes) is not list:
            raise ValueError("dimensoes não é uma lista")
        elif len(dimensoes) != 3:
            raise ValueError("somente três dimensões de objeto são aceitas")
        elif any(l <= 0 for l in dimensoes):
            raise ValueError("dimensoes deve conter apenas valores positivos")
        self.dimensoes = dimensoes

    def calcularVolume(self):
        '''
        a função calcularVolume retorna o volume em metros cúbicos do objeto,
        no caso dele ser um corpo macisso
        '''

        #calculando o volume comulativamente
        return reduce(lambda x,y: x*y, self.dimensoes)

    def mostrar(self):
        '''
        mostrar utiliza print para mostrar o objeto na tela,
        poderia ser implementado por __str__ e print
        '''

        print(f'{self.nome} -> X:{self.dimensoes[0]}, Y:{self.dimensoes[1]},'
            f' Z:{self.dimensoes[2]}')


def objetoDeTouple(touple_inicial):
    '''
    cria um objeto a partir de uma touple
    '''
    return Objeto(*touple_inicial)

if __name__ == "__main__":
    objeto = objetoDeTouple(("mesa", [0.5, 0.5, 0.5]))
    print(objeto.calcularVolume())
    objeto.mostrar()
```
