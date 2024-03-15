# Aqui se encontra o script em shell para dar o build na imagem da LISA
    # Porque um script só pra isso? Esse script garante que sempre que for dar um build todas as configs estajam certas e
    # que não haja conflito entre as imagens.

# Checa se quando rodar o comando você está na pasta correta.
if [[ $PWD = *LISA ]]; then
    cd docker
elif [[ ! $PWD = *LISA/docker ]]; then
    echo -e "You must be in either 'LISA' or the 'LISA/docker' directory to run this command."
    echo -e "Você precisa estar na pasta 'LISA' ou em 'LISA/docker' para rodar esse commando."
    return 1
fi

# Configs da build
# -f Dockerfile.lisa  (-f ou --file informa o arquivo a ser dado o build)
# -t LISA:noetic  (-t ou --tag é para o repositório e para a tag respectivamente, separados por dois pontos)
# --rm (Ouvi que é bom deixar, acho que remove as versões de builds antigas)

docker build \
    --network=host \
    -f Dockerfile.lisa \
    -t lisa:noetic \
    --rm \
    .