# Aqui se encontra o script em shell para dar o build na imagem da Lisa
    # Porque um script só pra isso? Esse script garante que sempre que for dar um build todas as configs estajam certas e
    # que não haja conflito entre as imagens.

# Checa se quando rodar o comando você está na pasta correta.
if [[ $PWD = *Lisa ]]; then
    cd docker
elif [[ ! $PWD = *Lisa/docker ]]; then
    echo -e "You must be in either 'Lisa' or the 'Lisa/docker' directory to run this command."
    echo -e "Você precisa estar na pasta 'Lisa' ou em 'Lisa/docker' para rodar esse commando."
    return 1
fi

# Configs da build
# -f Dockerfile.lisa  (-f ou --file informa o arquivo a ser dado o build)
# -t lisa:noetic  (-t ou --tag é para o repositório e para a tag respectivamente, separados por dois pontos)
# --rm (Ouvi que é bom deixar, acho que remove as versões de builds antigas)

docker build \
    --network=host \
    -f Dockerfile.lisa \
    -t lisa:noetic \
    --rm \
    .