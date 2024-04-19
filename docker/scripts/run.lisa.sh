# Aqui se encontra o script em shell para dar o run na imagem da LISA
    # Porque um script só pra isso? A cada run que você dá em um container é necessario informar a imagem e vários outros
    # parâmetros de vídeo, compartilhamento de pasta e etc. Aqui tudo isso é feito com um comando.
# Here you will find the commands to inicialize the docker image the rigth way
# Aqui você vai encontrar os comandos para inicializar a imagem docker do jeito bom

if [[ $PWD = *Lisa ]]; then

    #                       Configs para inicialização do container
    #                                -it    (Terminal interativo)
    #                             --user    (Setando qual usuário do container vc vai usar)
    #        --network=host e --ipc=host    (Pra ter acesso a internet)
    #               -v /dev/dri:/dev/dri    (Nesse caso to dando acesso a todas as pastas de driver, assim ele tem acesso ao driver de video do meu PC)
    # -v $PWD/lisa_ws:/home/lisa/lisa_ws    (Pra ter acesso a pasta de ROS do robo)
    #                -e DISPLAY=$DISPLAY    (Setando pra aparecer no display que estamos usando atualmente)
    #                        LISA:noetic    (A imagem docker que queremos)

    docker run -it \
    --user lisa \
    --network=host \
    --ipc=host \
    --device /dev/video0 \
    -v /dev/video0:/dev/video0 \
    -v /dev/dri:/dev/dri \
    -v $PWD/lisa_ws:/home/lisa/lisa_ws \
    -v $PWD/lisa_desktop:/home/lisa/lisa_desktop \
    -e DISPLAY=$DISPLAY \
    lisa:noetic

elif [[ ! $PWD = *spot-robot/docker ]]; then
    echo -e "You must be in 'LISA' directory to run this command."
    echo -e "Você deve estar na pasta 'LISA' para rodar esse comando."
    return 1
fi

# Aqui abaixo tem um comando pra quem tem placa da Nvidia eu acho, não funcionou no meu pq eu não uso esse X11-unix (minha teoria), mas
# você pode tentar.
# docker run -it --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=$DISPLAY spot-robot:spotnoetic