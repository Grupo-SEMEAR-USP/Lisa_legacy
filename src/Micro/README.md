# Envio de arquivos para ESP32 

O envio de arquivos para a ESP32 pode ser feito por qualquer serviço, nesse tutorial trabalhos com docker.

## Setup

Clonar repositório:
 `$ git clone git@github.com:lucasgpulcinelli/go_utils.git`

Vai para o respositorio go_utils e cria arquivos importantes:
 `$ touch get_file post_file`

**[FEDORA]** Mudar permissão dos arquivos:
 `$ chcon -R -v -t container_file_t get_file post_file`

## Servidor para envio de arquivos

Inicializar docker:
 `$ sudo systemtl start docker`

Inicializa o servidor que envia o arquivo get_file:
 `$ docker-compose -f compose-yamls/httptransfer-compose.yaml --project-directory . up`

Insere arquivo para ser enviado no get_file:
 `$ mv <caminho>/<arquivo>.mp3 data/get_file`

## Conectando ESP32 no servidor
Vai para o repositorio:
 `$ cd Lisa/src/Micro`

Edita o arquivo em `main/wifi.c`:
Altere o client config:
```
    esp_http_client_config_t config = {
        .host = "HOST",
        .path = "/",
        .port = 8080,
        .event_handler = http_event_handler,
        .user_data = local_response_buffer,
        .disable_auto_redirect = true,
    };
```

mudar `.host` para `.host = <COMPUTER IP>`
mudar `.port` para `.port = 80`

<br>

```
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "SSID", //temporary
            .password = "PASSWORD",
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
```

Inserir o SSID e a senha da rede que o computador está conectado

- Disponibiliza as ferramentas do idf.py em PATH e inicializa um venv de python:
`$HOME/esp/esp-idf/export.sh`
- Build do projeto: 
`idf.py build`
- alterar permissão com `chmod` para o usuário ter acesso a placa
`sudo chmod 777 /dev/ttyUSB0`
- Envia o projeto para a ESP32
`idf.py flash`
- Abre o log da ESP32
`idf.py monitor`