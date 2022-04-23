#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

// idealmente, colocam-se essas definicoes em runtime, mas, para facilitar
// testes, ou troque essas linhas por #defines ou compile o codigo com a opcao
// "-D", que permite passar ao compilador definicoes de preprocessamento
#ifndef REDE | SENHA | SERVIDOR
#error defines de informacao de internet faltando
#endif


void setup(){
    Serial.begin(115200);

    // conecta a rede
    Serial.println("\nConectando a " REDE);

    WiFi.begin(REDE, SENHA);
    while(WiFi.status() != WL_CONNECTED){
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    Serial.print("Conectado, ip: ");
    Serial.println(WiFi.localIP());
}

// Fica conectando ao servidor recebendo dados, coloca na tela o tempo de
// transmissao e os primeiros 100 bytes de dados
void loop(){
    String resposta;

    // se estamos conectados a internet
    if(WiFi.status() != WL_CONNECTED){
        unsigned long tempo_inicial = millis();

        // pega a resposta do servidor
        pedidoHttpGET(resposta);

        // coloca na tela o tempo em ms que levou
        Serial.println(millis() - tempo_inicial);
    }
    else{
        Serial.println("WiFi desconectado");
        delay(1000);
        return;
    }

    for(int i = 0; i < 100; i++){
        Serial.print(resposta[i]);
    }
    Serial.println();
    delay(1000);
}

void pedidoHttpGET(String& resposta){
    WiFiClient cliente_wifi;
    HTTPClient cliente_http;

    // inicializa a conexao com o servidor
    cliente_http.begin(cliente_wifi, SERVIDOR);

    // Faz o pedido em si por WiFi
    int retorno = cliente_http.GET();
    Serial.println(retorno);

    if(retorno < 0){
        Serial.println("Erro de HTTP");
        resposta = "";
        return;
    }

    resposta = cliente_http.getString();

    // termina a conexao
    cliente_http.end();
}
