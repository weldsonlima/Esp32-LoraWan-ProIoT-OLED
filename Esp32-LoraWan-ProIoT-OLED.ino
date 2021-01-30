

/*Referências
Guia_Primeiros_Passos_Rede_LoRaWAN.pdf
https://github.com/phfbertoleti/dummy_esp32_lorawan
https://wiki.keyestudio.com/KS0515_Keyestudio_FSK_Wireless_Shield_RFM69HCW_868mhz(Black_and_Eco-friendly)
http://wiki.dragino.com/index.php?title=Lora_Shield
https://github.com/mcci-catena/arduino-lmic (biblioteca alternativa adaptada para o arduino)

*/

/* Projeto: projeto básico / dummy de end-device LoRaWAN (ABP)
 *          com ESP32. Este projeto envia por LoRaWAN um numero aleatório
 *          (em formato textual/ASCII).
 *          
 * Autor: Pedro Bertoleti
 * 
 * IMPORTANTE:
 * 1) Esse projeto faz uso da biblioteca "MCCI LoRaWAN LMIC Library". 
 *    Utilize preferencialmente a versão 2.3.2 da mesma.
 * 2) Antes de compilar, não se esqueça de deixar o arquivo lmic_project_config.h   
 *    (dentro na pasta da biblioteca: project_config/lmic_project_config.h com o 
 *     conteúdo abaixo:
 *
 *    // project-specific definitions
 *    //#define CFG_eu868 1
 *    //#define CFG_us915 1
 *    #define CFG_au921 1
 *    //#define CFG_as923 1
 *    // #define LMIC_COUNTRY_CODE LMIC_COUNTRY_CODE_JP      
 *    //#define CFG_in866 1
 *    #define CFG_sx1276_radio 1
 *    //#define LMIC_USE_INTERRUPTS
 *    map(value, fromLow, fromHigh, toLow, toHigh)
*/
#include <lmic.h> // Utilizando versão 3.3.0
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "heltec.h"
#include "dht.h"
#include <math.h>


/* Definições gerais */
#define BAUDRATE_SERIAL_DEBUG   115200

/* Definições do rádio LoRa (SX1276) */
#define GANHO_LORA_DBM          20 //dBm

#define RADIO_RESET_PORT        14
#define RADIO_MOSI_PORT         27
#define RADIO_MISO_PORT         19
#define RADIO_SCLK_PORT         5
#define RADIO_NSS_PORT          18
#define RADIO_DIO_0_PORT        26
#define RADIO_DIO_1_PORT        35
#define RADIO_DIO_2_PORT        34

#define payloadSize             34 

#define pinLDR 2
#define pinDHT11 13
#define SensorPin 37 //Sensor de chuva

dht DHT; //VARIÁVEL DO TIPO DHT

/* Constantes do rádio LoRa: GPIOs utilizados para comunicação
   com rádio SX1276 */
const lmic_pinmap lmic_pins = {
  .nss = RADIO_NSS_PORT,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RADIO_RESET_PORT,
  .dio = {RADIO_DIO_0_PORT, RADIO_DIO_1_PORT, LMIC_UNUSED_PIN},
};

/* Constantes do LoraWAN */
/* - Chaves (network e application keys) */
static const PROGMEM u1_t NWKSKEY[16] = {  }; //coloque aqui sua network session key
static const u1_t PROGMEM APPSKEY[16] = {  }; //coloque aqui sua application session key

/* - Device Address */
static const u4_t DEVADDR = ;

/* - Tempo entre envios de pacotes LoRa */
const unsigned TX_INTERVAL = 600; //600s = 600 segundos 

/* Variáveis e objetos globais */
static osjob_t sendjob; //objeto para job de envio de dados via ABP

/* Callbacks para uso cpm OTAA apenas (por este projeto usar ABP, isso, eles 
 *  estão vazios) */
void os_getArtEui (u1_t* buf) 
{ 
    /* Não utilizado neste projeto */  
}

void os_getDevEui (u1_t* buf) 
{ 
    /* Não utilizado neste projeto */  
}

void os_getDevKey (u1_t* buf) 
{ 
    /* Não utilizado neste projeto */  
}

/* Callback de evento: todo evento do LoRaAN irá chamar essa
   callback, de forma que seja possível saber o status da 
   comunicação com o gateway LoRaWAN. */
void onEvent (ev_t ev) 
{
    //Serial.print(os_getTime());
    //Serial.print(": ");
    //Serial.println(ev);
    
    switch(ev) 
    {
        case EV_SCAN_TIMEOUT:
            //Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            //Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            //Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            //Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            LMIC_setLinkCheckMode(1);
            LMIC_setAdrMode(1);
        //LMIC_startJoining();
            //Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            //Serial.println(F("EV_JOINED"));
            break;
        case EV_JOIN_FAILED:
            //Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            //Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            //Serial.println (millis());
           // Serial.println(F("EV_TXCOMPLETE (incluindo espera pelas janelas de recepção)"));

            /* Verifica se ack foi recebido do gateway */
            if (LMIC.txrxFlags & TXRX_ACK)
              //Serial.println(F("Ack recebido"));

            /* Verifica se foram recebidos dados do gateway */  
            if (LMIC.dataLen) 
            {
                //Serial.println(F("Recebidos "));
                //Serial.println(LMIC.dataLen);
                //Serial.println(F(" bytes (payload) do gateway"));
              
                /* Como houve recepção de dados do gateway, os coloca
                   em um array para uso futuro. */
                if (LMIC.dataLen == 1) 
                {
                    uint8_t dados_recebidos = LMIC.frame[LMIC.dataBeg + 0];
                    //Serial.print(F("Dados recebidos: "));
                    //Serial.write(dados_recebidos);
                }

                /* Aguarda 100ms para verificar novos eventos */
                delay(100);
             
            }
            
            /* Agenda próxima transmissão de dados ao gateway, informando daqui quanto tempo deve
               ocorrer (TX_INTERVAL) e qual função chamar para transmitir (do_send).
               Dessa forma, os eventos serão gerados de forma periódica. */
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;

        case EV_LOST_TSYNC:
            //Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            //Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            //Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            //Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            //Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            //Serial.println(F("EV_TXSTART"));
            //Serial.println (millis());
            //Serial.println(LMIC.freq);
            break;
        default:
            //Serial.print(F("Evento desconhecido: "));
            //Serial.println((unsigned) ev);
            break;
    }
}

/* Função para envio de dados ao gateway LoRaWAN */
void do_send(osjob_t* j)
{
    int sensorValue = -1; //Higrômetro
    int ldrValue = -1; //Luminosidade do módulo LDR
    int saleSize = 1; // descunto do valor do payload
    sensorValue = map(analogRead(SensorPin), 0, 5000, 0, 100); // Leitura do Higrômetro
    ldrValue = map(analogRead(pinLDR), 0, 5000, 100, 0);
    //map(value, fromLow, fromHigh, toLow, toHigh)

    //saleSize
    DHT.read11(pinDHT11);
    //String mydata_str = "";
    static uint8_t mydata[payloadSize];
    char mydata_str[payloadSize]={0};

// Montando o JSON
    DynamicJsonDocument json(JSON_OBJECT_SIZE(4));
    json["H"] = sensorValue;
    json["T"] = (int)DHT.temperature;
    json["U"] = (int)DHT.humidity;
    json["L"] = ldrValue;

  //  Apenas para justar o tamanho do payload LoRaWAN
 
        if ((sensorValue >= 100)&&(sensorValue < 1000)) {
            saleSize++;
        }
        else if ((sensorValue >= 10)&&(sensorValue < 100)) {
            saleSize = saleSize + 2;
        }
        else if ((sensorValue >= 0)&&(sensorValue < 10)) {
            saleSize = saleSize + 3;
        }
 

        if ((ldrValue >= 100)&&(ldrValue <1000)) {
            saleSize++;
        }
        else if ((ldrValue >= 10)&&(ldrValue < 100)) {
            saleSize = saleSize + 2;
        }
        else if ((ldrValue >= 0)&&(ldrValue < 10)) {
            saleSize = saleSize + 3;
        }

    serializeJson(json, mydata_str);  
    //Serial.print(F("JSON:"));
    //Serial.println(mydata_str);
    //Serial.print(F("JSON Payload Size:"));
    //Serial.println(payloadSize-saleSize);
        Heltec.display -> clear();
        Heltec.display -> drawString(0,0, (String) os_getTime());
        Heltec.display -> drawString(0,10, mydata_str);
        Heltec.display -> display();

    /* Verifica se já há um envio sendo feito.
       Em caso positivo, o envio atual é suspenso. */
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        //Serial.println(F("OP_TXRXPEND: ha um envio ja pendente, portanto o atual envio nao sera feito"));
    } 
    else 
    {
        /* Aqui, o envio pode ser feito. */
        /* O pacote LoRaWAN é montado e o coloca na fila de envio. */
        LMIC_setTxData2(1, (uint8_t *)&mydata_str, payloadSize-saleSize, 0);

        
        //Serial.println(F("Pacote LoRaWAN na fila de envio."));       
    }


}

void setup() 
{

    Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);
    Heltec.display->clear();
    pinMode(pinDHT11, INPUT);
    pinMode(SensorPin, INPUT);

    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    int b;
    
    /* Inicializa serial de debug */
    //Serial.begin(BAUDRATE_SERIAL_DEBUG);

    /* Inicializa comunicação SPI com rádio LoRa */
    SPI.begin(RADIO_SCLK_PORT, RADIO_MISO_PORT, RADIO_MOSI_PORT);

    /* Inicializa stack LoRaWAN */
    os_init();
    LMIC_reset();

    /* Inicializa chaves usadas na comunicação ABP */
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);

    /* Faz inicializações de rádio pertinentes a região do 
       gateway LoRaWAN (ATC / Everynet Brasil) */
    for (b=0; b<8; ++b) 
        LMIC_disableSubBand(b);

    LMIC_enableChannel(0); // 915.2 MHz
    LMIC_enableChannel(1); // 915.4 MHz
    LMIC_enableChannel(2); // 915.6 MHz
    LMIC_enableChannel(3); // 915.8 MHz
    LMIC_enableChannel(4); // 916.0 MHz
    LMIC_enableChannel(5); // 916.2 MHz
    LMIC_enableChannel(6); // 916.4 MHz
    LMIC_enableChannel(7); // 916.6 MHz

    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);

    /* Data rate para janela de recepção RX2 */
    LMIC.dn2Dr = DR_SF12CR;

    /* Configura data rate de transmissão e ganho do rádio LoRa (dBm) na transmissão */
    LMIC_setDrTxpow(DR_SF12, GANHO_LORA_DBM);

    /* Força primeiro envio de pacote LoRaWAN */
    do_send(&sendjob);
}

void loop() 
{
    os_runloop_once();    
}
