/*********************************************************************************************************************************************************************
**  Title: Modbus/MQTT/PID application for ESP32 to control a tank system                                                                                           **
**  Author: Daniel Lopes Martins (danlartin@gmail.com)                                                                                                              **
**  Year: 2021-2022                                                                                                                                                      **
**  Version: 1.5.0                                                                                                                                                  **
**  Description: This application has two functions, capture data from sensors (modbus slaves) via Modbus protocol and making it available to other devices         **
**               (modbus masters) via Modbus and MQTT protocol, in addition to running a PID controller, using sensors data and setpoint from the Modbus Master     **
**               or MQTT, to control a tank system with two coupled tanks and one pump.                                                                             **
**********************************************************************************************************************************************************************/

#include <WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <EspMQTTClient.h>
#include <PID_v1.h>
#include <ArduinoOTA.h>
#include <NTPClient.h>

// ################################################################# Dados de configuração ###########################################################################
// ####################################################################### Serial ####################################################################################
const long baud = 115200;                       // Baud Rate do serial monitor

// ######################################################################## Rede #####################################################################################
//IPAddress gwInd(10, 13, 103, 40);             // Endereço do GW-YK-R1.5
IPAddress gwInd(10, 13, 103, 42);               // Endereço do GW-YK-R2.0
//IPAddress gwInd(10, 13, 103, 39);             // Endereço do ESP32 simulando um GW (LAMP)
IPAddress espIP(10, 13, 103, 37);               // Endereço do ESP32 (LAMP)
IPAddress subnet(255, 255, 225, 0);             // subrede (LAMP)
IPAddress gatewaySTD(10, 13, 96, 1);            // gateway padrão (LAMP)
IPAddress primaryDNS(10, 13, 96, 17);           // DNS primário (LAMP)
IPAddress secondaryDNS(192, 168, 1, 4);         // DNS secundário (LAMP)
const char ssid[] = "LampLABA";                 // nome da rede Lab A
const char pass[] = "udtqcSSONDodtqq";          // senha da rede */

const char ntp_server[] = "a.st1.ntp.br";       //servidor NTP

WiFiUDP udp;
NTPClient ntp(udp, ntp_server, -3 * 3600, 1000);/* Cria um objeto "NTP" com as configurações.utilizada no Brasil */
String hora; 

// ######################################################################## Modbus ###################################################################################
int updateTimeMsMBMS = 20;                      // Tempo de atualização do mestre e escravo modbus do esp32 (em milisegundos)
const int GWIREGLVL1 = 13;                      // Offset registrador de entrada do level do tanque 1 (float) do GW
const int ESPIREGLVL1 = 100;                    // Offset registrador de entrada do level do tanque 1 (float) do ESP
const int ESPHREGPIDSP = 104;                   // Offset registrador de armazenamento do setpoint do controlador PID  (float) de ESP
const int ESPHREGPIDME = 106;                   // Offset registrador de armazenamento do modo de execução do controlador PID (int16) de ESP
const int ESPHREGPIDKP = 108;                   // Offset registrador de armazenamento da constante proporcional do controlador PID (float) de ESP
const int ESPHREGPIDKI = 110;                   // Offset registrador de armazenamento da constante integrativa do controlador PID (float) de ESP
const int ESPHREGPIDKD = 112;                   // Offset registrador de armazenamento do constante derivativa do controlador PID (float) de ESP
TickType_t lastWakeTimeS;                       // Variável de acompanhamento de tempo do processo do escravo
TickType_t lastWakeTimeM;                       // Variável de acompanhamento de tempo do processo do Mestre
const int UPDATE_M_COUNT = 50;                  // Quantidade de execuções do mestre para ele requisitar novos dados
uint8_t ucm = UPDATE_M_COUNT-1;                 // Contador de atualizações do mestre
const int UPDATE_S_COUNT = 50;                  // Quantidade de execuções do escravo local para ele publicar dados
uint8_t ucs = UPDATE_S_COUNT-1 + 10;            // Contador de atualizações do escravo local que é defasado da execução do mestre

typedef union floatModbus_t{                    // Tipo union para converter 2 uint16 em um float
  uint16_t dVec[2];                             // Como os dados lidos pela biblioteca são armazenados em uint16 (2 bytes)
  float dFloat;                                 // com o Union, esses valores armazenados sequencialmente num vetor de
};                                              // uint16 de 2 posições são convertidos em um float (4 bytes) corretamente.

floatModbus_t Lv1;                              // Variável global de armazenamento e conversão de registradores para float para o nivel 1
floatModbus_t SP1;                              // Variável global de armazenamento e conversão de registradores para float para o SP do tanque 1  (ou voltage da bomba)

ModbusIP mbM,mbS;                          // Objetos do ModbusIP (mestre e escravo respectivamente)

static SemaphoreHandle_t mutexV;                // Mutex usado para compartilhamento de dados entre as tasks

// ######################################################################## DAC/PWM ##################################################################################
#define USEDAC  false                           // COnfiguração para uso do DAC ou PWM
#define PUMP_DAC_PIN 25                         // Pino 25 corresponde ao GPIO 25 (DAC1)
#define PUMP_PWM_PIN 32                         // Pino 34 corresponde ao GPIO 32 (PWM) (exceção das GPIO 34-39, que sao apenas entrada)
#define PWM_CH 0                                // Canal do PWM
#define PWM_RES 10                              // Resolução (em bits) do PWM
#define PWM_FREQ 30000                          // Frequencia do PWM (acima da audível e compatível como o motor)

// ########################################################################## PID ####################################################################################
#define PIDOUTPUTMIN  0.0                       // Limite mínimo da saída do PID
#define PIDOUTPUTMAX  3.2                       // Limite máximo da saída do PID
int updateTimeMsPID = 1000;                     // Tempo de atualização de variáveis que o PID manipula (em milisegundos)
TickType_t lastWakeTimePID;                     // Variável de acompanhamento de tempo do processo do PID
double Input, Setpoint, Output;                 // Variáveis de entrada e saída do PID
floatModbus_t Kp;                               // Variável armazenamento e conversão de registradores para float para a constante proporcional do controlador PID
floatModbus_t Ki;                               // Variável armazenamento e conversão de registradores para float para a constante integrativa do controlador PID
floatModbus_t Kd;                               // Variável armazenamento e conversão de registradores para float para a constante derivativa do controlador PID
int modo =  1;                                  // Modo do controlador PID (AUTOMATIC - ligado; MANUAL (default caso não configurado) - desligado)
int PWM_val;                                    // Armazena o valor de tensão a ser enviado para bomba na resolução de 8 Bits (0 - 255) do PWM 
#define KPINITIAL 0.005                       // Parâmetros inicial de ajuste do Kp do PID (ajuste default)
#define KIINITIAL 0.0004                        // Parâmetros inicial de ajuste do Ki do PID (ajuste default)
#define KDINITIAL 0.0005                             // Parâmetros inicial de ajuste do Kd do PID (ajuste default)

PID myPID(&Input,                               // Criando o controlador PID, definindo a variável de entrada
          &Output,                              // A saída
          &Setpoint,                            // O setpoint
          KPINITIAL,                            // A constante proporcional
          KIINITIAL,                            // A constante integrativa
          KDINITIAL,                            // A constante derivativa
          DIRECT);                              // A direção do controlador (DIRECT para a saída deve aumentar para um erro positivo, setpoint acima da entrada. INDIRECT a saída deve diminuir com um erro positivo).

// ######################################################################### MQTT ####################################################################################
EspMQTTClient client(
  "10.13.103.28",                               // Endereço IP onde o broker está rodando
  1883,                                         // Porta do serviço do broker
  "esp32",                                      // Omita este parametro para disabilitar a autenticação MQTT
  "adminadmin",                                 // Omita este parametro para disabilitar a autenticação MQTT
  "ESP32TankSystem"                             // Nome do cliente que o identifica unicamente
);

floatModbus_t SP1Temp;                          // Variável temporária de armazenamento entre MQTT e Modbus para o SetPoint do tanque 1 (ou voltage da bomba)

// #################################################################### Funções e tasks ##############################################################################
void writePump(int out){                                                    // Função para enviar a tensão da bomba
  //Serial.print("Enviando: ");                                               // Envia na serial o texto da tensão para fins de debug
  //Serial.println(out);                                                      // Envia na serial a tensão recebida para fins de debug
  if(USEDAC){                                                               // Veirifca se o usuário deseja o uso do DAC ou PWM
    dacWrite(PUMP_DAC_PIN, out);                                            // Envia o valor para a GPIO 25 (DAC1) do Esp32
  }else{
    ledcWrite(PWM_CH, out);                                                 // Envia o valor para a GPIO 32 (PWM) do Esp32
  }
}                                                                           // Fim da Função*/

void onMQTTMessageReceivedSP1V(const String& message){                      // Função para converter a string do MQTT para o SP/Tensão
  SP1Temp.dFloat = message.toFloat();                                       // Converte o texto para float
  mbS.Hreg(ESPHREGPIDSP, SP1Temp.dVec[1]);                                  // Converte o dado float para armazenamento dos registradores do escravo modbus
  mbS.Hreg(ESPHREGPIDSP+1, SP1Temp.dVec[0]);                                // Converte o dado float para armazenamento dos registradores do escravo modbus
  Serial.println(SP1Temp.dFloat);                                           // Envia na serial a tensão recebida
}                                                                           // Fim da Função*/

void onMQTTMessageReceivedPIDMODE(const String& message){                   // Função para converter a string do MQTT para o modo de execução do PID
  modo = message.toInt();                                                   // Converte o texto para int
  mbS.Hreg(ESPHREGPIDME, (uint16_t)modo);                                   // Converte o dado int para armazenamento do registrador do escravo modbus
  Serial.println(modo);                                                     // Envia na serial o modo recebido
}                                                                           // Fim da Função*/SetTunings(Kp, Ki, Kd) 


void onConnectionEstablished(){                                             // Função executada no estabelecimento da conexão mQTT
  Serial.println("MQTT Connected...");                                      // Exibe a msg que a conexão foi estabelecida
  client.subscribe("/TkSys/W/PID/SPVolt", onMQTTMessageReceivedSP1V);       // Assina o tópico "/TkSys/W/PID/SPVolt" para receber a Setpoint/Tensão via MQTT
  client.subscribe("/TkSys/W/PID/Mode", onMQTTMessageReceivedPIDMODE);      // Assina o tópico "/TkSys/W/PID/Mode" para receber o modo de funcionamento do PID via MQTT
  client.publish("/TkSys/R/Lv1",String(Lv1.dFloat,2));          // Publica uma mensagem inicial para o tópico "/TkSys/R/Lv1"
  client.publish("/TkSys/R/PID/SPVolt",String(SP1.dFloat,2));   // Publica uma mensagem inicial para o tópico "/TkSys/R/PID/SPVolt"
  client.publish("/TkSys/R/PID/Mode", String(modo));             // Publica uma mensagem inicial para o tópico "/TkSys/R/PID/Mode"
  client.publish("/TkSys/R/PID/output", String(PWM_val));
}                                                                           // Fim da função*/

long remap(long in, long minin, long maxin, long minout, long maxout){      // Reimmplementação da função map, necessário pois a função original apresentou erro na conversão da escala invertida
  return (in - minin)*(maxout - minout)/(maxin - minin) + minout;
}                                                                           // Fim da função*/

void TaskPID(void *arg) {                                                   // Task PID
  lastWakeTimePID = xTaskGetTickCount();                                    // Armazena o tempo de execução atual
  while(1) {                                                                // Loop infinito da task
    ArduinoOTA.handle();                                                    // Aguardando envio do novo código para atualizar
    vTaskDelayUntil(&lastWakeTimePID, pdMS_TO_TICKS(updateTimeMsPID));      // Aguarda o tempo fixo de execução
    if(xSemaphoreTake(mutexV, pdMS_TO_TICKS(updateTimeMsPID/2))){           // Tenta pegar o mutex da tensão pelo periodo máximo da metade do tempo de atualização da task
      Input = (double)Lv1.dFloat;                                           // Converte o valor do tank em double e envia para a entrada do PID
      Setpoint = (double)SP1.dFloat;
      xSemaphoreGive(mutexV);                                               // Devolve o mutex da tensão
      if(modo == MANUAL){                                                   // Modo do PID: MANUAL (0) ou modo AUTOMATIC (não 0)
        PWM_val = remap((long)(Setpoint*100+0.5), 0, 320, 1024, 0);                // Converte o Setpoint double com uma casa decimal para um PWM/DAC (resolução 1024-0, invertido por causa do circuito de acionamento)
      }else{
        myPID.Compute();                                                    // Computa a iteração atual do PID
        PWM_val = remap((long)(Output*100+0.5), 0, 320, 1024, 0);                  // Converte o Output double com uma casa decimal para um PWM/DAC (resolução 1024-0, invertido por causa do circuito de acionamento)
      }
      writePump(PWM_val);                                                   // Manda escrever a saída do PID na bomba
      hora = ntp.getFormattedTime();
      client.publish("/TkSys/R/Time", hora);
      client.publish("/TkSys/R/PID/output", String(PWM_val));
    }else{                                                                  
      Serial.println("Falha ao pegar o mutex de tensão (PID)");             // Informa na serial (monitor) falha ao pegar o mutex
    }                                                                       
  }                                                                         
}                                                                           // Fim da task*/

void TaskModbusMS(void *arg) {                                              // Task Modbus Master e Slave
  lastWakeTimeM = xTaskGetTickCount();                                      // Armazena o tempo de execução atual
  floatModbus_t tempData1;                                                  // Variáveis locais para nível 1 e 2
  while(1) {                                                                // Loop infinito da task
// ############################################################################## MASTER ############################################################################################
    if (!ucm--){                                                            // Requisita dados a cada 1 segundo (seguindo as configurações)
      if (mbM.isConnected(gwInd)) {                                         // Checa se a conexão para o escravo Modbus está estabelecida
        mbM.readIreg(gwInd, GWIREGLVL1, tempData1.dVec, 2);                 // Manda comando de leitura do registrador lv1 do escravo Modbus
      } else {                                                              
        mbM.connect(gwInd);                                                 // Tenta conectar se não tiver conexâo
      }                                                                     
      ucm = UPDATE_M_COUNT-1;                                               // Atualiza o contador do mestre
    }                                                                       
    mbM.task();                                                             // Executa a tarefa do modbus Master
    
    vTaskDelayUntil(&lastWakeTimeM, pdMS_TO_TICKS(updateTimeMsMBMS));       // Aguarda o tempo fixo de execução
// ############################################################################## SLAVE #############################################################################################
    if (!ucs--) {                                                           // Requisita dados a cada 1 segundo (seguindo as configurações)
      Lv1.dVec[1] = tempData1.dVec[0];                                      // Inversão dos registradores para exibição correta do float
      Lv1.dVec[0] = tempData1.dVec[1];                                      // Inversão dos registradores para exibição correta do float
      mbS.Ireg(ESPIREGLVL1,   Lv1.dVec[1]);                                 // Assinala o mesmo valor do registrador de leitura do nivel 1 para o servidor
      mbS.Ireg(ESPIREGLVL1+1, Lv1.dVec[0]);                          
      client.publish("/TkSys/R/Lv1", String(Lv1.dFloat,2));                 // Publica o nível do tanque 1 sob o topico "/TksSys/R/Lv1"

      if(xSemaphoreTake(mutexV, pdMS_TO_TICKS(updateTimeMsMBMS/2))){        // Tenta pegar o mutex da tensão pelo periodo máximo da metade do tempo de atualização da task
        SP1.dVec[1] = mbS.Hreg(ESPHREGPIDSP);                               // Converte o valor do registrador de amazenamento para float
        SP1.dVec[0] = mbS.Hreg(ESPHREGPIDSP+1);                             // Converte o valor do registrador de amazenamento para float
        client.publish("/TkSys/R/PID/SPVolt",
                      String(SP1.dFloat,2));                    // Publica o valor do SetPoint ou do valor de tensão para o tópico "/TkSys/R/PID/SPVolt"
        modo = (int)mbS.Hreg(ESPHREGPIDME);                                 // Converte o valor do registrador de amazenamento referente ao modo para int
        myPID.SetMode(modo);                                                // Define o novo modo no controlador PID
        client.publish("/TkSys/R/PID/Mode", String(modo));       // Publica o valor do modo para o tópico "/TkSys/R/PID/Mode"
        xSemaphoreGive(mutexV);                                             // Devolve o mutex da tensão
      }else{                                                                
        Serial.println("Falha ao pegar o mutex de tensão (Slave)");         // Informa na serial (monitor) falha ao pegar o mutex
      }                                                                     
      ucs = UPDATE_S_COUNT-1;                                               // Atualiza o contador do escravo                                        
    }                                                                       
    mbS.task();                                                             // Executa a tarefa do modbus Slave
    client.loop();                                                          // Executa as tarefas do MQTT
  }                                                                         
}                                                                           // Fim da task*/

void setup() {
  Serial.begin(baud);                                                       // Configura a taxa da porta serial do monitor
  if (!WiFi.config(espIP, gatewaySTD, subnet, primaryDNS, secondaryDNS)) {  // Configura os dados do wifi (IP, Gateway padrão, Máscara de sub-rede, dns padrão e secundário) 
    Serial.println("STA Failed to configure");                              // Se falha, avisa pela serial (monitor)
  }
  WiFi.begin(ssid, pass);                                                   // Configura a rede e senha a qual deve se conectar o esp32 e inicia a conexão
  
  while (WiFi.status() != WL_CONNECTED) {                                   // Enquanto não conecta, fica aguardando
    delay(500);
    Serial.print(".");
  }

  WiFi.setSleep(false);                                                     // Desliga o modo power save do wifi (diminui a latência)
  //Definições de callbacks
  ArduinoOTA                                                                //Criação do objeto para envio do código via OTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  
  ntp.begin();               

  ArduinoOTA.begin();                                                       // Iniciando OTA
  
  Serial.println("");                                                       // Informa na serial (monitor):
  Serial.println("WiFi connected");                                         // se a rede foi conectada
  Serial.println("IP address: ");                                           
  Serial.println(WiFi.localIP());                                           // o endereço IP
  
  mbM.client();                                                             // Inicia a configuração do Client Modbus (Mestre)
  mbS.server();                                                             // Inicia a configuração do Servidor Modbus (Escravo)
  mbS.addIreg(ESPIREGLVL1, 0, 2);                                           // Adiciona ao escravo os registradores para o nível do tanque 1
  mbS.addHreg(ESPHREGPIDSP, 0, 2);                                          // Adiciona ao escravo os registradores para a setpoint do tanque 1 (ou tensão da bomba)
  mbS.addHreg(ESPHREGPIDME, 0, 1);                                          // Adiciona ao escravo o registrador para o modo de execução do PID
  mbS.addHreg(ESPHREGPIDKP, 0, 2);                                          // Adiciona ao escravo o registrador para a constante proporcional do PID
  mbS.addHreg(ESPHREGPIDKI, 0, 2);                                          // Adiciona ao escravo o registrador para a constante integrativa do PID
  mbS.addHreg(ESPHREGPIDKD, 0, 2);                                          // Adiciona ao escravo o registrador para a constante derivativa do PID
  // Cria o semáforo
  mutexV = xSemaphoreCreateMutex();                                         // cria o semáforo para a manipulação da tensão
  
  //Inicia o PID
  Setpoint = 0.0;                                                           // Define o valor de Setpoint inicial
  SP1.dFloat = (float)Setpoint;                                             // Atribui o setpoint a variável correspondente para o modbus
  mbS.Hreg(ESPHREGPIDSP, SP1.dVec[1]);                                      
  mbS.Hreg(ESPHREGPIDSP+1, SP1.dVec[0]);                                    // Armazena o valor de Sp no registrador correspondente
  myPID.SetMode(modo);                                                      // Define o modo do controlador PID
  mbS.Hreg(ESPHREGPIDME, (uint16_t)modo);                                   // Armazena o valor inicial do modo no registrador correspondente no modbus escravo
  myPID.SetSampleTime(updateTimeMsPID);                                     // Define a taxa de atualização do PID
  myPID.SetOutputLimits(PIDOUTPUTMIN,PIDOUTPUTMAX);                         // Define os valores minimo e máximo da saída do PID
  Kp.dFloat = KPINITIAL;                                                    // Armazena o valor default do Kp na variável correspondente
  mbS.Hreg(ESPHREGPIDKP, Kp.dVec[1]);                                       
  mbS.Hreg(ESPHREGPIDKP+1, Kp.dVec[0]);                                     // Armazena o valor de Kp no registrador correspondente no modbus escravo
  Ki.dFloat = KIINITIAL;                                                    // Armazena o valor default do Ki na variável correspondente
  mbS.Hreg(ESPHREGPIDKI, Ki.dVec[1]);                                       
  mbS.Hreg(ESPHREGPIDKI+1, Ki.dVec[0]);                                     // Armazena o valor de Ki no registrador correspondente no modbus escravo
  Kd.dFloat = KDINITIAL;                                                    // Armazena o valor default do Kd na variável correspondente
  mbS.Hreg(ESPHREGPIDKD, Kd.dVec[1]);                                       
  mbS.Hreg(ESPHREGPIDKD+1, Kd.dVec[0]);                                     // Armazena o valor de Kd no registrador correspondente no modbus escravo
  Lv1.dFloat = 0.0;                                                         // Assinala o valor inicial da variável de leitura do nivel 1 para o servidor
  mbS.Ireg(ESPIREGLVL1,   Lv1.dVec[1]);                                     
  mbS.Ireg(ESPIREGLVL1+1, Lv1.dVec[0]);                                     // Assinala o valor inicial do registrador de leitura do nivel 1 para o servidor
  
  ledcAttachPin(PUMP_PWM_PIN, PWM_CH);                                      // Assinala o pino do PWM para a bomba com o canal do PWM utilizado
  //ledcAttachPin(2, PWM_CH);                                                 // Assinala o pino do 2 (led embutido) com o canal do PWM utilizado para fins de debug
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);                                     // Configura o canal do PWM com a frequência e a resolução escolhidas
  ntp.forceUpdate();
  
  if(mutexV != NULL){                                                       // Se a criação foi bem sucedida ativa as tasks
    // Cria as tasks para o PID e o Modbus master e slave                     
    xTaskCreatePinnedToCore(TaskPID,                                        // Nome da task a ser executada
                            "TaskPIDOnProCore",                             // Nome da task para debug
                            2048,                                           // Tamanho da pilha da task
                            NULL,                                           // Argumento a ser passado para a task
                            5,                                              // Prioridade da task
                            NULL,                                           // ID único da task, normalmente não usado
                            PRO_CPU_NUM);                                   // Afinidade da task (processador para o qual deve ser atribuida)*/
    xTaskCreatePinnedToCore(TaskModbusMS,                                   // Nome da task a ser executada
                            "TaskModbusMSOnAppCore",                        // Nome da task para debug
                            2048,                                           // Tamanho da pilha da task
                            NULL,                                           // Argumento a ser passado para a task
                            4,                                              // Prioridade da task
                            NULL,                                           // ID único da task, normalmente não usado
                            APP_CPU_NUM);                                   // Afinidade da task (processador para o qual deve ser atribuida)*/
    
    
    //client.enableDebuggingMessages();                                     // Habilita as mensagens de depuração do MQTT a serem enviadas pela serial (Monitor)
  }else{                                                                    
    Serial.println("Falha ao criar o mutex. Programa não iniciado!");       // Se a criação do mutex não foi bem sucedida, envia aviso pela serial
  }
}

void loop() {
}