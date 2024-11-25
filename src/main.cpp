/* Blue_Pill_Lora_Transmitter_with_RFM95_BPLTwR_23.09.2021-01
Teste do STM32L476 Nucleo com o módulo LoRa RFM95
  Funcionou ok e transmite para o ESP32 com o NRF95 qdo o botão do pushbotton é pressionado.
  Ligação do LoRa RFM95 - STM32 Bluepill
  Semtech SX1276/77/78/79	Arduino(STM32F103CB)
    VCC	3.3V
    GND	GND
    SCK	SCK (PA5)
    MISO	MISO (PA6)
    MOSI	MOSI (PA7)
    NSS	(PA4)
    RESET	(PA0)
    DIO0	(PA1)
  * 12.01.2021 - Funcionou, está recebendo com o Bluepill ligado direto no Lora.
  * 17.01.2021 - revisando e passando para a placa feita com STM32 Bluepill - Ok envia Distância e nome do sensor e contagem de pacote
  * implementando o deep sleep no stm32: OK
  * Tentando salvar dados na memo rtc: não dá
  * Tentando implementar o RTC clock no próprio RTC do STM32:
  * TODO: ten que ver para usar o próprio RTC do STM32 e melhorar a lógica
  * Configurando LoRa Simple Gateway/Node Exemple: 
    https://github.com/sandeepmistry/arduino-LoRa/tree/master/examples/LoRaSimpleNode
  * Ajustando para que entre em deepSleep assim que receber a hora do Gateway
  * 02.02.21 - vendo se resolve o volta a receber com reset no1 PA0 ==> resolvido recebe ok a data e hora do gateway
  * 05.02.21 - Ajustando para acordar do DS a cada 29min
  * 10.02.21 - O TinyRTC está configurado e mostrando a hora corretamente.
  * entrou em operação, a principio esta ok, Está em testes.
  * 17.02.2021 logica para se nao receber a data do gateway depois de 2min entra e acorda a cada minuto do deep sleep
  * 01.03.21 teste do IWatchdog
  * 02.03.21 - Limpando o Sketch e reativando nos sensores para testes
  *          - Definida a primeira versão de produçãoem testes nos sensores
  * 23.09.21 - Ajustes finais antes do segundo envio para Caçador
  * 04.10.21 - fica acorado no máximo 1h senão dorme por 24hs
  * 29.10.2023 - Tentando resolver o problema do Sensor-01 de Caçador que não está atualizando os gráficos, parece que fica hibernando por 24hs.
  *  Estava hibernando por 24hs, corrigi. À principio resolveu e o Sensor-01 que está com essa versão, voltou a funcionar.
  *  O Sensor-02 está com outra versão: BPLTwR_04.10.2021-05-LSE
  * 
  * 23.11.2024  - Alterando o código para ler e hibernar por 1 minuto entre leituras so ultrasom.
  *             - 1min on e 1min off, ajustar para não ficar tanto tempo on e desligar assim que transmitir
  * 24.11.2024  - alteramdo a função readUltrasom() para trabalhar com a mediana
  * 
  * TODO:
  * se funcionar sem hibernação, ajustar a leitura do ultrassom pela mediana.
  * reativar a hibernação e fazer dormir por 1min
  * Precisa medir com a régua, para saber se está funcional
*/  

#include <stdlib.h>

//Identificação para o gateway saber quem está enviando os dados
#define ID "Sensor_BluePill-01" //<=== MUDAR AQUI ====

#define enableSerialLog //Enable Serial debug
#define enableWatchDog //desativado o watchdog
#include <Arduino.h>
#include <STM32LowPower.h> //Deep Sleep
int vaiDormir_flag = 0;
#define enableRTCstm32 //STM32 internal RTC
#ifdef enableRTCstm32
  #include <STM32RTC.h>
#endif
//#define enableTinyRTC //não usado pois está bugado

#ifdef enableWatchDog
  #include <IWatchdog.h>
  const int ledPin = PB13;
#endif
#ifdef enableTinyRTC
  //Armazenamento da data e hora
  int   year    = 0;
  int   month   = 0;
  int   day     = 0;
  int   hours    = 0;
  int   minutes     = 0;
  int   seconds     = 0;
  float timezone = 0;
  String DATE_FULL3;

  // Date and time functions using a DS1307 RTC connected via I2C and Wire lib
  #include "RTClib.h"

  RTC_DS1307 rtc;

  char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

  void startTinyRTC() {
    while (!Serial); // wait for serial port to connect. Needed for native USB

    if (! rtc.begin()) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      //abort();
    }

    if (! rtc.isrunning()) {
      Serial.println("RTC is NOT running, let's set the time!");
      // When time needs to be set on a new device, or after a power loss, the
      // following line sets the RTC to the date & time this sketch was compiled
      ////rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }

    // When time needs to be re-set on a previously configured device, the
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  void setTime(){
      // Set the time
      //rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
      rtc.adjust(DateTime(year - 2000, month, day, hours, minutes, seconds));
  }
  void readTimeTinyRTC() {
      DateTime now = rtc.now();
      char DATE_FULL_RTC[] = "DD/MM/YYYY, hh:mm:ss";
      /*
      Serial.print(now.year(), DEC);
      Serial.print('/');
      Serial.print(now.month(), DEC);
      Serial.print('/');
      Serial.print(now.day(), DEC);
      Serial.print(" (");
      Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
      Serial.print(") ");
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      Serial.print(now.second(), DEC);
      Serial.println();
      */
      Serial.print("Data e hora atual: "); Serial.println(now.toString(DATE_FULL_RTC));
      //Prepara para atualizar o grafico a cada 30minutos
      /*if (now.minute() == 30 or now.minute() == 0) {
        //Serial.println(rtc.getHours());
        //Serial.println(rtc.getMinutes());
        Serial.println("Time of Transmission to Server");
      }*/
  }
#endif

//Protótipos das funções
void readUltrasom();
float calcularMediana(int *array, int tamanho);
int comparar(const void *a, const void *b);

#ifdef enableRTCstm32
  boolean onReceive_flag = 0;
  /* Get the rtc object */
  STM32RTC& rtc = STM32RTC::getInstance();
  byte startUpMinute = 0;
  /* Change these values to set the current initial time */
   byte seconds = 0;
   byte minutes = 0;
   byte hours = 0;

  /* Change these values to set the current initial date */
   byte weekDay = 0;
   byte day = 0;
   byte month = 0;
   byte year = 0;

  void setupRTC(){
      // Select RTC clock source: LSI_CLOCK, LSE_CLOCK or HSE_CLOCK.
      // By default the LSI is selected as source.
      //rtc.setClockSource(STM32RTC::LSI_CLOCK); //3V3 ligado com diodo no VBAT
      rtc.setClockSource(STM32RTC::LSE_CLOCK); //3V3 ligado com diodo no VBAT
      rtc.begin(); // initialize RTC 24H format
  }  

  void setTime(){
      // Set the time
      rtc.setHours(hours);
      rtc.setMinutes(minutes);
      rtc.setSeconds(seconds);

      // Set the date
      rtc.setWeekDay(weekDay);
      rtc.setDay(day);
      rtc.setMonth(month);
      rtc.setYear(year);
  }

  void readTime(){
    // Print date...
    Serial.println("Data e hora armazenada no RTC Local");
    Serial.printf("%02d/%02d/%02d ", rtc.getDay(), rtc.getMonth(), rtc.getYear());

   // ...and time
   Serial.printf("%02d:%02d:%02d.%03d\n", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), rtc.getSubSeconds());
  }
#endif

#define enableUltraSom
#ifdef enableUltraSom
  #include <NewPing.h>
  
  const unsigned int trigPin = PA3; 
  const unsigned int echoPin = PA2; 
  long lastEchoDistance = 0;          // we want to keep these values after reset
  unsigned long pulseLength = 0;
  unsigned long distanciaLida = 0;
  unsigned long qtdMaxLeituras = 0;
  boolean ultrasonicActive  = true;

  //=============================================================================================================
  // Ultrasonic Distance Sensor setup
  void ultrasonic_setup(){
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);  
    #ifdef enableSerialLog
      Serial.println("Setting up Ultrasonic Sensor.");
    #endif
  } //end ultrasonic_setup

//   void readUltrasom() { //Meu código original
//     // ===== Sensor Ultrasonic =====

//     // precisa do for pq o primeiro valor da leitura da zero e se tiver só uma leitura ele entra em deep sleep com valor pulseLength=0                                                   
//         checadistancia: //Label para goto
//         for (int i=0  ; i < 3; i++) {
//             digitalWrite(trigPin, LOW);                    // send low to get a clean pulse     
//             delayMicroseconds(5);                          // let it settle
//             digitalWrite(trigPin, HIGH);                   // send high to trigger device
//             delayMicroseconds(10);                         // let it settle
//             pulseLength = pulseIn(echoPin, HIGH);          // measure pulse coming back   
//             distanciaLida = pulseLength / 58;             // calculate distance (cm)
//             Serial.print("distanciaLida: "); Serial.println(distanciaLida);
//             delay(50);

//             // Elimina picos de leitura erroneos do sensor mas só fica aqui por qtdMaxLeituras para evitar loop infinito
//             if (distanciaLida > 500 and qtdMaxLeituras < 200){ //descarta leituras maiores que 500cm
//               qtdMaxLeituras = qtdMaxLeituras + 1;
//               goto checadistancia;
//             }//if distanciaLida
//         } //for

//         if ((distanciaLida - lastEchoDistance) >= 1){            // check for change in distance só manda msg se mudar o valor > 1cm 
//             lastEchoDistance = distanciaLida;
//             //#ifdef enableSerialLog
//               Serial.print("Distancia Lida pelo Sensor Ultrasonico: ");
//               Serial.print(distanciaLida); Serial.println("cm");
//               Serial.println("");
//             //#endif
//             delay(50);
//         } //if
//       delay(500); 
//       }


// Função adaptada para calcular a mediana de 10 leituras do sensor ultrassônico e mostrá-las como distância lida
void readUltrasom() {
  int leituras[10];
  int tamanhoArray = sizeof(leituras) / sizeof(leituras[0]);

  // Faz 10 leituras consecutivas para calcular a mediana e eliminar leituras indevidas e outliers devido a reflexão do ultrasom
  for (int i = 0; i < tamanhoArray; i++) { //for1
    checadistancia: // Label para goto
    // for (int j = 0; j < 3; j++) { //for2
      digitalWrite(trigPin, LOW);
      delayMicroseconds(5);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      pulseLength = pulseIn(echoPin, HIGH);
      distanciaLida = pulseLength / 58;
      delay(50);

      if (distanciaLida > 500 || distanciaLida < 0) { //elimina leituras erroneas acima ou abaixo do range do sensor
        // qtdMaxLeituras = qtdMaxLeituras + 1;
        goto checadistancia;
      }
    // } //for2
    leituras[i] = distanciaLida;
  } //for1

  // Calcula a mediana
  float mediana = calcularMediana(leituras, tamanhoArray);

  // Verifica se a mediana mudou significativamente
  if (abs(mediana - lastEchoDistance) >= 1) { // check for change in distance só manda msg se mudar o valor > 1cm
    lastEchoDistance = mediana;

    Serial.print("Distancia Lida pelo Sensor Ultrasonico (mediana): ");
    Serial.print(mediana); 
    Serial.println("cm");
    Serial.println("");
  }

  delay(50); //para economizar bateria, pode-se reduzir esse tempo
}

// Função para calcular a mediana
float calcularMediana(int *array, int tamanho) {
  qsort(array, tamanho, sizeof(int), comparar);

  if (tamanho % 2 == 0) {
    return (float)(array[tamanho / 2 - 1] + array[tamanho / 2]) / 2;
  } else {
    return (float)array[tamanho / 2];
  }
}

int comparar(const void *a, const void *b) {
  return (*(int *)a - *(int *)b);
}

#endif //enableUltraSom

//////////////////////////////////////////////////// vaiDormir() //////////////////////////////////////////////////// 
 void vaiDormir() {
    if (vaiDormir_flag == 2) { //Não encontrou o gateway e hibernará por 1 minuto
      #ifdef enableWatchDog
        IWatchdog.reload();
      #endif
      // Serial.println("Não encontrou o Gateway, hibernando por 1 minuto..."); 
      Serial.println("Hibernando por 1 minuto..."); 
      delay (10);
      // LowPower.shutdown(1000 * 60 * 30); //hiberna por 30 min
      LowPower.shutdown(1000 * 60); //hiberna por 1 min
    }
    //Entra em Deep Sleep e acorda em horas cheias hh:00 ou hh:30
    if (vaiDormir_flag == 1) {
    //   //DateTime now = rtc.now();
    //   //int sleepTime = 59 - now.minute(); //TinyRTC
      #ifdef enableWatchDog
        IWatchdog.reload();
      #endif
      Serial.println("Hibernando por 1 minuto..."); 
      delay (10);
      LowPower.shutdown(1000 * 60); //hiberna por 1 min
    //   int sleepTime = 59 - rtc.getMinutes(); //STM32RTC

    //   if ( sleepTime <= 30) {
    //     //#ifdef enableSerialLog
    //       Serial.print("Vai dormir por: "); Serial.print(sleepTime); Serial.println(" minutos");
    //       delay (100);
    //     //#endif
    //     //vai dormir...
    //     LowPower.shutdown(1000 * 60 * sleepTime); //D.S por 1000ms* 60s * sleepTime/
    //   }

    //   else  {
    //     sleepTime = sleepTime -30;
    //     //#ifdef enableSerialLog
    //       Serial.print("Vai dormir por: "); Serial.print(sleepTime); Serial.println(" minutos");
    //       delay (100);
    //     //#endif
    //     //vai dormir...
    //     LowPower.shutdown(1000 * 60 *sleepTime); //D.S por 1000ms* 60s * sleepTime/
    //   }

    }
 
 }

#define enableLoRa
#ifdef enableLoRa
//Libraries for LoRa
  #include <SPI.h>
  #include <LoRa.h>
  //#include <LoRa_stm32.h> //lib alterada para o stm32
  //define the pins used by the LoRa transceiver module
  #define SCK PA5
  #define MISO PA6
  #define MOSI PA7
  #define SS PA4
  #define RST PA0
  #define DIO0 PA1
  const int csPin = PA4;          // LoRa radio chip select
  const int resetPin = PA0;        // LoRa radio reset
  const int irqPin = PA1;          // change for your board; must be a hardware interrupt pin

  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  #define BAND 915E6  //Brasil : 902-928 MHz

  //packet counter
  /////int readingID = 0; //passando para a memoria RTC (ver em qual sketch foi feito)
  int counter = 0;
  long readingID = 0;
  
  String LoRaMessage = "";
#endif

#ifdef enableLoRa
  void LoRa_rxMode(){
    LoRa.enableInvertIQ();                // active invert I and Q signals
    LoRa.receive();                       // set receive mode
  }

  void LoRa_txMode(){
    LoRa.idle();                          // set standby mode
    LoRa.disableInvertIQ();               // normal mode
  }

//==================================== Lora Callback void onReceive ===================================================
void onReceive(int packetSize) {
  onReceive_flag = 1;
  #ifdef enableSerialLog
    Serial.print("Pacote LoRa Recebido do Gateway: ");
  #endif
  
  int rssi;
  String dia01;
  String mes01;
  String ano01;
  String hora01;
  String minuto01;
  String segundo01;
  String dia02;
  String mes02;
  String ano02;
  String hora02;
  String minuto02;
  String segundo02;

	while (LoRa.available()) {
		//Aqui eu faço ele entrar em deep sleep assim que receber a mensagem do Gateway
    
    String LoRaData = LoRa.readString();
		// LoRaData format: dia/mes@ano_hora,minuto%segundo
		// String example:2/2@21_20,57%27
		Serial.print("LoRaData: "); Serial.println(LoRaData);
    //recebimento da data01
    int pos1 = LoRaData.indexOf('/');
		int pos2 = LoRaData.indexOf('@');
		int pos3 = LoRaData.indexOf('_');
    int pos4 = LoRaData.indexOf(',');
		int pos5 = LoRaData.indexOf('%');

    //recebimento da data02
    int pos6 = LoRaData.indexOf('|');
		int pos7 = LoRaData.indexOf('?');
		int pos8 = LoRaData.indexOf('#');
    int pos9 = LoRaData.indexOf('$');
		int pos10 = LoRaData.indexOf('!');   
    int pos11 = LoRaData.indexOf('*');   

    dia01 = LoRaData.substring(0, pos1);
		mes01 = LoRaData.substring(pos1 +1, pos2);
		ano01 = LoRaData.substring(pos2+1, pos3);
    hora01 = LoRaData.substring(pos3+1, pos4);
    minuto01 = LoRaData.substring(pos4+1, pos5);
		segundo01 = LoRaData.substring(pos5+1, pos6);  

		dia02 = LoRaData.substring(pos6 + 1, pos7);
		mes02 = LoRaData.substring(pos7 +1, pos8);
		ano02 = LoRaData.substring(pos8+1, pos9);
    hora02 = LoRaData.substring(pos9+1, pos10);
    minuto02 = LoRaData.substring(pos10+1, pos11);
		segundo02 = LoRaData.substring(pos11+1, LoRaData.length());  

    Serial.println("Data e hora recebidas do Gateway:");
    Serial.print("Dia:"); Serial.println(dia01);
    Serial.print("Mês:"); Serial.println(mes01);
    Serial.print("Ano:"); Serial.println(ano01);
    Serial.print("Hora:"); Serial.println(hora01);
    Serial.print("Minuto:"); Serial.println(minuto01);
    Serial.print("Segundo:"); Serial.println(segundo01);
  
    // //chega se a data não chegou com erro de transmissão
    // if (dia01 == dia02 and mes01 == mes02 and ano01 == ano02 and hora01 == hora02 and minuto01 == minuto02 and segundo01 == segundo02 )
    // {
    //   hours = hora01.toInt();
    //   minutes = minuto01.toInt();
    //   seconds = segundo01.toInt();
    //   rtc.setTime(hours, minutes, seconds);

    //   day = dia01.toInt();
    //   month = mes01.toInt();
    //   year = ano01.toInt();
    //   rtc.setDate(weekDay, day, month, year);
    //   vaiDormir_flag = 1;
    //   readTime();

    //   enableBackupDomain();
    //   setBackupRegister(3, 10); //indica que tem que entrar em deep sleep
    //   disableBackupDomain();
    // }
      // Get RSSI
      #ifdef enableSerialLog
        //rssi = LoRa.packetRssi();
        //Serial.print(" with RSSI ");    
        //Serial.println(rssi);
      #endif

  }
}
  
  void LoRa_sendMessage(String message) {
    LoRa_txMode();                        // set tx mode
    LoRa.beginPacket();                   // start packet
    LoRa.print(message);                  // add payload
    LoRa.endPacket(true);                 // finish packet and send it
  }

  void onTxDone() {
    #ifdef enableSerialLog
      Serial.println("TxDone");
    #endif
    LoRa_rxMode();
  }

  boolean runEvery(unsigned long interval)
  {
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      return true;
    }
    return false;
  }

void checkonReceive() { //TODO ver se essa é a funçã oque recebe o retorno do gateway
    if (onReceive_flag == 0) {
      int time = (rtc.getMinutes() - startUpMinute);
      Serial.print("time:   "); Serial.println(time);
      if (rtc.getMinutes() - startUpMinute >= 2 ) {
      //#ifdef enableSerialLog
            Serial.print("Não recebeu a Data do Gateway! Vai dormir por 30 segundos ");
            delay (100);
          //#endif
          //vai dormir por 30 segundos...
          // LowPower.shutdown(1000 * 30); //D.S por 1000ms* 30s * sleepTime/
        }
    }
}

boolean runClockEvery(unsigned long interval)
  {
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      return true;
    }
    return false;
  }

  //=============================================================================================================
  //Initialize LoRa module
  void start_LoRa(){
    //LoRa.setTxPower(20);
    //SPI LoRa pins
    LoRa.setPins(csPin, resetPin, irqPin);
    //LoRa.setPins(Lora_SS, Lora_RST, Lora_DIO0); //pinos definidos diretamente na lib
    //SPI.begin(SCK, MISO, MOSI, SS); //pinos definidos diretamente na lib
    

    while (!LoRa.begin(BAND) && counter < 10) {
      Serial.print(".");
      counter++;
      delay(500);
    }
    if (counter == 10) {
      Serial.println("LoRa initialization Failed!"); 
          // delay (100);
    }
        if (counter < 10) {
          #ifdef enableSerialLog
            Serial.println("LoRa initialization OK!"); 
          #endif
    }

    //Setup receiver para receber o update da hora:
    #ifdef enableSerialLog
      Serial.println("LoRa Receiver Callback com LoRa Reset na PA0");
      Serial.println("LoRa Simple Node");
      Serial.println("Only receive messages from gateways");
      Serial.println("Tx: invertIQ disable");
      Serial.println("Rx: invertIQ enable");
      Serial.println();
    #endif
    // register the receive callback
    LoRa.onReceive(onReceive); 
    LoRa.onTxDone(onTxDone);
    LoRa_rxMode();
  } //end start_LoRa

  void sendReadings() {
    if (runEvery(5000)) { // repeat every 5 sec //TODO se recebe confirmaçã ode recebiment odo gateway, não pode enviar mais para economizar bateria
      LoRaMessage = String(ID) + "/" + String(distanciaLida) + "&" + String(distanciaLida);
      //Send LoRa packet to receiver
      LoRa_sendMessage(LoRaMessage); // send a LoRaMessage
      #ifdef enableSerialLog
        Serial.print("Sending packet N°: ");   Serial.println(readingID);
        Serial.print("LoRaMessage: ");   Serial.println(LoRaMessage);
      #endif
      readingID++;
    }
  }

#endif //enableLoRa

//////////////////////////////////////////////////// sketchSetup //////////////////////////////////////////////////// 
// Mostra dados do sketch e configura a serial
void sketchSetup() {
  Serial.begin(115200); 
  Serial.print("\nIniciando Sensor "); Serial.println(String(ID)); 
    // Serial.println("\nIlha 3d");
    // Serial.println("\n(48) 99852-6523");
    // Serial.println("\nwww.ilha3d.com");
    // Serial.println("\n");
    Serial.println("System Version: SAPM_Sensor_BluePill_20241123-01 - mediana");
    Serial.println("");
}

void setup() {
  sketchSetup();
  pinMode(PC13, OUTPUT); // initialize digital pin PC13 (LED) as an output.
  
  #ifdef enableWatchDog // caso não encontrar o gateway vai hibernar
    enableBackupDomain();
    if ( getBackupRegister(2) != 0) { //indica que tem que entrar em deep sleep
      Serial.println("Sistema reinicializado pelo WatchDog ... === Irá entrar em hibernação ... ===");
      setBackupRegister(2, 0);
      delay(100);
      setupRTC();
      LowPower.begin();
      vaiDormir_flag = 2; //flag de hibernação 1min
      vaiDormir();    
    }

    if (getBackupRegister(3) != 0 ) { //indica que tem que entrar em deep sleep
      Serial.println("Sistema reinicializado pelo WatchDog preparando para hibernação...");
      setupRTC();
      enableBackupDomain();
      setBackupRegister(3, 0);
      delay(100);
      LowPower.begin();
      vaiDormir_flag = 1; //flag de deep sleep normal
      vaiDormir();
    }
    disableBackupDomain();
    IWatchdog.begin(10000000); // Init the watchdog timer with 10 seconds timeout
  #endif

  //Enable the LoRa power supply
  pinMode(PB13,OUTPUT); 
  digitalWrite(PB13, HIGH); 
  
  delay(25); //Enable MP2307 in the MINI360 power regulator, it is needed 16ms to activate Vout

  LowPower.begin(); //STM32 deep sleep

  #ifdef enableRTCstm32
    setupRTC();
    setTime();
    startUpMinute = rtc.getMinutes();
  #endif

  #ifdef enableTinyRTC
    startTinyRTC();
    //setTime(); //TODO: O TinyRTC já está sincronizado. Inibo o Time sync do GSM
  #endif

  ultrasonic_setup();
  start_LoRa();
  
  //readTimeTinyRTC();
}

void loop() {
  readUltrasom();
  sendReadings();

  if (runClockEvery(1000 * 10)) { //aqui diz qto tempo fica ativo?? 10s
    vaiDormir_flag = 2; //Flag que sinaliza que tem que hibernar POR 1MIN
    enableBackupDomain();
    setBackupRegister(2, 10); //indica que irá hibernar
    disableBackupDomain();
  }
  vaiDormir();
  //checkonReceive(); //TODO desativo pq não tem como diferenciar qdo volta do boot pelo watchdog precisaria ter um flag gravado em memo rtc
  // delay(60000); //faz uma leitura por minuto
  #ifdef enableWatchDog
    IWatchdog.reload();
  #endif
}
