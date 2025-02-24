/* Code for Sensor-02:
 * Blue_Pill_Lora_Transmitter_with_RFM95_BPLTwR_23.09.2021-01
 * Alexandre Nuernberg - alexandreberg@gmail.com
 * 
 * Project available on: https://github.com/alexandreberg/SAPM_Sensor_BluePill
 * 
 * RFM95 LoRa Connection - STM32 Bluepill
 *    VCC - 3.3V
 *    GND - GND
 *    SCK - SCK (PA5)
 *    MISO - MISO (PA6)
 *    MOSI - MOSI (PA7)
 *    NSS - (PA4)
 *    RESET - (PA0)
 *    DIO0 - (PA1)
 * 
 * BackupRegisters:
 * See https://community.st.com/t5/stm32-mcus/how-to-use-the-stm32-s-backup-registers/ta-p/49892
 *
 * Backup registers can be written/read and protected and have the option of being preserved in VBAT mode when the VDD domain is powered off. 
 * The BackUp Registers are part of the RTC peripheral so we will need to enable the RTC to be able to access them. 
 * The STM32 Blue Pill, which typically uses the STM32F103C8T6 microcontroller, has 10 backup registers.
 * Each register is 16 bits wide, providing a total of 20 bytes of data that can be stored in the backup domain.
 * 
 * (!) To be able to preserve the backup registers through a power cycle, VBAT must remain powered when VDD is removed, this is called the VBAT mode.
 *
 * 23.11.2024 - Changing the code to read and hibernate for 1 minute between ultrasound readings.
 *             - 1min on and 1min off, adjust to not stay on for so long and turn off as soon as it transmits
 * 24.11.2024 - changing the readUltrasonic() function to work with the median
 * 24.12.2024 - Cleaning and organizing the file
 * 09.feb.2025 - changing code from if (readingDistance > 500 || readingDistance < 0) { to:
 *              if (readingDistance > 500 || readingDistance <= 0) {  to see if eliminates de zero value readings in the gateway.
 * 
 * BackupRegister Values:
 * Register - Value - Description
 * 0 - 
 * 1 - 
 * 2 - != 0 - indicates that  STM32 should to go into deepsleep
 * 3 - != 0 -//indicates that  have to go into deepsleep
 *
 * goToSleep_flag = 0; i boot flag
 * goToSleep_flag = 1; //hibernation flag normal deepsleep ?????
 * goToSleep_flag = 2; //hibernation flag 1min ?????
 * 
 * TODO:
 * reactivate hibernation and make it sleep for 1min
 * OK - implementing sleep in LoRa module (14 feb 2025)
 * OK - Changing clock source from LSE_CLOCK to LSI_CLOCK
 * OK - Disabling BackupRegisters
 * - Reactivate IWatchdog 
 * - Restart ultrassonic and lora modules
 * - Test power consumption
 * - See if do i need backup registers to work with the watchdog
 * - Takes too much time to send the LoRa packets (it repeats many times the ultrasonic reading cycle, change to send the LoRa message as soon sa take the 1st US reading)
 * 
 */  

/*********************************************** Sensor Description ***********************************************/
//REVIEWED
#define sensor_id "Sensor_02"         // <<=== Sensor identification  ==>> CHANGE HERE!!
#define sensor_location "Lab"         // <<=== Sensor location        ==>> CHANGE HERE!!

/*********************************************** Macro Definitions ***********************************************/
//REVIEWED
//Enable (uncommenting) or disable (commenting out) services and periferals
#define enableSerialLog         // enable Serial debug on console
#define enableWatchDog          // enable watchdog for deepsleep
#define enableUltrasonic        // enable Ultrasonic Sensor
#define enableRTCstm32          // using STM32 internal RTC Clock
//#define enableTinyRTC         // TODO: not used because de SPI bus freezes and loose the connection!
#define enableLoRa              // enable LoRa communication

/*********************************************** Library Definitions ***********************************************/
//REVIEWED
#include <Arduino.h>
#include <stdlib.h>
#include <STM32LowPower.h>      //Deep Sleep for STM32
#include <SPI.h>

#ifdef enableRTCstm32
  #include <STM32RTC.h>
#endif

#ifdef enableWatchDog
  #include <IWatchdog.h>
#endif

#ifdef enableTinyRTC
  #include "RTClib.h"           //Date and time functions using a DS1307 RTC connected via I2C and Wire lib (Not used because it is loosing connection with the SPI BUS)
#endif

#ifdef enableUltrasonic
  #include <NewPing.h>
#endif

#ifdef enableLoRa
  #include <LoRa.h>
#endif

/*********************************************** Global Variables ***********************************************/
//REVIEWED
String version = "System Version: SAPM_Sensor_BluePill_20250220-01 - median - LoRa Sleep - LSI_CLOCK - no BackupRegisters";  // ==> CHANGE HERE! <==

#ifdef enableWatchDog
  const int ledPin = PB13;      // TODO: Just to have visual information that it is working. 
#endif

#ifdef enableTinyRTC            // Not used
  //Store date and time
  int   year    = 0;
  int   month   = 0;
  int   day     = 0;
  int   hours    = 0;
  int   minutes     = 0;
  int   seconds     = 0;
  float timezone = 0;
  String DATE_FULL3;

  RTC_DS1307 rtc;

  char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
#endif //enableTinyRTC

#ifdef enableRTCstm32           // Working
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
  
#endif //enableRTCstm32

#ifdef enableUltrasonic
  const unsigned int triggerPin = PA3; 
  const unsigned int echoPin    = PA2; 
  // long lastEchoDistance = 0;             // We want to keep these values after reset
  unsigned long pulseLength = 0;
  unsigned long readingDistance = 0;        // Measured distance in centimeters
  // unsigned long maxReadingNumber = 0;    // Number of ultrasonic readings to do the calculation of mean and average
  // boolean ultrasonicActive  = true;
#endif //enableUltrasonic

int goToSleep_flag = 0;         // Flag to enter in deep sleep mode

#ifdef enableLoRa
  //define the pins used by the LoRa transceiver module
  #define SCK           PA5
  #define MISO          PA6
  #define MOSI          PA7
  #define SS            PA4
  #define RST           PA0
  #define DIO0          PA1
  const int csPin =     PA4;         // LoRa radio chip select
  const int resetPin =  PA0;         // LoRa radio reset
  const int irqPin =    PA1;         // Change for your board; must be a hardware interrupt pin of the STM32 Bluepill

  // Define LoRa Communication Band:
  #define BAND 915E6  /*  915E6 for Brazil (902-928 MHz)
                          433E6 for Asia
                          866E6 for Europe
                          915E6 for North America */

  int lora_startup_counter = 0;     // Counter to check if LoRa chip started communication propperly
  long readingID = 0;               // Sending packet N°
  
  String LoRaMessage = "";          // String to store the LoRa Message that should be sent
#endif  //enableLoRa

/*********************************************** Function Prototypes ***********************************************/
//REVIEWED
void activate_power_monitor_pins();
void sketchSetup();
void readUltrasonic();
float calculateMedian(int *array, int arraySize);
int compareReadings(const void *a, const void *b);

#ifdef enableTinyRTC
  //void startTinyRTC();
  //void setTime();
  //void readTimeTinyRTC();
#endif //enableTinyRTC

#ifdef enableRTCstm32
  void setupRTC();
  void setTime();
  void readTime();
#endif //enableRTCstm32

#ifdef enableUltrasonic
  void ultrasonic_setup();
  void readUltrasonic();
  float calculateMedian(int *array, int arraySize);
  int compareReadings(const void *a, const void *b);
#endif //enableUltrasonic

#ifdef enableLoRa
  void LoRa_rxMode();
  void LoRa_txMode();
  void onReceive(int packetSize);
  void LoRa_sendMessage(String message);
  void onTxDone();
  boolean runEvery(unsigned long interval);
  void checkonReceive();
  boolean runClockEvery(unsigned long interval);
  void start_LoRa();
  void sendReadings();
#endif //enableLoRa

void goToSleep();

/*********************************************** End Function Prototypes *******************************************/
// TODO: Need to be better documented and clarified!!!!
void setup() {
  activate_power_monitor_pins();      // Activate logic ppins to measure power consumption
  sketchSetup();                      // Setup of the Serial log and initial serial setup
  

  // pinMode(PC13, OUTPUT);              // Initialize digital pin PC13 (LED) as an output.
  
  // #ifdef enableWatchDog               // TODO: if the LoRa gateway is not found go to deep sleep ==> é necessário?
    // enableBackupDomain();             // Function of .platformio\packages\framework-arduinoststm32\cores\arduino\stm32\backup.h 

    // if ( getBackupRegister(2) != 0) { // indicates that  STM32 should to go into deepsleep
      // Serial.println("Sistema reinicializado pelo WatchDog ... === Irá entrar em hibernação ... ===");
      // setBackupRegister(2, 0);
      // delay(100);
      // setupRTC();
      // LowPower.begin();
      // goToSleep_flag = 2; //hibernation flag 1min ?????
      // goToSleep();    
    // }

    // // if (getBackupRegister(3) != 0 ) { //indicates that  have to go into deepsleep
    //   Serial.println("Sistema reinicializado pelo WatchDog preparando para hibernação...");
    //   setupRTC();
    //   // enableBackupDomain();
    //   // setBackupRegister(3, 0);
    //   // delay(100);
    //   LowPower.begin();
    //   goToSleep_flag = 1; //hibernation flag normal deepsleep ?????
    //   goToSleep();
    // }
    // disableBackupDomain();
    // IWatchdog.begin(10000000); // Init the watchdog timer with 10 seconds timeout
  // #endif

  // Enable the LoRa power supply activating the enable pin of the PS (Not used in this project)
  // pinMode(PB13,OUTPUT); 
  // digitalWrite(PB13, HIGH); 
  // delay(25); //Enable MP2307 in the MINI360 power regulator, it is needed 16ms to activate Vout

  LowPower.begin(); //STM32 deep sleep

  #ifdef enableRTCstm32
    setupRTC();
    // setTime(); // Not used in this project
    // startUpMinute = rtc.getMinutes(); // Not used in this project
  #endif

  #ifdef enableTinyRTC
    startTinyRTC(); // Not used in this project
    //setTime(); //TODO: O TinyRTC já está sincronizado. Inibo o Time sync do GSM
  #endif

  ultrasonic_setup();
  start_LoRa();
  
  //readTimeTinyRTC();  // Not used in this project
}

/*********************************************** loop () ***********************************************/
void loop() {
  // TODO: 
  readUltrasonic();
  sendReadings();

  // if (runClockEvery(1000 * 10)) { //Does it say here how long it stays active?? 10s
  //   goToSleep_flag = 2; //Flag that indicates that have to hibernate FOR 1MIN
  //   // enableBackupDomain();
  //   // setBackupRegister(2, 10); 
  //   // disableBackupDomain();
  // }
  goToSleep();
  //checkonReceive(); //TODO desativo pq não tem como diferenciar qdo volta do boot pelo watchdog precisaria ter um flag gravado em memo rtc
  // delay(60000); //faz uma leitura por minuto
  #ifdef enableWatchDog
    IWatchdog.reload();
  #endif

  Serial.println("Em loop...");
  readTime();
  
  delay(1000);
}

/*********************************************** End loop () ***********************************************/

/*********************************************** Function Definitions ***********************************************/
//////////////////////////////////////////////////// sketchSetup //////////////////////////////////////////////////// 
//REVIEWED
// Shows system infomation and configures serial interface
void sketchSetup() {
  Serial.begin(115200); 

  Serial.print("\nStarting Sensor: " + String(sensor_id) + " on " + String(sensor_location));
  Serial.println("\nIlha 3d");
  Serial.println("\nwww.ilha3d.com");
  Serial.println("\n");
  Serial.println(String (version));
  Serial.println("");
}

#ifdef enableTinyRTC
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
#endif //enableTinyRTC

#ifdef enableRTCstm32
void setupRTC(){
      // Select RTC clock source: LSI_CLOCK, LSE_CLOCK or HSE_CLOCK.
      // By default the LSI is selected as source.
      rtc.setClockSource(STM32RTC::LSI_CLOCK); 
      // rtc.setClockSource(STM32RTC::LSE_CLOCK); //3V3 wired with a diode on VBAT
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
#endif //enableRTCstm32

#ifdef enableUltrasonic
  //REVIEWED
  // Ultrasonic Distance Sensor setup
  void ultrasonic_setup(){
    digitalWrite(PB12, 0);        // put power monitor pin in 1 so we can see it in Nordic Power Profiller
    Serial.println("Power monitor D7 - PB12 = 0 Func: ultrasonic_setup()");
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);  
    #ifdef enableSerialLog
      Serial.println("Setting up Ultrasonic Sensor.");
    #endif
  } //end ultrasonic_setup

//   void readUltrasonic() { //Meu código original
//     // ===== Sensor Ultrasonic =====

//     // precisa do for pq o primeiro valor da leitura da zero e se tiver só uma leitura ele entra em deep sleep com valor pulseLength=0                                                   
//         check_distance: //Label para goto
//         for (int i=0  ; i < 3; i++) {
//             digitalWrite(triggerPin, LOW);                    // send low to get a clean pulse     
//             delayMicroseconds(5);                          // let it settle
//             digitalWrite(triggerPin, HIGH);                   // send high to trigger device
//             delayMicroseconds(10);                         // let it settle
//             pulseLength = pulseIn(echoPin, HIGH);          // measure pulse coming back   
//             readingDistance = pulseLength / 58;             // calculate distance (cm)
//             Serial.print("readingDistance: "); Serial.println(readingDistance);
//             delay(50);

//             // Elimina picos de leitura erroneos do sensor mas só fica aqui por maxReadingNumber para evitar loop infinito
//             if (readingDistance > 500 and maxReadingNumber < 200){ //descarta readings maiores que 500cm
//               maxReadingNumber = maxReadingNumber + 1;
//               goto check_distance;
//             }//if readingDistance
//         } //for

//         if ((readingDistance - lastEchoDistance) >= 1){            // check for change in distance só manda msg se mudar o valor > 1cm 
//             lastEchoDistance = readingDistance;
//             //#ifdef enableSerialLog
//               Serial.print("Distancia Lida pelo Sensor Ultrasonico: ");
//               Serial.print(readingDistance); Serial.println("cm");
//               Serial.println("");
//             //#endif
//             delay(50);
//         } //if
//       delay(500); 
//       }

// Function adapted to calculate the median of 11 readings from the ultrasonic sensor and display it as the read distance
//REVIEWED
void readUltrasonic() {
  digitalWrite(PB13, 0);        // put power monitor pin in 1 so we can see it in Nordic Power Profiller
  Serial.println("Power monitor - D6 - PB13 = 0 Func. readUltrasonic()");
  int readings[11];                           // 11 readings (To calculate the median it is better to use an odd number)
  int ultrasonic_readings_array_size = sizeof(readings) / sizeof(readings[0]); 

  #ifdef enableSerialLog
    Serial.println("ultrasonic_readings_array_size = " + String(ultrasonic_readings_array_size) );
  #endif

  // Take 11 consecutive readings to calculate the median and eliminate undue readings and outliers due to ultrasound reflection:
  for (int i = 0; i < ultrasonic_readings_array_size; i++) { //for1
    check_distance: // Label for goto
    // for (int j = 0; j < 3; j++) { //TODO: for2 introduces the possibility of averaging or applying other filtering techniques to improve the accuracy of the distance measurements.
      digitalWrite(triggerPin, LOW);
      delayMicroseconds(5);
      digitalWrite(triggerPin, HIGH);
      delayMicroseconds(10);
      pulseLength = pulseIn(echoPin, HIGH);
      readingDistance = pulseLength / 58;   // Measured distance in centimeters
      delay(50);

      if (readingDistance > 500 || readingDistance < 0) { //TODO: check zero case. Eliminate erroneous readings above or below the sensor range
        goto check_distance;
      }
    // } //for2
    readings[i] = readingDistance;

    #ifdef enableSerialLog
      Serial.println("readings-" + String(i) + " = " + String(readings[i]) );
    #endif
  } //for1

  float median = calculateMedian(readings, ultrasonic_readings_array_size);    // Calculate the Median

  // TODO: Verifica se a median mudou significativamente
  // if (abs(median - lastEchoDistance) >= 1) { // check for change in distance só manda msg se mudar o valor > 1cm
    // lastEchoDistance = median;

    #ifdef enableSerialLog
      Serial.println("Distance Read by the Ultrasonic Sensor (median): " + String(median) + "cm");
      readingDistance = median; // Prepare to send the value over LoRa

    #endif
  // }

  delay(50); //para economizar bateria, pode-se reduzir esse tempo
}

// // Função para calcular a median
// float calculateMedian(int *array, int arraySize) {
//   qsort(array, arraySize, sizeof(int), compareReadings);

//   if (arraySize % 2 == 0) {
//     return (float)(array[arraySize / 2 - 1] + array[arraySize / 2]) / 2;
//   } else {
//     return (float)array[arraySize / 2];
    
//   }
// }

// int compareReadings(const void *a, const void *b) {
//   return (*(int *)a - *(int *)b);
// }

// Function to calculate the median
// TODO: improve commenting
float calculateMedian(int *array, int arraySize) {
  qsort(array, arraySize, sizeof(int), compareReadings);

  // Print the sorted values
  #ifdef enableSerialLog
    Serial.println("Sorted distance readings:");
    for (int i = 0; i < arraySize; i++) {
      Serial.println(array[i]);
    }
    Serial.println("\n");
  #endif

  if (arraySize % 2 == 0) {
    return (float)(array[arraySize / 2 - 1] + array[arraySize / 2]) / 2;
  } else {
    return (float)array[arraySize / 2];
  }
}

int compareReadings(const void *a, const void *b) {
  return (*(int *)a - *(int *)b);
}
#endif //enableUltrasonic

//////////////////////////////////////////////////// goToSleep() //////////////////////////////////////////////////// 
 void goToSleep() {
    if (goToSleep_flag == 2) 
    { 
      //Não encontrou o gateway e hibernará por 1 minuto
      #ifdef enableWatchDog
        IWatchdog.reload();
      #endif
      // Serial.println("Não encontrou o Gateway, hibernando por 1 minuto..."); 
      Serial.println("Hibernando por 1 minuto..."); 

      // TODO:
      LoRa.end(); // Ends Lora
      Serial.println("Módulo LoRa Finalizado: LoRa.end()");

      LoRa.sleep(); //Puts LoRa chip in sleep mode
      Serial.println("Módulo LoRa em modo de hibernação: LoRa.sleep()");

      delay (10);
      LowPower.shutdown(1000 * 60); //hiberna por 1 min
    } // end if

    // TODO: excluir
    // Entra em Deep Sleep e acorda em horas cheias hh:00 ou hh:30
    if (goToSleep_flag == 1) 
    {
    //   //DateTime now = rtc.now();
    //   //int sleepTime = 59 - now.minute(); //TinyRTC
      #ifdef enableWatchDog
        IWatchdog.reload();
      #endif
      Serial.println("Hibernando por 1 minuto..."); 

      // LoRa.sleep(); //Puts LoRa chip in sleep mode
      // Serial.println("Módulo LoRa em modo de hibernação.");

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

    } // end if
 
 }

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
  onReceive_flag = 1;                     // Entered the function onReceive
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
    //   goToSleep_flag = 1;
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

  //REVIEWED
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

void checkonReceive() { //TODO ver se essa é a função que recebe o retorno do gateway
    if (onReceive_flag == 0) {
      int time = (rtc.getMinutes() - startUpMinute);
      Serial.print("time:   "); Serial.println(time);
      if (rtc.getMinutes() - startUpMinute >= 2 ) {
      //#ifdef enableSerialLog
            Serial.print("Did not receive the Date from the Gateway! Going to sleep for 30 seconds");
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
  //REVIEWED
  void start_LoRa(){
    digitalWrite(PB14, 0);        // put power monitor pin in 1 so we can see it in Nordic Power Profiller
    Serial.println("Power monitor - D5 - PB14 = 0 Func. start_LoRa()");
    //LoRa.setTxPower(20);    // Change transmission power
    
    LoRa.setPins(csPin, resetPin, irqPin);    //SPI LoRa pins
    //LoRa.setPins(Lora_SS, Lora_RST, Lora_DIO0); //pinos definidos diretamente na lib
    //SPI.begin(SCK, MISO, MOSI, SS); //pinos definidos diretamente na lib

    while (!LoRa.begin(BAND) && lora_startup_counter < 10) {
      Serial.print(".");
      lora_startup_counter++;
      delay(500);
    }
    if (lora_startup_counter == 10) {
      Serial.println("LoRa initialization Failed!"); 
          // delay (100);
    }
        if (lora_startup_counter < 10) {
          #ifdef enableSerialLog
            Serial.println("LoRa initialization OK!"); 
          #endif
    }

    //Setup receiver para receber o update da hora:
    #ifdef enableSerialLog
      Serial.println("LoRa Receiver Callback with LoRa Reset in PA0");
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
    // if (runEvery(5000)) { // repeat every 5 sec 
    if (runEvery(1000)) { // repeat every 1 sec 
    //TODO se recebe confirmação de recebimento do gateway, não pode enviar mais para economizar bateria ver email: Checagem de Retorno de mensagem LoRa

      digitalWrite(PB15, 0);        // put power monitor pin in 1 so we can see it in Nordic Power Profiller
      Serial.println("Power monitor - D4 - PB15 = 0 Func: sendReadings()");

      //TODO: Do I know it the receiver received the LoRa message? how?
      LoRaMessage = String(sensor_id) + "/" + String(readingDistance) + "&" + String(readingDistance);
      //Send LoRa packet to receiver
      LoRa_sendMessage(LoRaMessage); // send a LoRaMessage

      #ifdef enableSerialLog
        Serial.print("Sending packet N°: ");   Serial.println(readingID);
        Serial.print("LoRaMessage: ");   Serial.println(LoRaMessage);
      #endif

      readingID++;
      goToSleep_flag = 2; //Flag that indicates that have to hibernate FOR 1MIN
    }
  }

#endif //enableLoRa

void activate_power_monitor_pins()
{
  pinMode (PB12, OUTPUT);
  pinMode (PB13, OUTPUT);
  pinMode (PB14, OUTPUT);
  pinMode (PB15, OUTPUT);
  digitalWrite(PB12, 1);        // put power monitor pin in 1 so we can see it in Nordic Power Profiller
  // Serial.println("Power monitor - PB12 = 1");
  digitalWrite(PB13, 1);        // put power monitor pin in 1 so we can see it in Nordic Power Profiller
  // Serial.println("Power monitor - PB13 = 1");
  digitalWrite(PB14, 1);        // put power monitor pin in 1 so we can see it in Nordic Power Profiller
  // Serial.println("Power monitor - PB15 = 1");
  digitalWrite(PB15, 1);        // put power monitor pin in 1 so we can see it in Nordic Power Profiller
  // Serial.println("Power monitor - PB15 = 1");
} //end activate_power_monitor_pins
/*********************************************** End Function Definitions ********************************************/