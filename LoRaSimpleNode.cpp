/*
  This code uses InvertIQ function to create a simple Gateway/Node logic.

  Gateway - Sends messages with enableInvertIQ()
          - Receives messages with disableInvertIQ()

  Node    - Sends messages with disableInvertIQ()
          - Receives messages with enableInvertIQ()

  With this arrangement a Gateway never receive messages from another Gateway
  and a Node never receive message from another Node.
  Only Gateway to Node and vice versa.

  This code receives messages and sends a message every second.

  InvertIQ function basically invert the LoRa I and Q signals.

  See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
  for more on InvertIQ register 0x33.

Changes: 
- The starting point for this project is: https://github.com/akshayabali/pico-lora/
- Corrections regarding how interupts are handled were made;
- Library for SD card was added;
- Library for ssd1306 was added;
- Library for DHT22 was added;


*/

#include "stdlib.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "stdio.h"
#include "string.h"
#include <sstream>

#include <string>
//Inlude Library
#include "LoRa-RP2040.h"
#include "ssd1306.h"
#include "dht.h"
#include "sd_card.h"
#include "ff.h"

#define SENSORDATA0 100
#define GPIO_DHT_PIN 14

#define SENSOR 14

//system State

#define READ_SENSOR_DATA 0                              //read sensor data, save data to SD card, LORA module in Listen mode
#define BEGIN_SEND_DATA_2_GATEWAY 1                     //Connect to the gateway, receive comands from the gateway(what data to receive), send to the Gateway the size of the data)
#define ONGOING_SEND_DATA_2_GATEWAY 2                   //send data to the Gateway
#define END_SEND_DATA_2_GATEWAY 3                       //check if all the data was sent/received

#define SEND_DATA_SENSOR_1 1
#define READY_2_RECEIVE 2
#define END_TRANSMISSION 3
#define GOOD_BYE 4

using std::string;
using namespace std;
const long frequency = 433E6;  // LoRa Frequency

const int csPin = 8;          // LoRa radio chip select
const int resetPin = 9;        // LoRa radio reset
const int irqPin = 13;          // change for your board; must be a hardware interrupt pin
const char *words[]= {"SSD1306", "DISPLAY", "DRIVER"};
const char *call[]= {"Call111","Call222","Call333"};

//variables used for LCD
ssd1306_t disp ={};
int counter=0;

//system state
char SystemState = READ_SENSOR_DATA;
//Network settings
uint8_t localAddress = 0x78;     // address of this device | 0x78 = x //node x 
uint8_t destination = 0x7A;      // destination to send to | 0x7A = z //Gateway

//Parse message data
uint8_t received_localAddress = 0x00;     // address of this device | 0x7A = z
uint8_t received_destination = 0x00;      // destination to send to | 0x78 = x
uint8_t received_mes_counter = 0x00;
uint8_t received_command = 0;
uint32_t nr_of_transceived_data = 0;
//message info variables
uint8_t msgCount = 0;            // count of outgoing messages


char first_sent_message = 0;

//declaration for i2c lcd initialization 
void setup_gpios(void);

//dht22

void SerializeInt32(char (&buf)[5], int32_t val);
int32_t ParseInt32(const char (&buf)[5]);

//SD card data
  FRESULT fr;
  FATFS fs;
  FIL fil;
  int ret;
  char buf[128];
  char filename[] = "temp_data.txt";
  int sd_ptr_ind = 0;
  int nr_of_data_entries_sd = 0;


//debug sd
char debug_sd[5] = {'A','A','A','A','\0'};

char local_copy_SD[34560] = {}; //enough space to store 1 reading/h for 60 days

  //dht
  dht_reading result;
  uint8_t strT[6];
  uint8_t strH[6];


void LoRa_rxMode();
void LoRa_txMode();
string read_sd_data(int nrofbytes );
string read_sd_data_to_buffer(int nrofbytes );
void LoRa_sendMessage(string message);
void PacketAnalyzer(string message);
void onReceive(int packetSize);
void onTxDone();
bool runEvery(unsigned long interval);
bool runEvery_2(unsigned long interval);
bool runEvery_3(unsigned long interval);
bool runEvery_4(unsigned long interval);
bool runEvery_5(unsigned long interval);
void read_dht_save_sd(void);
void read_datasize_from_sd(void); //calculate the number of sensor readings and save the number in : nr_of_data_entries_sd

int main(){

//sd card


    //timestamp - will be added to each measurement in SD card
    char system_time[5]={0,0,0,0,'\0'};

    //sd debug
    int sd_ptr_ind = 0;
    char run_every_4_loops = 0;
    int hhh = 0;

    string message = "X";

    //state machine
    char try_2_send = 0;


  


  stdio_init_all();
  setup_default_uart();
  setup_gpios();
  gpio_init(DHT_PIN);



  disp.external_vcc=false;
  ssd1306_init(&disp, 128, 64, 0x3C, i2c1);
  ssd1306_clear(&disp);

  sleep_ms(2000);

for(int i=0; i<sizeof(words)/sizeof(char *); ++i) {
            ssd1306_draw_string(&disp, 8, 24, 2, words[i]);
            ssd1306_show(&disp);
            sleep_ms(800);
          ssd1306_clear(&disp);
        }
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    printf("LoRa init failed. Check your connections.\n");
    while (true);                       // if failed, do nothing
  }

  printf("LoRa init succeeded.\n");
  printf("\n");
  printf("LoRa Node\n");
  printf("Only receive messages from gateways\n");
  printf("Tx: invertIQ disable\n");
  printf("Rx: invertIQ enable\n");
  printf("\n");

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();

  //sd begin
    // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        //while (true);
    }
   

    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        //while (true);
    }

    // Open file for writing ()
    fr = f_open(&fil, filename, FA_WRITE | FA_OPEN_EXISTING| FA_OPEN_APPEND); 
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        //while (true);
    }

    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        //while (true);
    }

    // Open file for reading
    fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        //while (true);
    }

    // Print every line in file over serial
    printf("Reading from file '%s':\r\n", filename);
    printf("---\r\n");
    while (f_gets(buf, sizeof(buf), &fil)) {
        printf(buf);
    }
    printf("\r\n---\r\n");

    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        //while (true);
    }


    // Unmount drive
    f_unmount("0:");

  //sd end

  while (1)  {

    switch (SystemState)
      {
        case READ_SENSOR_DATA:
          if (runEvery_2(10000))
          {
            read_dht_save_sd();
            printf("\nReading data and save it to sd\n");
          }
        break;

        /*
        case BEGIN_SEND_DATA_2_GATEWAY:
        
            nr_of_transceived_data = 0;
            nr_of_data_entries_sd = 0;
            read_datasize_from_sd();
            read_sd_data_to_buffer(nr_of_data_entries_sd);
            message = "S";
            LoRa_sendMessage(message); // send a message
            printf("Send BEGIN_SEND_DATA_2_GATEWAY!");
            //send back to READ_SENSOR_DATA if more then 3 attemts to sent passed
            try_2_send++;
            printf("\nDebug 558\n");
            //SystemState = ONGOING_SEND_DATA_2_GATEWAY;
             busy_wait_ms(100);
        break;
        */

        case ONGOING_SEND_DATA_2_GATEWAY:
        ///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!addd timing - to be executed every 100ms
          if(runEvery_4(6000))
          {
            if(first_sent_message==0)
            {
            nr_of_transceived_data = 0;
            nr_of_data_entries_sd = 0;
            read_datasize_from_sd();
            read_sd_data_to_buffer(nr_of_data_entries_sd);
              message = "S";
            } else
            {
              message = "D";
            }

            LoRa_sendMessage(message); // send a message
            printf("\nSend ONGOING_SEND_DATA_2_GATEWAY!\n");
            printf("\nnr_of_transceived_data = %d | nr_of_data_entries_sd = %d \n",nr_of_transceived_data, nr_of_data_entries_sd);
            if(nr_of_transceived_data>=nr_of_data_entries_sd)
            {
              //SystemState = END_SEND_DATA_2_GATEWAY;
              printf("\nSend Closing the transmission!\n");
              printf("\n last Values: nr_of_transceived_data = %d | nr_of_data_entries_sd = %d \n",nr_of_transceived_data, nr_of_data_entries_sd);
              message = "C";
              LoRa_sendMessage(message); // send a message
              SystemState = READ_SENSOR_DATA;
              nr_of_transceived_data = 0;
              nr_of_data_entries_sd = 0;
              first_sent_message = 0;
              break;
            } else
            {
            LoRa_sendMessage(message); // send a message
            printf("\nSend ONGOING_SEND_DATA_2_GATEWAY!\n");
            printf("\nnr_of_transceived_data = %d | nr_of_data_entries_sd = %d \n",nr_of_transceived_data, nr_of_data_entries_sd);
            }


            first_sent_message=1;
          }
        break;
          /*
        case END_SEND_DATA_2_GATEWAY:
          if(runEvery_5(1000))
          {
            message = "C";
            LoRa_sendMessage(message); // send a message
            SystemState = READ_SENSOR_DATA;
            // delete data from SD carda after confirmation
            printf("\nSend END_SEND_DATA_2_GATEWAY!\n");
          }

        break;
        */

        default:
          //read_dht_save_sd();
          SystemState = READ_SENSOR_DATA;
        break;
      }

  }

  return 0;
}



void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  //LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
  //LoRa.enableInvertIQ();                // active invert I and Q signals
}
string read_sd_data(int nrofbytes ){

  int sd_read_loop_idx = 0;
      //sd begin
    // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        //while (true);
    }
   

    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        //while (true);
    }
     // Open file for reading
    fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        //while (true);
    }

    // Print every line in file over serial
    printf("Reading from file '%s':\r\n", filename);
    printf("---\r\n");
    

    fr = f_lseek(&fil,(sd_ptr_ind));
    if (fr != FR_OK) {
        printf("ERROR: Could not Move file pointer of the file object (%d)\r\n", fr);
        //while (true);
    }
    
    while (f_gets(buf, nrofbytes, &fil)) {
        //printf(buf);
        sd_read_loop_idx++;
        nr_of_transceived_data++;
        if(sd_read_loop_idx>=1)
        {
          sd_read_loop_idx=0;
          sd_ptr_ind=sd_ptr_ind+(nrofbytes);
          break;
        }
        
    }
    printf(buf);
    printf("\r\n---\r\n");

    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        //while (true);
    }

    f_unmount("0:");

return "test";
}

string read_sd_data_to_buffer(int nrofbytes ){

  int sd_read_loop_idx = 0;
      //sd begin
    // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        //while (true);
    }
   

    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        //while (true);
    }
     // Open file for reading
    fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        //while (true);
    }

    // Print every line in file over serial
    printf("Reading from file '%s':\r\n", filename);
    printf("---\r\n");
    
    
    while (f_gets(local_copy_SD, nrofbytes, &fil)) {
        printf(buf);
        sd_read_loop_idx++;
        if(sd_read_loop_idx>=nrofbytes)
        {
          sd_read_loop_idx=0;
          break;
        }
    }
    printf("\r\n---\r\n");

    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        //while (true);
    }

    f_unmount("0:");

return "test";
}


void LoRa_sendMessage(string message) {
  char data_size[5]={'0','0','0','0','\0'};
  char data_chunk[5]={'0','0','0','0','\0'};
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  //LoRa.write(msgCount);                 // add message ID
  LoRa.write((msgCount%10)+48);            // add message ID - transformed from integer to string 0...9
  LoRa.print(message.c_str());          // add payload


  //data_index++;                          //increment sensor_data_test index

  if (SystemState == ONGOING_SEND_DATA_2_GATEWAY)
  {
    if(first_sent_message==0)
    {
      SerializeInt32(data_size, nr_of_data_entries_sd);
      for (int i = 0; i < 4; i++)
      {
        LoRa.write(data_size[i]);
      }
      
    }else{
      read_sd_data(26);
      SerializeInt32(data_chunk, nr_of_transceived_data);
      for (int j = 0; j < 4; j++)
        {
          LoRa.write(data_chunk[j]);
        }
      for(int i=0; i<26;i++)
        {
          LoRa.write(buf[i]);
        }
    }


  } else
  {
    LoRa.write(nr_of_data_entries_sd);
  }

  LoRa.endPacket(true);                 // finish packet and send it
  msgCount++;                           // increment message ID
  printf("\n transmission done\n");

}

void system_state_transition(char received_command_G)
{
    switch(SystemState)
    {
    case READ_SENSOR_DATA: 
        if(received_command_G==SEND_DATA_SENSOR_1)
        {
            SystemState = ONGOING_SEND_DATA_2_GATEWAY;
        }
    break;
    /*
    case BEGIN_SEND_DATA_2_GATEWAY: 
        if(received_command_G==READY_2_RECEIVE)
        {
            SystemState = ONGOING_SEND_DATA_2_GATEWAY;
        }
    break;
    */
    case ONGOING_SEND_DATA_2_GATEWAY:
        //here the gateway should listen, not sending messages
        SystemState = ONGOING_SEND_DATA_2_GATEWAY;
    break;
    case END_SEND_DATA_2_GATEWAY:
        SystemState=READ_SENSOR_DATA;
    break;
    
    default:
    break;
    }
}

void PacketAnalyzer(string message)
{
  int str_length = message.length();
  int index = 0;

  received_destination = message[index++];
  received_localAddress = message[index++];
  received_mes_counter = message[index++];
  received_command =  message[index++]-'0';


  if(received_destination==localAddress && received_localAddress==destination)
  {
    system_state_transition(received_command);
  } else
  {
    //this gateway is not a friend
    printf("\nThe message we received is NOT from a known gateway\n");
    return;
  }


printf("\n received_localAddress = %d, received_destination = %d ", received_localAddress,received_destination);
printf("\n received_mes_counter = %d, received_command = %d ", received_mes_counter,received_command);
printf("\nstr_length = %d , SystemState = %d, ", str_length, SystemState);


}

void onReceive(int packetSize) {
  string message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }

  printf("Node Receive: ");
  printf("%s\n",message.c_str());

  PacketAnalyzer(message);
}

void onTxDone() {
  printf("TxDone\n");
  LoRa_rxMode();
}

bool runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = to_ms_since_boot(get_absolute_time());
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

bool runEvery_2(unsigned long interval)
{
  static unsigned long previousMillis2 = 0;
  unsigned long currentMillis = to_ms_since_boot(get_absolute_time());
  if (currentMillis - previousMillis2 >= interval)
  {
    previousMillis2 = currentMillis;
    return true;
  }
  return false;
}

bool runEvery_3(unsigned long interval)
{
  static unsigned long previousMillis3 = 0;
  unsigned long currentMillis = to_ms_since_boot(get_absolute_time());
  if (currentMillis - previousMillis3 >= interval)
  {
    previousMillis3 = currentMillis;
    return true;
  }
  return false;
}

bool runEvery_4(unsigned long interval)
{
  static unsigned long previousMillis4 = 0;
  unsigned long currentMillis = to_ms_since_boot(get_absolute_time());
  if (currentMillis - previousMillis4 >= interval)
  {
    previousMillis4 = currentMillis;
    return true;
  }
  return false;
}

bool runEvery_5(unsigned long interval)
{
  static unsigned long previousMillis5 = 0;
  unsigned long currentMillis = to_ms_since_boot(get_absolute_time());
  if (currentMillis - previousMillis5 >= interval)
  {
    previousMillis5 = currentMillis;
    return true;
  }
  return false;
}
void setup_gpios(void) {
    i2c_init(i2c1, 400000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);
}

void SerializeInt32(char (&buf)[5], int32_t val)
{
    uint32_t uval = val;
    buf[0] = uval;
    buf[1] = uval >> 8;
    buf[2] = uval >> 16;
    buf[3] = uval >> 24;
    //end of string
    buf[4] = '\0';
}

int32_t ParseInt32(const char (&buf)[5])
{
    // This prevents buf[i] from being promoted to a signed int.
    uint32_t u0 = buf[0], u1 = buf[1], u2 = buf[2], u3 = buf[3];
    uint32_t uval = u0 | (u1 << 8) | (u2 << 16) | (u3 << 24);
    return uval;
}

void read_dht_save_sd(void)
{
  static uint16_t data_counter = 0;
    //sd begin
    // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        //while (true);
    }
   

    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        //while (true);
    }
   read_from_dht(&result);
        strT[0] = result.temp_celsius / 100 + '0';
        strT[1] = result.temp_celsius % 100 / 10 + '0';
        //printf("\n temp: %d \n", result.temp_celsius);
        strT[2] = 46;
        strT[3] = result.temp_celsius % 10 + '0';
        strT[4] = 67;
        strT[5] = '\0';
        strH[0] = result.humidity / 100 + '0';
        strH[1] = result.humidity % 100 / 10 + '0';
        //printf("\n humidity: %d \n", result.humidity);
        strH[2] = 46;
        strH[3] = result.humidity % 10 + '0';
        strH[4] = 37;
        strH[5] = '\0';
        printf("\n temp: %s \n", strT);
        printf("\n humidity: %s \n", strH);
        printf("\nSize of int: %d \n", sizeof(int));

         // Open file for writing ()
    fr = f_open(&fil, filename, FA_WRITE | FA_OPEN_EXISTING| FA_OPEN_APPEND); 
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        //while (true);
    }
    //reset system time
    //system_time[0]=0;system_time[1]=0;system_time[2]=0;system_time[3]=0;system_time[4]='\0';
    //SerializeInt32(system_time,to_ms_since_boot(get_absolute_time()));

    //data index
    data_counter++;
    if(data_counter>9999)
    {
      data_counter = 0;
    }
    debug_sd[4]='\0';
    debug_sd[3]=(data_counter)%10+'0';
    debug_sd[2]=(data_counter/10)%10+'0';
    debug_sd[1]=(data_counter/100)%10+'0';
    debug_sd[0]=(data_counter/1000)+'0';
    
    // Write data from sensor to file
    ret = f_printf(&fil, "T=");
    ret = f_printf(&fil, (char *)strT);
    ret = f_printf(&fil, "?H=");
    ret = f_printf(&fil, (char *)strH);
    ret = f_printf(&fil, "?TS=");
    ret = f_printf(&fil, (char *)debug_sd);
    ret = f_printf(&fil, "\r\n");

    if (ret < 0) {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        f_close(&fil);
        //while (true);
    }
    
    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        //while (true);
    }

    f_unmount("0:");
}

void read_datasize_from_sd(void)
{
    //sd begin
    // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        //while (true);
    }
   

    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        //while (true);
    }
    // Open file for reading ()
    fr = f_open(&fil, filename, FA_READ | FA_OPEN_EXISTING| FA_OPEN_APPEND); 
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        //while (true);
    }

    nr_of_data_entries_sd = (int)f_tell(&fil)/26;
    printf("\r\nnr_of_data_entries_sd: (%d)\r\n", nr_of_data_entries_sd);

     // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        //while (true);
    }

    f_unmount("0:");
}