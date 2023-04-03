#include <Arduino.h> // I guess this one is needed to get the available functions?

#include "TWAI.h"
#include "esp_check.h"

// CAN parameters and variables (depending on which board used). In this example, the ezsbc high voltage esp32 board was used.
#define TX gpio_num_t(14) 
#define RX gpio_num_t(13)
const uint8_t rx_queue_len = 10;       // Receive Queue size
const uint8_t tx_queue_len = 10;
int CAN_DEVICE_ID{10};

// CANOpen addresses
int tpdo_1{0}; //{CAN_DEVICE_ID + 0x180};
int tpdo_2{0}; //{CAN_DEVICE_ID + 0x280};
int tpdo_4{0}; //{CAN_DEVICE_ID + 0x480};
int response_sdo{0}; //{CAN_DEVICE_ID + 0x580};
int heartbeat{0}; //{CAN_DEVICE_ID + 0x700};

int rpdo_1{0}; //{CAN_DEVICE_ID + 0x200};
int rpdo_2{0}; //{CAN_DEVICE_ID + 0x300};
int rpdo_3{0}; //{CAN_DEVICE_ID + 0x400};
int rpdo_4{0}; //{CAN_DEVICE_ID + 0x500};
int request_sdo{0}; //{CAN_DEVICE_ID + 0x600};

uint32_t filterValue{CAN_DEVICE_ID}; //This defines which bits are to be accepted in a CAN Message ID
constexpr uint32_t filterMask = 0x780; //This defines which bits are masked (low = masked, high = ignored) //127 = all the desired bits whereas the acceptance mask filter should have all these zero and the rest set to positive, hence the rest can vary but the ones with zero MUST comply. //This defines which of the bits in the whole CAN message are to be compared with the filter accepted bits

// CanBus Tx check timer micros
unsigned long int canSendTimerNow{0};
unsigned long int canSendTimerLoop{0};
unsigned long int canSendLimit{5000};

// Task handles
TaskHandle_t BLDCTaskHandle;
TaskHandle_t CANUpdateHandle;
SemaphoreHandle_t xMutex = NULL;  // Create a mutex object that will be used when getting and setting the targetRPMs to avoid race conditions to the same resource.
SemaphoreHandle_t sendCANMutex = NULL;

// Heartbeat timer
unsigned long t_HB = 0;
unsigned long timeNow_HB = 0;
unsigned long diff_HB{0};
unsigned long update_rate_HB{1000};
bool sendHB{false};

void setup() 
{
  Serial.begin(115200);

  delay(250);
  xMutex = xSemaphoreCreateMutex();  // Init mutex object.
  sendCANMutex = xSemaphoreCreateMutex();
  delay(250);

  // Initialize all canbus variables. These can be changed using a switch or similar on HW
  filterValue = CAN_DEVICE_ID;
  // CANOpen addresses
  tpdo_1 = CAN_DEVICE_ID + 0x180;
  tpdo_2 = CAN_DEVICE_ID + 0x280;
  tpdo_4 = CAN_DEVICE_ID + 0x480;
  response_sdo = CAN_DEVICE_ID + 0x580;
  heartbeat = CAN_DEVICE_ID + 0x700;
  
  rpdo_1 = CAN_DEVICE_ID + 0x200;
  rpdo_2 = CAN_DEVICE_ID + 0x300;
  rpdo_3 = CAN_DEVICE_ID + 0x400;
  rpdo_4 = CAN_DEVICE_ID + 0x500;
  request_sdo = CAN_DEVICE_ID + 0x600;
  
  // ---- CAN settings ----
  // Modes: TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK
  ESP32TWAI::initialize(RX, TX, 500000, TWAI_MODE_NORMAL, true, rx_queue_len, tx_queue_len, filterValue, filterMask , false); /
  Serial.println(" Installing driver");
  esp_err_t install_err = twai->install();
  if (install_err == ESP_OK)
  {
      Serial.println("TWAI INSTALL: ok");
      Serial.print("Start driver.");
      esp_err_t start_err = twai->start(); 
      if (start_err == ESP_OK) 
      {
          Serial.println(" Running");
      } 
      else 
      {
          Serial.printf(" Starting driver failed with error 0x%04X. Restarting in 5 seconds\n", start_err);
          delay(3000);
          esp_restart();
      }
  }
  else 
  {
      Serial.printf("[TWAI] Installing driver failed with error 0x%04X. Restarting in 5 seconds\n", install_err);
      delay(5000);
      esp_restart();
  }

  twai->onBusOff([]() {
      Serial.println(" Bus is off");
  });

  twai->onBusRecovered([](){
      Serial.println(" Bus sucessfully recovered");
  });

  twai->onRXQueueFull([](){
      Serial.println(" RX queue is full. Flushing it");
  });
  
  // -----------------------------------------

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "BLDC loop",     /* name of task. */
    4096,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &BLDCTaskHandle,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code,   /* Task function. */
    "Task2",     /* name of task. */
    4096,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &CANUpdateHandle,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
  delay(500);
}

// Empty loop since the cores are spinning their respective tasks defined beneath.
void loop() 
{
  
}

//Task1code: Run the stepper loop (this one is actually NOT using any delays or anything but rather is using a "trigger"
void Task1code( void * pvParameters ) 
{
//  Serial.print("PID running on core ");
//  Serial.println(xPortGetCoreID());

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(50));
    // DO stuff here such as running BLDC motors, checking sensors or whatever
  }
}

//Task2code: Receive and handle CAN messages.
void Task2code( void * pvParameters ) 
{
//  Serial.print("Update task running on core ");
//  Serial.println(xPortGetCoreID());

  while(1)
  {
    // If new received CAN message, then print it to the screen and update target speed as RPM.

    read_can();
    
    // Send HB at instructed rate
    timeNow_HB = millis();
    diff_HB = timeNow_HB - t_HB;
    if (diff_HB >= update_rate_HB)
    {
      if (sendHB)
      {
        send_HB(); 
      }
        t_HB = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

/* Populate the send message and send to CAN

*/
void send_HB()
{
    twai->txMessage.identifier = heartbeat;
    twai->txMessage.extd = 0;
    twai->txMessage.rtr = 0;
    twai->txMessage.data_length_code = 8;

    twai->txMessage.data[0] = 0;
    twai->txMessage.data[1] = 0;
    twai->txMessage.data[2] = 0;

    twai->txMessage.data[3] = 0;
    twai->txMessage.data[4] = 0;
    twai->txMessage.data[5] = 0;
    twai->txMessage.data[6] = 0;
    twai->txMessage.data[7] = 0;
  
    twai->txMessage.data[7] = emptyTankCounter;
    send_data();  
}

/* Populate the send message and send to CAN

*/
void send_status()
{
    twai->txMessage.identifier = tpdo_1;
    twai->txMessage.extd = 0;
    twai->txMessage.rtr = 0;
    twai->txMessage.data_length_code = 8;

    twai->txMessage.data[0] = 0;
    twai->txMessage.data[1] = 0;
    twai->txMessage.data[2] = 0;

    twai->txMessage.data[3] = 0;
    twai->txMessage.data[4] = 0;
    twai->txMessage.data[5] = 0;
    twai->txMessage.data[6] = 0;
    twai->txMessage.data[7] = 0;
    twai->txMessage.data[7] = emptyTankCounter;
    send_data();
}


void read_can()
{
  twai->readAlerts();

  while (twai->availableMessages() > 0)  // Read and process all available messages! 
  {
    //Check if message and CANBus is OK
    esp_err_t esp_status = twai->receiveMessage();
    switch (esp_status) 
    {
    case ESP_OK:
        break;
    case ESP_ERR_TIMEOUT:
        Serial.println("TWAI RX: ESP_ERR_TIMEOUT");
        break;
    case ESP_ERR_INVALID_ARG:
        Serial.println("TWAI RX: ESP_ERR_INVALID_ARG");
        break;
    case ESP_ERR_INVALID_STATE:
        Serial.println("TWAI RX: ESP_ERR_INVALID_STATE");
        break;
    default:
        Serial.println("TWAI RX: unknow error");
        break;
    }
    //Process message
//    Serial.printf(" Received message with ID: 0x%04x\n", twai->rxMessage.identifier);
    if (twai->rxMessage.identifier == rpdo_1)
    {
//      Serial.printf("Set motor speed or something");
    }
    else if(twai->rxMessage.identifier == rpdo_2)
    {
      // ping();
    }
    else if(twai->rxMessage.identifier == rpdo_3)
    {
      // NOT USED
    }
    else if(twai->rxMessage.identifier == rpdo_4)
    {
      // NOT USED
    }
    else if(twai->rxMessage.identifier == request_sdo)
    {
      // NOT USED
    }
  }
}

// ## Send msg on CAN network ##
void send_data()
{
  if (xSemaphoreTake (sendCANMutex, (150 * portTICK_PERIOD_MS)))  // take the mutex within 250ms which should be more than enough. We do not want to be stuck waiting for the mutex forever!
  {
//    Serial.print("message ID: ");Serial.println(twai->txMessage.identifier);
     esp_err_t esp_state = twai->sendMessage(); //  This just queues the message for transmission and does not necessarily send anything on the bus.

    switch (esp_state) 
    {
          case ESP_OK:
              break;
          case ESP_ERR_INVALID_ARG:
              Serial.println("TWAI TX: ESP_ERR_INVALID_ARG");
              break;
          case ESP_ERR_TIMEOUT:
              Serial.println("TWAI TX: ESP_ERR_TIMEOUT");
              break;
          case ESP_FAIL:
              Serial.println("TWAI TX: ESP_FAIL");
              break;
          case ESP_ERR_INVALID_STATE:
              Serial.println("TWAI TX: ESP_ERR_INVALID_STATE");
              break;
          case ESP_ERR_NOT_SUPPORTED:
              Serial.println("TWAI TX: ESP_ERR_NOT_SUPPORTED");
              break;
          default:
              Serial.println("TWAI TX: unknow error");
              break;
      }

      canSendTimerNow = 0;
      canSendTimerLoop = 0;
      canSendTimerNow = micros();
      delay(1);
      canSendTimerLoop = micros();
      while (!twai->txSuccess() && canSendTimerLoop - canSendTimerNow < canSendLimit)
      {
        delay(25);
        canSendTimerLoop = micros();
      }
      if (canSendTimerLoop - canSendTimerNow >= canSendLimit)
      {
        Serial.print("CAN TX complete timeout for: ");Serial.print(canSendTimerLoop - canSendTimerNow);Serial.print(" microseconds with a of: ");Serial.print(canSendLimit);Serial.println(" microseconds..");
        Serial.println("CANBus will become error passive probably...");
      }
      
  }
  xSemaphoreGive(sendCANMutex);
  
}