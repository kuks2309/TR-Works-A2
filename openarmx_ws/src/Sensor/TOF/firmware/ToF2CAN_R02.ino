/* ESP32 TWAI transmit example.
  This transmits a message every second.

  Connect a CAN bus transceiver to the RX/TX pins.
  For example: SN65HVD230

  The API gives other possible speeds and alerts:
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html

  created 27-06-2023 by Stephan Martin (designer2k2)
*/

#include "driver/twai.h"
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 20
#define TX_PIN 21
#define ADDR0 10
#define ADDR1 7
#define ADDR2 6
#define TXID 0x53

// Interval:
#define POLLING_RATE_MS 1000

static bool driver_installed = false;

//unsigned long previousMillis = 0;  // will store last time a message was send
byte ADDR;
void setup() {
  // Start Serial:
  pinMode(ADDR0, INPUT_PULLUP);
  pinMode(ADDR1, INPUT_PULLUP);
  pinMode(ADDR2, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("VL53L0X to CAN test.");
  ADDR = 0x50 + digitalRead(ADDR2) * 8 + digitalRead(ADDR1) * 4 + digitalRead(ADDR0) * 2;
  Wire.begin(3, 4);
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect TX alerts and Bus-Off errors
  uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;
}

byte Range[2];
static void send_message() {
  // Send message

  // Configure message to transmit
  twai_message_t message;
  message.identifier = TXID;
  message.data_length_code = 3;
  //  for (int i = 0; i < 4; i++) {
  message.data[0] = Range[0];
  message.data[1] = Range[1];
  message.data[2] = ADDR;
  //  }

  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
    printf("Message queued for transmission\n");
  } else {
    printf("Failed to queue message for transmission\n");
  }
}

void loop() {
  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    ADDR = 0x50 + digitalRead(ADDR2) * 8 + digitalRead(ADDR1) * 4 + digitalRead(ADDR0) * 2;
    Serial.printf("Received CAN ID: 0x%X, Data[0]: %d\n", rx_msg.identifier, rx_msg.data[0]);
    if (rx_msg.identifier != ADDR || rx_msg.data[0] != 0x53 ) return;
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    Range[0] = (measure.RangeMilliMeter >> 8) & 0xFF;
    Range[1] = measure.RangeMilliMeter & 0xFF;
    Serial.printf("Range is %d mm \n", measure.RangeMilliMeter);
    send_message();
  }
}
