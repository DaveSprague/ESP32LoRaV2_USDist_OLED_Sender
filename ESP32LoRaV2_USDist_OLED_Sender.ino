/*
  This is a simple example show the Heltec.LoRa sended data in OLED.

  The onboard OLED display is SSD1306 driver and I2C interface. In order to make the
  OLED correctly operation, you should output a high-low-high(1-0-1) signal by soft-
  ware to OLED's reset pin, the low-level signal at least 5ms.

  OLED pins to ESP32 GPIOs via this connecthin:
  OLED_SDA -- GPIO4
  OLED_SCL -- GPIO15
  OLED_RST -- GPIO16
  
  by Aaron.Lee from HelTec AutoMation, ChengDu, China
  成都惠利特自动化科技有限公司
  https://heltec.org
  
  this project also realess in GitHub:
  https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
*/

#include "heltec.h"

#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6
//
unsigned int counter = 0; // maintain a count of distance measurements

// variables for the LoRa radio
String rssi = "RSSI --";
String packSize = "--";
String packet ;

char buff[5];  // used to hold data read from the Maxbotix Ultrasonic distance sensor

//HardwareSerial Matbotix_Serial(1);

void setup()
{
  
  //WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
// 
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Starting...");
  Heltec.display->display();
  delay(1000);

  // Serial.begin(115600);  // we don't need this since enabling Serial on the Heltec sets it to 115600
  // Serial1.begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert)
  Serial1.begin(9600, SERIAL_8N1, 22, -1);
  delay(1000);
}

void loop()
{
  int startTime = millis();

  // flush input buffer containing old data from the Maxbotix Ultrasonic Distance sensor
  // data packets from the Maxbotix sensor consist of the letter 'R' followed by
  //   four digits giving the distance in millimeters
  while (Serial1.available()) {
    Serial1.read();
  }
  
  // wait for an R
  while (Serial1.available() && Serial1.read() != 'R')
  {
    
  }
  //Read the R
  Serial1.readBytes(buff, 1);
  
  // read the four digits following the 'R'
  int strLen = Serial1.readBytes(buff, 4);
  Serial.print("Length: ");
  Serial.print(strLen);
  Serial.print(" Value: ");
  Serial.print(buff);
//  Serial.print(" vBat: ");
//  Serial.print(vBat, 3);
  
  // Display the latest reading from the Maxbotis sensor on the OLED
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, "Sensed Distance: ");
  Heltec.display->drawString(90, 0, buff);
  Heltec.display->drawString(0, 20, "String Len ");
  Heltec.display->drawString(90,20, String(strLen));
  Heltec.display->drawString(0, 30, "Count: ");
  Heltec.display->drawString(90,30, String(counter));
  Heltec.display->display();
  
  // send packet
  LoRa.beginPacket();
  
  /*
  * LoRa.setTxPower(txPower,RFOUT_pin);
  * txPower -- 0 ~ 20
  * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
  *   - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
  *   - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
  */
  // can we just do this next line in the setup() function or is there some advantage to doing it here
  LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  // the messaeg we send over LoRa is a 'd' followed by four digit distance in mm,
  //   followed by 'mm' followed by the packet count.
  LoRa.print("d");
  LoRa.print(buff);
  LoRa.print("mm");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;
  delay(500);  // wait so that we transmit approx. one distance measurement per second
  int endTime = millis();
  Serial.print(" Loop time (msec): ");
  Serial.println(endTime - startTime);
}
