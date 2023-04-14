// #define USE_WIFI

#include <std_msgs/String.h>
#include <Adafruit_INA260.h>
#include "std_msgs/Float32.h"
#include <ros.h>


#ifdef USE_WIFI

#include <ESP8266WiFi.h>
//Set the rosserial_python server address
IPAddress server(192,168,0,1);
const char *ssid = "roboy-robot";
const char *password = "**************8";

// Set the rosserial socket server port default to 11411
const uint16_t serverPort = 11411;

#endif



// Loop exproximativly at 1Hz
#define timeInterval 1000

Adafruit_INA260 ina260 = Adafruit_INA260();

ros::NodeHandle nh;
// Make a chatter publisher
// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);

// std_msgs::String chair_status_msg;
// ros::Publisher status_pub("/roboy/pinky/middleware/espchair/status", &status_msg);

// std_msgs::String battery_status_msg;
// ros::Publisher status_pub("/roboy/pinky/middleware/battery/status", &battery_status_msg);

std_msgs::Float32 battery_status_voltage_msg;
ros::Publisher voltage_status_pub("/roboy/pinky/middleware/battery/status/voltage", &battery_status_voltage_msg);

std_msgs::Float32 battery_status_current_msg;
ros::Publisher current_status_pub("/roboy/pinky/middleware/battery/status/current", &battery_status_current_msg);

std_msgs::Float32 battery_status_power_msg;
ros::Publisher power_status_pub("/roboy/pinky/middleware/battery/status/power", &battery_status_power_msg);

std_msgs::String battery_status_log_msg;
ros::Publisher log_msg_pub("/roboy/pinky/middleware/battery/status/log", &battery_status_log_msg);
// the capacityPercentage caculation will be moved to the Node-red
// std_msgs::Float32 battery_status_capacity_msg;
// ros::Publisher capacity_status_pub("/roboy/pinky/middleware/battery/status/capacityPercentage", &battery_status_capacity_msg);

// ros::Subscriber<std_msgs::String> sub("blinkm", light_cb);
// Be polite and say hello
// char hello[13] = "hello world!";

void setup()
{
  // configure the PWM
  // analogWriteFreq(1000);
  // analogWriteRange(53687); esp8266 has a known issue with analogWriteRange bigger than 53687

  // Use ESP8266 serial to monitor the process
  // delay(1000);
  Serial.begin(9600);
  
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("Serial Connected");

  #ifdef USE_WIFI
  Serial.println("Serial Connected");
  #endif

  if (!ina260.begin())
  {
    #ifdef USE_WIFI
    Serial.println("Couldn't find INA260 chip");
    #endif

    while (1)
      ;
  }

  #ifdef USE_WIFI
  Serial.println("Found INA260 chip");
  printBatteryStatus();
  #endif

  #ifdef USE_WIFI
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  
  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);

  #endif

  // #ifndef USE_WIFI
  // nh.getHardware()->setBaudRate(115200);
  // #endif
  nh.initNode();

  #ifdef USE_WIFI
  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
  #endif

  // Start to be polite
  nh.advertise(voltage_status_pub);
  nh.advertise(current_status_pub);
  nh.advertise(power_status_pub);
  // nh.advertise(log_msg_pub);

}
void printBatteryStatus()
// void printBatteryStatus(bool printStatus = false)
{
  float voltage = ina260.readBusVoltage() / 1000.0;
  float current = ina260.readCurrent() / 1000.0;
  float power = ina260.readPower() / 1000.0;

  Serial.print("Current: ");
  Serial.print(voltage);
  Serial.println(" A");

  Serial.print("Bus Voltage: ");
  Serial.print(current);
  Serial.println(" V");

  Serial.print("Power: ");
  Serial.print(power);
  Serial.println(" W");


}
void publishBatteryVoltageStatus()
{
  float voltage;
  voltage = ina260.readBusVoltage()/1000.0;
  battery_status_voltage_msg.data = voltage;
  voltage_status_pub.publish(&battery_status_voltage_msg);
}
void publishBatteryCurrentStatus()
{
  battery_status_current_msg.data = ina260.readCurrent()/1000.0;
  current_status_pub.publish(&battery_status_current_msg);
}
void publishBatteryPowerStatus()
{
  battery_status_power_msg.data = ina260.readPower()/1000.0;
  power_status_pub.publish(&battery_status_power_msg);
}
// void publishBatteryCapacityStatus()
// {
//   float voltage;
//   voltage = ina260.readBusVoltage() / 1000.0;
//   battery_status_capacity_msg.data = getVoltageToCapacityPercentage(voltage);
//   capacity_status_pub.publish(&battery_status_capacity_msg);
// }
void publishBatteryStatus()
{
  publishBatteryVoltageStatus();
  publishBatteryCurrentStatus();
  publishBatteryPowerStatus();
  // publishBatteryCapacityStatus();
}

float getVoltageToCapacityPercentage(float voltage)
{
  const float Vmin = 23.7;
  const float Vmax = 29.4; 
  if (voltage > Vmax)
  {
    return 100.0;
  }
  else if (voltage < Vmin)
  {
    return 0.0;
  }
  else
    return (int)((voltage - Vmin) / (Vmax - Vmin)*100)/1.0;
}

void loop()
{
  
  
  if (nh.connected())
  {

    #ifdef USE_WIFI
    Serial.println("Connected");
    // Say hello

    // str_msg.data = hello;
    // chatter.publish(&str_msg);
    #endif
    publishBatteryStatus();
  }
  else
  {
    Serial.println("Not Connected to ros");
    printBatteryStatus();
    Serial.println("");
  }
  nh.spinOnce();
  delay(timeInterval);
}

