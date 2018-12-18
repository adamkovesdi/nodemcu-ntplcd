#include <Wire.h>  // This library is already built in to the Arduino IDE
#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <LiquidCrystal_I2C.h> //This library you can add via Include Library > Manage Library > 
#include <RTClib.h>

/* 2018.11.27. Reference for future me... THANKS adamkov
 *  i2c LCD - Nodemcu
 *  VCC - 3V3
 *  GND - GND
 *  SDL - D2
 *  SCL - D1
 *  
 *  RTC - Nodemcu
 *  SCL - D1
 *  SDL - D2
 *  VCC - D3
 *  GND - D4
 *  
 *  To set clock short D8 to 3V3
 *  
 */


LiquidCrystal_I2C lcd(0x3F, 20, 4);
RTC_DS1307 rtc;
#define RTCSETPIN   15

byte hourval, minuteval, secondval;

time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void printLcdDigits(int digits);
void sendNTPpacket(IPAddress &address);

IPAddress timeServer(148, 6, 0, 1); // time-a.timefreq.bldrdoc.gov

const int timeZone = 1;     // Central European Time
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

void setup() {
  pinMode(RTCSETPIN, INPUT_PULLUP);
  // pinMode(RTCSETPIN, INPUT);
  pinMode(0, OUTPUT); digitalWrite(0,HIGH);
  pinMode(2, OUTPUT); digitalWrite(2,LOW);
  
  Serial.begin(9600);
  while (!Serial) ; // Needed for Leonardo only
  delay(250);
  Serial.println("TimeNTP Example");
  Serial.print("Connecting to wifi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print("IP number assigned by DHCP is ");
  Serial.println(WiFi.localIP());
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);

  lcd.init();   // initializing the LCD
  lcd.backlight(); // Enable or Turn On the backlight
}

time_t prevDisplay = 0; // when the digital clock was displayed
void loop() {
  DateTime rtctime = rtc.now();

  if (timeStatus() != timeNotSet) {
    int val=digitalRead(RTCSETPIN);
    if (now() != prevDisplay) { //update the display only if time has changed
      prevDisplay = now();
      digitalClockDisplay();

      
      secondval = second();
      minuteval = minute();
      hourval = hour();
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("NTP time: ");
      lcd.print(hourval);
      printLcdDigits(minuteval);
      printLcdDigits(secondval);

      lcd.setCursor(0, 2);
      lcd.print("RTC time: ");
      lcd.print(rtctime.hour());
      printLcdDigits(rtctime.minute());
      printLcdDigits(rtctime.second());
    }
    if (val)
    {
      Serial.println("Setup RCT to NTP time");
      rtc.adjust(now()+1); // +1 seconds to avoid set delay
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("RTC synced to NTP");
      lcd.setCursor(0,1);
      lcd.print("end of operation");
      delay(5000);
    }
  }
  delay(100);
}

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());
  Serial.println();
}

void printDigits(int digits) {
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void printLcdDigits(int digits) {
  // utility for digital clock display: prints preceding colon and leading 0
  lcd.print(":");
  if (digits < 10)
    lcd.print('0');
  lcd.print(digits);
}
/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
