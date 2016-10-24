#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

// RF24 radio(CE,CSN);
RF24 radio(49,53);
const uint64_t pipes[2] = { 0xDEDEDEDEE8LL, 0xDEDEDEDEE4LL};

boolean stringComplete = false;
static int dataBufferIndex = 0;
boolean stringOverflow = false;
char charOverflow = 0;
String SendPayload = "";
char RecvPayload[31] = "";
char serialBuffer[31] = "";

void setup(void) {
  Serial.begin(115200);
  Serial1.begin(115200);
  printf_begin();
  radio.begin();
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(70);
  radio.enableDynamicPayloads();
  radio.setRetries(0,15);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.startListening();
  radio.printDetails();
  Serial.println();
  Serial.println("RF Chat V01.0");
  delay(500);
}
void loop(void) {
  nRF_receive();
  serial_receive();
}

void serialEvent1() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    SendPayload += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void nRF_receive(void) {
  int len = 0;
  if ( radio.available() ) {
    len = radio.getDynamicPayloadSize();
    radio.read(&RecvPayload,len);
    RecvPayload[len] = 0;
    Serial.print(RecvPayload);
    Serial.println();
    String a=String(RecvPayload);
    if (a.startsWith("MOTOR")) {
      a= a.substring(6);
      Serial1.println(a);
    }
    RecvPayload[0] = 0;
  }
}

void serial_receive(void) {
  if (stringComplete) {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(0,pipes[0]);
    radio.stopListening();
    char c[31]="";
    SendPayload.toCharArray(c,SendPayload.length());
    delay(2);
    //Delay is necessary, else transmission stops
    radio.write(c,sizeof(c));
    Serial.println(c);
    stringComplete = false;
    
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
    radio.startListening();
    SendPayload="";
    dataBufferIndex = 0;
  }
}
