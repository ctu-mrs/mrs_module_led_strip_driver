extern "C" {
#include <llcp.h>
}

#include "msgs.h"

#define F_CPU 8000000L

#define EN_12V A3
#define OUT_A 9
#define OUT_B 10
#define OUT_VBAT 8
#define PWM_RESOLUTION 100


#define TX_BUFFER_LEN 255
uint8_t tx_buffer[TX_BUFFER_LEN];
LLCP_Receiver_t llcp_receiver;
uint16_t num_cmds_received = 0;
long last_hb = millis();

uint8_t out_A = 0;
uint8_t out_B = 0;
bool out_VBAT = false;

void setup() {
  pinMode(EN_12V, OUTPUT);
  pinMode(OUT_A, OUTPUT);
  pinMode(OUT_B, OUTPUT);
  pinMode(OUT_VBAT, OUTPUT);
  digitalWrite(OUT_A, LOW);
  digitalWrite(OUT_B, LOW);
  digitalWrite(OUT_VBAT, LOW);
  digitalWrite(EN_12V, HIGH);

  TCCR1A = _BV(COM1A0) | _BV(COM1B0) |  _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);

  TCCR1B = _BV(WGM13) | _BV(WGM12)
           | _BV(CS11) | _BV(CS10);
  ICR1 = PWM_RESOLUTION;

  setPWM(out_A, out_B);

  Serial.begin(57600);
  llcp_initialize(&llcp_receiver);


}

void loop() {
  if (receive_message()) {

    digitalWrite(OUT_VBAT, out_VBAT);
    setPWM(out_A, out_B);

  }
  /*
    delay (20);

    pwm_value +=  1;

    if (pwm_value == 0 || pwm_value > 100) {
    pwm_value = 0;
    setPWM(0, 0);
    output = !output;

    digitalWrite(OUT_VBAT, HIGH);
    delay(30);
    digitalWrite(OUT_VBAT, LOW);
    delay(30);
    digitalWrite(OUT_VBAT, HIGH);
    delay(30);
    digitalWrite(OUT_VBAT, LOW);
    delay(30);
    digitalWrite(OUT_VBAT, HIGH);
    delay(30);
    digitalWrite(OUT_VBAT, LOW);
    delay(30);
    }

    if (output) {

    setPWM(pwm_value, 0);

    } else {
    setPWM(0, pwm_value);
    }*/
  if (millis() - last_hb >= 1000) {
    last_hb = millis();
    send_heartbeat();
  }
  delay(1);
}

bool receive_message() {
  uint16_t msg_len;
  bool got_valid_msg = false;
  LLCP_Message_t* llcp_message_ptr;

  while (Serial.available() > 0) {
    bool checksum_matched;
    uint8_t char_in = Serial.read();
    //individual chars are processed one by one by llcp, if a complete message is received, llcp_processChar() returns true
    if (llcp_processChar(char_in, &llcp_receiver, &llcp_message_ptr, &checksum_matched)) {
      if (checksum_matched) {
        switch (llcp_message_ptr->payload[0]) {
          case CMD_MSG_ID: {

              cmd_msg *received_msg = (cmd_msg *)llcp_message_ptr;

              out_A = received_msg->set_out_a;
              out_B = received_msg->set_out_b;
              out_VBAT = received_msg->set_out_vbat;

              got_valid_msg = true;
              num_cmds_received++;
              break;
            }
        }

        return true;
      }
    }
  }
  return got_valid_msg;
}

void setPWM(uint8_t pwm_a, uint8_t pwm_b) {
  if (pwm_a > PWM_RESOLUTION) {
    pwm_a = PWM_RESOLUTION;
  } if (pwm_b > PWM_RESOLUTION) {
    pwm_b = PWM_RESOLUTION;
  }

  OCR1A = PWM_RESOLUTION - pwm_a;
  OCR1B = PWM_RESOLUTION - pwm_b;

}

void send_heartbeat() {

  heartbeat_msg my_msg;
  uint16_t msg_len;

  // fill the message with data

  my_msg.id = HEARTBEAT_MSG_ID;
  my_msg.is_ok = true;
  my_msg.cmds_received = num_cmds_received;

  //llcp_prepareMessage will fill your TX buffer while returning the number of bytes written
  msg_len = llcp_prepareMessage((uint8_t*)&my_msg, sizeof(my_msg), tx_buffer);

  //send the message out over the serial line
  for (int i = 0; i < msg_len; i++) {
    Serial.write(tx_buffer[i]);
  }
}
