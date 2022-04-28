#include <EEPROM.h>
#include "ModbusXT.h"
#include <SoftwareSerial.h>

void(* resetFunc) (void) = 0;

SoftwareSerial BT(10, 11); // Setting up software serial com. which will be used in bluetooth

#define TIMEOUT 500   //Timeout for a failed packet. Timeout need to larger than polling. #500 is the old value
#define POLLING 2     //Wait time to next request

#define BAUD        57600  
#define RETRIES     15   //How many time to re-request packet frome slave if request is failed. #10 is the old value
#define BYTE_FORMAT SERIAL_8E1
#define TxEnablePin 2   //Arduino pin to enable transmission

#define print1(x)  Serial.print(x)   // simple defining operations for better writing
#define println1(x) Serial.println(x)

#define totalProduct 50     // product limit
#define sensorPin 3         // product counting switch
#define yellowPin 4         // yellow light relay pin which controls 220VAC
                            // red and yellow lights are controlled over a timer relay and motion sensor



//Name for register in regs[]
enum {
  button1,
  button2,
  button3,
  number_entry,
  password_entry,
  slider,
  total_packets,
  total_requests,
  total_failed,
  transfer_rate,
  transfer_delay,
  led_grn,
  led_blue,
  led_red,
  graph,
  button4,
  time_passed,
  TOTAL_REGS //=17
};

// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum {
  PACKET1,
  PACKET2,
  NO_OF_PACKET  //=2
};

// Masters register array
uint16_t regs[TOTAL_REGS];

int previousStatus = 0;  // 
int currentStatus;


int cTime = 0;
int pTime = 0;

//Modbus packet
Packet packets[NO_OF_PACKET];

// Access individual packet parameter. Uncomment it if you know what you're doing
// packetPointer packet1 = &packets[PACKET1];
// packetPointer packet2 = &packets[PACKET2];

int graph_value = 0;
int previousValue = 0;
int slider_value = 0;
long sm,em,dm;
uint16_t temp,num;
int dataAdress = 0;


char BTdata;

const uint8_t hmiID = 1;  //ID of HMI. The ID need to match, unless program will not work

//Modbus Master class define
Modbus master;  

void bluetoothCom(){
  if(BT.available()){
    
    BTdata = BT.read();

    if(BTdata = '1')
      BT.println(graph_value);
      
    
    }
  
  
  }


void setup()
{
  //Config packets and register
  master.configure(packets, NO_OF_PACKET, regs);

  //Config individual packet: (packet, ID, Function, Address, Number of register or data, start register in master register array)
  master.construct(&packets[PACKET1], hmiID, READ_HOLDING_REGISTERS, 0, 6, 0);

  master.construct(&packets[PACKET2], hmiID, PRESET_MULTIPLE_REGISTERS, 100, 11, 6);
  

  //Start Modbus
  master.begin(&Serial, BAUD, BYTE_FORMAT, TIMEOUT, POLLING, RETRIES, TxEnablePin);

  Serial.begin(57600);  //debug on serial0
  // this might be serial1 or smth


  pinMode(13, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  BT.begin(9600);
  
  pinMode(sensorPin, INPUT);
  
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);

}


void loop()
{
  
  master.update();  //polling
  
  bluetoothCom();

  int tTime = millis() / 1000;
  sm = millis();
  cTime = millis();
  // counting produced goods.
  currentStatus = digitalRead(sensorPin);

  // println(cTime-pTime);
  if(currentStatus != previousStatus && currentStatus == HIGH && cTime!=pTime ){
    graph_value++;
    EEPROM.put(dataAdress, graph_value);
    delay(10);
      
  }

  EEPROM.get(dataAdress, graph_value);
  delay(10);
  
  previousStatus = currentStatus;
  delayMicroseconds(800);
  pTime = cTime;

  regs[total_packets] = NO_OF_PACKET;             //Total number of packet, here is 2
  regs[total_requests] = master.total_requests(); //Update all requested packets. Take a look on ModbusXT.h
  regs[total_failed] = master.total_failed();     //Update all failed packet
  regs[graph] = graph_value;  //Update graph value
  regs[time_passed] = tTime; // Update time value


  //If button is press, turn on HMI's LED
  for (uint8_t i=0;i<3;i++)
  {
    if (regs[i] == 1)      
      regs[i+11] = 1;
    else
      regs[i+11] = 0;
  }
  if(regs[0] == 1){
    EEPROM.put(dataAdress, 0);
    delay(1000);
    resetFunc();
    
    }

  if ( digitalRead(sensorPin) ){
    
    regs[button4] = 1;
    digitalWrite(4, LOW);
    delayMicroseconds(50);
    digitalWrite(5, HIGH);
    }
  else if(!digitalRead(sensorPin)){
    regs[button4] = 0;
    digitalWrite(4, HIGH);
    delayMicroseconds(50);
    digitalWrite(5, LOW);
    }

  //If Led green is on (or button 0 = 1) turn on Led on arduino
  if (regs[led_grn])
    digitalWrite(13, HIGH);
  else
    digitalWrite(13, LOW);

  //Print number value on HMI
  if (num != regs[number_entry] )
  {
    num = regs[number_entry];

  }

  //Print slider value on HMI
  if (slider_value != regs[slider] )
  {
    slider_value = regs[slider];

  }

  //update transfer rate and transfer delay
  if ( (sm-dm) > 1000) //update 1s
  {
    dm = sm;
    regs[transfer_rate] = regs[total_requests] - temp;
    temp = regs[total_requests];

    regs[transfer_delay] = (unsigned int) ((NO_OF_PACKET*100000UL)/regs[transfer_rate]);
  }


  
  if (regs[total_failed] > 15){
       
    //Start Modbus
    master.begin(&Serial, BAUD, BYTE_FORMAT, TIMEOUT, POLLING, RETRIES, TxEnablePin);
  
    // Serial.begin(57600);  //debug on serial0
      }
  
}//end loop
