namespace { //anonymous namespace workarount to avoid compilation error http://stackoverflow.com/a/28181183
#define DEBUG
//#define DefEmulating
#define DefMotorSpeed
//#define DefOneWire
//#define DefSoftSerial //if will use softserial to control
}

#if defined DefSoftSerial
#include <SoftwareSerial.h>
#endif

//#if defined DefMotorSpeed
//#include <math.h>
//#endif
//
//#if defined DefOneWire
//#include <stdio.h>
//#include <stdint.h>
//#endif

//Pin Definitions
#if defined DefMotorSpeed
/*Motor Pins*/
#define TurretLeft 2   //D2 white  /white
#define TurretRight 4  //D4 yellow /black

#define LeftForward 10 //D7 brown
#define LeftReverse 6  //D8 green

#define RightForward 9 //D13 red
#define RightReverse 5 //D12 black

//static const int TurretLeft = 2;    //D2 white  /white
//static const int TurretRight = 4;   //D4 yellow /black
//
//static const int LeftForward = 10;   //D7 brown
//static const int LeftReverse = 6;   //D8 green
//
//static const int RightForward = 9; //D13 red
//static const int RightReverse = 5; //D12 black
#endif

#if defined DefOneWire
//PWM Code Pins
#define PinPPM 8
//static const int PinPPM = 8;
#endif

#if defined DefSoftSerial
//SoftwareSerial Pins
static const int softSerialTX = 12;
static const int softSerialRX = 11;

SoftwareSerial softSerial(softSerialRX, softSerialTX); // RX, TX
#endif

#if defined DefMotorSpeed
// Control Motor Definitions
static const int LF =20;
static const int LR =21;
static const int RF =22;
static const int RR =23;

int MinSpeed = 120;
int MaxSpeed = 255;
int MinDistance = 10;
#endif

#if defined DefOneWire
//PWM/PPM Signal Definitions
static const uint16_t SYNC_US = 1300;
static const uint16_t SPACING_US = 500;
static const uint16_t SIGNAL_US = 800;
static const uint8_t CYCLES = 2;

//PWM/PPM Signail Constants - bit position
static const uint8_t BIT_LEFT_UP      = 0; //0b0000000000000001;
static const uint8_t BIT_LEFT_DOWN    = 1; //0b0000000000000010;
static const uint8_t BIT_RIGHT_UP     = 2; //0b0000000000000100;
static const uint8_t BIT_RIGHT_DOWN   = 3; //0b0000000000001000;
static const uint8_t BIT_TURRET_LEFT  = 4; //0b0000000000010000;
static const uint8_t BIT_TURRET_RIGHT = 5; //0b0000000000100000;
static const uint8_t BIT_BUTTON_FIRE  = 6; //0b0000000001000000;
static const uint8_t BIT_BUTTON_POWER = 7; //0b0000000010000000;
static const uint8_t BIT_ONE_CMD      = 8; //0b0000000100000000;

uint16_t CodeToSend = 0x0000;
#endif

//joy minmax is -32768 to 32767 or 0-65535

struct Descriptor_t {
  uint8_t headerType; // 0x00
  uint8_t Type;
  uint8_t b1; //buttonsP1;
  uint8_t b2; //buttonsP2;
  uint8_t b3; //AxisA; //min 1 max 255 neutral 128
  uint8_t b4; //AxisB;
  uint8_t b5; //AxisC;
  uint8_t b6; //AxisD;
  uint16_t CRC;
};

Descriptor_t Descriptor;
static const uint8_t SizeOfDescriptor = 10;

//ATMEGA328 Serial buffer holds 64 bytes
static const uint8_t BufferSize = 32;
uint8_t BufferData[BufferSize];
uint8_t BufferIndex = 0;

uint8_t ByteLength = 0;
uint8_t CharFound = 0;

int isRunning = 0;

// "Delay" variables
unsigned long msgRecTimeOut = 500UL; //if serial not receive data stop motors and/or clear serial buffer
unsigned long msgRecLastMillis = 0UL;

unsigned long msgSendInverval = 300UL; //to keep toy working between wifi data receptions (latency issues)
unsigned long msgSentLastMillis = 0UL;

unsigned long msgToggleInverval = 300UL; //TODO: re-send interval in micros for Speed Control -> for 2nd Stage
unsigned long msgToggleLastMicros = 0UL;

uint8_t getBit(uint8_t Byte, uint8_t index) {
  byte mask = (byte)(1 << index);
  return (Byte & mask) != 0;
}

uint8_t setBit8(uint8_t Byte, uint8_t index){
  uint8_t mask = (1 << index);
  return (Byte | mask);
}

uint8_t clearBit8(uint8_t Byte, uint8_t index){
  uint8_t mask = (1 << index);
  return (Byte & ~mask);
}

void printBits8(uint8_t Byte){
  for(uint8_t mask = 0x80; mask; mask >>= 1){
    if(mask & Byte)
      Serial.print('1');
    else
      Serial.print('0');
  }
}

void printBits16A(uint16_t Byte){
  for(uint16_t mask = 0x8000; mask; mask >>= 1){
    if(mask & Byte)
      Serial.print('1');
    else
      Serial.print('0');
  }
}

void printBits16(uint16_t Code){
  for (int i = 0; i < 16; i++) {
    uint8_t bit = (Code & 0x8000) >> 15;
    Serial.print(bit);
    Code <<= 1;
  }
}

void SerialRead(uint8_t inByte) {
  if(BufferIndex > BufferSize){
    #if defined DEBUG
    Serial.println(F("#Buffer overflow"));
    #endif

    BufferReset();

    //#if defined DefSoftSerial
    //softSerial.flush();
    //#else
    //Serial.flush();
    //#endif
    return;
  }

  if(ByteLength == 0 && CharFound == 0) /*wait for sync byte*/{
    /*if char # (DEC 35, HEX 0x23) found, will wait for '\n' postfix*/
    if (inByte == 0x23 /*# - 35 - 0x23*/){ 
      #if defined DEBUG
      Serial.println(F("#CharFound"));
      #endif
      CharFound = 1;
    }
    /*if char SOH (DEC 1, HEX 0x21 found, will wait for SizeOfDescriptor*/
    else if (inByte == 0x01 /*SOH - 1 - 0x21*/){
      #if defined DEBUG
      Serial.println(F("#ByteLength"));
      #endif
      ByteLength = 1;
    }
  }

  //prefix found start buffering
  if((ByteLength != 0) | (CharFound != 0)) {
    BufferData[BufferIndex] = inByte;

    #if defined DefEmulating
    Serial.print(F("#read "));
    Serial.println(BufferIndex,10);
    #endif
  }

  /*wait for a full byte pack*/
  if(ByteLength != 0){
    #if defined DefEmulating
    Serial.print(F("#ByteSync "));
    Serial.println(BufferIndex);
    #endif
    if (BufferIndex >= SizeOfDescriptor -1) {
      #if defined DEBUG
      Serial.println(F("#read bytes"));
      #endif

      uint8_t byteStart = BufferIndex - (SizeOfDescriptor -1);
      //uint16_t crcPack = ((uint16_t)BufferData[BufferIndex] << 8) | BufferData[BufferIndex - 1];

      uint16_t crcPack = (uint16_t)((BufferData[BufferIndex] << 8) + (BufferData[BufferIndex -1] & 0xFF));
      uint16_t crc = getCRCMODBUS(BufferData, byteStart, SizeOfDescriptor - 2);

      #if defined DefEmulating
      Serial.print(F("#from byte: "));
      Serial.print(byteStart,10);
      Serial.print(F(" to byte: "));
      Serial.print(BufferIndex,10);

      uint8_t bytePos = 0;
      for(uint8_t i = byteStart; i < SizeOfDescriptor + byteStart; i++){
        Serial.print(F(" byte"));
        Serial.print(bytePos);
        Serial.print(F(": "));
        printBits8(BufferData[i]);
        bytePos++;
      }
      
      Serial.print(F(" CRC: "));
      Serial.print(crcPack, 10);
      Serial.print(F(", CRC check: "));
      Serial.print(crc);
      Serial.println();
      #endif
      
      //check for CRC
      if(crc == crcPack){
        memcpy(&Descriptor,&BufferData[byteStart],SizeOfDescriptor); //Copy memory from buffer to struct

        byte* my_s_bytes = reinterpret_cast<byte*>(&Descriptor);

        #if defined DefEmulating
        Serial.print(F("#Bytes"));
        for(uint8_t i = 0; i < SizeOfDescriptor; i++){
          Serial.print(F(" byte"));
          Serial.print(i);
          Serial.print(F(": "));
          printBits8(my_s_bytes[i]);
        }
             
        Serial.print(F(" Size: "));
        Serial.print(sizeof(Descriptor));
        Serial.print(F(" headerType: "));
        Serial.print(Descriptor.headerType);
        Serial.print(F(" Type: "));
        Serial.print(Descriptor.Type);
        Serial.print(F(" b1: "));
        Serial.print(Descriptor.b1);
        Serial.print(F(" b2: "));
        Serial.print(Descriptor.b2);
        Serial.print(F(" b3: "));
        Serial.print(Descriptor.b3,10);
        Serial.print(F(" b4: "));
        Serial.print(Descriptor.b4,10);
        Serial.print(F(" b5: "));
        Serial.print(Descriptor.b5,10);
        Serial.print(F(" b6: "));
        Serial.print(Descriptor.b6,10);
        Serial.print(F(" CRC: "));
        Serial.print(Descriptor.CRC,10);
        Serial.println();
        #endif

        #if defined DefOneWire
        if(Descriptor.Type == 0x01)
          ControlOnePin(Descriptor.b1); //Process the descriptor to tank control
        #endif

        #if defined DefMotorSpeed
        if(Descriptor.Type == 0x01)
          ControlDir(Descriptor.b1); //8 buttons 7-0
        if(Descriptor.Type == 0x02)
          DriveMotorSpeed(Descriptor.b3, Descriptor.b4, Descriptor.b5, Descriptor.b6); //LM RM LS RS
        if(Descriptor.Type == 0x03)
          DriveMotorAxis(Descriptor.b3, Descriptor.b4); //axis X,Y
        #endif

        BufferReset();
        return;
      }
      else {
        Serial.println(F("#Invalid CRC"));
      }
    }
  }
  else if(CharFound != 0){
    if((char)BufferData[BufferIndex] == '\n'){
      #if defined DEBUG
      Serial.println(F("#read chars"));
      #endif

      //Serial.print("#BufferData: ");
      //Serial.println((char*)BufferData);

      Serial.print("#Data: ");
      for(int i = 0; i < 32; i++)
        Serial.print((char)BufferData[i]);
      Serial.println("\n");
      
      //char ping[] = {'#','P','I','N','G'};
      //if(charComp((char*)BufferData, ping,0,5) != 0){
      if(BufferData[0] == '#' && BufferData[1] == 'P' && BufferData[2] == 'I' && BufferData[3] == 'N' && BufferData[4] == 'G'){
        BufferData[2] = 'O';
        BufferData[BufferIndex] = '\n';
        Serial.write(BufferData, sizeof(BufferData));

        //char datepart[22]; /*16 - 0 - 15, */
        //memcpy(&datepart,&BufferData, 22);

        //datepart[2] = 'O';
        //datepart[21] = '\n';

        //Serial.print("#datepart: ");
        //for(int i = 0; i < 16; i++)
        //  Serial.print((char)datepart[i]);
        //Serial.println();

        //Serial.write(datepart, sizeof(datepart));
      }

      #if defined DefMotorSpeed
      ExecuteCommand((char*)BufferData);
      #endif

      BufferReset();
      return;
    }
  }
  //set next buffer index
  if((ByteLength != 0) | (CharFound != 0)) {
    BufferIndex++;
  }
}

uint8_t charComp(const char *data1, const char *data2, uint8_t start, uint8_t length){
  for(uint8_t i = start; i < length;i++){
    if(data2[i] != data1[i]){
      return 0;
    }
  }

  return 1;
}

int getposition(const char *array, size_t size, char c){
    for (size_t i = 0; i < size; i++)
        if (array[i] == c)
            return (int)i;
    return -1;
}

void BufferReset(){
  #if defined DEBUG
  Serial.println(F("#BufferReset"));
  #endif

  memset(BufferData, 0, BufferSize); //Clear the buffer
  BufferIndex = 0; //reset the index
  ByteLength = 0;
  CharFound = 0;

  //Clear descriptor
  Descriptor.headerType = 0x00;
  Descriptor.Type = 0x00;
  Descriptor.b1 = 0x00;
  Descriptor.b2 = 0x00;
  Descriptor.b2 = 0x00;
  Descriptor.b4 = 0x00;
  Descriptor.b5 = 0x00;
  Descriptor.b6 = 0x00;
  Descriptor.CRC = 0x00;
}

void receiveTimeOut(){
  //Timer to check Last message received and stops the motor
  if ((millis() - msgRecLastMillis) >= msgRecTimeOut) {
    if(isRunning != 0) {
      //#if defined DEBUG
      Serial.println(F("#ReceiveTimeOut-isRunning"));
      //#endif

      #if defined DefMotorSpeed
      digitalWrite(LeftForward,LOW); 
      digitalWrite(LeftReverse,LOW);
      digitalWrite(RightForward,LOW); 
      digitalWrite(RightReverse,LOW);

      digitalWrite(TurretRight,LOW);
      digitalWrite(TurretLeft,LOW);
      #endif
      
      #if defined DefOneWire
      //Stop if is running
      CodeToSend = 0x0000;
        SendCode(CodeToSend);
      #endif

      isRunning = 0;
    }

    if(BufferData[0] != 0x00){
      //#if defined DEBUG
      Serial.println(F("#ReceiveTimeOut-BufferData"));
      //#endif
      BufferReset();
    }
  }

  #if defined DefOneWire
  if(isRunning != 0 /*running*/){
    if ((millis() - msgSentLastMillis) >= msgSendInverval) {
      //Serial.println(F("#Send by interval");
      SendCode(CodeToSend); //continue send until stop by command or receive time out
    }
  }
  #endif
}

uint16_t getCRCMODBUS(const uint8_t *data, uint8_t byteStart, uint8_t Size){
  uint16_t crc = 0xFFFF;
  //uint16_t val = 0;

  uint8_t End = Size + byteStart;
  #if defined DefEmulating
  Serial.print(F("#getCRCMODBUS from "));
  Serial.print(byteStart,10);
  Serial.print(", Size: ");
  Serial.print(End,10);
  #endif

  for (uint8_t pos = byteStart; pos < End; pos++){
    #if defined DefEmulating
    Serial.print(", byte");
    Serial.print(pos,10);
    Serial.print(": ");
    Serial.print(data[pos], 10);
    #endif

    crc ^= (uint16_t)data[pos];          // XOR byte into least sig. byte of crc

    for (uint8_t i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }

  #if defined DefEmulating
  Serial.println();
  #endif
  // Note, crc has low and high bytes swapped, so use it accordingly (or swap bytes)
  //val = (uint16_t)((crc & 0xff) << 8);
  //val = (uint16_t)(val + ((crc >> 8) & 0xff));
  //System.out.printf("Calculated a CRC of 0x%x, swapped: 0x%x\n", crc, val);
  //return val;
  return crc;
}

#if defined DefMotorSpeed
void ExecuteCommand(char* DataString){
  int errors = 0;

  char *chpt = strtok(DataString, "("); //Split string into tokens
  if (chpt == NULL) {
    #if defined DEBUG
    Serial.println(F("#strtok returns NULL"));
    #endif
  }
  else {
    if(strcmp(chpt, "#M") == 0){
      int LD = 0;
      int RD = 0;
      int LSpeed = 0;
      int RSpeed = 0;

      chpt = strtok(NULL, ",");
      if (chpt == NULL) {
        #if defined DEBUG
        Serial.println(F("#First param strok returns NULL"));
        #endif
        errors = 1;
      }
      else {
        LD = atoi(chpt);
      }

      if (errors == 0) {
        chpt = strtok(NULL, ",");
        if (chpt == NULL) {
          #if defined DEBUG
          Serial.println(F("#Second param strok returns NULL"));
          #endif
          errors = 1;
        }
        else {
          RD = atoi(chpt);
        }
      }

      if (errors == 0) {
        chpt = strtok(NULL, ",");
        if (chpt == NULL) {
          #if defined DEBUG
          Serial.println(F("#Third param strok returns NULL"));
          #endif
          errors = 1;
        }
        else {
          LSpeed = atoi(chpt);
        }
      }

      if (errors == 0) {
        chpt = strtok(NULL, ",");
        if (chpt == NULL) {
          #if defined DEBUG
          Serial.println(F("#Fourth param strok returns NULL"));
          #endif
          errors = 1;
        }
        else {
          RSpeed = atoi(chpt);
        }
      }

      if (errors == 0) {
        #if defined DefMotorSpeed
        DriveMotorSpeed(LD, RD, LSpeed, RSpeed);
        #endif
      }
    }
  }
}

void DriveTankAndSpeed(){

}

void ControlDir(uint8_t buttons){
  isRunning = 0;

  ////P1-0 Power
  //if(getBit(buttons,0) == 1){
  //}

  ////P1-1 Fire
  //if(getBit(buttons,1) == 1){
  //}

  //P1-2 Turret Right
  if(getBit(buttons,2) == 1){
    #if defined DefEmulating
    Serial.println(F("#Turret Right"));
    #endif

    digitalWrite(TurretLeft,LOW);
    digitalWrite(TurretRight,HIGH);
    isRunning++;
  }
  //P1-3 Turret Left
  else if(getBit(buttons,3) == 1){
    #if defined DefEmulating
    Serial.println(F("#Turret Left"));
    #endif

    digitalWrite(TurretRight,LOW);
    digitalWrite(TurretLeft,HIGH);
    isRunning++;
  }
  //Turret Stop
  else {
    #if defined DefEmulating
    Serial.println(F("#Turret Stop"));
    #endif

    digitalWrite(TurretRight,LOW);
    digitalWrite(TurretLeft,LOW);
  }

  #if defined StickUpDown
  //P1-4 Right Down
  if(getBit(buttons,4) == 1){
    digitalWrite(RightForward, LOW);
    digitalWrite(RightReverse,HIGH);
  }
  //P1-5 Right UP
  else if(getBit(buttons,5) == 1){
    digitalWrite(RightReverse,LOW);
    digitalWrite(RightForward, HIGH);
  }

  //P1-6 Left Down
  if(getBit(buttons,6) == 1){
    analogWrite(LeftForward, HIGH);
    digitalWrite(LeftReverse, LOW);
  }
  //P1-7 Left UP
  else if(getBit(buttons,7) == 1){
    digitalWrite(LeftReverse,LOW);
    analogWrite(LeftForward, HIGH);
  }
  #endif

  int Speed = 255;
  int HalfSpeed = 180;

  //D-PAD
  //Set from D=PAD bits
  //P1-4 UP + P1-6 LEFT
  if(getBit(buttons,4) == 1 && getBit(buttons,6) == 1){
    #if defined DefEmulating
    Serial.println(F("#UP + LEFT"));
    #endif

    digitalWrite(LeftReverse,LOW);
    digitalWrite(RightReverse,LOW);
    analogWrite(LeftForward, HalfSpeed);
    analogWrite(RightForward, Speed);
    isRunning++;
  }
  //P1-4 UP + P1-7 RIGHT
  else if(getBit(buttons,4) == 1 && getBit(buttons,7) == 1){
    #if defined DefEmulating
    Serial.println(F("#UP + RIGHT"));
    #endif

    digitalWrite(LeftReverse,LOW);
    digitalWrite(RightReverse,LOW);
    analogWrite(LeftForward, Speed);
    analogWrite(RightForward, HalfSpeed);
    isRunning++;
  }
  //P1-4 UP
  else if(getBit(buttons,4) == 1){
    #if defined DefEmulating
    Serial.println(F("#UP"));
    #endif
    digitalWrite(LeftReverse,LOW);
    digitalWrite(RightReverse,LOW);
    analogWrite(LeftForward, Speed);
    analogWrite(RightForward, Speed);
    isRunning++;
  }
  //P1-5 DOWN + P1-6 LEFT
  else if(getBit(buttons,5) == 1 && getBit(buttons,6) == 1){
    #if defined DefEmulating
    Serial.println(F("#DOWN + LEFT"));
    #endif
    digitalWrite(LeftForward,LOW);
    digitalWrite(RightForward,LOW);
    analogWrite(LeftReverse, HalfSpeed); 
    analogWrite(RightReverse, Speed);
    isRunning++;
  }
  //P1-5 DOWN + P1-7 RIGHT
  else if(getBit(buttons,5) == 1 && getBit(buttons,7) == 1){
    #if defined DefEmulating
    Serial.println(F("#DOWN + RIGHT"));
    #endif
    digitalWrite(LeftForward,LOW);
    digitalWrite(RightForward,LOW);
    analogWrite(LeftReverse, Speed);
    analogWrite(RightReverse, HalfSpeed);
    isRunning++;
  }
  //P1-5 DOWN
  else if(getBit(buttons,5) == 1){
    #if defined DefEmulating
    Serial.println(F("#DOWN"));
    #endif
    digitalWrite(LeftForward,LOW);
    digitalWrite(RightForward,LOW);
    analogWrite(LeftReverse, Speed); 
    analogWrite(RightReverse, Speed);
    isRunning++;
  }
  //P1-6 LEFT
  else if(getBit(buttons,6) == 1){
    #if defined DefEmulating
    Serial.println(F("#LEFT"));
    #endif
    digitalWrite(LeftForward,LOW);
    digitalWrite(RightReverse,LOW);
    analogWrite(LeftReverse, HalfSpeed);
    analogWrite(RightForward, HalfSpeed);
    isRunning++;
  }
  //P1-7 RIGHT
  else if(getBit(buttons,7) == 1){
    #if defined DefEmulating
    Serial.println(F("#RIGHT"));
    #endif
    digitalWrite(LeftReverse,LOW);
    digitalWrite(RightForward,LOW);
    analogWrite(LeftForward, HalfSpeed);
    analogWrite(RightReverse, HalfSpeed);
    isRunning++;
  }
  //STOP
  else {
    #if defined DefEmulating
    Serial.println(F("#STOP"));
    #endif
    digitalWrite(LeftForward,LOW); 
    digitalWrite(LeftReverse,LOW);
    digitalWrite(RightForward,LOW); 
    digitalWrite(RightReverse,LOW);
  }
}

void DriveMotorSpeed(uint8_t LD, uint8_t RD, uint8_t LSpeed, uint8_t RSpeed){
  #if defined DefEmulating
  Serial.print(F("#LD: "));
  Serial.print(LD, 10);
  Serial.print(F(", RD: "));
  Serial.print(RD, 10);
  Serial.print(F(", LS: "));
  Serial.print(LSpeed, 10);
  Serial.print(F(", RS: "));
  Serial.println(RSpeed, 10);
  #endif

  isRunning = 0;

  if (LD == LF) {
    digitalWrite(LeftReverse,LOW);
    analogWrite(LeftForward, LSpeed);
    isRunning++;
  }
  else if (LD == LR) {
    digitalWrite(LeftForward,LOW);;
    analogWrite(LeftReverse, LSpeed);
    isRunning++;
  }
  else{
    digitalWrite(LeftForward,LOW);
    digitalWrite(LeftReverse,LOW);
  }

  if (RD == RF) {
    digitalWrite(RightReverse,LOW);
    analogWrite(RightForward, RSpeed);
    isRunning++;
  }
  else if (RD == RR) {
    digitalWrite(RightForward,LOW);
    analogWrite(RightReverse, RSpeed);
    isRunning++;
  }
  else {
    digitalWrite(RightForward,LOW);
    digitalWrite(RightReverse,LOW);
  }
}

void XYToDegrees(int xyX, int xyY, int& angle, int& distance){
  //double angle = 0;
  int originX = 127;
  int originY = 127;

  if (xyY <= originY) {
      if (xyX > originX) {
        //up right
        //angle = (double)(xyX - originX) / (double)(originY - xyY);
        //angle = atan(-angle);
        //angle = 270 - angle * 180 / 3.14159 /*PI*/;

        angle = (int) (270 - atan(-(double)(xyX - originX) / (double)(originY - xyY)) * 180 / 3.14159);
      }
      else if (xyX <= originX) {
        //up left
        //angle = (double)(originX - xyX) / (double)(originY - xyY);
        //angle = atan(angle);
        //angle = 270.0f - angle * 180.0f / 3.14159f /*PI*/;
        angle = (int)(270 - atan((double)(originX - xyX) / (double)(originY - xyY)) * 180 / 3.14159) /*PI*/;
      }
  }
  else if (xyY > originY) {
      if (xyX > originX) {
        //down right
        //angle = (double)(xyX - originX) / (double)(xyY - originY);
        //angle = atan(angle);
        //angle = 90.0f - angle * 180.0f / 3.14159f /*PI*/;
        angle = (int)(90 - atan((double)(xyX - originX) / (double)(xyY - originY)) * 180 / 3.14159 /*PI*/);
      }
      else if (xyX <= originX) {
        //down left
        //angle = (double)(originX - xyX) / (double)(xyY - originY);
        //angle = atan(-angle);
        //angle = 90.0f - angle * 180.0f / 3.14159f /*PI*/;
        angle = (int)(90 - atan(-(double)(originX - xyX) / (double)(xyY - originY)) * 180 / 3.14159f /*PI*/);
      }
  }

  distance = (int)sqrt(pow((xyX - originX), 2) + pow((xyY - originY), 2));
  //if (angle > 180) angle -= 360; //Optional. Keeps values between -180 and 180
}

void DriveMotorAxis(int xyX, int xyY){
  int distance = 0;
  int angle = 0;
  
  int LD = 0;
  int RD = 0;
  int L = 0;
  int R = 0;

  XYToDegrees(xyX,xyY,angle,distance);

  if (distance > MinDistance){
    if (angle >= 181 && angle <= 270){
      L = map_int(angle, 180, 270, MinSpeed, MaxSpeed);
      R = 255;
      LD = LF;
      RD = RF;
    } else if (angle >= 271 && angle <= 360){
      R = map_int(angle, 271, 360, MaxSpeed, MinSpeed);
      L = 255;
      LD = LF;
      RD = RF;
    }
    else if (angle >= 91 && angle <= 180) {
      L = map_int(angle, 91, 180, MaxSpeed, MinSpeed);
      R = 255;
      LD = LR;
      RD = RR;
    }
    else if (angle >= 1 && angle <= 90) {
      R = map_int(angle, 1, 90, MinSpeed, MaxSpeed);
      L = 255;
      LD = LR;
      RD = RR;
    }
  }

  R = map_uint8(distance, 1, 127, MinSpeed, R);
  L = map_uint8(distance, 1, 127, MinSpeed, L);

  //#if defined DefEmulating
  //Serial.print(F("#Angle: "));
  //Serial.print(angle,10);
  //Serial.print(F(" Distance: "));
  //Serial.println(distance,10);
  //#endif

  DriveMotorSpeed(LD, RD, L, R);
}

int map_int(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t map_uint8(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif

#if defined DefOneWire
void SendCode(uint16_t Code){
  //Serial.print(F("#SendCode took ");
  //unsigned long startMicros = micros();

  #if defined DefEmulating 
  Serial.print(F("#SendCode: "));
  for (int index = 0; index < 14; index++) {
    uint8_t bit = (CodeToSend & (1 << index)) != 0;
    Serial.print(bit);
  }
  Serial.println();
  #endif

  for(int x = 0; x < CYCLES; x++){
    pulseOut(PinPPM, SYNC_US, SYNC_US);
    for (int index = 0; index < 13; index++) {
      uint8_t bit = (Code & (1 << index)) != 0;
      if(bit == 1)
        pulseOut(PinPPM, SIGNAL_US, SIGNAL_US);
      else
        pulseOut(PinPPM, SPACING_US, SPACING_US);
    }
  }

  msgSentLastMillis = millis();//last sent time to continue sending

  //unsigned long DifMicros = micros() - startMicros;
  //Serial.print(DifMicros,10);
  //Serial.println();
}

void pulseOut(uint8_t pin, uint16_t MicroHigh, uint16_t MicoLow){
  //Serial.println(F("#"));
  //Serial.println(MicroHigh);

  digitalWrite(pin,HIGH);
  delayMicroseconds(MicroHigh);
  digitalWrite(pin, LOW);
  delayMicroseconds(MicoLow);
}

void ControlOnePin(uint8_t buttons){
  uint8_t pulses = 0;
  isRunning = 0;

  CodeToSend = 0x0000;
  if(buttons != 0x00){
    //P1-0 Power
    if(getBit(buttons,0) == 1){
      #if defined DefEmulating
      Serial.println(F("#Power"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_BUTTON_POWER);
      pulses++;
    }

    //P1-1 Fire
    if(getBit(buttons,1) == 1){
      #if defined DefEmulating
      Serial.println(F("#Fire"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_BUTTON_FIRE);
      pulses++;
    }

    //P1-2 Turret Right
    if(getBit(buttons,2) == 1){
      #if defined DefEmulating
      Serial.println(F("#Turret Right"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_TURRET_RIGHT);
      pulses++;
    }

    //P1-3 Turret Left
    else if(getBit(buttons,3) == 1){
      #if defined DefEmulating
      Serial.println(F("#Turret Left"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_TURRET_LEFT);
      pulses++;
    }

    #if defined UpDownX2
    //P1-4 Right Down
    if(getBit(buttons,4) == 1){
      CodeToSend = CodeToSend | (1 << BIT_RIGHT_DOWN);
    }
    //P1-5 Right UP
    else if(getBit(buttons,5) == 1){
      CodeToSend = CodeToSend | (1 << BIT_RIGHT_UP);
    }
    //P1-6 Left Down
    if(getBit(buttons,6) == 1){
      CodeToSend = CodeToSend | (1 << BIT_LEFT_DOWN);
    }
    //P1-7 Left UP
    else if(getBit(buttons,7) == 1){
      CodeToSend = CodeToSend | (1 << BIT_LEFT_UP);
    }
    #endif

    //D-PAD
    //Clear direction bits
    CodeToSend = CodeToSend & ~(1 << BIT_RIGHT_UP);
    CodeToSend = CodeToSend & ~(1 << BIT_RIGHT_DOWN);
    CodeToSend = CodeToSend & ~(1 << BIT_LEFT_UP);
    CodeToSend = CodeToSend & ~(1 << BIT_LEFT_DOWN);

    //Set from D=PAD bits
    //P1-4 UP + P1-6 LEFT
    if(getBit(buttons,4) == 1 && getBit(buttons,6) == 1){
      #if defined DefEmulating
      Serial.println(F("#UP + LEFT"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_RIGHT_UP);
      pulses++;
    }
    //P1-4 UP + P1-7 RIGHT
    else if(getBit(buttons,4) == 1 && getBit(buttons,7) == 1){
      #if defined DefEmulating
      Serial.println(F("#UP + RIGHT"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_LEFT_UP);
      pulses++;
    }
    //P1-4 UP
    else if(getBit(buttons,4) == 1){
      #if defined DefEmulating
      Serial.println(F("#UP"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_RIGHT_UP);
      CodeToSend = CodeToSend | (1 << BIT_LEFT_UP);
      pulses += 2;
    }

    //P1-5 DOWN + P1-6 LEFT
    else if(getBit(buttons,5) == 1 && getBit(buttons,6) == 1){
      #if defined DefEmulating
      Serial.println(F("#DOWN + LEFT"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_RIGHT_DOWN);
      pulses++;
    }
    //P1-5 DOWN + P1-7 RIGHT
    else if(getBit(buttons,5) == 1 && getBit(buttons,7) == 1){
      #if defined DefEmulating
      Serial.println(F("#DOWN + RIGHT"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_LEFT_DOWN);
      pulses++;
    }
    //P1-5 DOWN
    else if(getBit(buttons,5) == 1){
      #if defined DefEmulating
      Serial.println(F("#DOWN"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_RIGHT_DOWN);
      CodeToSend = CodeToSend | (1 << BIT_LEFT_DOWN);
      pulses += 2;
    }
    //P1-6 LEFT
    else if(getBit(buttons,6) == 1){
      #if defined DefEmulating
      Serial.println(F("#LEFT"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_RIGHT_UP);
      CodeToSend = CodeToSend | (1 << BIT_LEFT_DOWN);
      pulses += 2;
    }
    //P1-7 RIGHT
    else if(getBit(buttons,7) == 1){
      #if defined DefEmulating
      Serial.println(F("#RIGHT"));
      #endif

      CodeToSend = CodeToSend | (1 << BIT_RIGHT_DOWN);
      CodeToSend = CodeToSend | (1 << BIT_LEFT_UP);
      pulses += 2;
    }
  }

  if(pulses == 1)
    CodeToSend = CodeToSend | (1 << (BIT_ONE_CMD));

  SendCode(CodeToSend);

  if(CodeToSend != 0x0000)
    isRunning = 1;
}
#endif

void setup(){
  Serial.begin(115200); //Serial.begin(57600);
  Serial.println(F("#Serial Ready"));

  #if defined DefSoftSerial
  softSerial.begin(57600);
  softSerial.println(F("#Bluetooth Ready"));
  #endif

  #if defined DefMotorSpeed
  //Set Pins for Motors
  pinMode(TurretRight, OUTPUT);
  pinMode(TurretLeft, OUTPUT);

  pinMode(LeftReverse, OUTPUT);
  pinMode(LeftForward, OUTPUT);

  pinMode(RightReverse, OUTPUT);
  pinMode(RightForward, OUTPUT); 
  #endif

  #if defined DefOneWire
  //Set Receiver Pin
  pinMode(PinPPM, OUTPUT);
  digitalWrite(PinPPM,LOW);
  #endif
}

void loop(){
  #if defined DefSoftSerial
  if(softSerial.available() > 0){
    while (softSerial.available() > 0) {
      uint8_t inByte = (uint8_t)softSerial.read(); //cast signed to unsigned is ok becouse of the right 8 bits
      SerialRead(inByte);
    }
    msgRecLastMillis = millis(); //set last time received a message
  }
  #else
  //if (Serial.available() > 0) {
  //  char inChar = (char)Serial.read(); 

  //  if (inChar == '\n') {
  //    ExecuteCommand();
  //    BufferIndex = 0; 
  //    memset(BufferData, 0, 30);
  //  }
  //  else {
  //    BufferData[BufferIndex] = inChar;
  //    BufferIndex++;
  //  }

  //  msgRecLastMillis = millis();
  //}

  if(Serial.available() > 0){
    while (Serial.available() > 0) {
      uint8_t inByte = (uint8_t)Serial.read(); //cast signed to unsigned is ok becouse of the right 8 bits
      SerialRead(inByte);
    }
  
    msgRecLastMillis = millis(); //set last time received a message
  }
  #endif
  else
    receiveTimeOut();
}