// niet vergeten de juiste AD clock te zetten in de library van de AD9833!! 16MHz moet het zijn voor dit ontwerp.
// deze versie heeft de automatische gain aan de real en imag level, ipv het signaalniveau aan de ingang. een lomend gemiddelde zorgt voor de juiste gainsetting

#define DEBUG 1   //debug on/off, uncomment for debug

#include <SPI.h>
#include <MD_AD9833.h>
#include <Wire.h>

// Pins for SPI comm with the AD9833 IC
#define DATA  11  ///< SPI Data pin number
#define CLK   13  ///< SPI Clock pin number
#define FSYNC 10  ///< SPI Load pin number (FSYNC in AD9833 usage)
#define FSYNC2 9  ///< SPI Load pin number (FSYNC in AD9833 usage)
#define adress 0x2E  //potmeter MCP40d17 “0101110”
#define SigLvl A3 //analog input for siglevel
#define SigReal A0  // input for the real value
#define SigImag A1  //input for the imaginary value (90 degrees shifted signal)
#define SigRDY A2 // Data Ready pin
#define Switchpoint 12 //was 15 treshold for the gain switching.
#define EnableAmp 5 //enable amplifier, high is on
#define LEDSignal 6 //led signal
int OffsetValue, OffsetReal, OffsetImag;
int16_t Real[10], Imag[10], Siglevel[10];   //was uint...
uint8_t Gain[10];
float Alpha = 0.1, Realavg[10], Imagavg[10];
const long FreqList[11] = {100000, 125000, 150000, 175000, 200000, 225000, 250000, 275000, 300000}; // Frequency list for scanned frequencies.
bool okvar;

MD_AD9833 AD(FSYNC); // Hardware SPI DDS1
MD_AD9833  AD2(FSYNC2);  // Hardware SPI DDS2
//MD_AD9833 AD(DATA, CLK, FSYNC); // Arbitrary SPI pins


void SpiSendAll(uint16_t data) {// this is for simultaneously writing to all SPI devices , needed to sync the DDS'es
  digitalWrite(FSYNC, LOW);
  digitalWrite(FSYNC2, LOW);
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE2));
  digitalWrite(FSYNC, LOW);
  digitalWrite(FSYNC2, LOW);
  SPI.transfer16(data);
  digitalWrite(FSYNC, HIGH);
  digitalWrite(FSYNC2, HIGH);
  SPI.endTransaction();
}


void syncclocks() {
  uint16_t CReg;
  CReg = 0;
  bitSet(CReg, 13);  // always write 2 words consecutively check dit
  bitSet(CReg, 8);  // Reset is done on a 1 to 0 transition
  SpiSendAll(CReg);
  bitClear(CReg, 8);
  SpiSendAll(CReg);
}


void setup()  {
  uint32_t  ul;
  AD.begin();
  AD2.begin();
  syncclocks();
  AD.setPhase(0, (uint16_t)900);
  pinMode(SigRDY, OUTPUT);
  pinMode(EnableAmp, OUTPUT);
  pinMode(LEDSignal, OUTPUT);
  Serial.begin(115200);
  Wire.begin(); // join i2c bus (address optional for master)
  MD_AD9833::channel_t chan;
  MD_AD9833::mode_t mode;
  chan = MD_AD9833::CHAN_0;
  AD.setFrequency(chan, 200000);  //  ul = strtoul("2000", NULL, 10); sometimes needed?
  AD2.setFrequency(chan, 200000);
  offsetter();
  mode = MD_AD9833::MODE_SINE; AD.setMode(mode);
  mode = MD_AD9833::MODE_SINE; AD2.setMode(mode);
  setgain(0);
  Realavg[0] = 0; //doekle
  digitalWrite(EnableAmp, HIGH); 
}


void offsetter(void) {
  uint32_t  ul;
  int temp;
  //digitalWrite(EnableAmp, LOW);
  MD_AD9833::channel_t chan;
  MD_AD9833::mode_t mode;
  mode = MD_AD9833::MODE_OFF; AD.setMode(mode);// niet uitzetten, geeft onvoorspelbare pieken in het signaal
  mode = MD_AD9833::MODE_OFF; AD2.setMode(mode);// niet uitzetten, geeft onvoorspelbare pieken in het signaal
  setgain(0);
  delay(100);
  temp = analogRead(SigLvl);
  delay(1);
  temp += analogRead(SigLvl);
  delay(1);
  temp += analogRead(SigLvl);
  delay(1);
  temp += analogRead(SigLvl);
  OffsetValue = (temp / 4)-1;
  OffsetReal = analogRead(SigReal) - 512;
  OffsetImag = analogRead(SigImag) - 512;
  mode = MD_AD9833::MODE_SINE; AD.setMode(mode);   // turn on DDS transmitter again
  mode = MD_AD9833::MODE_SINE; AD2.setMode(mode);   // turn on DDS transmitter again
 // digitalWrite(EnableAmp, HIGH); //enable amplifier
}


void setgain(uint8_t Gainstep) { //0.1,1,10 = 0.1=1,0.5=11 1=22 5=93
  uint8_t Potval[5];
  Potval[0] = 1; //was 1
  Potval[1] = 5;  //tusseningekomen
  Potval[2] = 11;
  Potval[3] = 32;  //was 22
  Potval[4] = 93;
  Wire.beginTransmission(adress); // transmit to device
  Wire.write(byte(0x00));            // sends instruction byte
  Wire.write(Potval[Gainstep]);             // sends potentiometer value byte
  Wire.endTransmission();     // stop transmitting
}

void selftest(){
    int Value1,Value2;
    setgain(3);
    okvar=0;
    AD.setFrequency(0, FreqList[2]);
    AD2.setFrequency(0, FreqList[2]);
    syncclocks();    //synchronize the DDS to 90 degrees phase
    digitalWrite(EnableAmp, LOW);
    delay(2);

    Value1 = analogRead(SigReal)-OffsetReal;  // meten
    Value2 = analogRead(SigImag)-OffsetImag;  // meten

//      Serial.print(Value1-512);
//      Serial.print("  ");  
//      Serial.print(Value2-512);
//      Serial.print("  ");  
      
    if ( (abs(Value1-512)<20) | (abs(Value1-512)<20)){
      okvar=1;
      }

    digitalWrite(EnableAmp, HIGH);
    delay(2);
    Value1 = analogRead(SigReal)-OffsetReal;  // meten
    Value2 = analogRead(SigImag)-OffsetImag;  // meten

    if ( ((abs(Value1-512)>50) | (abs(Value1-512)>50)) & okvar ){
      okvar=1;
      digitalWrite(LEDSignal, HIGH); 
      }
      else{okvar=0;
      digitalWrite(LEDSignal, LOW); }

//      Serial.print(Value1-512);
//      Serial.print("  ");  
//      Serial.print(Value2-512);
//      Serial.print("  ");  
//      Serial.print(okvar*100);
//      Serial.println("  ");  
}



void measure(uint8_t index) {
  int Value;
  bool RDY = 0;
  static float Average;

  // Set measurement parameters, frequency.
  AD.setFrequency(0, FreqList[index]);
  AD2.setFrequency(0, FreqList[index]);
  syncclocks();    //synchronize the DDS to 90 degrees phase
  //digitalWrite(EnableAmp, HIGH);
  setgain(Gain[index]);  ///set the input gain for the input stage
  delay(3); //wait for DDS to settle 1ms, important delay, also for the demodulator input (2ms)to settle.

// Exponential averaging, Alpha is the forgetfactor**************************************************
  Average = Realavg[index];
  Value = analogRead(SigReal);  // meten
  Real[index] = Value - OffsetReal;  //offset
  Realavg[index] = (1 - Alpha) * Average + Alpha * Real[index]; //moving exp average
  Average = Imagavg[index];
  Value = analogRead(SigImag);  // meten
  Imag[index] = Value - OffsetImag;  //offset
  Imagavg[index] = (1 - Alpha) * Average + Alpha * Imag[index];

// set the input gain level, based on the average input level of the demodulators
  if ( (abs(Realavg[index]-512)>150) | (abs(Imagavg[index]-512)>150) )
     {
      if (Gain[index]>0){
        Gain[index]--;
      }
    }
if ( (abs(Realavg[index]-512)<50) & (abs(Imagavg[index]-512)<50) )    
    {
      if (Gain[index]<4){
        Gain[index]++;
      }
    }
  Siglevel[index] = analogRead(SigLvl);
}



void loop() {
  uint8_t Temp,  i, RCVchar, CRC;
  static int16_t counter = 0,counter2=0;
  //selftest();
  //digitalWrite(EnableAmp, LOW);
  for (i = 0; i < 9; i++) {   //1=0, i<9   //doekle debug
    measure(i);  }

digitalWrite(SigRDY, HIGH);  // signal when ready with measurements
#ifndef DEBUG
  while (!Serial.available()) {} //wait for serial input, normally
#endif
  digitalWrite(SigRDY, LOW);

  if (Serial.available()) {
    RCVchar = Serial.read();
    delayMicroseconds(400);
  }  // flush the character and start with giving data
  if (RCVchar == '1') {
    digitalWrite(SigRDY, LOW);  // signal when ready with measurements
    offsetter();
  }

  //offsetcalibration every 1000 samples, maybe removed?
  counter++;
  if (counter == 1000) {
    offsetter();
    counter = 0;
  }
  counter2++;
  if (counter2 == 50) { //100 is 4 seconden
    selftest();
    counter2 = 0;
  }

#ifdef DEBUG
//*************************DEBUG**************************

  
  //  for (i = 0; i < 9; i++) {
  //    Serial.println("  ");
  //    Serial.print(Gain[i]*100);
  //    Serial.print("  ");
  //    Serial.print(Siglevel[i]*10);
  //    Serial.print("  ");
  //    Serial.print(Real[i]);
  //    Serial.print("  ");
  //    Serial.print(Imag[i]);
  //    Serial.print("  ");
  //    analogReference(INTERNAL);
  //    delay(50);
  //    analogRead(A6);
  //    Serial.print(analogRead(A6)/8);// rough voltage
  //    analogReference(DEFAULT);
  //    delay(50);
  //  }
  
//    Temp =  Gain[0] << 5 ;
//    Temp +=  0x10 & (okvar << 4) ;
//    Temp += 0x0F & ((Siglevel[0]-OffsetValue));
//    Serial.println(Temp,BIN);

//  Serial.print(okvar*100);
//  Serial.print("  ");  
//  Serial.print(Imag[0]);
//  Serial.print("  ");
//  Serial.print(Imag[1]);
//  Serial.print("  ");
//  Serial.print(Imag[2]);
//  Serial.print("  ");
//  Serial.print(Imag[3]);
//  Serial.print("  ");
//  Serial.print(Imag[4]);
//  Serial.print("  ");
//  Serial.print(Imag[5]);
//  Serial.print("  ");
//  Serial.print(Imag[6]);
//  Serial.print("  ");
//  Serial.print(Imag[7]);
//  Serial.print("  ");
//  Serial.println(Imag[8]);


 
//  Serial.print(Real[0]);
//  Serial.print("  ");
//  Serial.print(Real[1]);
//  Serial.print("  ");
//  Serial.print(Real[2]);
//  Serial.print("  ");
//  Serial.print(Real[3]);
//  Serial.print("  ");
//  Serial.print(Real[4]);
//  Serial.print("  ");
//  Serial.print(Real[5]);
//  Serial.print("  ");
//  Serial.print(Real[6]);
//  Serial.print("  ");
//  Serial.print(Real[7]);
//  Serial.print("  ");
//  Serial.println(Real[8]);

//********************END OF DEBUG**************************
#else
//********************END OF DEBUG**************************

  
  CRC = 0;
  Serial.write(0xf0); //ID bytes
  Serial.write(0xe0);  //ID bytes
  for (i = 0; i < 9; i++) {
    Temp =  Gain[0] << 5 ;
    Temp +=  0x10 & (okvar << 4) ;
    Temp += 0x0F & ((Siglevel[0]-OffsetValue));
    CRC += Temp;
    Serial.write(Temp);
    Temp = Real[i];
    CRC += Temp;
    Serial.write(Temp);
    Temp = Real[i] >> 8;
    CRC += Temp;
    Serial.write(Temp);
    Temp = Imag[i];
    CRC += Temp;
    Serial.write(Temp);
    Temp = Imag[i] >> 8;
    CRC += Temp;
    Serial.write(Temp);
  }
  Serial.write(CRC);
#endif  //END DEBUG STATeMENT

  while (Serial.available()) {
    RCVchar = Serial.read(); //flush the buffer, in case an extra character was sent. disabling multiplle sends.
  }
}
