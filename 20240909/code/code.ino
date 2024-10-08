#define TSL1401_CLK 22
#define TSL1401_SI 23
#define NPIXELS 12

byte Pixel[NPIXELS];    // Field for measured values <0-255>
int LineSensor_Data[NPIXELS];

#define FASTADC 1
// defines for setting and clearing register bits

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup()
{
    Serial.begin(9600); 
    Serial.println("TSL1401 Camera Initialized");
    digitalWrite(TSL1401_CLK, LOW);
    digitalWrite(TSL1401_SI, LOW);

    pinMode(TSL1401_CLK, OUTPUT);
    pinMode(TSL1401_SI, OUTPUT);
    
    #if FASTADC
      // set prescale to 16
      sbi(ADCSRA, ADPS2);
      cbi(ADCSRA, ADPS1);
      cbi(ADCSRA, ADPS0);
    #endif
}

void read_TSL1401_camera(void)
{
  digitalWrite(TSL1401_SI, HIGH);
  digitalWrite(TSL1401_CLK, HIGH);
  delayMicroseconds(1);
  digitalWrite(TSL1401_CLK, LOW);
  digitalWrite(TSL1401_SI, LOW);
  delayMicroseconds(1);

  for(int i =0; i< 128; i++)
  {
    digitalWrite(TSL1401_CLK, HIGH);
    delayMicroseconds(1);
    Pixel[i] = analogRead(A0);
    digitalWrite(TSL1401_CLK, LOW);
    delayMicroseconds(1);
  }
}

void send_camera_data_serial()
{
  for(int i =0; i < NPIXELS; i++)
  {
    Serial.println(Pixel[i]);
  }
  
  for(int i= 0; i < 64; i++)
  {
      Serial.println(0);
  }
}

void loop() 
{
  read_TSL1401_camera();
  send_camera_data_serial();
  delay(100);
}
