#define TSL1401_CLK 22 //  test pin
#define TSL1401_SI 23 //  test pin
#define NPIXELS 128

byte Pixel[NPIXELS];
int LineSensor_Data[NPIXELS];
byte threshold_data[NPIXELS];

#define FATASDC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void read_TSL1401_camera(void)
{
  digitalWrite(TSL1401_SI, HIGH);
  digitalWrite(TSL1401_CLK, HIGH);
  delayMicroseconds(1);
  digitalWrite(TSL1401_SI, LOW);
  digitalWrite(TSL1401_CLK, LOW);
  delayMicroseconds(1);

  for(int i =0; i< NPIXELS; i++)
  {
    Pixel[i] = analogRead(A0);
    digitalWrite(TSL1401_CLK, HIGH);
    delayMicroseconds(1);
    digitalWrite(TSL1401_CLK, LOW);
    delayMicroseconds(1);
  }
}

void send_camera_data_serial(void)
{
    for(int i=0;i<128;i++)
  {
    Serial.print(Pixel[i]);        // 첫 번째 값: Pixel 값 출력
    Serial.print(",");            // 탭으로 구분
    Serial.println(threshold_data[i]); // 두 번째 값: threshold_data 값 출력
  }
  for(int i = 0; i< 64; i++) {
    Serial.println(0);
  }
}

void line_threshold(int threshold){
  for (int i = 0; i < NPIXELS; i++){
    if (Pixel[i] >= threshold){
      threshold_data[i] = 0;
    } else{
      threshold_data[i] = 255;
    }
  }
}

void setup() 
{
  // put your setup code here, to run once:
  pinMode(TSL1401_CLK, OUTPUT);
  pinMode(TSL1401_SI, OUTPUT);
  digitalWrite(TSL1401_CLK, LOW);
  digitalWrite(TSL1401_SI, LOW);
   #if FASTADC
   sbi(ADCSPA,ADPS2);
   cbi(ADCSPA,ADPS1);
   cbi(ADCSPA,ADPS0);
   #endif

   Serial.begin(115200);
   Serial.println("Camera, threshold");
}

void loop() 
{
  // put your main code here, to run repeatedly:
  read_TSL1401_camera();
  line_threshold(128);
  send_camera_data_serial();
  delay(100);
}
