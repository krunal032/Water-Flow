#include <SoftwareSerial.h>
/* Create object named SIM900 of the class SoftwareSerial */
SoftwareSerial SIM900(7, 8);

char msg;
int flowPin1 = 2;    //This is the input pin on the Arduino
int flowPin2 = 3;    //This is the input pin on the Arduino
double flowRate1;    //This is the value we intend to calculate. 
double flowRate2;    //This is the value we intend to calculate. 
volatile int count1; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.  
volatile int count2; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.  
double diff;    //This is the value of difference
double output=0;//for difference
double output1=0;//for PH value
int msgStatus=0;
/*#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0; */

String url = String("\"AT+HTTPPARA=\"URL\",\"api.thingspeak.com/update?api_key=3N3IFGCE5I4KFTAV&field1=");
String url1 = String("&field2=");
String url2 = String("\"");

void openConnection()
{
  Serial.println("HTTP get method :");
  Serial.print("AT\\r\\n");
  SIM900.println("AT"); /* Check Communication */
  delay(5000);
  ShowSerialData(); /* Print response on the serial monitor */
  delay(5000);
  /* Configure bearer profile 1 */
  Serial.print("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\\r\\n");    
  SIM900.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");  /* Connection type GPRS */
  delay(5000);
  ShowSerialData();
  delay(5000);
  Serial.print("AT+SAPBR=3,1,\"APN\",\"www\"\\r\\n");  
  SIM900.println("AT+SAPBR=3,1,\"APN\",\"www\"");  /* APN of the provider */
  delay(5000);
  ShowSerialData();
  delay(5000);
  Serial.print("AT+SAPBR=1,1\\r\\n");
  SIM900.println("AT+SAPBR=1,1"); /* Open GPRS context */
  delay(5000);
  ShowSerialData();
  delay(5000);
  Serial.print("AT+SAPBR=2,1\\r\\n");
  SIM900.println("AT+SAPBR=2,1"); /* Query the GPRS context */
  delay(5000);
  ShowSerialData();
  delay(5000);
  Serial.print("AT+HTTPINIT\\r\\n");
  SIM900.println("AT+HTTPINIT"); /* Initialize HTTP service */
  delay(5000); 
  ShowSerialData();
  delay(5000);
  Serial.print("AT+HTTPPARA=\"CID\",1\\r\\n");
  SIM900.println("AT+HTTPPARA=\"CID\",1");  /* Set parameters for HTTP session */
  delay(5000);
  ShowSerialData();
  delay(15000); 
}

void closeConnection()
{
  Serial.print("AT+HTTPTERM\\r\\n");  
  SIM900.println("AT+HTTPTERM");  /* Terminate HTTP service */
  delay(5000);
  ShowSerialData();
  delay(5000);
  Serial.print("AT+SAPBR=0,1\\r\\n");
  SIM900.println("AT+SAPBR=0,1"); /* Close GPRS context */
  delay(5000);
  ShowSerialData();
  delay(5000);
}


//For Sending SMS
int SendMessage()
{
  Serial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  SIM900.println("AT+CMGS=\"+919033440365\"\r"); // Replace x with mobile number
  delay(1000);
  SIM900.println("Leakage");// The SMS text you want to send
  delay(100);
   SIM900.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
  return 1;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(flowPin1, INPUT);           //Sets the pin as an input
  pinMode(flowPin2, INPUT);           //Sets the pin as an input
  attachInterrupt(digitalPinToInterrupt(flowPin1), Flow1, RISING);  //Configures interrupt 0 (pin 2 on the Arduino Uno) to run the function "Flow"  
  attachInterrupt(digitalPinToInterrupt(flowPin2), Flow2, RISING);  //Configures interrupt 0 (pin 2 on the Arduino Uno) to run the function "Flow"  
  Serial.begin(9600);  //Start Serial

  openConnection();
}

double Flowsensor()
{

// put your main code here, to run repeatedly:  
  count1 = 0;      // Reset the counter so we start counting from 0 again
  count2 = 0;      // Reset the counter so we start counting from 0 again
  interrupts();   //Enables interrupts on the Arduino
  delay (1000);   //Wait 1 second 
  noInterrupts(); //Disable the interrupts on the Arduino
  
  //Start the math
  flowRate1 = (count1 * 2.25);        //Take counted pulses in the last second and multiply by 2.25mL 
  flowRate1 = flowRate1 * 60;         //Convert seconds to minutes, giving you mL / Minute
  flowRate1 = flowRate1 / 1000;       //Convert mL to Liters, giving you Liters / Minute

 
  
  //Start the math
  flowRate2 = (count2 * 2.25);        //Take counted pulses in the last second and multiply by 2.25mL 
  flowRate2 = flowRate2 * 60;         //Convert seconds to minutes, giving you mL / Minute
  flowRate2 = flowRate2 / 1000;       //Convert mL to Liters, giving you Liters / Minute

  diff=flowRate1-flowRate2;
  Serial.println("\nSensor-1");
  Serial.println(flowRate1);         //Print the variable flowRate to Serial
  Serial.println("Sensor-2");
  Serial.println(flowRate2);         //Print the variable flowRate to Serial
 // Serial.println("Difference ");
  //Serial.println(diff);         //Print the variable flowRate to Serial
  return diff;
  
}
void Flow1()
{
   count1++; //Every time this function is called, increment "count" by 1
}

void Flow2()
{
   count2++; //Every time this function is called, increment "count" by 1
}
/*double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

double ph()
{
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
  //Serial.print("Voltage:");
    //    Serial.print(voltage,2);
      //  Serial.print("    pH value: ");
  //Serial.println(pHValue,2);
    //    digitalWrite(LED,digitalRead(LED)^1);
        printTime=millis();
    
  }
  return pHValue;
}


*/
void loop() {

  String urlfinal= String(url)+int(flowRate1)+String(url1)+String(flowRate2)+String(url2);
  delay(5000);

    output=Flowsensor();
  Serial.println("Difference :: ");
 Serial.print(output, DEC);//flow sensor difference
 Serial.println("");

  Serial.print("AT+HTTPPARA=\"URL\",\"api.thingspeak.com/update?api_key=3N3IFGCE5I4KFTAV&field1=\"\\r\\n");
  SIM900.println(urlfinal);  /* Set parameters for HTTP session */
  delay(3000);
  ShowSerialData();
  delay(3000);
  Serial.print("AT+HTTPACTION=0\\r\\n");
  SIM900.println("AT+HTTPACTION=0");  /* Start GET session */
  delay(3000);
  ShowSerialData();
  delay(1000);
  Serial.print("AT+HTTPREAD\\r\\n");
  SIM900.println("AT+HTTPREAD");  /* Read data from HTTP server */
  delay(1000);
  ShowSerialData();
  delay(1000);

  delay(5000);

 /*output1=ph();
  Serial.println("PH Value :: ");
 Serial.print(output1, DEC);//pH value
 Serial.println("");
 */
 Serial.println(msgStatus);
  
  if(output>100 && msgStatus == 0)
  {
    msgStatus = SendMessage();
  }

  /* static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
  Serial.print("Voltage:");
        Serial.print(voltage,2);
        Serial.print("    pH value: ");
  Serial.println(pHValue,2);
    //    digitalWrite(LED,digitalRead(LED)^1);
        printTime=millis();
  }*/
  
}

void ShowSerialData()
{
  while(SIM900.available()!=0)  /* If data is available on serial port */
  Serial.write(char (SIM900.read())); /* Print character received on to the serial monitor */
}


