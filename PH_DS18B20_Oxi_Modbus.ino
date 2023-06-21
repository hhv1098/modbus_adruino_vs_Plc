/*!
Du an Dam tom: Do Oxi, Do PH, Do nhiet do
Design: Nguyen Duc Dien 15/3/2022
 */

#include "DFRobot_PH.h" 
#include <EEPROM.h>

#include <OneWire.h>
#include <DallasTemperature.h>


#include <SPI.h>
#include <Ethernet.h>
#include <Modbus.h>
#include <ModbusIP.h>

#define PH_PIN A1
float voltage,phValue,temperature,DO_Value;
uint16_t phValue_Send,temperature_Send,DO_Value_Send;
DFRobot_PH ph;
#define ONE_WIRE_BUS 9
//Thiết đặt thư viện onewire
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
//Khai bao Do oxi
#define DO_PIN A2
#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution
//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 1
#define READ_TEMP (26) //Current water temperature ℃, Or temperature sensor function
//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (2026) //mv
#define CAL1_T (26)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (820) //mv
#define CAL2_T (20)   //℃
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};
uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

//khai bao modbus
//Modbus Registers Offsets (0-9999)
const int SENSOR_HREG1 = 0; 
const int SENSOR_HREG2 = 1; 
const int SENSOR_HREG3 = 2; 
//ModbusIP object
ModbusIP mb;

void setup()
{
    Serial.begin(115200);  
    // The media access control (ethernet hardware) address for the shield
    byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  
    // The IP address for the shield
    byte ip[] = { 192, 168, 0, 120 };   
    //Config Modbus IP 
    mb.config(mac, ip);

    // Add SENSOR_IREG register - Use addIreg() for analog Inputs
    mb.addHreg(SENSOR_HREG1);
    mb.addHreg(SENSOR_HREG2);
    mb.addHreg(SENSOR_HREG3);
    
    ph.begin();
    sensors.begin();
    
}

void loop()
{
    static unsigned long timepoint = millis();
        if(millis()-timepoint>1000U){                  //time interval: 1s
        timepoint = millis();
        //temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
        mb.task();
        sensors.requestTemperatures();
        temperature=sensors.getTempCByIndex(0);
        voltage = analogRead(PH_PIN)/1024.0*5000;  // read the voltage
        phValue = ph.readPH(voltage,temperature);  // convert voltage to pH with temperature compensation
        //Doc oxi
        Temperaturet = (uint8_t)temperature;
        ADC_Raw = analogRead(DO_PIN);
        ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
        DO_Value=readDO(ADC_Voltage, Temperaturet)/1000;
        phValue_Send=phValue*100;
        temperature_Send=temperature*100;
        DO_Value_Send=DO_Value*100;;
       mb.Hreg(SENSOR_HREG1,temperature_Send);
       mb.Hreg(SENSOR_HREG2,phValue_Send);
       mb.Hreg(SENSOR_HREG3,DO_Value_Send); 
       Serial.print("temperature: ");
       Serial.print(temperature,1);
       Serial.print("^C  pH: ");
       Serial.print(phValue,2);
       //Serial.println("  DO:\t" + String(readDO(ADC_Voltage, Temperaturet)/1000) + "\t");
       Serial.print("   Temperaturet:\t" + String(Temperaturet));
       Serial.print("   ADC RAW:\t" + String(ADC_Raw) );
       Serial.print("  ADCVoltage:\t" + String(ADC_Voltage) );
       Serial.println("   DO:\t" + String(readDO(ADC_Voltage, Temperaturet)/1000));
    }
    ph.calibration(voltage,temperature);           // calibration process by Serail CMD
}
