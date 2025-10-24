#include "FineTuneBMX160.h"

using namespace FineTuneBMX160;
BMX160 bmx160;

void setup()
{
    Serial.begin(115200);
    while (!Serial);


    bmx160.begin();
}

void loop()
{
    float temperature;
    bmx160.getTemp(temperature);

    Serial.print("Temp: ");Serial.print(temperature);Serial.println(" ºC");
    
    delay(1000);
}
