/*
The Protector firmware  (MUST BE COMPILED AND FLASHED ON PLATFORMIO)

@Author: Christopher Mendez
@Date: 02/02/2023 (mm/dd/yy)

@Brief:
This code runs and Edge Impulse model for fall detection, send temperature, humidity, pressure, IAQ, CO2, activity
and inference result data through BLE to be viewed and recorded on "The Protector" mobile app.

@Note:
The Edge Impulse model library of your project must be in the ./lib folder.

@Tutorial:
Arduino Projects Hub link: 

*/

/*
BLE comunication protocol:

1 -- temp, hum, press
2 -- Iaq, Co2, Activity(256-16384)
3 -- Inference(still,walking,falling)

Example of data to be sent (one line at a time):

1,25.32,45.2,994.2
2,24,500,256
3,walking

*/

/* Activities -- activ.value data:

256 = Still activity started.
512 = Walking activity started.
1024 = Running activity started.
2048 = Bicycle activity started.
4096 = Vehicle activity started.
8192 = Tilting activity started.
16384 = Vehicle still.

*/

/*
Espected inference results:

1 -- still
2 -- falling
3 -- walking

*/

#include "Nicla_System.h"              // Nicla library to be able to enable battery charging and LED control
#include "Arduino_BHY2.h"              // Arduino library to read data from every built-in sensor of the Nicla Sense ME<the_protector_inferencing.h> // Edge Impulse Arduino Library of your trained model


// 1. Declaración de sensores inerciales y de actividad
SensorXYZ accel(SENSOR_ID_ACC);         // Acelerómetro[cite: 2, 5]
SensorXYZ gyro(SENSOR_ID_GYRO);         // Giroscopio[cite: 2, 5]
SensorActivity activ(SENSOR_ID_AR);     // Reconocimiento de actividad integrado

void setup() {
  Serial.begin(115200);
  while (!Serial); // Esperar a que el puerto serie se abra

  Serial.println("Inicializando BHI260AP (Fusión y Reconocimiento de Actividad)...");

  // 2. Inicialización del sistema principal y los sensores
  BHY2.begin();    // Iniciar sistema BHY2
  accel.begin();   // Habilitar acelerómetro[cite: 2, 5]
  gyro.begin();    // Habilitar giroscopio[cite: 2, 5]
  activ.begin();   // Habilitar reconocimiento de actividad
}

void loop() {
  static auto printTime = millis(); 

  // 3. Actualización continua del estado del sensor
  BHY2.update();   // Obligatorio para procesar nueva información

  // Imprimir datos cada 500 milisegundos
  if (millis() - printTime >= 500) {
    printTime = millis();
    
    // 4. Extracción de valores crudos de los 6 ejes
    float ax = accel.x(); //[cite: 2, 5]
    float ay = accel.y(); //[cite: 2, 5]
    float az = accel.z(); //[cite: 2, 5]
    
    float gx = gyro.x();  //[cite: 2, 5]
    float gy = gyro.y();  //[cite: 2, 5]
    float gz = gyro.z();  //[cite: 2, 5]

    // 5. Extracción de la actividad reconocida
    // Devuelve un número (como 256 o 512) dependiendo de la actividad detectada[cite: 5]
    uint16_t actividadID = activ.value();

    // Visualización en el Monitor Serie
    Serial.print("Acel(X,Y,Z): ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az); 
    
    Serial.print(" | Giro(X,Y,Z): ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.print(gz);

    Serial.print(" | Actividad: ");
    
    // Evaluación del ID de Actividad e impresión del nombre
    switch (actividadID) {
      case 256:
        Serial.println("Estar quieto (Still activity started)"); //[cite: 4]
        break;
      case 512:
        Serial.println("Caminando (Walking activity started)"); //[cite: 4]
        break;
      case 1024:
        Serial.println("Corriendo (Running activity started)"); //[cite: 4]
        break;
      case 2048:
        Serial.println("Bicicleta (Bicycle activity started)"); //[cite: 4]
        break;
      case 4096:
        Serial.println("Vehiculo en movimiento (Vehicle activity started)"); //[cite: 4]
        break;
      case 8192:
        Serial.println("Inclinando (Tilting activity started)"); //[cite: 4]
        break;
      case 16384:
        Serial.println("Vehiculo detenido (Vehicle still)"); //[cite: 4]
        break;
      case 0:
        Serial.println("Buscando actividad..."); 
        break;
      default:
        Serial.print("Desconocida (ID: ");
        Serial.print(actividadID);
        Serial.println(")");
        break;
    }
  }
}