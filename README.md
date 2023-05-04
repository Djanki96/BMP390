# README - BMP390 Sensor auslesen mit ESP32

Dieses Programm liest Druckdaten von einem BMP390-Sensor mit I2C und gibt sie auf der Konsole aus. Es wurde für die Verwendung auf einem ESP32-Board entwickelt.
# RBenötigte Hardware

ESP32-Board
BMP390-Sensor


# Abhängigkeiten

freertos/FreeRTOS.h
freertos/task.h
driver/i2c.h
stdio.h
sdkconfig.h

# Anschlüsse

Der SCL-Pin des BMP390-Sensors muss an GPIO22 und der SDA-Pin an GPIO21 der ESP32-Platine angeschlossen werden.
# Programmablauf

Das Programm liest kontinuierlich Druckdaten vom angeschlossenen BMP390-Sensor. Die Sensorregister werden entsprechend gesetzt und die Daten über I2C ausgelesen. Die Rohdaten werden dann in hPa-Einheiten umgerechnet und auf der Konsole ausgegeben.
# Anpassungsmöglichkeiten

Die I2C-Taktfrequenz kann durch Ändern der I2C_MASTER_FREQ_HZ-Konstante angepasst werden.
Die I2C-Adresse des BMP390-Sensors kann durch Ändern der BMP390_SENSOR_ADDR-Konstante angepasst werden.