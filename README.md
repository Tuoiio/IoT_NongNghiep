•	Microcontroller: STM32F103C8, ESP01

•	Hardware: LCD20X4, DS1307, DHT22, Module Relay, Moisture Sensor, Button

•	Peripherals: Timer, I/O

•	Protocol: I2C, OneWire, UART

•	Technical Skill: C

•	Enviroment development: KeilC, ArduinoIDE

•	Description:STM32 reads real-time data from DS1307 and temperature humidity from DHT22. Display parameter via 20x4 LCD screen. There are 4 buttons: Mode, Ok, Up, Down. There are 2 mode: Auto and Manual. 
Mode manual turn on and off the water pump, ventilation fan and heating lamp.
Mode automatic will rely on environmental parameters to control devices. Pump water  will depend on time and soil moisture. Ventilation fans rely on low humidity. Heat lamps rely on temperature. It can change time, date, warning high temperature and humidity, watering time, soil moisture turns the pump on and off.
