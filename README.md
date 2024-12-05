# DE2_MCV_environment-for-plants

## Topic

**Measurement/Control/Visualization of the environment for tropical plants**
The goal of the project would be to create a system that measures key environmental parameters (such as temperature, humidity, light levels, soil moisture) for tropical plants. This system should also allow the user to control or adjust environmental conditions and visualize the data.

Inspiration: [Climate Chamber System](https://projecthub.arduino.cc/ms_peach/climate-chamber-system-c545de)

### Team members

* František Hecl (responsible for programming and testing)
* Filip Rákos (responsible for programming and documentation)
* Marek Smolinský (responsible for programing and state diagrams)
* Kamil Soukup (responsible for programming, testing and GitHub)

## Hardware description

Describe your implementation and include block or circuit diagram(s).
![image](https://github.com/user-attachments/assets/920e9da3-cd24-4cf3-a0df-5ade5083d01e)

In this project, we used following resources:
* Source of 12 VDC
* Light emiting diode(s) (LED)
* NPN transistor
* I2C OLED display 1.3"
* I2C temperature and moisture sensor
* Capacitive soil moisture sensor
* Fan
* Light-dependent resistor (fotorezistor)
* Resistor 200 Ohm

Light-dependent resistor regulates LEDs using PWN regulated by ADC of the Arduino depending on the lighting.\
On the I2C bus, there are OLED display and temperature / moisture sensor. On the OLED are displayed information about temperature, lighting, air moisture and soil moisture. Capacitive soil moisture sensor is another AD input (A1).\
NPN transistor is used as a switch to turn on / off fan dependending on moisture and temperature inside the terrarium. 12 VDC source is to power the fan. 

## Software description

Put flowchats/state diagrams of your algorithm(s) and direct links to source files in PlatformIO `src` or `lib` folders. Present the libraries you used in the project.

Applications state diagram
![StavovyDiagram drawio (3)](https://github.com/user-attachments/assets/37ba01d4-3370-4fd0-b595-e0f5e3dbbd7d)



## Instructions and photos

Describe how to use the application. Add photos or videos of your application.

## References and tools

1. [ADC from GitHub of Tomas Fryza](https://github.com/tomas-fryza/avr-course/tree/master/archive/labx-adc)
2. [Datasheet of ATmega328p](https://www.microchip.com/en-us/product/ATmega328p)
3. [OLED with humidity sensor from GitHub of Tomas Fryza](https://github.com/tomas-fryza/avr-course/tree/master/solutions/lab6-i2c-sensor-oled)
4. [Cirkit Designer IDE](https://app.cirkitdesigner.com/)
5. [Draw.io](https://app.diagrams.net)

