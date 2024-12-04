# DE2_MCV_environment-for-plants

## Topic

**Measurement/Control/Visualization of the environment for tropical plants**
The goal of the project would be to create a system that measures key environmental parameters (such as temperature, humidity, light levels, soil moisture) for tropical plants. This system should also allow the user to control or adjust environmental conditions and visualize the data.

Inspiration: [Climate Chamber System](https://projecthub.arduino.cc/ms_peach/climate-chamber-system-c545de)

### Team members

* František Hecl (responsible for programming and testing)
* Filip Rákos (responsible for programming and testing)
* Marek Smolinský (responsible for leading our team)
* Kamil Soukup (responsible for programming and testing)

## Hardware description

Describe your implementation and include block or circuit diagram(s).
![image](https://github.com/user-attachments/assets/920e9da3-cd24-4cf3-a0df-5ade5083d01e)

In this project, we used following resources:
* Source of 12 VDC
* Light emiting diode (LED)
* NPN transistor
* I2C OLED display 1.3"
* I2C temperature and moisture sensor
* Capacitive soil moisture sensor
* Fan
* Photocell
* Resistor 200 Ohm


## Software description

Put flowchats/state diagrams of your algorithm(s) and direct links to source files in PlatformIO `src` or `lib` folders. Present the libraries you used in the project.

![Snímek obrazovky 2024-12-04 175832](https://github.com/user-attachments/assets/0a1538ac-2a28-4376-aba4-c1dffa8382f1)
![Snímek obrazovky 2024-12-04 180738](https://github.com/user-attachments/assets/c8d589ad-aca9-4af2-b3a3-182ff79c5a48)


## Instructions and photos

Describe how to use the application. Add photos or videos of your application.

## References and tools

1. [ADC from GitHub of Tomas Fryza](https://github.com/tomas-fryza/avr-course/tree/master/archive/labx-adc)
2. [Datasheet of ATmega328p](https://www.microchip.com/en-us/product/ATmega328p)
3. [OLED with humidity sensor from GitHub of Tomas Fryza](https://github.com/tomas-fryza/avr-course/tree/master/solutions/lab6-i2c-sensor-oled)
4. [Cirkit Designer IDE](https://app.cirkitdesigner.com/)

