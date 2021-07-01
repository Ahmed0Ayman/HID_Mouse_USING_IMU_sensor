# HID_Mouse_USING_IMU_sensor


Hi everyone as far as we know the importance of USB protocol so first I’ll give you an introduction to USB protocol then I’ll move on to discuss this project so to introduce this importance of USB we have two point of view first from -> user point of view for USB is 
-	Hot plugged and play 
-	Not require any setting in most cases 
-	Inexpensive and versatile (support many different types of devise )
-	Still developed by USB organization util we now have USB 3.2 with speed up to 20Gbps
-	Standard connector for all devices don’t confused between USB protocol and TYPE C -> TYPE C this is only the shape of connector not more than this so TYPE C is a way to make standard as usb organization aimed to make usb id standard protocol for all devices connected with pc here also they want to make standard connector for all devices in the world . host and devices same connector , also can used with any other protocols and because it has multiple connections used with power delivery . 
-	Back word compatibility 
-	Run time address assignation up to 127 device and 0 is received as default address before enumeration process
-	Each device can support multiple function like stm32 debugger  

But with these all features we also have some of drawbacks  like 
Limitation is distance and master centric only 127 device at the same network  - not use broadcast messages 
USB is designed to be a standard protocol for all devices connected with PC so this make this protocol hieratical and more complicated but for your information USB support any type of communication requirements ex if you need stream audio or video you can use isochronous transfer 
Or if you need to transfer big data you can use bulk transfer and so one 
Data transfer in USB protocol is based on pipe method where this pipe from the device end with endpoint and at the host connect with the device driver  
Each transaction in USB is consist of 3 packets ( token – data – ack/handshake ) 
 And also usb protocol has 4 types of transactions ( control – bulk – isochronous – interrupt ) of course the usb device must at least support control packet 
In enumeration process is this process the device learn about device capability and also choose which configuration descriptor will be run at the time you need to know only one configuration run at a time but you can define many functionality for the same configuration  descriptor 
Control transfer consists of 3 stage ( setup – data – status ) talking about USB can’t be coverage in one paper this  huge protocol need a lot and a lot of words to tell you about all this protocol functionality so if you interested with USB we can search on internet and you can fend a lot of useful documentation is prefer “USB Complete The Developer's Guide 4th Ed” as start point 

If we returned back to our project here is usb stm32 usb as a device with HID class 
This class contain a lot of device like mouse and keyboard and game controller and so on this type of devices use interrupt endpoint 
I also use IMU sensor (MPU6050 ) this sensor use to measure the orientation around the axis 
This is  a brief about peripheral that used in this project ( ADC – DMA -TIM8 – I2C – USB_FS – EXTI) I uploaded all project folder to make it easy to you not only code but also the configurations 
You can also take a closer look for this project in my YouTube channel  to see this project of course I made source code with heavy comments to make you know what’s going on in this project 
In this project I used CMSIS – v2 operating system to make it easy for future upgrade 
If you have any questions feel free to ask .thank you for your time 


https://youtu.be/UinZJJhDKbc
