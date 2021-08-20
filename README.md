# 2021BEV 
2021 Bachelor's degree Graduation Project, Battery Electric Vehicle  
Department of Electrical and Conputer Engineering, Seoul National University  

- Advisor : [Professor Seung-ki Sul](http://eepel.snu.ac.kr/wordpress/professor/)

## Demo Video

### [Test driving 1 : Seoul National University Campus Road (youtube video)](https://youtu.be/YFAJCFfAURw)
- Path length : 4.06Km
- Maximum slope : 7%
- Maximum speed : 30.5km/h

### [Test driving 2 : Cooling Performance test (youtube video)](https://youtu.be/0JFbmtatzrk)
- place : Seoul National University Electric Power Research Institute Parking Lot
- settled down at 44 celcius degree
- continuous maximum q-axis current (30A)

## What I made

### Inverter
- 4 layer PCB artwork (1oz 2oz 2oz 1oz) 
- Used IPM([NFAL5065L4B](https://kr.mouser.com/datasheet/2/308/1/NFAL5065L4B_D-2317681.pdf)) as the power IGBT switch
- Electrically isolated control board area from the High Power Area including IPM and DC link

### DSP - controller
- TMS320F2839D Launchpad
- Built the q-axis current regulator(PI controller with feedforward) from the scratch including resgister setting(ePWM, eCAP, GPIO, Clock, Timer)
- Implemented rotor position and speed observer to estimate right position of rotor for DQ transformation and feedforward control

## Future work
- Robust control algorithm to prevent sudden current peak from the disturbance.   
  Since there is no mechanical damper between ground and motor wheel, Bumpy road surface directly affects the controller's performance
- Hall sensor type curernt sensor removal. because the IPM already provides extra ports to measure the phase current using small register.

## Photos
- [Bicycle Photo](https://photos.app.goo.gl/71CFiTBS6PbSPb559)
- [Inverter Photo](https://photos.app.goo.gl/JQpmmjE8fstSpoiK7)
