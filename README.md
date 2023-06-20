# jkbmsInterface
RS485 ttl interface for JKBMS


This is an extract of the functions etc from the ESP home repository to make it run standalone, and its rough
use at your own peril :P 

Just include the header, create an object "JKBMS jkbms" and call start. If all goes well you will have data
in the various variables. I didnt change any of the names so they are basically the same as the ESP home sensors
created by the original project. 

I've only used this with cpp, even the main program as a cpp. im sure there is a way to include it with c but 
I havent tried
