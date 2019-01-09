LoRa STM32 Source Code
===============
Source code for Dragino LoRa module base on STM32 Chip. 
Support Products: [LoRaST](http://www.dragino.com/products/lora/item/127-lora-st.html), [LSN50](http://www.dragino.com/products/lora/item/128-lsn50.html).

How To Use Source Code: [For LSN50](http://wiki.dragino.com/index.php?title=LoRa_Sensor_Node-LSN50#Program_LSN50)

Switch between LSN50 Code and LoRa ST default code:

In file Projects/Multi/Applications/LoRa/DRAGINO-LRWAN(AT)/inc/hw_conf.h

    #define LoRa_Sensor_Node /*LSN50*/      --- For LSN50
    //#define AT_Data_Send /*AT+SEND or AT+SENDB*/    --- For LoRa ST
	
	





