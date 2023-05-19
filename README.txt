
Author: Robert Sutherland
Copyright: all rights resevred
This is the firmware code for the SkeltonicsProject.

These are the following branches.
Each branch has a coresponding folder that contains its code.
------------------------------------
BT_7ChanADC_MVP

This is the final version of the code. This contains the script for both the UART and the Bluetooth communication protocols. Use this script only when running the skeltonics device.


3chan_ADC_DMA:

Early testing script to esablish the functionallity of ADCs and interupts and DMA. Do not use as a final version.


UART_TX_RX_Test

Early testing script to establish the functionaly of UART communcication. This included TX and RX functions. RX interupt and DMA.

MVP_UART_5ChanADC
First working version of device. However, this is an earlier prototype and should not be used as it lacks both channel filtering and Bluetooth capabilities. 


------------------------------------
OTHER FOLDERS:

MVP_UART_5ChanADC_SERIALwRESET_4_24: 
Contains a backup ziped folder. Use cautiously.

ARUDINO_LEO_ATCOMAND:

Contians a arudino project file that can be used to configure AT commands on new Bluetooth radio HC-05,HC-06. This is tricky. Consult the web for details. Baud rate for TX line is 38400. 
