# Operating Instructions
## Startup Procedure
Assuming the code is loaded onto the Teensy already (which is usually is, but if not see the section below), the startup procedure is as follows:
1. Connect the input power cables (the fat longer black/red wires with power-pole connectors - both wires are on the same side of the board) to the fuel cell.
2. Connect the output power cables (the fat shorter black/red wires with power-pole connectors - the black wire is on the bottom of the board and the red on top) to the load (i.e. a resistor, diode array, supercap array, and/or motor controller)
3. Turn on the flow meter
4. Make sure all the hydrogen tubes are connected as you want (i.e. regulator - flow meter - supply valve) and set the pressure on the regulator to ~7psi.
5. If there is a dangling blue wire coming from under the converter board, use the alligator clips from a power supply to give the blue wire 15-20V by connecting the blue wire to the power supply red and by clipping the power supply black to the fat black solder joint from step 1.  When you first connect/provide power, some random valves will flicker - this is normal.
6. Connect the Teensy to either your computer or to a power source via USB.
7. The fuel cell should now be running.  Opening a serial monitor on your computer should enable you to read the power stats coming from the fuel cell (columns 2-4).  Make sure the voltage is ~13-18V.  Check the operating point against existing IV curves such as the ones below to ensure healthy FC operation.

## Shutdown Procedure
The process of shutting down the FC is much simpler - simply disconnect the Teensy from your computer and disconnect the power supply.  It is preferable to, right before shutting down, do a quick purge by disconnecting and reconnecting the Teensy (it should purge for 3 seconds on startup).  After disconnecting all electronics, turn off the hydrogen and disconnect the tube between the FC and the supply valve.
## Arduino Code
The “Controller” folder currently contains the code uploaded to the Arduino for controlling the fuel cell and hybrid power system.  As of now, the most updated code is “constPowerIn”, but I (Gerry) will be restructuring this shortly and will likely be renaming everything.  Either way, “constPowerIn” is capable of reading all stats and controlling the fuel cell.  It does not yet have short circuit capabilities, nor is the closed loop control actually implemented yet.  It also is only capable of sending information out, not receiving information/commands.  The data sent to the Serial Monitor is of separated into 10 columns:
* Time
* I/V/P from the FC
* I/V/P out to the load
* Converter Duty Cycle	
* Converter Efficiency
* FC temperature
You need to download “TeensyLoader” to upload code to the Teensy via the Arduino interface.
