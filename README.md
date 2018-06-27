# OpenCTDwithConduino
OpenCTD with a low cost Conduino conductivity sensor

*********************************************************************************************************************************************
This version is the basic design. During testing, a major drift in the reading was discovered. Two revisions were made, with some improvement. Continuing research is being done on Version 3 - see a separate entry for that, OpenCTDwithConduinoV3.
*********************************************************************************************************************************************

This project merges the Oceanography for Everyone Open CTD with the Conduino conductivity sensor.

Open CTD:
https://github.com/OceanographyforEveryone/OpenCTD

Conduino:
Marco Carminati, Paolo Luzzatto-Fegiz,
Conduino: Affordable and high-resolution multichannel water conductivity sensor using micro USB connectors,
In Sensors and Actuators B: Chemical, Volume 251, 2017, Pages 1034-1041, ISSN 0925-4005,
https://doi.org/10.1016/j.snb.2017.05.184.

Follow the basic assembly instructions from OpenCTD along with the included instructions for constructing
a Conduino.

PCBs can be ordered from OSHPARK, or Eagle and Gerber files are included here.

The system goes into a 2" PVC pipe and seals with Marine Epoxy and a 3D printed plug (file included here). 
See Conduino Pipe Assembly.pdf file.

First test was taking it into the surf and turbulence here at Spanish Bay for a few minutes, see excel file for that gem.

March 25, 2018

3/27/18
slight modification to the schematic to take away the filter on the network analyzer buffer (C5 and R3). It was giving a gigantic time constant to the measurement.
