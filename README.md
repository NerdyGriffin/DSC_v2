# DSC_v2
UI and real-time equipment control software for a Differential Scanning Calorimetry Prototype System, as part of my undergraduate physics research.
Version 2 is remade for use with a Feather M0 Express

### DSC_v2: UI and control systems for DSC prototype system

Copyright (C) 2020 Christian Kunis

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

You may contact the author at ckunis.contact@gmail.com

NOTE: This project is currently a work-in-progress. Not all features have been implemented and all code may be subject to change.

## Setup Instructions

1. Plug in the Feather M0 Express to the computer via a USB cable.
2. If you have not done so already, open the `dsc_arduino` sketch in the Arduino IDE and upload it to the Adafruit Feather M0 Express.
3. Make a note of which serial port the arduino board is connected to (Example: "COM3"), as you will need to set the same port in the experiment UI

## Experiment Instructions

Run the `DSC_Experiment_UI.mlapp` in MATLAB 2020b or later
(More detailed instructions coming in the future)

## Analysis Instructions

Run the `DSC_Data_Analysis_UI.mlapp` in MATLAB 2020b or later
(More detailed instructions coming in the future)

## Previous version

The old version of this project was created to use a NI USB-6211 DAQ Box.

Please note that the pervious version is deprecated and is no longer being support.

The link is provided here for reference purposes only.

The previous version can be found here: https://github.com/NerdyGriffin/DSC_UI
