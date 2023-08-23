# Sensor Array Assembly Instructions
## Software

Follow the [How To Use](https://github.com/markfrosty/Tree-Sensorization#how-to-use) section in the main [README](https://github.com/markfrosty/Tree-Sensorization) in order to properly upload the programs to the sensor array.

## 3D Printing 

All 3D-printed components were printed with PLA on a Prusa MK3. 30% infill, 0.10 mm DETAIL setting, and supports everywhere were used to achieve high-quality parts.
The parts to be printed are as follows:

  | Item  | Quanity | Note | 
  | ------------- | ------------- | ------------- |
  | branch_mount_base | 3 | Houses all components and affixes to branch |
  | Arduino_mount_battery_cover | 3 | Covers power system components and allows Arduino to mount |
  | Arduino_cover | 4 | Provides protection for central and peripheral sensors |
  | central_base_plate | 1 | Central device mount point | 

## Final Assembly

1. Assuming all parts are present and/or printed, gather the following:

  | Item  | Quanity | Note | 
  | ------------- | ------------- | ------------- |
  | branch_mount_base | 3 | Houses all components and affixes to branch |
  | Arduino_mount_battery_cover | 3 | Covers power system components and allows Arduino to mount |
  | Arduino_cover | 4 | Provides protection for central and peripheral sensors |
  | central_base_plate | 1 | Central device mount point |
  | M2 x 0.4 mm thread, 4 mm length heat set insert | 19 | Each housing requrires 5 and the RP2040 base plate requires 4 | 
  | M1.4 x 0.3 mm thread, 3 mm length heat set insert | 16 | Allow Arduino to be screwed down|
  | Laser cut heat set insert jig | 1 | Allows precise placement of inserts |

2. Affix heat set insert jig to an Arduino mount battery cover using double-sided tape or a clamp, preheat soldering iron (Optional: an M2 heat set insert tool is very helpful for installing both sizes of inserts), place 4x M1.4 x 0.3 mm thread, 3 mm length heat-set inserts in each of the holes in the center of the jig, use a soldering iron to get the inserts started and as far down as possible without severely burning the jig(some burning is expected to occur), remove jig once mostly inserted remove the jig, finish installing inserts until flush with the base.
  - Note: For best results, let the heat and the weight of the soldering iron do the work when installing the heat set inserts. This will prevent plastic from filling the inside of the insert
3. Repeat step 2 on the central base plate. Place 4x M2 x 0.4 mm thread, 4 mm length heat-set inserts in each corner hole of the jig and install them using the same process as before. Ensure they are flush with the plate when fully installed.
4. Affix heat set insert jig to a branch mount base using double-sided tape or a clamp, if using double sided tape ensure it will not be burned when installing inserts, place 4x M2 x 0.4 mm thread, 4 mm length heat-set inserts in each corner hole of the jig, install inserts, remove jig if still affixed, ensure inserts are flush, place 1x M2 x 0.4 mm thread, 4 mm length heat set insert in the hole located in the battery compartment, hold the insert straight and in the right location with metal tweezers and install, ensure flush
5. Place a small drop of solder between the VUSB pads on the bottom of the Arduino Nano 33 BLEs. 
![Untitled drawing (2)](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/455c9c20-9969-40a6-8b64-cd466689e224)
6. Screw Arduino Nano 33 BLE peripherals into Arduino mount battery covers, tighten them in a criss-cross pattern, ensure the Arduino Nano 33 BLE has the U-blox logo aligned with the wire pass-through hole on the Arduino mount battery cover
![IMG_4497](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/0083c70d-cd2e-49dd-875d-0785a54f176e)
7. Cut 3 pieces (one red, one green, one black) of 22AWG wire about 35-40 mm in length, strip about 4 mm of each end
8. Cut 2 pieces (one blue, one black) of 22AWG wire about 40 mm in length for the blue piece and around 42 mm for the black wire as it has a longer run, stip about 4 mm of each end
9. On the Adafruit LiPo Backpack there are two unlabeled pads with a trace connecting them, take a razor blade and cut the trace between these two pads, place a stripped end of the blue wire inside the innermost switch hole and ensure the wire is flush with but not poking out the bottom of the board, solder from the top as to keep the bottom of the board as flat as possible, repeat this with the black wire in the outside switch hole ensuring it is flush with the bottom, solder the black wire to the middle prong of the switch, solder the blue wire with one of the outside pins and the switch opposite that pin in the "off" position
![Untitled drawing (1)](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/491cec2f-ef18-4e87-a0b6-6de6af9b3467)
10. On the Adafruit LiPo Backpack place a stripped end of the shorter black wire in the 5V hole, ensure flush with the bottom of the board, solder from the top, place the green wire in the G hole, ensure flush with the bottom of the board, solder from the top, place the red wire in the BAT hole, ensure flush with the bottom of the board, solder from the top
    - I found this order of soldering made it easiest to solder each wire with less obstruction
11. Place a bead of hot glue around the solder joints on the battery backpack and the switch
12. Thread the other stripped end of the red, green, and black wires through the wire pass-through hole and into the Arduino Nano 33 BLE, the black wire going from 5V to the unmarked pad below the VUSB pad soldered earlier (4th hole from the antenna end and next to RST), the green wire going from G to GND, the red wire going from BAT to VIN, once the wires are correctly located in the Arduino Nano 33 BLE solder them from the top side of the board
    - In the end it should look like this:
![complete wiring except for switch](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/94f8000a-5bfa-4ce9-91a6-35f8edfb2a91)




