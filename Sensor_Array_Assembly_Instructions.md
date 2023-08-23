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
  | Arduino Nano 33 BLE | 3 | Peripheral device boards |
  | Arduino Nano RP2040 Connect | 1 | Central device board |
  | branch_mount_base | 3 | Houses all components and affixes to branch |
  | Arduino_mount_battery_cover | 3 | Covers power system components and allows Arduino to mount |
  | Arduino_cover | 4 | Provides protection for central and peripheral sensors |
  | central_base_plate | 1 | Central device mount point |
  | M2 x 0.4 mm thread, 4 mm length heat set insert | 19 | Each branch mount base requires 5 and the RP2040 base plate requires 4 | 
  | M1.4 x 0.3 mm thread, 3 mm length heat set insert | 16 | Allow Arduino to be screwed down|
  | M2 x 0.4 mm thread, 6 mm length screw | 7 | The central base plate requires 4 and the branch mount base plate requires 1 each to affix the battery backpack | 
  | M2 x 0.4 mm thread, 12 mm length screw | 12 | Each branch mount base requires 4 to affix the Arduino cover | 
  | M1.4 x 0.3 mm thread, 3 mm length screw | 16 | Each Arduino mount battery cover requires 4 and the central base plate requires 4| 
  | Laser cut heat set insert jig | 1 | Allows precise placement of inserts |
  | Multi-bit screw driver | N/A | 1.3 mm and 1.5 mm hex bits are needed for installation of screws |
  | Double sided tape | 1 | Secures jig |
  | Metal tweezers | 1 | Helpful for installing heat set inserts | 
  | 22 AWG Solid Core wire | N/A | 4 colors are needed, kit in BOM should cover this | 
  | 3-Pin Slide Switch | 3 | On/Off switch for peripherals | 
  | Super glue | 1 | Used to affix switch |

2. Affix heat set insert jig to the central base plate using double-sided tape or a clamp, preheat soldering iron (Optional: an M2 heat set insert tool is very helpful for installing both sizes of inserts), place 4x M1.4 x 0.3 mm thread, 3 mm length heat-set inserts in each of the holes in the center of the jig, use a soldering iron to get the inserts started and as far down as possible without severely burning the jig (some burning is expected to occur), remove jig once mostly inserted, finish installing inserts until flush with the base, reinstall jig, place 4x M2 x 0.4 mm thread, 4 mm length heat-set inserts in each corner hole of the jig, install them using the same process as before, ensure they are flush with the plate when fully installed
  - Note: For best results, let the heat and the weight of the soldering iron do the work when installing the heat set inserts. This will prevent plastic from filling the inside of the insert.
3. Screw Arduino Nano RP2040 Connect central into the central base plate using 4x M1.4 x 0.3 mm thread, 3 mm long screws, tighten them in a criss-cross pattern, align Arduino cover with reset switch and USB port, using 4x M2 x 0.4 mm thread, 6 mm long screws, thread screws through Arduino cover and into heat-set inserts in the central base plate, shift 3D printed parts as needed to make edges flush with one another, tighten using a criss-cross pattern
    - In the end, it should look like this:
![central anatomy](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/25ffffb9-9093-4d28-b38b-cadc32f2d09e)
4. Affix heat set insert jig to an Arduino mount battery cover using double-sided tape or a clamp, preheat soldering iron (Optional: an M2 heat set insert tool is very helpful for installing both sizes of inserts), place 4x M1.4 x 0.3 mm thread, 3 mm length heat-set inserts in each of the holes in the center of the jig, use a soldering iron to get the inserts started and as far down as possible without severely burning the jig (some burning is expected to occur), remove jig once mostly inserted, finish installing inserts until flush with the base.
5. Affix heat set insert jig to a branch mount base using double-sided tape or a clamp, if using double sided tape ensure it will not be burned when installing inserts, place 4x M2 x 0.4 mm thread, 4 mm length heat-set inserts in each corner hole of the jig, install inserts, remove jig if still affixed, ensure inserts are flush, place 1x M2 x 0.4 mm thread, 4 mm length heat set insert in the hole located in the battery compartment, hold the insert straight and in the right location with metal tweezers and install, ensure flush
6. Place a small drop of solder between the VUSB pads on the bottom of the Arduino Nano 33 BLEs. 
![Untitled drawing (2)](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/455c9c20-9969-40a6-8b64-cd466689e224)
7. Screw Arduino Nano 33 BLE peripheral into the Arduino mount battery cover using 4x M1.4 x 0.3 mm thread, 3 mm long screws, tighten them in a criss-cross pattern, ensure the Arduino Nano 33 BLE has the U-blox logo is aligned with the wire pass-through hole on the Arduino mount battery cover
![IMG_4497](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/0083c70d-cd2e-49dd-875d-0785a54f176e)
8. Cut 3 pieces (one red, one green, one black) of 22AWG wire about 35-40 mm in length, strip about 4 mm of each end
9. Cut 2 pieces (one blue, one black) of 22AWG wire about 40 mm in length for the blue piece and around 42 mm for the black wire as it has a longer run, stip about 4 mm of each end
10. On the Adafruit LiPo Backpack there are two unlabeled pads with a trace connecting them, take a razor blade and cut the trace between these two pads, place a stripped end of the blue wire inside the innermost switch hole and ensure the wire is flush with but not poking out the bottom of the board, solder from the top as to keep the bottom of the board as flat as possible, repeat this with the black wire in the outside switch hole ensuring it is flush with the bottom, solder the black wire to the middle prong of the switch, solder the blue wire with one of the outside pins and the switch opposite that pin in the "off" position
![Untitled drawing (1)](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/491cec2f-ef18-4e87-a0b6-6de6af9b3467)
10. On the Adafruit LiPo Backpack place a stripped end of the shorter black wire in the 5V hole, ensure flush with the bottom of the board, solder from the top, place the green wire in the G hole, ensure flush with the bottom of the board, solder from the top, place the red wire in the BAT hole, ensure flush with the bottom of the board, solder from the top
    - I found this order of soldering made it easiest to solder each wire with less obstruction
11. Place a bead of hot glue around the solder joints on the battery backpack and the switch
12. Thread the other stripped end of the red, green, and black wires through the wire pass-through hole and into the Arduino Nano 33 BLE, the black wire going from 5V to the unmarked pad below the VUSB pad soldered earlier (4th hole from the antenna end and next to RST), the green wire going from G to GND, the red wire going from BAT to VIN, once the wires are correctly located in the Arduino Nano 33 BLE solder them from the top side of the board
    - As shown here:
![complete wiring except for switch](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/94f8000a-5bfa-4ce9-91a6-35f8edfb2a91)
13. Using an M2 x 0.4 mm thread 6 mm long screw attach the battery backpack to the branch mount base battery compartment, and press the switch into the switch hole making it mostly flush with the outside face, glue the switch in place by running a small bead of super glue around where the switch contacts the housing on the outside
    - If using super glue, some may seep into the switch and glue it in position. I have been able to take a screwdriver or tweezers and break the switch free from the glue and work it back and forth to clear the glue and make the switch function as desired.
    - I found letting the glue dry with the base nearly vertical was helpful for reducing the glue that could potentially seep into the switch.
![IMG_4502](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/e46fec22-fe01-4b23-86b8-d232c6ff184e)
14. Plug the battery into the backpack and place it inside the battery compartment, train wires in order to allow for closing as necessary, push Arduino mount battery cover down and align edges with branch mount base, align Arduino cover with reset switch and USB port, using 4x M2 x 0.4 mm thread, 12 mm long screws, thread screws through Arduino cover and Arduino mount battery cover and into heat-set inserts in branch mount base, shift 3D printed parts as needed to make edges flush with one another, tighten using a criss-cross pattern
    - In the end it should look like this:
![Untitled drawing (2)](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/4cd098a2-0a75-4fb8-a00f-725101107233)
15. Depending on resources either waterjet neoprene foam using the flat pattern provided or laser cut a template from acrylic using the same flat pattern and trace the template on the foam using a razor blade, peel the backing and apply to curved branch contact point on the bottom of the branch mount base
16. Thread velcro through the middle hole on the branch mount base
17. Using a paint marker or a way to mark the sensors, mark on and off on either side of the switch and mark which peripheral number each board is somewhere clearly visible on the peripheral
18. The battery backpack used allows the battery to be charged through the Arduino's micro-USB port, simply plug the board in with a micro-USB cable and charge like any other USB device

Repeat these steps a total of 3 times in order to build the peripherals and assemble the sensor array.

At this point the peripheral should be completed and ready to be affixed to a branch and relay data.
