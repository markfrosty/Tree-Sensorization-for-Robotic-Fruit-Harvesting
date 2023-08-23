# Sensor Array Assembly Instructions
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
  - REPEAT 3X FOR ALL BATTERY COVERS
  - Note: For best results, let the heat and the weight of the soldering iron do the work when installing the heat set inserts. This will prevent plastic from filling the inside of the insert
3. Repeat step 2 on the central base plate. Place 4x M2 x 0.4 mm thread, 4 mm length heat-set inserts in each corner hole of the jig and install them using the same process as before. Ensure they are flush with the plate when fully installed.
4. Affix heat set insert jig to a branch mount base using double-sided tape or a clamp, if using double sided tape ensure it will not be burned when installing inserts, place 4x M2 x 0.4 mm thread, 4 mm length heat-set inserts in each corner hole of the jig, install inserts, remove jig if still affixed, ensure inserts are flush, place 1x M2 x 0.4 mm thread, 4 mm length heat set insert in the hole located in the battery compartment, hold the insert straight and in the right location with metal tweezers and install, ensure flush
  - REPEAT 3X FOR ALL BRANCH MOUNT BASES
5. Place a small drop of solder betwwen the VUSB pads on the bottom of the Arduino Nano 33 BLE's. 
![Untitled drawing (2)](https://github.com/markfrosty/Tree-Sensorization/assets/124550575/7624766f-17d6-443f-958f-a6a27c505f3b)
6. Screw Arduino Nano 33 BLE peripherals into Arduino mount battery covers, tighten in criss-cross pattern

