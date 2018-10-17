## Display drivers

A multilayered set of display drivers for LCD and OLED displays written in plain C.

The lowest layer is the hardware interface specific for each MCU supported. It should be fairly easy to add additional drivers.
Above that is the graphics primitives layer, for LCDs this is split in two - one addtional layer is for supporting the display model.
On top of the graphics primitives layer a UI layer may be added, this uses the heap for storing the widget descriptors so it is not suited for processors with limited memory.

Font support is provided by an implementation of a rendering engine for H. Reddmann fonts.
Since most fonts I use is copyrighted I do not include these, however they are easy to make from TrueType fonts with H. Reddmanns tools.

For color displays RGB888 format is used to specify the color, translation to the appropriate pixel format is provided by the drivers.

Credits:

Parts of the code has been based on code sourced from the internet and possibly modified for my needs.

* Display drivers by [RobG](https://forum.43oh.com/profile/73-robg/) @ 43oh.com
* [Font parsing and rendering](https://www.mikrocontroller.net/topic/99701#865331) by H. Reddmann
* [8x8 proportional font](https://github.com/mueschel/lcdlib/blob/master/Fonts/font_proportional_8px.c) by mueschel
* [Jpg decompression](http://elm-chan.org/fsw/tjpgd/00index.html) by ChaN
* [Touch calibration](https://www.embedded.com/design/system-integration/4023968/How-To-Calibrate-Touch-Screens) by Carlos E. Vidales
* [Quick select routine](http://ndevilla.free.fr/median/median/) by Nicolas Devillard

There may be bits and pieces gleaned from other work too, but I have not kept track of where I found these.
If you feel that some parts should be credited, or removed, then please contact me.

Licensing for the different source files is not consistent, some does not provide it at all.
Sources without licensing info is assumed to be in the public domain.
Please check the source code for details before use.
