## Display drivers

A multilayered set of display drivers for LCD and OLED displays written in plain C.

The UI library uses the graphics primitives provided by the LCD library for rendering. It is completely event driven and requires heap memory for storing the widget descriptors.

Some example canvases can be found [here](https://github.com/terjeio/GRBL_MPG_DRO_BoosterPack/tree/master/Firmware/MPG%20and%20DRO%20processor/canvas).

Widgets:

* Canvas (usually the whole display)
* Frame
* Button
* List
* ListElement
* Image
* Label
* TextBox - numeric input needs improvement (right justification)
* CheckBox

Events:
* NullEvent - continuous @ 10mS
* PointerDown
* PointerUp
* PrePointerChange
* PointerChanged
* PointerEnter
* PointerLeave
* ListOffStart
* ListOffEnd
* WidgetPainted
* WidgetClose
* WidgetClosed
* KeyDown
* KeyUp
* Validate
* Remote
* UserDefined

Support for input sources:

* Navigator
* IR Remote Control
* Keyboard Input
* Touch Input

The input sources must be provider by the user, and is done via a well defined API.
