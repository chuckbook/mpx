class OLED -- OLED control for the ESG OLED module
==================================================

The OLED class is used to control the OLED on the ESG OLED module.
The OLED is a 128x96 pixel full color screen part.

The module must be connected to the I2C2 bus, and then
an OLED object is made using::

    OLED = pyb.OLED()

Then you can use::

    OLED.string('Hello world!\r\n')  # print text to the screen

Constructors
------------

.. class:: pyb.OLED()

   Construct an OLED object.

Methods
-------

.. method:: OLED.res()

   Reset the OLED module.

.. method:: OLED.onoff(on)

   Turn display on or off. Screen content is preseved.

.. method:: OLED.font(font)

   Select one of four fonts. Fonts have different sizes. The biggest size implements
   only digits and some separators and signs.
   
.. method:: OLED.pos(x, y)

   Set the position ``(x, y)``.

.. method:: OLED.colors(fg, bg)

   Set the character and the background color. A color is a 16-bit value.

.. method:: OLED.pen(lc, fc)
   Set the line and fill color. A color is a 16-bit value.

.. method:: OLED.string(s)

   Show the string s at current position with current font and colors.

.. method:: OLED.rect(xs, ys, w, h)

   Draw a w*h pixel rectangle at position x,y. The outline is the pen lc color.
   Fill color is pen fc.

Constants
---------

.. data:: color10[10]

   A list of predefined colors (0: black, 1: brown, ... ,9: white)

