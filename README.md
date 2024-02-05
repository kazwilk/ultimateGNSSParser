# ultimateGNSSParser

This library parses the given NMEA 0183 messages. The library parses these messages: ___$xxGSV___, ___$xxGSA___, ___$xxRMC___, ___$xxGGA___, ____$xxVTG___, ___$xxGLL___, ___$xxGBS___, ___$xxGST___
User can add his own parser for any other messages and/or replace library parser for particular message with his own parser.

* The messages are given to the library by callbacks. Library stores the NMEA 0183 messages data into the own structure.

* Thanks to the callbacks mechanism the data can be captured from any device connected via UART, i2c, SPI, USB, etc.

* Timeout support has also been provided.

* The library code has been quite thoroughly tested on many receivers from different manufacturers.

Check the [Wiki](https://github.com/kazwilk/ultimateGNSSParser/wiki) pages for more details.
