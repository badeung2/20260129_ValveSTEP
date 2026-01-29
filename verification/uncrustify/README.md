## Uncrustify configuration
--------------
### General
* Customer: Melexis BU-Actuators
* Description: Default Uncrustify configuration file.

### Getting started
Install uncrustify from http://uncrustify.sourceforge.net/

* Check code style
```sh
$ uncrustify -c uncrustify.cfg --check -l C $(SOURCE_FILE_LIST)
```

* Update source files with the correct style
```sh
$ uncrustify -c uncrustify.cfg --replace --no-backup -l C $(SOURCE_FILE_LIST)
```

### License
Copyright (C) 2017 Melexis N.V.

The Software is being delivered 'AS IS' and Melexis, whether explicitly or implicitly, makes no warranty as to its Use or performance.
The user accepts the Melexis Firmware License Agreement.

Melexis confidential & proprietary
