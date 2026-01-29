## PC-Lint configuration
--------------
### General
* Customer: Melexis BU-Actuators
* Description: Default PC-Lint configuration files.

### Getting started
Install pc-lint from http://www.gimpel.com/html/pcl.htm

* lint make target for mlx16-gcc
```
lint:
# get correct gcc include directories
	@$(ECHO) $(subst /cygdrive/c,C:,$(foreach v,$(shell echo | mlx16-cpp -Wp,-v 2>&1),$(if $(findstring "/,$(v)), ,$(if $(findstring /,$(v)),-i\"$(v)\",)))) > $(LINT_DIR)/gcc-include-path.lnt
# get all include directories
	@$(ECHO) $(subst /cygdrive/c,C:,$(subst -I,,$(foreach v,$(CPPFLAGS),$(if $(findstring /,$(v):0:1),-i\"$(v)\",)))) > $(LINT_DIR)/project_include_path.lnt
	@$(ECHO) | $(CC) -E -dM $(INSTSET) $(CFLAGS) $(CPPFLAGS) - > $(LINT_DIR)/lint_cmac.h
	$(LINT) -i$(LINT_DIR) common-options.lnt co-gcc.lnt project-options.lnt temp.lnt $(filter %.c, $(SRCS)) > $(TARGET)_lint.txt
	$(LINT) -i$(LINT_DIR) common-options.lnt co-gcc.lnt au-misra2.lnt project-options.lnt temp.lnt $(filter %.c, $(SRCS)) > $(TARGET)_misra_lint.txt
```

### License
Copyright (C) 2017 Melexis N.V.

The Software is being delivered 'AS IS' and Melexis, whether explicitly or implicitly, makes no warranty as to its Use or performance.
The user accepts the Melexis Firmware License Agreement.

Melexis confidential & proprietary
