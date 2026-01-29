This is the release repository which contains artifacts of the ROM and Flash library from https://gitlab.melexis.com/swcc-si-projects/82100 plus build system and some more user files.

Normally the repository adapts the output from the package job marked with tag `PLTF_CAMCU_*` (like https://gitlab.melexis.com/swcc-si-projects/82100/builds/454440/artifacts/browse/package/PLTF_CAMCU_0_5_0/)

In the case the package job isn't available, the next steps can be taken to update or add new chip revison support:
- define which chip revision has to be changed/added to this repositoy
- update/add the *-rom-defs.ld in ./lib/ldscripts (e.g. https://gitlab.melexis.com/bu-act/firmware/library_platform_camcu/blob/develop/lib/ldscripts/81113xAA-rom-defs.ld) taken from corresponding ROM build job like from https://gitlab.melexis.com/swcc-si-projects/82100/builds/449065/artifacts/file/build/lib/81113/rom-defs.ld 
- update/add the ROM .h files in ./include/<chip_revision> taken from e.g. https://gitlab.melexis.com/swcc-si-projects/82100/builds/449065/artifacts/browse/build/lib/81113/
- update/add the Flash .h files in ./include/<chip_revision> taken from e.g. https://gitlab.melexis.com/swcc-si-projects/82100/builds/449086/artifacts/browse/build/flash/81113/local/ (replacing even previously added ROM .h files)
- update/add the Flash .a files in ./lib/<chip_revision> taken from e.g. https://gitlab.melexis.com/swcc-si-projects/82100/builds/449086/artifacts/browse/build/flash/81113/local/
- if new chip revision is introduced then the build system shall know about new PROJECT_ID: new chip revision shall be added here to https://gitlab.melexis.com/bu-act/firmware/library_platform_camcu/blob/develop/config/81113.mk and default cpp-flags file shall be created for new chip revision (e.g. copying the https://gitlab.melexis.com/bu-act/firmware/library_platform_camcu/blob/develop/config/81113xAA-cpp-flags.mk)