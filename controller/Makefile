.PHONY: create_folder autoconf_post

ROOT_LOC := $(CURDIR)
ROOT_KCONFIG := $(ROOT_LOC)/Kconfig
KCONFIG_CONFIG_LOC := $(ROOT_LOC)/configs

ifeq ("$(origin DEF)", "command line")
DEFCONFIG_FILE := $(KCONFIG_CONFIG_LOC)/$(DEF)
else
DEFCONFIG_FILE := $(KCONFIG_CONFIG_LOC)/controller_defconfig
endif

KCONFIG_CONFIG := $(KCONFIG_CONFIG_LOC)/.config
KCONFIG_CONFIG_OUTPUT_LOC := $(KCONFIG_CONFIG_LOC)/generated
export KCONFIG_CONFIG

ifeq ($(OS),Windows_NT)
	AUTOCONF_POST_FILE := autoconf_post.exe
else
	AUTOCONF_POST_FILE := autoconf_post
endif

menuconfig: autoconf_post
	menuconfig $(ROOT_KCONFIG)
	cd $(KCONFIG_CONFIG_OUTPUT_LOC) && genconfig $(ROOT_KCONFIG) && mv config.h autoconf.h
	configs/generated/$(AUTOCONF_POST_FILE) configs/generated/autoconf.h

savedefconfig:
	cp $(KCONFIG_CONFIG_LOC)/.config $(DEFCONFIG_FILE)

defconfig: autoconf_post
	cd configs && defconfig --kconfig $(ROOT_KCONFIG) $(DEFCONFIG_FILE)
	cd configs/generated && genconfig $(ROOT_KCONFIG) && mv config.h autoconf.h
	configs/generated/$(AUTOCONF_POST_FILE) configs/generated/autoconf.h

autoconf_post: create_folder
	cd $(ROOT_LOC)/../components/autoconf_post && make all
	cp $(ROOT_LOC)/../components/autoconf_post/$(AUTOCONF_POST_FILE) configs/generated/$(AUTOCONF_POST_FILE)

create_folder:
	mkdir -p configs/generated
