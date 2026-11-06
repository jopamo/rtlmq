# SPDX-License-Identifier: GPL-2.0-only

ifneq ($(KERNELRELEASE),)

# Public source is production-specialized before it reaches this tree.
GCOV_PROFILE := n
ccflags-y += -I$(src)

RTLMQ_KERNEL_CONFIG_SHA256 := $(shell \
	sha256sum "$(objtree)/.config" 2>/dev/null | awk '{print $$1}')
ifeq ($(RTLMQ_KERNEL_CONFIG_SHA256),)
$(error could not hash target kernel configuration $(objtree)/.config)
endif
CFLAGS_rtlmq_profile.o += \
	-DRTLMQ_KERNEL_CONFIG_SHA256=\"$(RTLMQ_KERNEL_CONFIG_SHA256)\"
ifneq ($(wildcard $(objtree)/include/generated/utsversion.h),)
CFLAGS_rtlmq_profile.o += -DRTLMQ_HAVE_UTSVERSION
endif

# Authoritative shipped source inventory for both in-tree and external-module
# Kbuild paths. Keep generated files and local-only helpers out of this list.
rtlmq-y += rtlmq_main.o
rtlmq-y += rtlmq_admin.o
rtlmq-y += rtlmq_chip.o
rtlmq-y += rtlmq_8126_mac.o
rtlmq-y += rtlmq_desc.o
rtlmq-y += rtlmq_hw.o
rtlmq-y += rtlmq_mac.o
rtlmq-y += rtlmq_8125_mac.o
rtlmq-y += rtlmq_profile.o
rtlmq-y += rtlmq_layout.o
rtlmq-y += rtlmq_runtime.o
rtlmq-y += rtlmq_lifecycle.o
# Reset admission, pending batches, epochs, and work callback share one owner.
rtlmq-y += rtlmq_reset.o
rtlmq-y += rtlmq_pci.o
rtlmq-y += rtlmq_ring.o
rtlmq-y += rtlmq_rx_ring.o
# RX ring lifetime/storage is separate from the NAPI-owned buffer/refill
# lifecycle. Keep allocation, DMA sync, recycle, and descriptor refill in the
# hot owner rather than regrowing the cold ring-lifetime file.
rtlmq-y += rtlmq_rx_refill.o
# RX owns packet consumption and borrowed-buffer copy construction.
rtlmq-y += rtlmq_rx.o
rtlmq-y += rtlmq_xsk.o
rtlmq-y += rtlmq_tx.o
# TX is split only where ownership changes: ring lifetime/storage, queue
# flow-control, completion/reclaim, and recovery.
# Diagnostic gates stay out of the publish fast path.
rtlmq-y += rtlmq_tx_ring.o
rtlmq-y += rtlmq_tx_queue.o
# TX reclaim owns buffer release, DMA unmap/rollback, and BQL completion.
rtlmq-y += rtlmq_tx_reclaim.o
# Recovery owns both the early completion-liveness resweep and the later
# netdev-watchdog validation/escalation transaction.
rtlmq-y += rtlmq_tx_recovery.o
# Serialization is part of the production mechanism.
rtlmq-y += rtlmq_tx_serial.o
# Production service requests have a bounded, passive tracepoint.
rtlmq-y += rtlmq_irq.o
rtlmq-y += rtlmq_irq_mask.o
rtlmq-y += rtlmq_irq_layout.o
# NAPI owns RX/TX service, completion, registration, and queue bindings.
rtlmq-y += rtlmq_napi.o
rtlmq-y += rtlmq_msix.o
rtlmq-y += rtlmq_coalesce.o
rtlmq-y += rtlmq_tc.o
rtlmq-y += rtlmq_ethtool.o
rtlmq-y += rtlmq_regdump.o
rtlmq-y += rtlmq_stats.o
rtlmq-y += rtlmq_phy.o
rtlmq-y += rtlmq_phy_config.o
rtlmq-y += rtlmq_rss.o
rtlmq-y += rtlmq_fw.o
rtlmq-y += rtlmq_ptp.o
rtlmq-y += rtlmq_ptp_clock.o
# PTP register-bank serialization and family backend mechanics.
rtlmq-y += rtlmq_ptp_hw.o
rtlmq-y += rtlmq_ptp_request.o
rtlmq-$(CONFIG_RTLMQ_LEDS) += rtlmq_led.o
# External Kbuild overrides do not rewrite the kernel's generated autoconf.
ccflags-y += -DRTLMQ_LEDS_ENABLED=$(if $(filter y m,$(CONFIG_RTLMQ_LEDS)),1,0)

# When this file is entered from an external-module build, emit obj-m. When it
# is copied into a kernel tree, keep the normal obj-$(CONFIG_RTLMQ) path.
ifneq ($(KBUILD_EXTMOD),)
obj-m += rtlmq.o
else
obj-$(CONFIG_RTLMQ) += rtlmq.o
endif

else

RUNNING_KDIR := /lib/modules/$(shell uname -r)/build
KDIR ?= $(RUNNING_KDIR)
KERNEL_MAKE_DIR := $(abspath $(KDIR))

# KDIR may name either a conventional prepared tree or the source side of a
# split build. Match kernel make semantics: O= wins over KBUILD_OUTPUT, and
# normalize it before changing into the kernel source directory.
ifneq ($(strip $(O)),)
RTLMQ_KBUILD_OUTPUT := $(abspath $(O))
else ifneq ($(strip $(KBUILD_OUTPUT)),)
RTLMQ_KBUILD_OUTPUT := $(abspath $(KBUILD_OUTPUT))
endif
ifneq ($(strip $(RTLMQ_KBUILD_OUTPUT)),)
KBUILD_DIR := $(RTLMQ_KBUILD_OUTPUT)
KBUILD_OUTPUT_ARG := O=$(KBUILD_DIR)
else
KBUILD_DIR := $(KERNEL_MAKE_DIR)
KBUILD_OUTPUT_ARG :=
endif
KBUILD_MAKE := $(MAKE) -C $(KERNEL_MAKE_DIR) $(KBUILD_OUTPUT_ARG)

# Module BTF is optional at runtime. Keep it opt-in for external modules:
# split/distilled BTF generated from an ambiguous or mismatched base can make
# an otherwise valid module unloadable. In-tree builds retain the target
# kernel's normal CONFIG_DEBUG_INFO_BTF_MODULES policy.
RTLMQ_MODULE_BTF ?= n
ifeq ($(RTLMQ_MODULE_BTF),y)
RTLMQ_BTF_KBUILD_ARG := CONFIG_DEBUG_INFO_BTF_MODULES=y
else ifeq ($(RTLMQ_MODULE_BTF),n)
RTLMQ_BTF_KBUILD_ARG := CONFIG_DEBUG_INFO_BTF_MODULES=
else
$(error RTLMQ_MODULE_BTF must be y or n)
endif

.PHONY: all modules production production-kdir-ready clean install

LOCAL_CLEAN_FILES := .*.cmd *.cmd *.d *.o *.ko *.ko.* *.mod *.mod.c \
			     Module.symvers modules.order
LOCAL_CLEAN_DIRS := .tmp_versions

all: modules

modules:
	$(KBUILD_MAKE) M=$(CURDIR) \
		$(RTLMQ_BTF_KBUILD_ARG) modules

production-kdir-ready:
	@test -f "$(KERNEL_MAKE_DIR)/Makefile" || { \
		echo "KDIR=$(KERNEL_MAKE_DIR) is not a kernel make tree (missing Makefile)" >&2; \
		exit 1; \
	}
	@test -f "$(KBUILD_DIR)/include/config/auto.conf" || { \
		echo "kernel output $(KBUILD_DIR) is not configured/prepared (missing include/config/auto.conf)" >&2; \
		exit 1; \
	}
	@test -s "$(KBUILD_DIR)/Module.symvers" || { \
		echo "kernel output $(KBUILD_DIR) lacks a non-empty Module.symvers; build the target kernel before producing a release module" >&2; \
		exit 1; \
	}
	@test -f "$(KBUILD_DIR)/.config" || { \
		echo "kernel output $(KBUILD_DIR) lacks .config" >&2; \
		exit 1; \
	}
	@test -f "$(KBUILD_DIR)/include/generated/utsversion.h" || { \
		echo "kernel output $(KBUILD_DIR) lacks generated UTS build identity" >&2; \
		exit 1; \
	}
	@command -v sha256sum >/dev/null 2>&1 || { \
		echo "sha256sum is required to bind the module to the target kernel configuration" >&2; \
		exit 1; \
	}

production: production-kdir-ready
	$(KBUILD_MAKE) M=$(CURDIR) clean
	$(KBUILD_MAKE) M=$(CURDIR) \
		$(RTLMQ_BTF_KBUILD_ARG) \
		modules

clean:
	@if [ -f "$(KERNEL_MAKE_DIR)/Makefile" ]; then \
		$(KBUILD_MAKE) M=$(CURDIR) clean || true; \
	else \
		echo "Skipping kernel-tree clean: KDIR=$(KERNEL_MAKE_DIR) is unavailable"; \
	fi
	rm -f $(LOCAL_CLEAN_FILES)
	rm -rf $(LOCAL_CLEAN_DIRS)

install:
	$(KBUILD_MAKE) M=$(CURDIR) modules_install

endif
