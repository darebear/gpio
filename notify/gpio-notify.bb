DESCRIPTION = "interrupt-based gpio signalling with user-space pin reading"
PRIORITY = "optional"
SECTION = "base"
LICENSE = "GPL"
RDEPENDS = "kernel (${KERNEL_VERSION})"
DEPENDS = "virtual/kernel"

PR = "r4"

SRC_URI = "\
   file://gpio-event-drv.h\
   file://gpio-notify-drv.c\
   file://gpio-notify.c\
   file://gpio.c\
   file://gpio.h\
   file://spi.c\
   file://spi.h\
   file://dcb.h\
   file://Makefile\
   "

S = "${WORKDIR}/gpio-notify"

inherit module-base

addtask builddir after do_fetch before do_unpack
addtask movesrc after do_unpack before do_patch

EXTRA_OEMAKE = 'CROSS_COMPILE="${CROSS_COMPILE}" \
                KERNELDIR="${KERNEL_SOURCE}" \
                CC="${CC}" \
                '

PARALLEL_MAKE = ""

do_builddir () {
   mkdir -p ${S}
}

do_movesrc () {
   cd ${WORKDIR}
   mv gpio-*.* gpio.* spi.* dcb.h Makefile ${S}
}

do_configure () {
   echo "Nothing to configure for gpio-notify"
}

do_compile () {
   unset CFLAGS CPPFLAGS CXXFLAGS LDFLAGS
   cd ${S}
	oe_runmake   
}

do_install () {
   # install programs to bindir
   install -m 0755 -d ${D}${bindir}
	install -m 0755  ${S}/gpio-notify ${D}${bindir}

   # kernel module installs with other modules
   install -m 0755 -d ${D}${base_libdir}/modules/${KERNEL_VERSION}/extra/
   # use cp instead of install so the driver doesn't get stripped
   cp ${S}/gpio-notify-drv.ko ${D}${base_libdir}/modules/${KERNEL_VERSION}/extra/
}

PACKAGES = "${PN}"
FILES_${PN}  = "${bindir}/gpio-notify"
FILES_${PN} += "${base_libdir}/modules/${KERNEL_VERSION}/extra/gpio-notify-drv.ko"
