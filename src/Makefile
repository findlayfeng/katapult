# Main code build rules

src-y += sched.c bootentry.c command.c flashcmd.c initial_pins.c
src-$(CONFIG_ENABLE_LED) += led.c

# The serial mux: the units and the admin channel (mux.c), the identity it
# answers a scan with (generic/mux_chipid.c), the reset a REQUEST_BOOTLOADER
# ends in (generic/armcm_reset.c - the deployer builds have it already), and the
# hash that identity is derived with, the same one canserial_set_uuid uses.
# SERIAL and CANSERIAL exclude each other on every arch, so fasthash.c is never
# listed twice, and the output directory is asked for here because the arch
# makefiles that build fasthash for CAN are not the ones a UART build reaches.
src-$(CONFIG_MUX_SERIAL) += mux.c generic/mux_chipid.c generic/armcm_reset.c
src-$(CONFIG_MUX_SERIAL) += ../lib/fast-hash/fasthash.c
dirs-$(CONFIG_MUX_SERIAL) += lib/fast-hash

deployer-y += deployer.c
