#ifndef __MUX_H
#define __MUX_H

#include <stddef.h> // size_t
#include <stdint.h> // uint8_t, uint32_t

// The serial mux: the unit format a shared byte stream carries, and the device
// half katapult answers it with.
//
// A unit is a checked two-byte header naming an arbitration id, followed by one
// unmodified frame. The byte stream a device reads is therefore either raw
// frames (a path with one device on it) or units (a path with several), and the
// first byte says which: a header always starts with a bit set that no frame's
// first byte has.
//
// The specification is docs/klippy/developer-manual/serial-mux-protocol.md and
// src/mux.c in the klipperx checkout; the definitions here mirror it. What is
// katapult's own is the frame behind a header:
//
//  - a device's own frames are katapult's blocks (command.h MESSAGE_*:
//    `01 88 <cmd> <word_cnt> ... <crc> 99 03`). The host carries a katapult
//    block whole inside a unit and calls it a tunnel unit;
//  - the admin channel's frames are klipper's `KXID` pseudo-frames
//    (MUX_KXID_* below), because that is what the host builds for every
//    command it addresses to the admin id.
//
// The two shapes never collide: a katapult block starts with the `01 88` tag
// pair, a `KXID` frame with a length byte in 5..=64, and neither has the
// header's bit 7 set.

// The id a device is addressed by, as arbitration ids: CAN's mapping, and the
// same one the host uses. Both answer 0 for an id the wire format cannot
// address (the range is the single byte the admin message carries), so 0 is
// also "no node's id".
#define MUX_ARB_BASE 0x100
#define MUX_MAX_NODE_ID 0xff
uint32_t mux_arb_id(uint32_t nodeid);
uint32_t mux_reply_arb_id(uint32_t nodeid);

// The admin channel, spelled with CAN's own values (generic/canserial.h): the
// ids sit outside the assigned band, the even one listens and the odd one
// answers, and the command and response bytes mean what they mean there.
#define MUX_ADMIN_LISTEN 0x3f0
#define MUX_ADMIN_REPLY 0x3f1
#define MUX_CMD_QUERY_UNASSIGNED 0x00
#define MUX_CMD_SET_KLIPPER_NODEID 0x01
#define MUX_RESP_NEED_NODEID 0x20

// The two commands that name a device's own firmware: the two resets a device
// can be asked for. Both are addressed to the device (the port index they carry
// is a bridge's routing, and there is no bridge in this build, so it is read no
// further) and neither carries a receipt.
//
// MUX_CMD_REQUEST_BOOTLOADER is CAN's CANBUS_CMD_REQUEST_BOOTLOADER by value
// and meaning and leaves the device in its bootloader, for a flashing tool to
// write it - the request signature and reset the double reset into this
// firmware uses (generic/armcm_reset.c try_request_canboot).
#define MUX_CMD_REQUEST_BOOTLOADER 0x02
// MUX_CMD_RESTART_DEVICE is the other reset: the firmware itself comes back up
// instead of waiting in a bootloader, which for this firmware means starting
// the application (canboot.h application_jump).
#define MUX_CMD_RESTART_DEVICE 0x05

// The application byte this build answers a scan with, so the host names it
// right: CAN's CANBUS_CMD_SET_CANBOOT_NODEID by value, which is what the host's
// UnassignedNode::application_name() reads as "CanBoot". Klipper builds send no
// byte (read as Klipper), so the shape with the byte is the one to use here.
#define MUX_APP_CANBOOT 0x11

// A uuid is six bytes, most significant first.
#define MUX_UUID_LEN 6

// The two bytes a unit's header occupies.
#define MUX_HEADER_BYTES 2

// The band of legal codewords that name no address: a reader drops a unit whose
// header decodes into it.
#define MUX_HEADER_RESERVED_FIRST 0x3f2
#define MUX_HEADER_MAX 0x3ff

// What reading a header off the front of a buffer found.
enum mux_header_scan {
    MUX_HEADER_COMPLETE,   // a whole arbitration id, in *consumed bytes
    MUX_HEADER_NEED_MORE,  // the buffer ends inside the header
    MUX_HEADER_INVALID,    // not a header: framing bit, check bits, or reserved
};
enum mux_header_scan mux_decode_header(const uint8_t *buf, uint_fast8_t len
                                       , uint32_t *arb, uint_fast8_t *consumed);
uint_fast8_t mux_encode_header(uint8_t *buf, uint32_t arb);

// The `KXID` pseudo-frame: klipper's frame shape (MESSAGE_HEADER_SIZE 2,
// MESSAGE_TRAILER_SIZE 3, MESSAGE_SYNC 0x7e) whose sequence byte is 0x20, a
// value no ordinary klipper frame uses, and whose payload begins with the
// marker. The assignment handshake and every admin command ride it, differing
// only in the bytes after the marker.
//
// It is klipper's frame, not katapult's, and the lengths below are klipper's
// (MESSAGE_MIN 5, MESSAGE_MAX 64) rather than this firmware's own 8..128: the
// host builds these frames with klipper's rule, and a frame's bytes are what
// the other side reads. The byte after the header is a length, so the parser
// here reads it with klipper's positions - MUX_KXID_*, never katapult's
// MESSAGE_*, whose positions belong to a block.
#define MUX_KXID_SEQ_BYTE 0x20
#define MUX_KXID_MAGIC "KXID"
#define MUX_KXID_MAGIC_LEN 4
#define MUX_KXID_POS_LEN 0
#define MUX_KXID_POS_SEQ 1
#define MUX_KXID_HEADER_BYTES 2
#define MUX_KXID_TRAILER_BYTES 3
#define MUX_KXID_POS_CRC 3        // the CRC's high byte: at len - 3, len - 2
#define MUX_KXID_POS_SYNC 1       // the trailer's SYNC byte: at len - 1
#define MUX_KXID_SYNC 0x7e
#define MUX_KXID_FRAME_MIN 5
#define MUX_KXID_FRAME_MAX 64

// The decoded body of a `KXID` frame: what follows the marker. An empty body is
// the assignment handshake itself; otherwise the first byte is a command.
struct mux_kxid {
    uint8_t *args;
    uint_fast8_t args_len;
};
int mux_parse_kxid(uint8_t *frame, uint_fast8_t frame_len, struct mux_kxid *out);

// The units this protocol carries, each answering the bytes it wrote or 0 when
// the buffer is too small (or the id names no address).
uint_fast8_t mux_kxid_frame(uint8_t *buf, uint_fast8_t buf_size
                            , const uint8_t *payload, uint_fast8_t payload_len);
uint_fast8_t mux_kxid_unit(uint8_t *buf, uint_fast8_t buf_size, uint32_t arb
                           , const uint8_t *payload, uint_fast8_t payload_len);
uint_fast8_t mux_need_nodeid(uint8_t *buf, uint_fast8_t buf_size
                             , const uint8_t *uuid, uint8_t app);
uint_fast8_t mux_assignment_reply(uint8_t *buf, uint_fast8_t buf_size
                                  , uint32_t nodeid);

// The device half: who this device is inside the mux, and how a stream reaches
// it. The identity is the six bytes derived from the chip id; the node id
// starts at 0 ("no address yet") and is adopted from the assignment.
void mux_set_uuid(const uint8_t *uuid);
const uint8_t *mux_uuid(void);
uint32_t mux_node_id(void);

// The header our own frames go out with: 0 bytes while the device has no node
// id (raw frames, exactly the old wire format), else the reply header naming
// us. Whoever writes a frame upward calls this first.
uint_fast8_t mux_out_header(uint8_t *buf);

// Resolve the units at the front of an incoming byte buffer, in place.
//
// This is the device's whole input path: the serial console hands its bytes
// here before the ordinary frame parser reads them (generic/serial_irq.c).
// Admin units are answered or consumed; one of our own frames is left at the
// front with its header stripped, which is the frame the parser then reads. Raw
// frames - a path with one device on it - are left alone, which is what keeps
// the old wire format working.
//
// Returns 1 when the front is an ordinary frame the caller's parser should look
// at, 0 when there is nothing to parse: the front is a unit that was handled,
// or one that has not arrived whole yet. In the second case the caller must not
// parse - a unit's leading bytes are not a frame, and reading them as one would
// lose the unit.
int mux_input(uint8_t *buf, size_t *len);

// Bytes the mux sends upward (its answers on the admin channel): implemented by
// whoever carries the stream, because only it knows where the transmit buffer
// is (generic/serial_irq.c).
void mux_send(const uint8_t *bytes, uint_fast8_t len);

#endif // mux.h
