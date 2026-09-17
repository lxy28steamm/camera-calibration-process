#!/usr/bin/env python3
"""Internal-only Kalibr schema v2 calibration writer.

This tool deliberately lives outside the customer delivery package.  It
reuses the customer tool only for UVC transport discovery and readback, while
the credential, upload, commit, and activation protocol remains here.
"""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import struct
import sys
import textwrap
import zlib


def load_readonly_transport():
    if __package__:
        from . import yctc_uvc_xu_procotol
        return yctc_uvc_xu_procotol
    transport_path = Path(__file__).with_name("yctc_uvc_xu_procotol.py")
    if not transport_path.is_file():
        raise RuntimeError("customer read-only UVC transport module is missing: {}".format(transport_path))

    module_name = "yctc_xu_readonly_transport"
    spec = importlib.util.spec_from_file_location(module_name, str(transport_path))
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load customer read-only UVC transport module")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


base = load_readonly_transport()
UvcXuError = base.UvcXuError


YCTC_XU_CMD_BEGIN_WRITE = 0x01
YCTC_XU_CMD_COMMIT = 0x02
YCTC_XU_CMD_ACTIVATE = 0x03
YCTC_XU_CMD_ABORT = 0x04
YCTC_XU_CMD_UNLOCK_WRITE = 0x06
YCTC_XU_COMMAND_PROTOCOL_VERSION = 1
YCTC_XU_COMMAND_FLAG_WRITE_UNLOCKED = 0x01
YCTC_XU_COMMAND_FLAG_SESSION_AUTHORIZED = 0x02
DEFAULT_UNLOCK_SECRET_ENV = "YCTC_XU_UNLOCK_SECRET"


def encode_calibration_serial_number(serial_number):
    if not serial_number:
        raise UvcXuError("--serial-number is required for a schema v2 calibration write")
    try:
        data = str(serial_number).encode("ascii")
    except UnicodeEncodeError as exc:
        raise UvcXuError("--serial-number must use printable ASCII") from exc
    if len(data) > base.CALIBRATION_SN_SIZE:
        raise UvcXuError("--serial-number must be at most {} bytes".format(base.CALIBRATION_SN_SIZE))
    if not data or any(byte < 0x21 or byte > 0x7E for byte in data):
        raise UvcXuError("--serial-number must use printable ASCII without spaces")
    return data + (b"\x00" * (base.CALIBRATION_SN_SIZE - len(data)))


def build_kalibr_yaml_blob(yaml_payload, serial_number):
    payload = bytes(yaml_payload)
    try:
        base.validate_kalibr_yaml_payload(payload)
    except ValueError as exc:
        raise UvcXuError(str(exc)) from exc
    if len(payload) > 0xFFFFFFFF:
        raise UvcXuError("Kalibr YAML payload is too large")

    serial_field = encode_calibration_serial_number(serial_number)
    header = struct.pack(
        "<4sHHI",
        base.CALIBRATION_BLOB_MAGIC,
        base.CALIBRATION_SCHEMA_V2,
        0,
        len(payload),
    )
    header += serial_field
    header += b"\x00" * (base.CALIBRATION_HEADER_SIZE - len(header))
    if len(header) != base.CALIBRATION_HEADER_SIZE:
        raise AssertionError("unexpected calibration header size")

    blob_without_tail_crc = header + payload
    return blob_without_tail_crc + struct.pack("<I", zlib.crc32(blob_without_tail_crc) & 0xFFFFFFFF)


def read_calib_command(backend, args):
    payload = base.xu_get_cur(backend, args.unit_id, base.SELECTOR_CALIB_COMMAND, base.YCTC_XU_COMMAND_LEN)
    flags, state, protocol_version, unlock_nonce, remaining_window_ms, authorized_session_id = struct.unpack(
        "<BBHIII", payload)
    return {
        "flags": flags,
        "write_unlocked": bool(flags & YCTC_XU_COMMAND_FLAG_WRITE_UNLOCKED),
        "session_authorized": bool(flags & YCTC_XU_COMMAND_FLAG_SESSION_AUTHORIZED),
        "state": state,
        "state_name": {
            0: "IDLE",
            1: "RECEIVING",
            2: "READY",
            3: "ERROR",
        }.get(state, "UNKNOWN"),
        "protocol_version": protocol_version,
        "unlock_nonce": unlock_nonce,
        "remaining_window_ms": remaining_window_ms,
        "authorized_session_id": authorized_session_id,
        "raw_hex": payload.hex(),
    }


def pack_calib_command(command, schema_version=0, session_id=0, arg0=0, arg1=0):
    values = {
        "command": (command, 0xFF),
        "schema_version": (schema_version, 0xFFFF),
        "session_id": (session_id, 0xFFFFFFFF),
        "arg0": (arg0, 0xFFFFFFFF),
        "arg1": (arg1, 0xFFFFFFFF),
    }
    for name, (value, maximum) in values.items():
        if int(value) < 0 or int(value) > maximum:
            raise UvcXuError("{} is outside its protocol range".format(name))
    return struct.pack(
        "<BBHIII",
        int(command),
        0,
        int(schema_version),
        int(session_id),
        int(arg0),
        int(arg1),
    )


def pack_write_transfer(session_id, offset, data):
    chunk = bytes(data)
    if not chunk or len(chunk) > base.YCTC_XU_TRANSFER_DATA_MAX:
        raise UvcXuError("write chunk length must be in range 1..{}".format(base.YCTC_XU_TRANSFER_DATA_MAX))
    if int(session_id) <= 0 or int(session_id) > 0xFFFFFFFF:
        raise UvcXuError("write session_id must be in range 1..0xffffffff")
    if int(offset) < 0 or int(offset) > 0xFFFFFFFF:
        raise UvcXuError("write offset must fit uint32")
    return struct.pack(
        "<IIHH",
        int(session_id),
        int(offset),
        len(chunk),
        base.crc16_modbus(chunk),
    ) + chunk + (b"\x00" * (base.YCTC_XU_TRANSFER_DATA_MAX - len(chunk)))


def calculate_unlock_token(secret, session_id, unlock_nonce, info):
    try:
        secret_bytes = str(secret).encode("utf-8")
    except UnicodeEncodeError as exc:
        raise UvcXuError("unlock secret cannot be encoded as UTF-8") from exc
    if not secret_bytes or b"\x00" in secret_bytes:
        raise UvcXuError("unlock secret must be non-empty and cannot contain NUL")
    if int(session_id) <= 0 or int(session_id) > 0xFFFFFFFF:
        raise UvcXuError("write session_id must be in range 1..0xffffffff")
    token_payload = secret_bytes[:96] + struct.pack(
        "<IIIIH",
        int(unlock_nonce),
        int(session_id),
        int(info["payload_crc32"]),
        int(info["payload_length"]),
        int(info["schema_version"]),
    )
    return zlib.crc32(token_payload) & 0xFFFFFFFF


def resolve_unlock_token(args, session_id, command, info):
    if args.unlock_token is not None:
        return int(args.unlock_token), "explicit_token"
    secret = args.unlock_secret or os.environ.get(DEFAULT_UNLOCK_SECRET_ENV, "")
    if not secret:
        raise UvcXuError(
            "set --unlock-token, --unlock-secret, or {} before a calibration write".format(
                DEFAULT_UNLOCK_SECRET_ENV))
    return calculate_unlock_token(secret, session_id, command["unlock_nonce"], info), "secret"


def write_session_id_from_args(args):
    if int(args.session_id) != 0:
        return int(args.session_id)
    session_id = int.from_bytes(os.urandom(4), "little")
    return session_id if session_id != 0 else 1


def require_calib_status(status, expected_state, stage, expected_received_length=None):
    if int(status["state"]) != int(expected_state) or int(status["status_code"]) != 0:
        raise UvcXuError("{} failed: CALIB_STATUS {}".format(stage, json.dumps(status, sort_keys=True)))
    if expected_received_length is not None and int(status["received_length"]) != int(expected_received_length):
        raise UvcXuError(
            "{} received length mismatch: {} != {}".format(
                stage, status["received_length"], expected_received_length))


def set_calib_command(backend, args, command, stage):
    try:
        base.xu_set_cur(backend, args.unit_id, base.SELECTOR_CALIB_COMMAND, command)
    except Exception as exc:
        try:
            status = base.read_calib_status(backend, args)
        except Exception:
            status = None
        if status is not None:
            raise UvcXuError("{} rejected: CALIB_STATUS {}".format(
                stage, json.dumps(status, sort_keys=True))) from exc
        raise


def run_calib_write_kalibr(backend, args):
    with open(args.write_kalibr_yaml, "rb") as fp:
        yaml_payload = fp.read()
    blob = build_kalibr_yaml_blob(yaml_payload, args.serial_number)
    if len(blob) > int(args.max_blob_size):
        raise UvcXuError("schema v2 blob length {} exceeds --max-blob-size {}".format(
            len(blob), args.max_blob_size))

    protocol_probe = base.run_probe(backend, args) if args.validate_protocol else None
    current_info = base.read_calib_info(backend, args)
    command_before = read_calib_command(backend, args)
    if int(command_before["protocol_version"]) != YCTC_XU_COMMAND_PROTOCOL_VERSION:
        raise UvcXuError("unsupported CALIB_COMMAND protocol version: {}".format(
            command_before["protocol_version"]))

    session_id = write_session_id_from_args(args)
    if (command_before["write_unlocked"] and
            int(command_before["authorized_session_id"]) not in (0, session_id)):
        raise UvcXuError("another host has an active calibration write authorization")

    unlock_token, unlock_token_source = resolve_unlock_token(args, session_id, command_before, current_info)
    set_calib_command(
        backend,
        args,
        pack_calib_command(
            YCTC_XU_CMD_UNLOCK_WRITE,
            session_id=session_id,
            arg0=command_before["unlock_nonce"],
            arg1=unlock_token,
        ),
        "UNLOCK_WRITE",
    )
    command_unlocked = read_calib_command(backend, args)
    if (not command_unlocked["write_unlocked"] or
            int(command_unlocked["authorized_session_id"]) != session_id):
        raise UvcXuError("UNLOCK_WRITE did not authorize the requested session")

    blob_crc32 = zlib.crc32(blob) & 0xFFFFFFFF
    transaction_started = False
    activated = False
    try:
        set_calib_command(
            backend,
            args,
            pack_calib_command(
                YCTC_XU_CMD_BEGIN_WRITE,
                schema_version=base.CALIBRATION_SCHEMA_V2,
                session_id=session_id,
                arg0=len(blob),
                arg1=blob_crc32,
            ),
            "BEGIN_WRITE",
        )
        transaction_started = True
        begin_status = base.read_calib_status(backend, args)
        require_calib_status(begin_status, 1, "BEGIN_WRITE", expected_received_length=0)

        chunk_count = 0
        for offset in range(0, len(blob), base.YCTC_XU_TRANSFER_DATA_MAX):
            chunk = blob[offset:offset + base.YCTC_XU_TRANSFER_DATA_MAX]
            base.xu_set_cur(
                backend,
                args.unit_id,
                base.SELECTOR_CALIB_TRANSFER,
                pack_write_transfer(session_id, offset, chunk),
            )
            chunk_count += 1

        transfer_status = base.read_calib_status(backend, args)
        require_calib_status(transfer_status, 1, "TRANSFER", expected_received_length=len(blob))
        set_calib_command(
            backend,
            args,
            pack_calib_command(YCTC_XU_CMD_COMMIT, session_id=session_id),
            "COMMIT",
        )
        commit_status = base.read_calib_status(backend, args)
        require_calib_status(commit_status, 2, "COMMIT", expected_received_length=len(blob))
        set_calib_command(
            backend,
            args,
            pack_calib_command(YCTC_XU_CMD_ACTIVATE, session_id=session_id),
            "ACTIVATE",
        )
        activated = True
    except Exception:
        if transaction_started and not activated:
            try:
                base.xu_set_cur(
                    backend,
                    args.unit_id,
                    base.SELECTOR_CALIB_COMMAND,
                    pack_calib_command(YCTC_XU_CMD_ABORT, session_id=session_id),
                )
            except Exception:
                pass
        raise

    activate_status = base.read_calib_status(backend, args)
    require_calib_status(activate_status, 0, "ACTIVATE", expected_received_length=0)
    active_info = base.read_calib_info(backend, args)
    if (int(active_info["schema_version"]) != base.CALIBRATION_SCHEMA_V2 or
            int(active_info["payload_length"]) != len(blob) or
            int(active_info["payload_crc32"]) != blob_crc32):
        raise UvcXuError("active CALIB_INFO does not match the committed schema v2 blob")

    readback_blob, readback_info = base.read_calibration_blob(backend, args, session_id=session_id)
    if readback_blob != blob:
        raise UvcXuError("schema v2 calibration readback does not match the uploaded blob")
    parsed = base.parse_calibration_blob(readback_blob)
    yaml_out = ""
    if args.yaml_out:
        yaml_out = base.write_kalibr_yaml_output(args.yaml_out, readback_blob)

    result = {
        "ok": True,
        "operation": "calib-write-kalibr",
        "backend": backend.name,
        "input_file": os.path.abspath(args.write_kalibr_yaml),
        "serial_number": args.serial_number,
        "schema_version": base.CALIBRATION_SCHEMA_V2,
        "yaml_length": len(yaml_payload),
        "blob_length": len(blob),
        "blob_crc32": blob_crc32,
        "session_id": session_id,
        "unlock_token_source": unlock_token_source,
        "chunk_count": chunk_count,
        "current_info": current_info,
        "begin_status": begin_status,
        "transfer_status": transfer_status,
        "commit_status": commit_status,
        "activate_status": activate_status,
        "active_info": active_info,
        "readback_info": readback_info,
        "yaml_out": yaml_out,
        "header": parsed["header"],
    }
    if protocol_probe is not None:
        result["protocol_probe"] = protocol_probe
    return result


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Internal-only YCTC Kalibr schema v2 calibration writer.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent(
            """\
            This tool is not part of the customer protocol package.

            Linux example:
              YCTC_XU_UNLOCK_SECRET='device-specific-secret' %(prog)s \\
                --device /dev/video1 \\
                --write-kalibr-yaml calibration_stride3-camchain-imucam.yaml \\
                --serial-number ZXCZ-T7-KALIBR-0001

            Windows PowerShell example:
              $env:YCTC_XU_UNLOCK_SECRET = 'device-specific-secret'
              py -3 %(prog)s --backend windows-ks \\
                --write-kalibr-yaml calibration_stride3-camchain-imucam.yaml \\
                --serial-number ZXCZ-T7-KALIBR-0001
            """
        ),
    )
    parser.add_argument("--backend", choices=("auto", "linux-ioctl", "windows-ks", "pyusb"), default="auto")
    parser.add_argument("--list", action="store_true", help="list candidate devices/interfaces and exit")
    parser.add_argument("--device", help="Linux video node, for example /dev/video1 or video1")
    parser.add_argument("--vid", type=base.parse_u16, help="USB vendor id filter, for example 0x1234")
    parser.add_argument("--pid", type=base.parse_u16, help="USB product id filter, for example 0x5678")
    parser.add_argument("--index", type=int, help="select an entry printed by --list")
    parser.add_argument("--interface", type=base.parse_u8, help="USB VideoControl interface number for pyusb")
    parser.add_argument("--bus", type=base.parse_int, help="pyusb bus filter")
    parser.add_argument("--address", type=base.parse_int, help="pyusb address filter")
    parser.add_argument("--xu-guid", type=base.parse_guid, default=base.DEFAULT_XU_GUID,
                        help="Windows KS Extension Unit GUID, default {}".format(base.DEFAULT_XU_GUID))
    parser.add_argument("--node-id", type=base.parse_int,
                        help="Windows KS topology node id override; auto-scanned by default")
    parser.add_argument("--timeout-ms", type=int, default=3000, help="USB control transfer timeout, default 3000")
    parser.add_argument("--skip-probe", action="store_true", help="Linux: skip GET_LEN/GET_INFO support check")
    parser.add_argument("--no-claim", action="store_true", help="pyusb: do not claim the VideoControl interface")
    parser.add_argument("--unit-id", type=base.parse_u8, default=base.DEFAULT_UNIT_ID,
                        help="XU unit id, default 0x0a")
    parser.add_argument("--write-kalibr-yaml", default="", metavar="FILE",
                        help="UTF-8 Kalibr camchain-imucam YAML to upload as schema v2")
    parser.add_argument("--serial-number", default="",
                        help="printable ASCII serial number stored in the schema v2 blob")
    parser.add_argument("--unlock-secret", default="",
                        help="write credential; prefer {} instead".format(DEFAULT_UNLOCK_SECRET_ENV))
    parser.add_argument("--unlock-token", type=base.parse_u32,
                        help="precomputed write unlock token; requires a non-zero --session-id")
    parser.add_argument("--chunk-size", type=int, default=base.YCTC_XU_TRANSFER_DATA_MAX,
                        help="readback chunk size, range 1..48, default 48")
    parser.add_argument("--session-id", type=base.parse_int, default=0,
                        help="transaction session id; a random non-zero value is generated when omitted")
    parser.add_argument("--max-blob-size", type=int, default=8192, help="maximum accepted blob size, default 8192")
    parser.add_argument("--yaml-out", default="", help="write verified Kalibr YAML readback to this path")
    parser.add_argument("--validate-protocol", action="store_true",
                        help="probe documented selector metadata before the write")
    return parser


def configure_transport_args(args):
    args.xu_expected_selector_override = base.SELECTOR_CALIB_COMMAND
    args.xu_expected_selector_requires_set = True
    for name, value in (
            ("recovery", False),
            ("get_output_mode", False),
            ("set_output_mode", None),
            ("get_exposure_time", False),
            ("set_exposure_time_us", None),
            ("get_system_gain", False),
            ("set_system_gain", None),
            ("get_bitrate", False),
            ("set_bitrate_kbps", None),
            ("calib_info", False),
            ("calib_status", False),
            ("probe", False),
            ("dry_run", False),
            ("recovery_value", None),
            ("recovery_payload", ""),
            ("out", ""),
            ("json_out", ""),
            ("no_json_out", True),
            ("strict_parse", False),
            ("parse_file", ""),
            ("expected_file", ""),
            ("expected_md5", "")):
        setattr(args, name, value)


def validate_args(args):
    if args.timeout_ms <= 0:
        raise UvcXuError("--timeout-ms must be positive")
    if args.chunk_size <= 0 or args.chunk_size > base.YCTC_XU_TRANSFER_DATA_MAX:
        raise UvcXuError("--chunk-size must be in range 1..{}".format(base.YCTC_XU_TRANSFER_DATA_MAX))
    if args.session_id < 0 or args.session_id > 0xFFFFFFFF:
        raise UvcXuError("--session-id must fit uint32")
    if args.node_id is not None and (args.node_id < 0 or args.node_id > 0xFFFFFFFF):
        raise UvcXuError("--node-id must fit uint32")
    if args.max_blob_size <= 0:
        raise UvcXuError("--max-blob-size must be positive")
    if args.list:
        if args.write_kalibr_yaml:
            raise UvcXuError("--list cannot be combined with --write-kalibr-yaml")
        return
    if not args.write_kalibr_yaml:
        raise UvcXuError("--write-kalibr-yaml is required unless --list is used")
    encode_calibration_serial_number(args.serial_number)
    if args.unlock_token is not None and args.unlock_secret:
        raise UvcXuError("--unlock-token and --unlock-secret are mutually exclusive")
    if args.unlock_token is not None and args.session_id == 0:
        raise UvcXuError("--unlock-token requires an explicit non-zero --session-id")
    if (args.unlock_token is None and not args.unlock_secret and
            not os.environ.get(DEFAULT_UNLOCK_SECRET_ENV, "")):
        raise UvcXuError(
            "set --unlock-token, --unlock-secret, or {} before a calibration write".format(
                DEFAULT_UNLOCK_SECRET_ENV))


def run_device_operation(args):
    backend = base.build_backend(args)
    if args.list:
        base.print_target_list(backend.list_targets())
        return None
    try:
        return run_calib_write_kalibr(backend, args)
    finally:
        backend.close()


def main(argv=None):
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    configure_transport_args(args)
    validate_args(args)
    result = run_device_operation(args)
    if result is not None:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False, indent=2), file=sys.stderr)
        sys.exit(1)
