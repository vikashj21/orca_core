#!/usr/bin/env python
"""Change one isolated Feetech motor's ID without changing its baud or mode.

Power off before connecting ONLY the motor to change, then restore power.
Example: uv run python scripts/change_feetech_id.py --port /dev/ttyUSB0 --new-id 17
Read-only scan: uv run python scripts/change_feetech_id.py --port /dev/ttyUSB0 --diagnose
"""

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from orca_core.hardware.feetech import (
    COMM_SUCCESS,
    INST_PING,
    PortHandler,
    SMS_STS_ID,
    SMS_STS_BAUD_RATE,
    SMS_STS_LOCK,
    SMS_STS_MODE,
    SMS_STS_TORQUE_ENABLE,
    sms_sts,
)
from orca_core.hardware.feetech_client import FEETECH_BAUD_RATE_MAP, FEETECH_MODELS


def motor_id(value: str) -> int:
    result = int(value)
    if not 0 <= result <= 253:
        raise argparse.ArgumentTypeError("motor ID must be between 0 and 253")
    return result


def checked(result: tuple[int, int], operation: str) -> None:
    comm, error = result
    if comm != COMM_SUCCESS or error:
        raise RuntimeError(f"{operation} failed: communication={comm}, motor error={error}")


def diagnose(port: str, baudrates, first_id: int, last_id: int) -> int:
    """Ping and read registers only; never connect through the motor client."""
    found = 0
    issues = False
    baudrates = list(baudrates)
    print(f"\nFeetech motor check — {port}")
    print("Read-only: motor settings stay unchanged.")
    if len(baudrates) * (last_id - first_id + 1) > 253:
        print("Scanning all these addresses can take several minutes; Ctrl+C stops the scan.")
    registers = {
        "stored_id": SMS_STS_ID,
        "baud_code": SMS_STS_BAUD_RATE,
        "torque": SMS_STS_TORQUE_ENABLE,
        "mode": SMS_STS_MODE,
        "eeprom_lock": SMS_STS_LOCK,
    }
    for baud in baudrates:
        print(f"\nScanning IDs {first_id}-{last_id} at {baud:,} bps...", flush=True)
        print(f"  {'ID':<5}{'Model':<20}{'Torque':<12}{'Mode':<14}{'Memory':<12}Result", flush=True)
        found_at_baud = 0
        handler = PortHandler(port)
        handler.baudrate = baud
        try:
            if not handler.openPort():
                raise RuntimeError(f"Cannot open {port} at {baud} bps")
            packet = sms_sts(handler)
            for candidate in range(first_id, last_id + 1):
                # SDK ping() also reads the model number and masks a valid
                # ping reply when that subsequent register read fails.
                _, comm, error = packet.txRxPacket([0, 0, candidate, 2, INST_PING, 0])
                if comm != COMM_SUCCESS:
                    handler.ser.reset_input_buffer()
                    continue
                found += 1
                found_at_baud += 1
                notes = [f"Motor reported error code {error}."] if error else []
                model, model_comm, model_error = packet.read2ByteTxRx(candidate, 3)
                if model_comm != COMM_SUCCESS or model_error:
                    handler.ser.reset_input_buffer()
                    model = None
                    notes.append("Ping replied, but model number could not be read.")
                values = {}
                for label, address in registers.items():
                    value, read_comm, read_error = packet.read1ByteTxRx(candidate, address)
                    if read_comm != COMM_SUCCESS or read_error:
                        handler.ser.reset_input_buffer()
                        values[label] = None
                        notes.append(f"{label.replace('_', ' ')}: READ FAILED "
                                     f"(communication={read_comm}, motor error={read_error}).")
                    else:
                        values[label] = value
                if (isinstance(values["stored_id"], int) and values["stored_id"] != candidate
                        or isinstance(values["baud_code"], int)
                        and values["baud_code"] != FEETECH_BAUD_RATE_MAP[baud]):
                    notes.append(f"Stored ID/baud differs from responding settings "
                                 f"(stored ID={values['stored_id']}, baud code={values['baud_code']}).")
                if values["eeprom_lock"] == 0:
                    notes.append("EEPROM is unlocked; diagnostic leaves it unchanged.")
                def setting(label, names):
                    value = values[label]
                    return "Unreadable" if value is None else names.get(value, f"Other ({value})")

                torque = setting("torque", {0: "Off", 1: "On"})
                mode = setting("mode", {0: "Servo", 1: "Wheel"})
                memory = setting("eeprom_lock", {0: "Unlocked", 1: "Locked"})
                model_name = ("Unreadable" if model is None
                              else FEETECH_MODELS.get(model, f"Unknown ({model})"))
                issues |= bool(notes)
                print(f"  {candidate:<5}{model_name:<20}{torque:<12}{mode:<14}{memory:<12}"
                      f"{'Check' if notes else 'OK'}", flush=True)
                for note in notes:
                    print(f"    ID {candidate}: {note}", flush=True)
            if not found_at_baud:
                print("  No motors responded at this speed.")
        finally:
            if handler.ser is not None:
                handler.closePort()
    if not found:
        print("No replies. Check power, wiring, port, and scan range; this does not prove memory corruption.")
        return 1
    print(f"\nScan complete: {found} motor response{'s' if found != 1 else ''} found.")
    print("Some checks need attention; see the notes above." if issues
          else "All responding motors passed the communication and ID/baud checks.")
    print("No settings changed. Motor movement and full memory integrity were not tested.")
    print("If expected motors are missing, check them individually: shared IDs can hide motors.")
    return 1 if issues else 0


def change_id(packet, current_id: int, new_id: int) -> None:
    model, comm, error = packet.ping(current_id)
    checked((comm, error), f"Ping current ID {current_id}")
    if current_id == new_id:
        print(f"Motor already responds at ID {new_id}; no settings written.")
        return

    _, comm, _ = packet.ping(new_id)
    if comm == COMM_SUCCESS:
        raise RuntimeError(f"ID {new_id} is already occupied; no settings written.")

    checked(packet.write1ByteTxRx(current_id, SMS_STS_TORQUE_ENABLE, 0), "Disable torque")
    checked(packet.unLockEprom(current_id), "Unlock EEPROM")
    # An interrupted/failed write may leave either ID active. Attempt to lock
    # whichever responds, including when the write acknowledgement is lost.
    try:
        checked(packet.write1ByteTxRx(current_id, SMS_STS_ID, new_id), "Write ID")
    finally:
        time.sleep(0.5)
        lock_id = None
        for candidate in (new_id, current_id):
            _, comm, error = packet.ping(candidate)
            if comm == COMM_SUCCESS and error == 0:
                lock_id = candidate
                break
        if lock_id is None:
            raise RuntimeError("Neither ID responds after writing; EEPROM lock could not be verified.")
        checked(packet.LockEprom(lock_id), f"Lock EEPROM at ID {lock_id}")

    found_model, comm, error = packet.ping(new_id)
    checked((comm, error), f"Verify new ID {new_id}")
    stored_id, comm, error = packet.read1ByteTxRx(new_id, SMS_STS_ID)
    checked((comm, error), "Read back ID")
    if stored_id != new_id or found_model != model:
        raise RuntimeError("Motor ID/model verification failed.")
    _, comm, _ = packet.ping(current_id)
    if comm == COMM_SUCCESS:
        raise RuntimeError("The old ID still responds. Check for multiple connected motors.")
    print(f"Verified ID {current_id} -> {new_id}. Baud and mode unchanged; torque remains off.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Motor adapter serial port")
    parser.add_argument("--current-id", type=motor_id, default=1)
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument("--new-id", type=motor_id)
    operation.add_argument("--diagnose", action="store_true", help="Read-only scan; no ID change")
    parser.add_argument("--min-id", type=motor_id, default=0, help="Diagnostic scan start (default: 0)")
    parser.add_argument("--max-id", type=motor_id, default=253, help="Diagnostic scan end (default: 253)")
    parser.add_argument("--baudrate", type=int,
                        choices=FEETECH_BAUD_RATE_MAP,
                        help="Existing baud: defaults to 1000000 for ID changes, all rates for diagnosis")
    args = parser.parse_args()
    if args.min_id > args.max_id:
        parser.error("--min-id must be <= --max-id")
    if args.diagnose:
        try:
            return diagnose(args.port, [args.baudrate] if args.baudrate else FEETECH_BAUD_RATE_MAP,
                            args.min_id, args.max_id)
        except (OSError, RuntimeError) as exc:
            print(f"Diagnostic error: {exc}", file=sys.stderr)
            return 1
        except KeyboardInterrupt:
            print("Diagnostic interrupted; no motor settings were written.")
            return 130
    handler = PortHandler(args.port)
    handler.baudrate = args.baudrate or 1_000_000
    try:
        if not handler.openPort():
            raise RuntimeError(f"Cannot open {args.port}")
        change_id(sms_sts(handler), args.current_id, args.new_id)
        return 0
    except (OSError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        if handler.ser is not None:
            handler.closePort()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit("Interrupted. The ID may already have changed; check before retrying.")
