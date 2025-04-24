import atexit
import logging
import time
from typing import Optional, Sequence, Tuple, Union

import dynamixel_sdk
import numpy as np

# Imports the control table addresses and sizes
# E-manual has descriptions of each data in the control table and value ranges
# E-manual Link: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
from ruka_hand.utils.control_table.control_table import *

# NON-CONTROL TABLE CONTSTANTS

COMM_SUCCESS = 0
PROTOCOL_VERSION = 2.0
BAUDRATE = 2000000

# Change to log file path if not testing
logging.basicConfig(level=logging.WARNING)


def dynamixel_cleanup_handler():
    """Cleanup function to ensure Dynamixels are disconnected properly."""
    open_clients = list(DynamixelClient.OPEN_CLIENTS)
    for open_client in open_clients:
        if open_client.port_handler.is_using:
            logging.warning("Forcing client to close.")
        open_client.port_handler.is_using = False
        open_client.disconnect()


def unsigned_to_signed(value: int, size: int) -> int:
    """Converts the given value from its unsigned representation."""
    bit_size = 8 * size
    if (value & (1 << (bit_size - 1))) != 0:
        value = -((1 << bit_size) - value)
    return value


class DynamixelClient:
    """Client for communicating with Dynamixel motors.

    NOTE: This only supports Protocol 2.
    """

    # The currently open clients.
    OPEN_CLIENTS = set()

    def __init__(
        self,
        motor_ids: Sequence[int],
        port: str = "/dev/ttyUSB0",
        lazy_connect: bool = False,
    ):
        """Initializes a new client.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The Dynamixel device to talk to. e.g.
                - Linux: /dev/ttyUSB0
                - Mac: /dev/tty.usbserial-*
                - Windows: COM1
            lazy_connect: If True, automatically connects when calling a method
                that requires a connection, if not already connected.
        """

        self.dxl = dynamixel_sdk
        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = BAUDRATE
        self.lazy_connect = lazy_connect
        self.port_handler = self.dxl.PortHandler(port)
        self.packet_handler = self.dxl.PacketHandler(PROTOCOL_VERSION)

        # GroupSyncWrite instaces to add/remove params for.
        # https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/api_reference/python/python_groupsyncwrite/#groupsyncwrite
        self._sync_writers = {}

        self.OPEN_CLIENTS.add(self)

    @property
    def is_connected(self) -> bool:
        return self.port_handler.is_open

    def connect(self):
        """Connects to the Dynamixel motors.

        NOTE: This should be called after all DynamixelClients on the same
            process are created.
        """
        assert not self.is_connected, "Client is already connected."

        if self.port_handler.openPort():
            logging.info("Succeeded to open port: %s", self.port_name)
        else:
            raise OSError(
                (
                    "Failed to open port at {} (Check that the device is powered "
                    "on and connected to your computer)."
                ).format(self.port_name)
            )

        if self.port_handler.setBaudRate(self.baudrate):
            logging.info("Succeeded to set baudrate to %d", self.baudrate)
        else:
            raise OSError(
                (
                    "Failed to set the baudrate to {} (Ensure that the device was "
                    "configured for this baudrate)."
                ).format(self.baudrate)
            )

    def disconnect(self):
        """Disconnects from the Dynamixel device."""
        if not self.is_connected:
            return
        if self.port_handler.is_using:
            logging.error("Port handler in use; cannot disconnect.")
            return
        # Ensure motors are disabled at the end.
        self.set_torque_enabled(False)
        self.port_handler.closePort()
        if self in self.OPEN_CLIENTS:
            self.OPEN_CLIENTS.remove(self)

    def check_connected(self):
        """Ensures the robot is connected."""
        if self.lazy_connect and not self.is_connected:
            self.connect()
        if not self.is_connected:
            raise OSError("Must call connect() first.")

    def reboot(self, retries: int = -1, retry_interval: float = 0.25):
        """Reboots all motors.

        Args:
            motor_ids: The motor IDs to configure.
            retries: The number of times to retry. If this is <0, will retry
                forever.
            retry_interval: The number of seconds to wait between retries.
        """
        remaining_ids = self.motor_ids
        while remaining_ids:
            for dxl_id in remaining_ids:
                dxl_comm_result, dxl_error = self.packet_handler.reboot(
                    self.port_handler, dxl_id
                )
                if dxl_comm_result != COMM_SUCCESS:
                    logging.error(
                        "%s" % self.packet_handler.getTxRxResult(dxl_comm_result)
                    )
                elif dxl_error != 0:
                    logging.error(
                        "%s" % self.packet_handler.getRxPacketError(dxl_error)
                    )
                else:
                    logging.info("Dynamixel[ID:%03d] has been rebooted" % dxl_id)
                    remaining_ids.remove(dxl_id)
            if remaining_ids:
                logging.error("Could not reboot for IDs: %s", str(remaining_ids))
            if retries == 0:
                break
            time.sleep(retry_interval)
            retries -= 1

    def handle_packet_result(
        self,
        comm_result: int,
        dxl_error: Optional[int] = None,
        dxl_id: Optional[int] = None,
        context: Optional[str] = None,
    ):
        """Handles the result from a communication request."""
        error_message = None
        if comm_result != COMM_SUCCESS:
            error_message = self.packet_handler.getTxRxResult(comm_result)
        elif dxl_error is not None:
            error_message = self.packet_handler.getRxPacketError(dxl_error)
        if error_message:
            if dxl_id is not None:
                error_message = "[Motor ID: {}] {}".format(dxl_id, error_message)
            if context is not None:
                error_message = "> {}: {}".format(context, error_message)
            logging.error(error_message)
            return False
        return True

    def set_torque_enabled(
        self, enabled: bool, retries: int = -1, retry_interval: float = 0.25
    ):
        """Sets whether torque is enabled for the motors.

        Args:
            enabled: Whether to engage or disengage the motors.
            retries: The number of times to retry. If this is <0, will retry
                forever.
            retry_interval: The number of seconds to wait between retries.
        """
        remaining_ids = list(self.motor_ids)
        while remaining_ids:

            for motor_id in remaining_ids:

                dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, motor_id, ADDR_TORQUE_ENABLE, int(enabled)
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
                    self.packet_handler.reboot(self.port_handler, motor_id)
                elif dxl_error != 0:
                    logging.error(
                        "%s" % self.packet_handler.getRxPacketError(dxl_error)
                    )
                    self.packet_handler.reboot(self.port_handler, motor_id)
                else:
                    remaining_ids.remove(motor_id)
                    logging.info(
                        "Dynamixel#%d has been successfully connected" % motor_id
                    )
            if remaining_ids:
                logging.error(
                    "Could not set torque %s for IDs: %s",
                    "enabled" if enabled else "disabled",
                    str(remaining_ids),
                )

            if retries == 0:
                break
            time.sleep(retry_interval)
            retries -= 1

    def sync_write(
        self,
        motor_ids: Sequence[int],
        values: Sequence[Union[int, float]],
        address: int,
        size: int,
    ):
        """Writes values to a group of motors.

        Args:
            motor_ids: The motor IDs to write to.
            values: The values to write.
            address: The control table address to write to.
            size: The size of the control table value being written to.
        """
        self.check_connected()
        key = (address, size)
        if key not in self._sync_writers:
            self._sync_writers[key] = self.dxl.GroupSyncWrite(
                self.port_handler, self.packet_handler, address, size
            )
        sync_writer = self._sync_writers[key]
        sync_writer.clearParam()

        errored_ids = []
        for motor_id, desired_pos in zip(motor_ids, values):
            value = int(desired_pos)
            value = value.to_bytes(size, byteorder="little")
            success = sync_writer.addParam(motor_id, value)
            if not success:
                errored_ids.append(motor_id)
        if errored_ids:
            logging.error("Sync write failed for: %s", str(errored_ids))

        comm_result = sync_writer.txPacket()
        self.handle_packet_result(comm_result, context="sync_write")

    def sync_read(self, motor_ids: Sequence[int], address: int, size: int):
        """Reads data from a group of motors

        Args:
            motor_ids: The motor IDs to read from
            address: The data's address in the control table to read from
            size: The length of the data being read
        """
        if len(motor_ids) > 3:
            bulk_data = self.sync_read(motor_ids[:3], address, size) + self.sync_read(
                motor_ids[3:], address, size
            )
            return bulk_data
        self.check_connected()
        sync_reader = self.dxl.GroupSyncRead(
            self.port_handler, self.packet_handler, address, size
        )

        errored_ids = []
        for motor_id in motor_ids:
            success = sync_reader.addParam(motor_id)
            if not success:
                errored_ids.append(motor_id)
        if errored_ids:
            logging.error("Sync write failed for: %s", str(errored_ids))

        # Transmit and receive packet and handle result
        comm_result = sync_reader.txRxPacket()
        self.handle_packet_result(comm_result, context="sync_write")

        # Checks whether there is available data in the data storage for each motor
        bulk_data = []
        errored_ids = []
        for motor_id in motor_ids:
            data = None
            available = sync_reader.isAvailable(motor_id, address, size)
            if not available:
                errored_ids.append(motor_id)
            # If available then read it and add it to the bulk data
            else:
                data = sync_reader.getData(motor_id, address, size)
            bulk_data.append(data)
        if errored_ids:
            logging.error("Bulk read data is unavailable for: %s", str(errored_ids))

        sync_reader.clearParam()
        return bulk_data

    # Common read calls

    def read_pos(self):
        return self.sync_read(
            self.motor_ids, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        )

    def read_goal_pos(self):
        return self.sync_read(self.motor_ids, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    def read_cur(self):
        bulk_data = self.sync_read(
            self.motor_ids, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT
        )
        # print(f"bulk_data: {bulk_data}")
        for i in range(len(bulk_data)):
            value = bulk_data[i]
            bulk_data[i] = value - 65536 if value >= 32768 else value
        return bulk_data

    def read_vel(self):
        return self.sync_read(
            self.motor_ids, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY
        )

    # Common write calls

    def set_pos(self, values):
        self.sync_write(
            list(self.motor_ids), values, ADDR_GOAL_POSITION, LEN_GOAL_POSITION
        )

    def set_pos_indv(self, motor_id, value):
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, ADDR_GOAL_POSITION, value
        )
        if dxl_comm_result != COMM_SUCCESS:
            logging.error("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            logging.error("%s" % self.packet_handler.getRxPacketError(dxl_error))
        else:
            logging.info(
                "Dynamixel %d has been successfully set position %d" % (motor_id, value)
            )

    def single_write(self, motor_id, value, addr):
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, addr, value
        )

        print(f"dxl_comm_result: {dxl_comm_result}, dxl_error: {dxl_error}")
        if dxl_comm_result != COMM_SUCCESS:
            logging.error("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            logging.error("%s" % self.packet_handler.getRxPacketError(dxl_error))
        else:
            logging.info(
                "Dynamixel %d has been successfully set gain %d" % (motor_id, value)
            )

    def single_read(self, motor_id, addr):

        value, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, addr
        )
        if result != COMM_SUCCESS:
            print(
                f"Failed to read at address {addr}: {self.packet_handler.getTxRxResult(result)}"
            )
        elif error != 0:
            print(
                f"Error at address {addr}: {self.packet_handler.getRxPacketError(error)}"
            )
        else:
            return value

    def read_single_cur(self, motor_id):
        # Read the 2-byte value from the specified address
        value, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, ADDR_PRESENT_CURRENT
        )
        if result != COMM_SUCCESS:
            print(
                f"Failed to read at address {ADDR_PRESENT_CURRENT}: {self.packet_handler.getTxRxResult(result)}"
            )
            return None
        elif error != 0:
            print(
                f"Error at address {ADDR_PRESENT_CURRENT}: {self.packet_handler.getRxPacketError(error)}"
            )
            return None
        else:
            # Convert the value to a signed 16-bit integer
            signed_value = value - 65536 if value >= 32768 else value
            return signed_value

    def __enter__(self):
        """Enables use as a context manager."""
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, *args):
        """Enables use as a context manager."""
        self.disconnect()

    def __del__(self):
        """Automatically disconnect on destruction."""
        self.disconnect()


# Register global cleanup function.
atexit.register(dynamixel_cleanup_handler)

# Command line controls for testing
if __name__ == "__main__":
    import argparse
    import itertools

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-m", "--motors", required=True, help="Comma-separated list of motor IDs."
    )
    parser.add_argument(
        "-d",
        "--device",
        default="/dev/tty.usbserial-FT8ISQ5P",
        help="The Dynamixel device to connect to.",
    )
    parser.add_argument(
        "-b", "--baud", default=57600, help="The baudrate to connect with."
    )
    parsed_args = parser.parse_args()

    motors = [int(motor) for motor in parsed_args.motors.split(",")]

    with DynamixelClient(motors, parsed_args.device, parsed_args.baud) as dxl_client:
        init_pos = np.array(
            [1000, 1040, 600, 1500, 800, 800, 1200, 1200, 2900, 2700, 2800]
        )
        fist_pos = np.array(
            [2700, 1500, 1500, 500, 1500, 1700, 2200, 2000, 2000, 1800, 2000]
        )
        fist_pos_6to11 = np.array([1700, 2200, 2000, 2000, 1800, 2000])
        dxl_client.set_torque_enabled(True, -1, 0.05)
        time.sleep(4)
        torque_now = dxl_client.sync_read(motors, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE)
        dxl_client.set_pos(init_pos)
        time.sleep(0.1)
        print("> Torque: {}".format(torque_now))
        for step in itertools.count():
            read_start = time.time()
            goal_pos = dxl_client.read_goal_pos()

            # Increases position every 50 steps
            if step > 0 and step % 50 == 0 and step < 500:
                for i in range(len(goal_pos)):
                    if i < 9:
                        goal_pos[i] += 100
                    else:
                        goal_pos[i] -= 100
                dxl_client.set_pos(goal_pos)

            pos_now = dxl_client.read_pos()
            cur_now = dxl_client.read_cur()
            if step % 5 == 0:
                elapsed = time.time() - read_start
                print("[{}] Frequency: {:.2f} Hz".format(step, 1.0 / (elapsed)))
                print("> Present Pos: {}".format(pos_now))
                print("> Preasent Current: {}".format(cur_now))
