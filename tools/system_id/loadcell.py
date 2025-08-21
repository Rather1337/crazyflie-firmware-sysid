"""Class to calibrate and read data from a ATI loadcell via EtherCAT."""

import time
import struct
import pysoem
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import os


class Loadcell:
    def __init__(self, interface: str, slave_id: int = 0, verbose: bool = False):
        self._verbose = verbose

        self._ethercat_master = pysoem.Master()
        self._ethercat_master.open(interface)

        if self._ethercat_master.config_init() <= 0:
            raise RuntimeError("No EtherCAT slave found")
        self._ethercat_master.config_map()
        self._ethercat_master.config_dc()  # optional, for sync

        self._slave_id = slave_id
        self.slave = self._ethercat_master.slaves[self._slave_id]
        self._get_counts_per_force()

        self._interface = interface
        self._slave_id = slave_id

        self._offset = np.zeros(6)  # fx, fy, fz, mx, my, mz
        self._counts_per_SI = np.array(
            [
                self._cp_force,
                self._cp_force,
                self._cp_force,
                self._cp_torque,
                self._cp_torque,
                self._cp_torque,
            ]
        )

    def _get_counts_per_force(self):
        cp_force_raw = self.slave.sdo_read(0x2040, 0x31, 4)  # count per force
        cp_torque_raw = self.slave.sdo_read(0x2040, 0x32, 4)  # count per torque
        self._cp_force = struct.unpack("<I", cp_force_raw)[0]
        self._cp_torque = struct.unpack("<I", cp_torque_raw)[0]
        if self._verbose:
            print(f"Counts/Force: {self._cp_force}, Counts/Torque: {self._cp_torque}")

    def calibrate(self):
        """Calibrate the offset of the loadcell."""
        if self._verbose:
            print("Calibrating loadcell. Don't apply any force.")
        self._offset = np.zeros(6)
        measurements = []
        for _ in range(100):
            measurements.append(self.read_data())
            time.sleep(0.01)  # wait for stable readings

        self._offset = np.mean(measurements, axis=0)

        if self._verbose:
            print(f"Calibration complete. Offset: {self._offset}")

    def read_data(self):
        """Read data from the loadcell."""
        self._ethercat_master.send_processdata()
        self._ethercat_master.receive_processdata(2000)

        # 0x6000 readings are typically in slave inputs; check byte offset
        # For example, assuming input mapping starts right away:

        # print(f"{master.slaves[0].input=}")
        data = self.slave.input  # raw byte array
        # Parse Fx, Fy, Fz, Tx, Ty, Tz (6 × int32)
        fx, fy, fz, tx, ty, tz = struct.unpack("<6i", data[0:24])
        self._reading = (
            np.array([fx, fy, fz, tx, ty, tz]) / self._counts_per_SI - self._offset
        )

        if self._verbose:
            txt = f"Fx: {self._reading[0]:.3f}, Fy: {self._reading[1]:.3f}, Fz: {self._reading[2]:.3f}, "
            txt += f"Tx: {self._reading[3]:.3f}, Ty: {self._reading[4]:.3f}, Tz: {self._reading[5]:.3f}"
            print(txt)

        return self._reading

    @staticmethod
    def list_interfaces():
        """List available EtherCAT interfaces."""
        print("Interfaces found:", pysoem.find_adapters())

    @staticmethod
    def list_slaves(interface: str):
        """List EtherCAT slaves on the specified interface."""
        master = pysoem.Master()
        master.open(interface)
        slaves = master.config_init()
        print("Number of slaves found:", slaves)
        if slaves:
            for i, slave in enumerate(master.slaves):
                print(f"Slave {i}: {slave.name}")
        master.close()


# Short test (Needs to be executed as sudo)
if __name__ == "__main__":
    Loadcell.list_interfaces()
    Loadcell.list_slaves("enp0s31f6")  # Replace with your actual interface

    loadcell = Loadcell("enp0s31f6", verbose=True)  # Replace with your actual interface
    loadcell.read_data()  # Read data from the loadcell
    loadcell.calibrate()  # Calibrate the loadcell

    while True:
        reading = loadcell.read_data()
        print(reading)
        time.sleep(0.01)
