#!/usr/bin/env python
import signal
import sys
from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


class IsseCrazyCopter(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.copterPosition = [0., 0., 0.]
        self.direction = [1.,1.,1.]
        self.functionality = {"arm": None, "disarm": None, "get_pos": None, "set_pos": None,
                              "get_arming": None, "setNeoPixelColor": None, "get_name": None, "set_name": None,
                              "get_rot": None, "set_rot": None,
                              "rotate": None}
    def SetArmingStatus(self, params: dict):
        formatPrint(self, f"Set Arming Status to {params}")
        p = params["SimpleBooleanParameter"]
        if p and self.functionality["arm"] is not None:
            self.functionality["arm"]()
        elif not p and self.functionality["disarm"] is not None:
            self.functionality["disarm"]()
        return params

    def GetArmingStatus(self, params: dict):
        if self.functionality["GetArmingStatus"] is not None:
            return {"SimpleBooleanParameter": self.functionality["get_arming"]()}
        return {"SimpleBooleanParameter": False}

    def SetPosition(self, params: dict) -> dict:
        try:
            p = params["Position3D"]
        except:
            return self.GetPosition(params)
        formatPrint(self, f"Flying to {p}")
        if self.functionality["set_pos"] is not None:
            self.functionality["set_pos"](p)
        else:
            pass
        return self.GetPosition({})

    def GetPosition(self, params: dict) -> dict:
        if self.functionality["get_pos"] is not None:
            pos = self.functionality["get_pos"]()
            self.copterPosition = pos
        return {"Position3D": self.copterPosition}

    def FlyToPosition(self, params: dict) -> dict:
        formatPrint(self, f"Flying to position {params}")
        return self.SetPosition(params)

    def setNeoPixelColor(self, params: dict) -> dict:
        if self.functionality["get_pos"] is None:
            return {}
        r = params["Red"]
        g = params["Green"]
        b = params["Blue"]
        deviceID = params["DeviceID"]
        if deviceID == "CrazyFly":
            if r is not None and g is not None and b is not None:
                self.functionality["setNeoPixelColor"](r, g, b)
        return {}

    def Settf_name(self, params: dict):
        tf_name = params["SimpleStringParameter"]
        if self.functionality["set_name"] is not None:
            self.position = self.functionality["set_name"](tf_name)
        return {"SimpleStringParameter": tf_name}

    def Gettf_name(self, params: dict):
        tf_name = "NO_ROS_CONNECTION"
        if self.functionality["get_name"] is not None:
            tf_name = self.position = self.functionality["get_name"]()
        return {"SimpleStringParameter": tf_name}

    def SetRotation(self, params: dict):
        quat = params["Quaternion"]
        if self.functionality["set_rot"] is not None:
            self.functionality["set_rot"](quat)
        return {"Quaternion": quat}

    def GetRotation(self, params: dict):
        quat = [0, 0, 0, 0]
        if self.functionality["get_name"] is not None:
            quat = self.functionality["get_rot"]()
        return {"Quaternion": quat}

    def RotateAroundAxis(self, params: dict):
        axis = params["Axis"]
        if axis == 'z':
            axis = [0, 0, 1]
        elif axis == 'y':
            axis = [0, 1, 0]
        elif axis == 'x':
            axis = [1, 0, 0]
        degree = params["SimpleDoubleParameter"]
        if self.functionality["get_name"] is not None:
            quat = self.functionality["rotate"](axis, degree)
        formatPrint(self, f"New Quaternion {quat}")
        return {"Quaternion": quat}

    def GetDirection(self, params: dict):
        return {"Vector3": self.direction}

    def SetDirection(self, params: dict):
        new_direction = params["Vector3"]
        self.direction = new_direction
        return self.GetDirection()

    def loop(self):
        pass


if __name__ == "__main__":
    try:
        port = None
        if len(sys.argv[1:]) > 0:
            port = int(sys.argv[1])
        server = VirtualCapabilityServer(port)
        cf = IsseCrazyCopter(server)
        cf.start()

        def signal_handler(sig, frame):
            cf.kill()
            server.kill()
        signal.signal(signal.SIGINT, signal_handler)
        cf.join()
        server.join()
        signal.pause()

        # Needed for properly closing, when program is being stopped with a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
