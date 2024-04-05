import trio
from typing import Tuple, Callable, Any, List, Optional, Dict, Union
from trio import sleep, sleep_until
import json
import time
from functools import partial
from dataclasses import dataclass


def warning(text: str):
    color = "\033[33m"
    reset = "\033[0m"
    print(f"{color}[{time.time()-start_time:.3f} WARNING] {text}{reset}")


def log(text: str):
    color = "\033[36m"
    reset = "\033[0m"
    print(f"{color}[{time.time() - start_time:.3f} LOG] {text}{reset}")


@dataclass
class TcpPort:
    """Convenience class to package all data relating to a TCP port maintained by the server. A port has a number,
    an informal name (corresponding to the same in skybrushd.jsonc), a function that gets called when a client
    connects to it, and a list of the streams connected to it."""
    port: int
    name: str
    func: Callable
    streams: List[trio.SocketStream]


class DroneHandler:
    def __init__(self, uav_id: str, stream: trio.SocketStream, color: str):
        self.traj = b''
        self.stream_data = b''
        self.transmission_active = False
        self.uav_id = uav_id
        self.stream = stream
        self.color = color

    def print(self, text):
        reset_color = "\033[0m"
        print(f"{self.color}[drone_{self.uav_id}]: {text}{reset_color}")

    async def takeoff(self, arg: bytes):
        try:
            arg = float(arg)
            self.print(f"Takeoff command dispatched to drone.")
            await sleep(0.01)
            await self.stream.send_all(b'ACK')  # reply with an acknowledgement
        except ValueError:
            warning("Takeoff argument is not a float.")
        except Exception as exc:
            warning(f"drone{self.uav_id}: Couldn't take off because of this exception: {exc!r}. ")

    async def land(self, arg: bytes):
        self.print(f"Land command dispatched..")
        await self.stream.send_all(b'ACK')  # reply with an acknowledgement

    async def handle_transmission(self):
        self.print(f"Transmission of trajectory started.")
        start_index = self.stream_data.find(b'{')
        # If the command was 'upload', then a json file must follow. If it doesn't (we can't find the beginning b'{'),
        # then the command or the file was corrupted.
        if start_index == -1:
            warning("Corrupted trajectory file.")
        else:
            self.traj = self.stream_data[start_index:]
            self.transmission_active = True  # signal we're in the middle of transmission
            while not self.traj.endswith(b'_EOF'):  # receive data until we see that the file has ended
                self.traj += await self.stream.receive_some()  # append the new data to the already received data
            self.traj = self.traj[:-len(b'_EOF')]  # once finished, remove the EOF indicator
            self.transmission_active = False  # then signal that the transmission has ended
            self.print(f"Transmission of trajectory finished.")

    async def upload(self, arg: bytes):
        await self.handle_transmission()
        trajectory_data = json.loads(self.traj.decode('utf-8'))
        f"Defined trajectory of length {trajectory_data.get('landingTime')} sec for drone {self.uav_id}"
        # await sleep(0.5)
        await self.stream.send_all(b'ACK')  # reply with an acknowledgement

    async def start(self, arg: bytes):
        is_valid, is_relative = self.get_traj_type(self, arg=arg)
        if is_valid:
            self.print(f"Started {'relative' if is_relative else 'absolute'} trajectory.")
            await self.stream.send_all(b'ACK')  # reply with an acknowledgement

    async def hover(self, arg: bytes):
        self.print(f"Hover command dispatched.")
        await self.stream.send_all(b'ACK')  # reply with an acknowledgement

    async def set_param(self, arg: bytes):
        param, value = arg.split(b'=')
        param = param.decode()
        value = float(value)
        self.print(f"setting param {param} to {value}")
        await self.stream.send_all(b'ACK')  # reply with an acknowledgement

    async def command(self, cmd: bytes, arg: bytes):
        self.print(f"Command received: {cmd.decode('utf-8')}")
        # await self.stream.send_all(b'Command received: ' + cmd)
        await self.tcp_command_dict[cmd][0](self, arg)

    async def listen(self):
        while True:
            if not self.transmission_active:
                try:
                    self.stream_data: bytes = await self.stream.receive_some()
                    if not self.stream_data:
                        break
                    cmd, arg = self.parse(self.stream_data)
                    if cmd == b'NO_CMDSTART':
                        self.print(f"Command is missing standard CMDSTART")
                        break
                    elif cmd == b'WRONG_CMD':
                        self.print(f"Command is not found in server side dictionary")
                        break
                    elif cmd is None:
                        warning(f"None-type command.")
                        break
                    else:
                        await self.command(cmd, arg)
                except Exception as exc:
                    warning(f"TCP handler crashed: {exc!r}")
                    break

    tcp_command_dict: Dict[
        bytes, Tuple[Callable[[Any, bytes], None], bool]] = {
        b"takeoff": (takeoff, True),
        b"land": (land, False),
        b"upload": (upload, True),
        b"hover": (hover, False),
        b"start": (start, True),
        b"param": (set_param, True)
    }

    def parse(self, raw_data: bytes, ) -> Tuple[Union[bytes, None], Union[bytes, None]]:
        data = raw_data.strip()
        if not data:
            return None, None
        data = data.split(b'_')
        if data[0] != b'CMDSTART':
            return b'NO_CMDSTART', None
        command = data[1]
        if command not in self.tcp_command_dict:
            return b'WRONG_CMD', None
        if self.tcp_command_dict[command][1]:  # This is a boolean signifying whether we expect an argument
            argument = data[2]
        else:
            argument = None
        return command, argument


class Server:
    """put stuff here that would be here in the server: mimic as closely as possible"""
    def __init__(self, object_registry: List[str]):
        self.object_registry: List[str] = object_registry
        self.drone_handlers: List[DroneHandler] = []
        self.PORT = 7000
        self.car_streams: List[trio.SocketStream] = []
        self.simulation_streams: List[trio.SocketStream] = []
        self.colors = {"04": "\033[92m",
                       "06": "\033[93m",
                       "07": "\033[94m",
                       "08": "\033[96m",
                       "09": "\033[95m"}
        # this would be read from skybrushd.jsonc:
        self.tcp_port_dict = {"drone": 7000,
                              "car": 7001,
                              "sim": 7002,
                              "lqr": 7003}
        self.ports: Dict[int, TcpPort] = {}

    def _set_port_func(self, port_name: str, func: Callable):
        try:
            port_num = self.tcp_port_dict[port_name]
            if port_num in self.ports:  # means we need to save its streams
                streams: List[trio.SocketStream] = self.ports[port_num].streams
                self.ports[port_num] = TcpPort(port=port_num, name=port_name, func=func, streams=streams)
            else:
                self.ports[port_num] = TcpPort(port=port_num, name=port_name, func=func, streams=[])
        except KeyError:
            warning(f"No such port in configuration: {port_name}")

    async def establish_drone_handler(self, drone_stream: trio.SocketStream, port: int):
        # when a client is trying to connect (i.e. a script wants permission to give commands to a drone),
        # we must check what uavs are recognised by the server:
        uav_ids = self.object_registry
        # take away the ones that already have a handler
        taken_ids = [handler.uav_id for handler in self.drone_handlers]
        # and what's left is the drones up for grabs
        available_ids = [drone_id for drone_id in uav_ids if drone_id not in taken_ids]
        if len(available_ids) != 0:  # if there are free drones, tell the user
            log(f"TCP connection made. Valid drone IDs: {uav_ids}. "
                f"Of these the following are not yet taken: {available_ids}")
            request = await drone_stream.receive_some()
            request = request.decode('utf-8')
            # The client will send a request for a drone handler beginning with REQ_, i.e.: REQ_06
            if 'REQ_' in request:
                requested_id = request.split('REQ_')[1]
                if requested_id not in available_ids:
                    warning(f"The requested ID isn't available.")
                    await drone_stream.send_all(b'ACK_00')
                    return
            else:
                warning(f"Wrong request.")
                await drone_stream.send_all(b'ACK_00')
                return
            color = self.colors[requested_id]
            handler = DroneHandler(requested_id, drone_stream, color)
            self.drone_handlers.append(handler)
            log(f"Made handler for drone {requested_id}. "
                f"Currently we have handlers for the following drones: "
                f"{[handler.uav_id for handler in self.drone_handlers]}")
            acknowledgement = f"ACK_{requested_id}"
            await drone_stream.send_all(acknowledgement.encode('utf-8'))
            await handler.listen()
            self.drone_handlers.remove(handler)
            log(f"Removing handler for drone {handler.uav_id}. Remaining handlers:"
                f"{[drone_handler.uav_id for drone_handler in self.drone_handlers]}")
        else:  # if there aren't any drones left, tell them that
            log(f"All drone IDs are accounted for.")
            await drone_stream.send_all(b'ACK_00')

    async def broadcast(self, stream: trio.SocketStream, port: int):
        streams = self.ports[port].streams
        streams.append(stream)
        log(f"Number of connections on port {port} changed to {len(streams)}")
        while True:
            try:
                data = await stream.receive_some()
                if data:
                    for target_stream in [other_stream for other_stream in streams if other_stream != stream]:
                        await target_stream.send_all(data)
                else:
                    break
            except trio.BrokenResourceError:
                break
        streams.remove(stream)
        log(f"Number of connections on port {port} changed to {len(streams)}")

    async def stream_lqr_to_cf(self):
        pass  #TODO

    def configure(self):
        self._set_port_func("drone", self.establish_drone_handler)
        self._set_port_func("car", self.broadcast)
        self._set_port_func("sim", self.broadcast)
        #  self._set_port_func("lqr", self.stream_lqr_to_cf)

    async def run(self):
        async with trio.open_nursery() as nursery:
            for port_num, tcp_port in self.ports.items():
                handler = partial(tcp_port.func, port=port_num)
                nursery.start_soon(partial(trio.serve_tcp, handler=handler, port=port_num, handler_nursery=nursery))
        start = None
        while start != "start":
            start = await trio.to_thread.run_sync(input, 'Type "start" to simulate a skyc start!\n')
        try:
            for stream in self.car_streams:
                print("STARTING CAR WROOM WROOM")
                await stream.send_all(b'6')
            for stream in self.simulation_streams:
                print("START SIMULATION!")
                await stream.send_all(b'00_CMDSTART_show_EOF')
        except Exception as exc:
            print(f"Exception: {exc!r}")


if __name__ == "__main__":
    DummyServer = Server(object_registry=["01", "02", "06", "07", "08", "09"])
    DummyServer.configure()  # in ctor?
    start_time = time.time()
    log("DUMMY SERVER READY! :)")
    trio.run(DummyServer.run)

