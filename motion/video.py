import asyncio
import atexit
import os
import socket

import numpy as np
import omni.ext
import omni.kit.app
import omni.usd
import toml
from omni.isaac.core import World
from omni.isaac.sensor import Camera


class MotionVideoExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()

        self.config = {
            "camera": None,
            "container": "motion-video-extension",
            "image": "ghcr.io/yongtang/gstreamer:latest",
            "server": "ws://127.0.0.1:7880",
            "stream": None,
            "token": None,
            "relay": False,
        }

        try:
            ext_manager = omni.kit.app.get_app().get_extension_manager()
            ext_id = ext_manager.get_extension_id_by_module(__name__)
            ext_path = ext_manager.get_extension_path(ext_id)
            config = os.path.join(ext_path, "config", "config.toml")
            print("[MotionVideoExtension] Extension config: {}".format(config))
            config = toml.load(config)
            print("[MotionVideoExtension] Extension config: {}".format(config))
            self.config["camera"] = (
                config.get("camera", self.config["camera"]) or self.config["camera"]
            )
            self.config["container"] = (
                config.get("container", self.config["container"])
                or self.config["container"]
            )
            self.config["server"] = (
                config.get("server", self.config["server"]) or self.config["server"]
            )
            self.config["stream"] = (
                config.get("stream", self.config["stream"]) or self.config["stream"]
            )
            self.config["token"] = (
                config.get("token", self.config["token"]) or self.config["token"]
            )
            self.config["relay"] = (
                config.get("relay", self.config["relay"]) or self.config["relay"]
            )
        except Exception as e:
            print("[MotionVideoExtension] Extension config: {}".format(e))

        assert self.config[
            "camera"
        ], "[MotionVideoExtension] Extension config: no camera"
        assert self.config["token"], "[MotionVideoExtension] Extension config: no token"

        print("[MotionVideoExtension] Extension config: {}".format(self.config))

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def on_startup(self, ext_id):
        async def v(self):
            try:
                while self.running:
                    try:
                        if hasattr(self, "camera") and self.camera:
                            image = self.camera.get_rgba()
                            if len(image):
                                image = np.array(image, dtype=np.uint8)  # RGBA
                                assert image.shape[-1] == 4, "camera {}".format(
                                    image.shape
                                )
                                image = image[
                                    :, :, :3
                                ]  # Remove alpha channel, keep RGB
                                assert image.shape[-1] == 3, "camera {}".format(
                                    image.shape
                                )

                                # buffer = io.BytesIO()
                                # PIL.Image.fromarray(image, mode="RGB").save(
                                #     buffer, format="JPEG2000", quality_mode="lossless"
                                # )
                                # self.socket.sendto(buffer.getvalue(), ("127.0.0.1", 6000))

                                self.socket.sendto(image.tobytes(), ("127.0.0.1", 6000))

                                # width, height = 128, 128
                                # header = f"P6\n{width} {height}\n255\n".encode()
                                # self.socket.sendto(
                                #     header + image.tobytes(), ("127.0.0.1", 6000)
                                # )
                    except asyncio.CancelledError:
                        raise
                    except Exception as e:
                        print("[MotionVideoExtension] Extension camera: {}".format(e))
                    await asyncio.sleep(1 / 30)  # Maintain ~30 FPS
            except asyncio.CancelledError:
                print("[MotionVideoExtension] Extension camera cancel")
            finally:
                print("[MotionVideoExtension] Extension camera exit")

        async def g(self):
            def k(self):
                print("[MotionVideoExtension] Extension exit start")
                if getattr(self, "process") and self.process is not None:
                    try:
                        print("[MotionVideoExtension] Extension container: kill")
                        os.killpg(os.getpgid(container_process.pid), signal.SIGTERM)
                        time.sleep(3)
                        os.killpg(os.getpgid(container_process.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        print("[MotionVideoExtension] Extension container: none")
                print("[MotionVideoExtension] Extension exit final")

            atexit.register(k, self)
            try:
                while self.running:
                    try:
                        print("[MotionVideoExtension] Extension container remove start")
                        p = [
                            "rm",
                            "-f",
                            self.config["container"],
                        ]
                        print(
                            "[MotionVideoExtension] Extension container remove {}".format(
                                p
                            )
                        )

                        self.process = await asyncio.create_subprocess_exec(
                            "docker",
                            *p,
                            stdout=asyncio.subprocess.PIPE,
                            stderr=asyncio.subprocess.STDOUT,
                            preexec_fn=os.setsid,
                        )

                        async for line in self.process.stdout:
                            print(
                                "[MotionVideoExtension] Extension container: {}".format(
                                    line.decode().strip()
                                )
                            )

                        await self.process.wait()
                        print("[MotionVideoExtension] Extension container remove final")

                        print("[MotionVideoExtension] Extension container launch start")
                        p = (
                            [
                                "run",
                                "-i",
                                "--rm",
                                "--net=host",
                                "--env=GST_DEBUG=2",
                                "--env=GST_DEBUG_NO_COLOR=1",
                                "--name",
                                self.config["container"],
                                self.config["image"],
                                "gst-launch-1.0",
                                "-v",
                                "udpsrc",
                                "port=6000",
                                "address=127.0.0.1",
                                "!",
                                "videoparse",
                                "width=128",
                                "height=128",
                                "format=rgb",
                                "framerate=30/1",
                                "!",
                                "videoconvert",
                                "!",
                                "video/x-raw,width=128,height=128,framerate=30/1,bitrate=250000",
                                "!",
                                "whipclientsink",
                                "signaller::whip-endpoint={}".format(
                                    self.config["stream"]
                                ),
                                "signaller::auth-token={}".format(self.config["token"]),
                            ]
                            if self.config["relay"]
                            else [
                                "run",
                                "-i",
                                "--rm",
                                "--net=host",
                                "--env=GST_DEBUG=2",
                                "--env=GST_DEBUG_NO_COLOR=1",
                                "--name={}".format(self.config["container"]),
                                self.config["image"],
                                "gst-launch-1.0",
                                "-v",
                                "udpsrc",
                                "port=6000",
                                "address=127.0.0.1",
                                "caps=video/x-raw,format=RGB,width=128,height=128,framerate=30/1",
                                "!",
                                "queue",
                                "max-size-buffers=5",
                                "leaky=downstream",
                                "!",
                                "videorate",
                                "!",
                                "video/x-raw,framerate=30/1",
                                "!",
                                "videoconvert",
                                "!",
                                "vp8enc",
                                "keyframe-max-dist=30",
                                "deadline=1",
                                "!",
                                "sink.video_0",
                                "livekitwebrtcsink",
                                "name=sink",
                                "signaller::ws-url={}".format(self.config["server"]),
                                "signaller::auth-token={}".format(self.config["token"]),
                            ]
                        )

                        print(
                            "[MotionVideoExtension] Extension container launch {}".format(
                                p
                            )
                        )
                        self.process = await asyncio.create_subprocess_exec(
                            "docker",
                            *p,
                            stdout=asyncio.subprocess.PIPE,
                            stderr=asyncio.subprocess.STDOUT,
                            preexec_fn=os.setsid,
                        )

                        async for line in self.process.stdout:
                            print(
                                "[MotionVideoExtension] Extension container: {}".format(
                                    line.decode().strip()
                                )
                            )

                        await self.process.wait()
                        print("[MotionVideoExtension] Extension container launch final")
                    finally:
                        print("[MotionVideoExtension] Extension container finish start")
                        p = [
                            "rm",
                            "-f",
                            self.config["container"],
                        ]
                        print(
                            "[MotionVideoExtension] Extension container finish {}".format(
                                p
                            )
                        )
                        self.process = await asyncio.create_subprocess_exec(
                            "docker",
                            *p,
                            stdout=asyncio.subprocess.PIPE,
                            stderr=asyncio.subprocess.STDOUT,
                            preexec_fn=os.setsid,
                        )

                        async for line in self.process.stdout:
                            print(
                                "[MotionVideoExtension] Extension container: {}".format(
                                    line.decode().strip()
                                )
                            )

                        await self.process.wait()
                        print("[MotionVideoExtension] Extension container finish final")
                        self.process = None
            except asyncio.CancelledError:
                print("[MotionVideoExtension] Extension camera cancel")
            finally:
                print("[MotionVideoExtension] Extension camera exit")

        self.running = True
        loop = asyncio.get_event_loop()
        self.camera_task = loop.create_task(v(self))
        self.container_task = loop.create_task(g(self))
        print("[MotionVideoExtension] Extension startup")

        self.subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self.world_callback, name="world_callback")
        )

    def world_callback(self, e):
        try:
            print("[MotionVideoExtension] Extension world wait")
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return
            print("[MotionVideoExtension] Extension stage ready {}".format(stage))

            world = World.instance()
            if not world:
                return
            if not world.stage:
                return
            print(
                "[MotionVideoExtension] Extension world: {} {}".format(
                    world, world.stage
                )
            )

            prim = stage.GetPrimAtPath(self.config["camera"])
            if not prim.IsValid():
                return
            print("[MotionVideoExtension] Extension prim ready {}".format(prim))

            self.camera = Camera(prim_path=self.config["camera"])
            self.camera.initialize()

            print(
                "[MotionVideoExtension] Extension camera ready {}".format(self.camera)
            )

            if self.subscription:
                self.subscription.unsubscribe()
            self.subscription = None
        except Exception as e:
            print("[MotionVideoExtension] Extension world exception {}".format(e))

    def on_shutdown(self):
        async def g(self):
            if hasattr(self, "container_task") and self.container_task:
                self.container_task.cancel()
                try:
                    await self.container_task
                except asyncio.CancelledError:
                    print("[MotionVideoExtension] Extension container cancel")
                except Exception as e:
                    print(
                        "[MotionVideoExtension] Extension container exception {}".format(
                            e
                        )
                    )

        async def v(self):
            if hasattr(self, "camera_task") and self.camera_task:
                self.camera_task.cancel()
                try:
                    await self.camera_task
                except asyncio.CancelledError:
                    print("[MotionVideoExtension] Extension camera cancel")
                except Exception as e:
                    print(
                        "[MotionVideoExtension] Extension camera exception {}".format(e)
                    )

        self.running = False
        loop = asyncio.get_event_loop()
        loop.run_until_complete(g(self))
        loop.run_until_complete(v(self))
        print("[MotionVideoExtension] Extension shutdown")
