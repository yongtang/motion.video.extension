import omni.ext
import omni.usd
import omni.kit.app
from omni.isaac.sensor import Camera
import asyncio, toml, os, socket, io
import numpy as np
import PIL.Image


class MotionVideoExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()

        self.config = {
            "camera": None,
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
        except Exception as e:
            print("[MotionVideoExtension] Extension config: {}".format(e))
        print("[MotionVideoExtension] Extension config: {}".format(self.config))

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def on_startup(self, ext_id):
        async def f(self):
            context = omni.usd.get_context()
            while context.get_stage() is None:
                print("[MotionVideoExtension] Extension stage wait")
                await asyncio.sleep(0.5)
            print("[MotionVideoExtension] Extension stage ready")

            stage = context.get_stage()
            print("[MotionVideoExtension] Extension stage {}".format(stage))

            if (
                self.config["camera"]
                and stage.GetPrimAtPath(self.config["camera"]).IsValid()
            ):
                self.camera = Camera(prim_path=self.config["camera"])
                self.camera.initialize()
                print("[MotionVideoExtension] Extension camera {}".format(self.camera))
            else:
                self.camera = None

        async def v(self):
            try:
                while self.running:
                    try:
                        image = self.camera.get_rgba()
                        if len(image):
                            image = np.array(image, dtype=np.uint8)  # RGBA
                            assert image.shape[-1] == 4, "camera {}".format(image.shape)
                            image = image[:, :, :3]  # Remove alpha channel, keep RGB
                            assert image.shape[-1] == 3, "camera {}".format(image.shape)

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

        self.running = True
        loop = asyncio.get_event_loop()
        loop.run_until_complete(f(self))
        self.camera_task = loop.create_task(v(self)) if self.camera else None
        print("[MotionVideoExtension] Extension startup")

    def on_shutdown(self):
        async def v(self):
            if getattr(self, "camera_task") and self.camera_task:
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
        loop.run_until_complete(v(self))
        print("[MotionVideoExtension] Extension shutdown")
