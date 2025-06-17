import time
from vuer import Vuer
from vuer.events import ClientEvent
from vuer.schemas import ImageBackground, group, Hands # WebRTCStereoVideoPlane, DefaultScene
from multiprocessing import Array, Process, shared_memory, Queue, Manager, Event, Semaphore
import numpy as np
import asyncio
from webrtc.zed_server import *
from vuer.schemas import DefaultScene, Ply, PointCloud
import open3d as o3d    
from multiprocessing import Process, Array, Value, shared_memory

from asyncio import sleep
import numpy as np
import os
from vuer import Vuer, VuerSession
from vuer.schemas import Ply, Scene, Pcd, PointCloud



class OpenTeleVision:
    def __init__(self, img_shape, shm_name, queue, toggle_streaming, init_pose=None, intrinsic_mat = None,   stream_mode= "pcd", cert_file="./cert.pem", key_file="./key.pem", ngrok=False):
        # self.app=Vuer()
        self.init_camera_pose = init_pose
        self.intrinsic_mat = np.array(intrinsic_mat)   
        # self.rescale = resolution_scale
                
        # self.rgb_width = 720
        # self.rgb_height = 960
        # self.depth_width = 192
        # self.depth_height = 256


        # self.display_width = int(self.rgb_width * self.rescale)
        # self.display_height = int(self.rgb_height * self.rescale)

 

        self.img_height, self.img_width = img_shape[:2]
        self.pcd = o3d.geometry.PointCloud()

        if ngrok:
            self.app = Vuer(host='0.0.0.0', queries=dict(grid=False), queue_len=3)
        else:
            self.app = Vuer(host='0.0.0.0', cert=cert_file, key=key_file, queries=dict(grid=False), queue_len=3)

        self.app.add_handler("HAND_MOVE")(self.on_hand_move)
        self.app.add_handler("CAMERA_MOVE")(self.on_cam_move)


        if stream_mode == "image":


            self.img_shape = (img_shape[0], 2*img_shape[1], 3)

            existing_shm = shared_memory.SharedMemory(name=shm_name)
            self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=existing_shm.buf)
            self.app.spawn(start=False)(self.main_image)


        elif stream_mode == "singlergb":
            self.img_shape = (img_shape[0], img_shape[1], 3)

            existing_shm = shared_memory.SharedMemory(name=shm_name)
            self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=existing_shm.buf)
            self.app.spawn(start=False)(self.main_single_image)

           
        elif stream_mode == "webrtc":
            self.app.spawn(start=False)(self.main_webrtc)
        
        elif stream_mode == "pcd":
            shm = shared_memory.SharedMemory(name=shm_name)
            # Create numpy arrays with offsets
                # Define shapes
            resolution_cropped = img_shape
            rgb_shape = (resolution_cropped[0], resolution_cropped[1], 3)
            depth_shape = (resolution_cropped[0], resolution_cropped[1])
            extrinsic_shape = (4, 4)

            # Calculate sizes in bytes
            rgb_size = np.prod(rgb_shape) * np.uint8().itemsize
            depth_size = np.prod(depth_shape) * np.float32().itemsize
            #extrinsic_size = np.prod(extrinsic_shape) * np.float32().itemsize

            self.rgb_array = np.ndarray(rgb_shape, dtype=np.uint8, buffer=shm.buf[:rgb_size])
            self.depth_array = np.ndarray(depth_shape, dtype=np.float32, buffer=shm.buf[rgb_size:rgb_size + depth_size])
            self.extrinsic_array = np.ndarray(extrinsic_shape, dtype=np.float32, buffer=shm.buf[rgb_size + depth_size:])

            self.app.spawn(start=False)(self.main_pcd)
            
        else:
            raise ValueError("stream_mode must be either 'webrtc' or 'image'")

        self.left_hand_shared = Array('d', 16, lock=True)
        self.right_hand_shared = Array('d', 16, lock=True)
        self.left_landmarks_shared = Array('d', 75, lock=True)
        self.right_landmarks_shared = Array('d', 75, lock=True)
        
        self.head_matrix_shared = Array('d', 16, lock=True)
        self.aspect_shared = Value('d', 1.0, lock=True)

        # if stream_mode == "webrtc":
        #     # webrtc server
        #     if Args.verbose:
        #         logging.basicConfig(level=logging.DEBUG)
        #     else:
        #         logging.basicConfig(level=logging.INFO)
        #     Args.img_shape = img_shape
        #     # Args.shm_name = shm_name
        #     Args.fps = 60

        #     ssl_context = ssl.SSLContext()
        #     ssl_context.load_cert_chain(cert_file, key_file)

        #     app = web.Application()
        #     cors = aiohttp_cors.setup(app, defaults={
        #         "*": aiohttp_cors.ResourceOptions(
        #             allow_credentials=True,
        #             expose_headers="*",
        #             allow_headers="*",
        #             allow_methods="*",
        #         )
        #     })
        #     rtc = RTC(img_shape, queue, toggle_streaming, 60)
        #     app.on_shutdown.append(on_shutdown)
        #     cors.add(app.router.add_get("/", index))
        #     cors.add(app.router.add_get("/client.js", javascript))
        #     cors.add(app.router.add_post("/offer", rtc.offer))

        #     self.webrtc_process = Process(target=web.run_app, args=(app,), kwargs={"host": "0.0.0.0", "port": 8080, "ssl_context": ssl_context})
        #     self.webrtc_process.daemon = True
        #     self.webrtc_process.start()
        #     # web.run_app(app, host="0.0.0.0", port=8080, ssl_context=ssl_context)

        self.process = Process(target=self.run)
        self.process.daemon = True
        self.process.start()

    
    def run(self):
        self.app.run()

    async def on_cam_move(self, event, session, fps=60):
        # only intercept the ego camera.
        # if event.key != "ego":
        #     return
        try:
            # with self.head_matrix_shared.get_lock():  # Use the lock to ensure thread-safe updates
            #     self.head_matrix_shared[:] = event.value["camera"]["matrix"]
            # with self.aspect_shared.get_lock():
            #     self.aspect_shared.value = event.value['camera']['aspect']
            self.head_matrix_shared[:] = event.value["camera"]["matrix"]
            self.aspect_shared.value = event.value['camera']['aspect']
        except:
            pass
        # self.head_matrix = np.array(event.value["camera"]["matrix"]).reshape(4, 4, order="F")
        # print(np.array(event.value["camera"]["matrix"]).reshape(4, 4, order="F"))
        # print("camera moved", event.value["matrix"].shape, event.value["matrix"])

    async def on_hand_move(self, event, session, fps=60):
        try:
            # with self.left_hand_shared.get_lock():  # Use the lock to ensure thread-safe updates
            #     self.left_hand_shared[:] = event.value["leftHand"]
            # with self.right_hand_shared.get_lock():
            #     self.right_hand_shared[:] = event.value["rightHand"]
            # with self.left_landmarks_shared.get_lock():
            #     self.left_landmarks_shared[:] = np.array(event.value["leftLandmarks"]).flatten()
            # with self.right_landmarks_shared.get_lock():
            #     self.right_landmarks_shared[:] = np.array(event.value["rightLandmarks"]).flatten()
            self.left_hand_shared[:] = event.value["leftHand"]
            self.right_hand_shared[:] = event.value["rightHand"]
            self.left_landmarks_shared[:] = np.array(event.value["leftLandmarks"]).flatten()
            self.right_landmarks_shared[:] = np.array(event.value["rightLandmarks"]).flatten()
        except: 
            pass
    
    async def main_webrtc(self, session, fps=60):
        session.set @ DefaultScene(frameloop="always")
        session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=False, showRight=False)
        session.upsert @ WebRTCStereoVideoPlane(
                src="https://192.168.8.102:8080/offer",
                # iceServer={},
                key="zed",
                aspect=1.33334,
                height = 8,
                position=[0, -2, -0.2],
            )
        while True:
            await asyncio.sleep(1)
        
    async def main_image(self, session, fps=60):
        session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=False, showRight=False)
        end_time = time.time()
        while True:
            start = time.time()
            # print(end_time - start)
            # aspect = self.aspect_shared.value
            display_image = self.img_array

            # session.upsert(
            # ImageBackground(
            #     # Can scale the images down.
            #     display_image[:self.img_height],
            #     # 'jpg' encoding is significantly faster than 'png'.
            #     format="jpeg",
            #     quality=80,
            #     key="left-image",
            #     interpolate=True,
            #     # fixed=True,
            #     aspect=1.778,
            #     distanceToCamera=2,
            #     position=[0, -0.5, -2],
            #     rotation=[0, 0, 0],
            # ),
            # to="bgChildren",
            # )

            session.upsert(
            [ImageBackground(
                # Can scale the images down.
                display_image[::2, :self.img_width],
                # display_image[:self.img_height:2, ::2],
                # 'jpg' encoding is significantly faster than 'png'.
                format="jpeg",
                quality=80,
                key="left-image",
                interpolate=True,
                # fixed=True,
                aspect=1.66667,
                # distanceToCamera=0.5,
                height = 8,
                position=[0, -1, 3],
                # rotation=[0, 0, 0],
                layers=1, 
                alphaSrc="./vinette.jpg"
            ),
            ImageBackground(
                # Can scale the images down.
                display_image[::2, self.img_width:],
                # display_image[self.img_height::2, ::2],
                # 'jpg' encoding is significantly faster than 'png'.
                format="jpeg",
                quality=80,
                key="right-image",
                interpolate=True,
                # fixed=True,
                aspect=1.66667,
                # distanceToCamera=0.5,
                height = 8,
                position=[0, -1, 3],
                # rotation=[0, 0, 0],
                layers=2, 
                alphaSrc="./vinette.jpg"
            )],
            to="bgChildren",
            )
            # rest_time = 1/fps - time.time() + start
            end_time = time.time()
            await asyncio.sleep(0.03)



    async def main_single_image(self, session, fps=60):
        session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=False, showRight=False)
        end_time = time.time()
        while True:
            start = time.time()
            # print(end_time - start)
            # aspect = self.aspect_shared.value
            display_image = self.img_array

            # session.upsert(
            # ImageBackground(
            #     # Can scale the images down.
            #     display_image[:self.img_height],
            #     # 'jpg' encoding is significantly faster than 'png'.
            #     format="jpeg",
            #     quality=80,
            #     key="left-image",
            #     interpolate=True,
            #     # fixed=True,
            #     aspect=1.778,
            #     distanceToCamera=2,
            #     position=[0, -0.5, -2],
            #     rotation=[0, 0, 0],
            # ),
            # to="bgChildren",
            # )

            session.upsert(
            
            

            [ImageBackground(
                # Can scale the images down.
                display_image[::1, ::1, :],
                # display_image[:self.img_height:2, ::2],
                # 'jpg' encoding is significantly faster than 'png'.
                format="jpeg",
                quality=80,
                key="left-image",
                interpolate=True,
                fixed=True,
                aspect=1.66667,
                distanceToCamera=1,
                height = 8,
                position=[0, 0, -30],
                #rotation=[0, 0, 0],
                layers=1, 
                alphaSrc="./vinette.jpg"


            ),
            ImageBackground(
                # Can scale the images down.
                display_image[::1, ::1, :],
                # display_image[self.img_height::2, ::2],
                # 'jpg' encoding is significantly faster than 'png'.
                format="jpeg",
                quality=80,
                key="right-image",
                interpolate=True,
                fixed=True,
                aspect=1.66667,
                distanceToCamera=1,
                height = 8,
                position=[0, 0, -30],
                #rotation=[0, 0, 0],
                layers=2, 
                alphaSrc="./vinette.jpg"
            )],
            


            to="bgChildren",
            )
            # rest_time = 1/fps - time.time() + start
            end_time = time.time()
            await asyncio.sleep(0.015)


    def get_global_xyz(self, depth, rgb, intrinsics, extrinsic, depth_scale=1000.0, only_confident=False):


        depth_o3d = o3d.geometry.Image(
            np.ascontiguousarray(depth_scale * depth).astype(np.float32)
        )
        rgb_o3d = o3d.geometry.Image(
            np.ascontiguousarray(rgb).astype(np.uint8)
        )

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d, depth_o3d, convert_rgb_to_intensity=False
        )

        camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            width=rgb.shape[0],
            height=rgb.shape[1],
            fx=intrinsics[0, 0] * rgb.shape[0]/ 960,
            fy=intrinsics[1, 1] * rgb.shape[0]/ 960,
            cx=intrinsics[0, 2] * rgb.shape[0]/ 960,
            cy=intrinsics[1, 2] * rgb.shape[0]/ 960,
        )


        temp = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, camera_intrinsics)
        temp.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])


        self.pcd.points = temp.points
        self.pcd.colors = temp.colors


        # Now transform everything by camera pose to world frame.







        self.pcd.transform(extrinsic)
        self.pcd.transform(np.linalg.inv(self.init_camera_pose))



        # o3d.io.write_point_cloud("sample_point_cloud.pcd", self.pcd)
        # print("point cloud written")
        # time.sleep(1000)



    async def main_pcd(self, sess: VuerSession):
        # setting the toneMappingExposure to a lower value to make the color look nicer.
        sess.set @ Scene(toneMappingExposure=0.4)
        while True:
            self.get_global_xyz(self.depth_array, self.rgb_array, self.intrinsic_mat, self.extrinsic_array, depth_scale=1000.0, only_confident=False)
            pcd = self.pcd

            sess.upsert @ PointCloud(
                        key="pointcloud",
                        vertices=np.array(pcd.points),
                        colors=np.array(pcd.colors),
                        position=[0, 1.4, 0],
                        size=0.008,
            )
            await asyncio.sleep(0.015)

        # while True:
        #     await sleep(1)


    
    # async def main_pcd(self, session, fps=60):

    #     session.set @ DefaultScene(frameloop="always")

    #     session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=False, showRight=False)
    #     end_time = time.time()
    #     while True:
    #         start = time.time()
    #         # print(end_time - start)
    #         # aspect = self.aspect_shared.value

    #         self.get_global_xyz(self.depth_array, self.rgb_array, self.intrinsic_mat, self.extrinsic_array, depth_scale=1000.0, only_confident=False)
            

    #         pcd = self.pcd
    #         output_file = "output.ply"

    #         # Save the point cloud to a PLY file
    #         o3d.io.write_point_cloud(output_file, pcd)


    #         session.upsert @ Ply(
    #             src= "output.ply",
    #             size=0.008,
    #             rotation=[- 0.5 * np.pi, 0, -0.5 * np.pi]
    #         )

    #         session.upsert(

    #                 PointCloud(
    #                     key="pointcloud",
    #                     vertices=np.array(pcd.points),
    #                     colors=np.array(pcd.colors),

    #                     # position=[0, 1.5, -0.4],
    #                     # position=[0, 1.5, -0.4],
    #                     position=[0, 1.4, 0.0],

    #                     size=0.05,
    #                 ),                
    #         to="bgChildren",
    #         )
    
    #         # # rest_time = 1/fps - time.time() + start
    #         end_time = time.time()
    #         #print(1/(end_time - start))
    #         await asyncio.sleep(0.015)


    @property
    def left_hand(self):
        # with self.left_hand_shared.get_lock():
        #     return np.array(self.left_hand_shared[:]).reshape(4, 4, order="F")
        return np.array(self.left_hand_shared[:]).reshape(4, 4, order="F")
        
    
    @property
    def right_hand(self):
        # with self.right_hand_shared.get_lock():
        #     return np.array(self.right_hand_shared[:]).reshape(4, 4, order="F")
        return np.array(self.right_hand_shared[:]).reshape(4, 4, order="F")
        
    
    @property
    def left_landmarks(self):
        # with self.left_landmarks_shared.get_lock():
        #     return np.array(self.left_landmarks_shared[:]).reshape(25, 3)
        return np.array(self.left_landmarks_shared[:]).reshape(25, 3)
    
    @property
    def right_landmarks(self):
        # with self.right_landmarks_shared.get_lock():
            # return np.array(self.right_landmarks_shared[:]).reshape(25, 3)
        return np.array(self.right_landmarks_shared[:]).reshape(25, 3)

    @property
    def head_matrix(self):
        # with self.head_matrix_shared.get_lock():
        #     return np.array(self.head_matrix_shared[:]).reshape(4, 4, order="F")
        return np.array(self.head_matrix_shared[:]).reshape(4, 4, order="F")

    @property
    def aspect(self):
        # with self.aspect_shared.get_lock():
            # return float(self.aspect_shared.value)
        return float(self.aspect_shared.value)

    
# if __name__ == "__main__":

    # image_queue = Queue()
    # toggle_streaming = Event()
    # resolution = (720, 1280)
    # crop_size_w = 340  # (resolution[1] - resolution[0]) // 2
    # crop_size_h = 270
    # resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)  # 450 * 600
    # img_shape = (2 * resolution_cropped[0], resolution_cropped[1], 3)  # 900 * 600
    # img_height, img_width = resolution_cropped[:2]  # 450 * 600
    
    # points = np.random.rand(100, 3)
    # colors = np.random.rand(100, 3)

    # shm = shared_memory.SharedMemory(create=True, size=np.prod(points.shape) * np.uint8().itemsize)
    # shm_name = shm.name
    # _shm = shared_memory.SharedMemory(create=True, size=np.prod(colors.shape) * np.uint8().itemsize)
    # _shm_name = _shm.name


    # tv = OpenTeleVision(resolution_cropped, shm.name, _shm_name, image_queue, toggle_streaming, ngrok=False)
    # # OpenTeleVision(resolution_cropped, cert_file="../cert.pem", key_file="../key.pem")
    # while True:
    #     # print(tv.left_landmarks)
    #     # print(tv.left_hand)
    #     # tv.modify_shared_image(random=True)
    #     time.sleep(1)


