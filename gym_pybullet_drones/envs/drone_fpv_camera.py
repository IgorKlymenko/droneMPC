import pybullet as p
import numpy as np
import math

class FPVCameraEnv:
    def __init__(self, drone_id, z_near=0.1, z_far=100, res_w=640, res_h=480, fov_w=60.0):
        """
        Initialize the FPV camera environment.

        Parameters:
        -----------
        drone_id : int
            The PyBullet ID of the drone.
        z_near : float
            The near clipping plane for the camera.
        z_far : float
            The far clipping plane for the camera.
        res_w : int
            The width of the camera image.
        res_h : int
            The height of the camera image.
        fov_w : float
            The horizontal field of view of the camera in degrees.
        """
        self.drone_id = drone_id
        self._z_near = z_near
        self._z_far = z_far
        self._width = res_w
        self._height = res_h
        self._fov_width_deg = fov_w
        self._focal_length_pix = (float(self._width) / 2) / math.tan((math.pi * self._fov_width_deg / 180) / 2)
        self._fov_height_deg = (math.atan((float(self._height) / 2) / self._focal_length_pix) * 2 / math.pi) * 180
        self._projection_matrix = self._compute_projection_matrix()

    def _compute_projection_matrix(self):
        return p.computeProjectionMatrixFOV(
            fov=self._fov_height_deg,
            aspect=float(self._width) / float(self._height),
            nearVal=self._z_near,
            farVal=self._z_far,
        )

    def capture_fpv(self):
        """
        Capture an FPV view from the drone's position and orientation.

        Returns:
        --------
        rgb_image : np.ndarray
            The RGB image from the FPV camera.
        """
        # Get drone's position and orientation
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)

        # Compute view matrix
        cam_rot_mat = p.getMatrixFromQuaternion(orn)
        forward_vec = [cam_rot_mat[0], cam_rot_mat[3], cam_rot_mat[6]]
        cam_up_vec = [cam_rot_mat[2], cam_rot_mat[5], cam_rot_mat[8]]
        cam_target = [
            pos[0] + forward_vec[0] * 10.0,
            pos[1] + forward_vec[1] * 10.0,
            pos[2] + forward_vec[2] * 10.0,
        ]
        view_matrix = p.computeViewMatrix(pos, cam_target, cam_up_vec)

        # Capture FPV image
        w, h, rgb, _, _ = p.getCameraImage(
            width=self._width,
            height=self._height,
            viewMatrix=view_matrix,
            projectionMatrix=self._projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )

        # Process the RGB image
        rgb_image = np.asarray(rgb).reshape(h, w, 4)[:, :, :3].astype(np.uint8)
        return rgb_image
