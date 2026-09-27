from abc import ABC, abstractmethod
import cv2
import numpy as np


class TagDetectorBase(ABC):
    """
    Abstract base class for tag detectors. This allows for flexibility in switching 
    between different tag detection libraries or standards.
    """

    @abstractmethod
    def detect_tags(self, gray_frame, camera_params, tag_size_m):
        """
        Detect tags in the provided grayscale frame.
        
        Args:
            gray_frame (numpy.ndarray): Grayscale image for tag detection.
            camera_params (list): Camera parameters [fx, fy, cx, cy].
            tag_size_m (float): Tag size in meters.

        Returns:
            List of detected tags. Each tag must have `corners` and `tag_id` attributes.
        """
        pass


class ApriltagDetector(TagDetectorBase):
    """
    Detector for AprilTags using the pyapriltags library.
    """

    def __init__(self, families, hamming_thr=0):
        from pyapriltags import Detector  # Import here to decouple dependencies
        self.detector = Detector(
            searchpath=['apriltags'],
            families=families,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.0,
            debug=0
        )
        self.hamming_thr = hamming_thr

    def detect_tags(self, gray_frame, camera_params, tag_size_m):
        tags = self.detector.detect(
            gray_frame,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size_m
        )
        # Filter tags based on Hamming threshold
        return [tag for tag in tags if tag.hamming <= self.hamming_thr]


class quARtet:
    """
    The quARtet class performs pose estimation using multiple tags (assumes four tags).
    It supports flexibility in the choice of the tag detection library or standard.
    """

    def __init__(
        self,
        tag_detector: TagDetectorBase,  # Abstract detector interface
        tag_ids,
        tag_size_mm,
        whole_size_mm,
        camera_params,
        dist_coeffs,
        tilt_mode="diagonal",
        vis_detected_markers_pos=True,
        XY_tilt_degree=18,
    ):
        # Convert parameters received in mm to m
        self.tag_size_m = tag_size_mm / 1000.0
        self.whole_size_m = whole_size_mm / 1000.0

        self.tag_detector = tag_detector  # Pass a detector instance
        self.tag_ids = tag_ids
        self.camera_params = camera_params
        self.dist_coeffs = dist_coeffs
        self.vis_detected_markers_pos = vis_detected_markers_pos

        self.XY_tilt_degree=XY_tilt_degree
        self.tilt_mode = tilt_mode
        self.tag_obj_points = self._calculate_corners_3d_positions(
            self.whole_size_m,
            self.tag_size_m,
            self.XY_tilt_degree,
            self.tilt_mode
        )

    def _draw_detected_frame_id(self, frame, corners, tag_id):
        """
        Draw the detected tag boundary and display its ID.
        """
        cv2.polylines(frame, [corners.astype(int).reshape((-1, 1, 2))], True, (255, 255, 0), 2)
        center = tuple(np.mean(corners, axis=0).astype(int).ravel())
        cv2.putText(frame, str(tag_id), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    def _draw_attitude(self, frame, rvec, tvec):
        """
        Draw xyz axes on the frame based on the pose estimation results.
        """
        fx, fy, cx, cy = self.camera_params
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)

        axis = np.float32([
            [self.tag_size_m, 0, 0],
            [0, self.tag_size_m, 0],
            [0, 0, self.tag_size_m] # -self.tag_size_m -> self.tag_size_m
        ]).reshape(-1, 3)

        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, self.dist_coeffs)
        origin, _ = cv2.projectPoints(np.float32([[0, 0, 0]]), rvec, tvec, camera_matrix, self.dist_coeffs)

        origin = tuple(origin[0].ravel().astype(int))
        imgpts = imgpts.astype(int)

        cv2.line(frame, origin, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
        cv2.line(frame, origin, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
        cv2.line(frame, origin, tuple(imgpts[2].ravel()), (0, 0, 255), 5)


    def _calculate_corners_3d_positions(self, L, l, theta_deg, tilt_mode="diagonal"):
        frame = (L-2*l)/2
        big_square_vertices=np.array([
            [-L/2,-L/2,0],
            [L/2,-L/2,0],
            [L/2,L/2,0],
            [-L/2,L/2,0],
        ])
        small_square_local_vertices=np.array([
            [0,0,0],
            [l,0,0],
            [l,l,0],
            [0,l,0],
        ])

        if tilt_mode == "diagonal":
            axis=np.array([1,-1,0])
            theta_rad=np.radians(-theta_deg)
        elif tilt_mode == "pitch":
            axis=np.array([-1,0,0])
            theta_rad=np.radians(theta_deg)
        elif tilt_mode == "roll":
            axis=np.array([0,-1,0])
            theta_rad=np.radians(-theta_deg)
        elif tilt_mode == "pitch_r":
            axis=np.array([-1,0,0])
            theta_rad=np.radians(theta_deg)
        elif tilt_mode == "roll_r":
            axis=np.array([0,-1,0])
            theta_rad=np.radians(-theta_deg)
        else:
            raise ValueError("Invalid tilt_mode specified. Choose 'diagonal', 'pitch', 'pitch_r', 'roll' or 'roll_r'.")

        axis=axis/np.linalg.norm(axis)
        K=np.array([
            [0,-axis[2],axis[1]],
            [axis[2],0,-axis[0]],
            [-axis[1],axis[0],0]
        ])
        I=np.eye(3)
        R=I+np.sin(theta_rad)*K+(1-np.cos(theta_rad))*(K@K)

        all_vertices=[]

        for i,big_vertex in enumerate(big_square_vertices):
            rotation_matrix_yaw=np.array([
                [np.cos(i*np.pi/2),-np.sin(i*np.pi/2), 0],
                [np.sin(i*np.pi/2),np.cos(i*np.pi/2),0],
                [0,0,1]
            ])
            global_vertices=(rotation_matrix_yaw@R@small_square_local_vertices.T).T+big_vertex
            if tilt_mode =="pitch_r" or tilt_mode == "roll_r":
                xy_center = np.mean(global_vertices, axis=0)
                if xy_center[0]>0:xL=frame*2+l
                else:xL=-frame*2-l
                if xy_center[1]>0:yL=frame*2+l
                else:yL=-frame*2-l

                global_vertices[:,:2] = np.array([xL,yL]) - global_vertices[:,:2]

            all_vertices.append(global_vertices)

        all_vertices = np.vstack(all_vertices)
        return all_vertices
    

    def detect(self, frame):
        """
        Detect tags in the frame and calculate the central pose (rvec, tvec).
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect tags using the provided detector
        tags = self.tag_detector.detect_tags(gray, self.camera_params, self.tag_size_m)

        tags_info = {self.tag_ids.index(tag.tag_id): tag for tag in tags if tag.tag_id in self.tag_ids}

        if len(tags_info) == 0:
            return np.nan, np.nan, [0]

        # Gather object points and image points for SolvePnP
        obj_points = []
        img_points = []
        detected_tag_ids = []
        for tag_no, tag in tags_info.items():
            corners = tag.corners.reshape(4, 2)
            obj_points.extend(self.tag_obj_points[tag_no*4:(tag_no+1)*4])
            img_points.extend(corners)
            detected_tag_ids.append(tag.tag_id)

            if self.vis_detected_markers_pos:
                self._draw_detected_frame_id(frame, corners, tag.tag_id)
        # SolvePnP using available points
        obj_points = np.array(obj_points, dtype=float)
        img_points = np.array(img_points, dtype=float)
        fx, fy, cx, cy = self.camera_params
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)
        
        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
        )

        if success:
            self._draw_attitude(frame, rvec, tvec)
            return tvec.T[0], rvec.T[0], detected_tag_ids

        return np.nan, np.nan, [0]
