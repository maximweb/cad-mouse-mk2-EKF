import numpy as np


class DipoleModel:

    _DEFAULT_GEOMETRY = {
        # The coordinate system is fixed to the case with
        # - USB port facing away from the user
        # - X-axis pointing to the right
        # - Y-axis pointing towards the USB port away from the user
        # - Z-axis pointing upwards
        # This aligns with sensor orientations and reading.
        #
        # We assume the center of the spring is at the origin (0, 0, 0).
        # This makes the origin also the pivot point of any rotation.
        #
        # The hall sensors are located at a fixed z-coordinate in an
        # equilateral triangle arrangement in the XY-plane.
        "sensor_z": -19.417,  # mm, top of sensor to middle of spring
        #
        "sensor1_x": 0,  # mm
        "sensor1_y": -16.51,  # mm
        "sensor2_x": -14.298,  # mm
        "sensor2_y": 8.255,  # mm
        "sensor3_x": 14.298,  # mm
        "sensor3_y": 8.255,  # mm
        # According to datasheet of hall sensors TLI493D-A2B6, the center of sensitivity is not centered.
        "sensor_offset": {
            "x": -2.9 / 2 + 1.15,  # mm, Fig 11 Package outline & Fig 4 Sensitivity area
            "y": 1.6 / 2 - 0.785,  # mm, Fig 11 Package outline & Fig 4 Sensitivity area
            "z": -0.65,  # mm, Fig 4 Sensitivity area
        },
        # Magnets are fixed to each other in a plane and equilateral triangle.
        # Only the neutral position of the magnets is parallel to the XY-plane.
        # Any rotation of the magnets will change the z-coordinate of each magnet.
        # Magnet dimensions
        "magnet_diameter": 6,  # mm
        "magnet_height": 6,  # mm
        # Magnet neutral position centers
        "magnet_neutral_z": -11.005,  # mm
        #
        "magnet1_neutral_x": 0,  # mm
        "magnet1_neutral_y": -16.5,  # mm
        "magnet2_neutral_x": -14.289,  # mm
        "magnet2_neutral_y": 8.25,  # mm
        "magnet3_neutral_x": 14.289,  # mm
        "magnet3_neutral_y": 8.25,  # mm
    }

    def __init__(
        self,
        n_dipoles: int = 1,
        magnets: tuple[bool, bool, bool] = (True, True, True),
        magnetic_moment: float | tuple[float, float, float] = 0.5,
        neutral_position_offsets: tuple[float, float, float] = (0.0, 0.0, 0.0),
        geometry: dict | None = None,
    ):
        """Initialize the DipoleModel.

        Args:
            n_dipoles (int, optional): Number of dipoles per magnet. Defaults to 1.
                Single dipole is assumed to be at the center of the magnet.
                Two dipoles are assumed to be at -h/4 and +h/4 along the magnet's axis, where h is the height of the magnet.
                Multiple dipoles are assumed to be evenly spaced along the magnet's axis.
            magnets (tuple[bool, bool, bool], optional): Which magnets are active. Defaults to (True, True, True).
            magnetic_moment (float | tuple[float, float, float], optional): Magnetic moment of the
                dipoles in A·m². Can be a single float (shared by all three magnets) or a tuple of
                three floats (one per magnet). The value is a fit parameter; a 6 mm × 6 mm NdFeB
                cylinder (Br ≈ 1 T) has m ≈ 0.135 A·m². Defaults to 0.5.
            neutral_position_offsets (tuple[float, float, float], optional):
                Offsets for the neutral position of the magnets X, Y, RZ in mm and degrees.
                Defaults to (0.0, 0.0, 0.0).
            geometry (dict | None, optional): Geometry configuration. Defaults to None.

        Raises:
            ValueError: If the number of dipoles is less than 1.
            ValueError: If the magnetic moment is not a float or a tuple of three floats.
        """

        # Input validation
        if n_dipoles < 1:
            raise ValueError("Number of dipoles must be at least 1.")
        self._n_dipoles = n_dipoles

        if (
            not isinstance(magnets, (tuple, list))
            or len(magnets) != 3
            or not all(isinstance(m, bool) for m in magnets)
            or not any(magnets)
        ):
            raise ValueError("Magnets must be a tuple of three booleans, and at least one magnet must be active.")
        self._magnets = magnets

        if isinstance(magnetic_moment, (int, float)):
            self._magnetic_moment_1 = magnetic_moment
            self._magnetic_moment_2 = magnetic_moment
            self._magnetic_moment_3 = magnetic_moment
        elif isinstance(magnetic_moment, (tuple, list)) and len(magnetic_moment) == 3:
            self._magnetic_moment_1, self._magnetic_moment_2, self._magnetic_moment_3 = magnetic_moment
        elif isinstance(magnetic_moment, np.ndarray) and magnetic_moment.shape == (3,):
            self._magnetic_moment_1, self._magnetic_moment_2, self._magnetic_moment_3 = magnetic_moment.tolist()
        else:
            raise ValueError("Magnetic moment must be a float or a tuple of three floats.")

        if isinstance(neutral_position_offsets, (tuple, list)) or len(neutral_position_offsets) != 3:
            self._neutral_X_offset, self._neutral_Y_offset, self._neutral_RZ_offset = neutral_position_offsets
        elif isinstance(neutral_position_offsets, np.ndarray) and neutral_position_offsets.shape == (3,):
            self._neutral_X_offset, self._neutral_Y_offset, self._neutral_RZ_offset = neutral_position_offsets.tolist()
        else:
            raise ValueError("Neutral position offsets must be a tuple of three floats.")

        if geometry is None:
            self._geometry = self._DEFAULT_GEOMETRY
        else:
            self._geometry = geometry

    def _get_sensor_positions(self) -> np.ndarray:
        """Get the positions of the sensors in the fixed coordinate system.

        Returns:
            np.ndarray: A 3x3 array where each row corresponds to a sensor's (x, y, z) position.
        """
        sensor_positions = np.array(
            [
                [
                    self._geometry["sensor1_x"] + self._geometry["sensor_offset"]["x"],
                    self._geometry["sensor1_y"] + self._geometry["sensor_offset"]["y"],
                    self._geometry["sensor_z"] + self._geometry["sensor_offset"]["z"],
                ],
                [
                    self._geometry["sensor2_x"] + self._geometry["sensor_offset"]["x"],
                    self._geometry["sensor2_y"] + self._geometry["sensor_offset"]["y"],
                    self._geometry["sensor_z"] + self._geometry["sensor_offset"]["z"],
                ],
                [
                    self._geometry["sensor3_x"] + self._geometry["sensor_offset"]["x"],
                    self._geometry["sensor3_y"] + self._geometry["sensor_offset"]["y"],
                    self._geometry["sensor_z"] + self._geometry["sensor_offset"]["z"],
                ],
            ]
        )
        return sensor_positions

    def _get_dipole_positions_neutral(self) -> np.ndarray:
        """Get the positions of the dipoles in the neutral position.

        n_magnets x n_dipoles x 3

        Dipoles are assumed to be evenly spaced along the magnet's axis, which is perpendicular to the XY-plane in the neutral position.

        Returns:
            np.ndarray: A 3D array where each slice corresponds to a magnet's dipole positions in the neutral position.
        """
        magnet_neutral_positions = np.array(
            [
                [
                    self._geometry["magnet1_neutral_x"],
                    self._geometry["magnet1_neutral_y"],
                    self._geometry["magnet_neutral_z"],
                ],
                [
                    self._geometry["magnet2_neutral_x"],
                    self._geometry["magnet2_neutral_y"],
                    self._geometry["magnet_neutral_z"],
                ],
                [
                    self._geometry["magnet3_neutral_x"],
                    self._geometry["magnet3_neutral_y"],
                    self._geometry["magnet_neutral_z"],
                ],
            ]
        )

        # Apply neutral position offsets
        # - Translation in X and Y
        # - Rotation around Z (RZ) in degrees
        RZ_rad = np.radians(self._neutral_RZ_offset)
        rotation_matrix = np.array(
            [
                [np.cos(RZ_rad), -np.sin(RZ_rad), 0],
                [np.sin(RZ_rad), np.cos(RZ_rad), 0],
                [0, 0, 1],
            ]
        )
        magnet_neutral_positions = (rotation_matrix @ magnet_neutral_positions.T).T
        magnet_neutral_positions[:, 0] += self._neutral_X_offset
        magnet_neutral_positions[:, 1] += self._neutral_Y_offset

        # Calculate dipole positions along the magnet's axis
        dipole_positions = np.zeros((3, self._n_dipoles, 3))
        h = self._geometry["magnet_height"]
        for i in range(3):  # For each magnet
            for j in range(self._n_dipoles):  # For each dipole
                # Dipoles are evenly spaced within [-h/4, +h/4] along the magnet axis.
                # For n=1: 0, for n=2: -h/4 and +h/4, for n>2: evenly spaced within +-h/4.
                z_offset = (
                    (j - (self._n_dipoles - 1) / 2) * (h / (2 * max(self._n_dipoles - 1, 1)))
                    if self._n_dipoles > 1
                    else 0
                )
                dipole_positions[i, j] = magnet_neutral_positions[i] + np.array([0, 0, z_offset])

        return dipole_positions

    @staticmethod
    def _apply_6dof_dipole_positions(
        neutral_dipole_positions: np.ndarray,
        x: float,
        y: float,
        z: float,
        RX_roll: float,
        RY_pitch: float,
        RZ_yaw: float,
    ) -> np.ndarray:
        """Apply 6DOF transformation to the dipole positions.

        Args:
            neutral_dipole_positions (np.ndarray): The neutral dipole positions (n_magnets x n_dipoles x 3).
            x (float): Translation along the X-axis.
            y (float): Translation along the Y-axis.
            z (float): Translation along the Z-axis.
            RX_roll (float): Rotation around the X-axis in radians.
            RY_pitch (float): Rotation around the Y-axis in radians.
            RZ_yaw (float): Rotation around the Z-axis in radians.

        Returns:
            np.ndarray: The transformed dipole positions after applying translation and rotation.
        """
        # Create rotation matrices for roll, pitch, and yaw
        R_roll = np.array(
            [
                [1, 0, 0],
                [0, np.cos(RX_roll), -np.sin(RX_roll)],
                [0, np.sin(RX_roll), np.cos(RX_roll)],
            ]
        )

        R_pitch = np.array(
            [
                [np.cos(RY_pitch), 0, np.sin(RY_pitch)],
                [0, 1, 0],
                [-np.sin(RY_pitch), 0, np.cos(RY_pitch)],
            ]
        )

        R_yaw = np.array(
            [
                [np.cos(RZ_yaw), -np.sin(RZ_yaw), 0],
                [np.sin(RZ_yaw), np.cos(RZ_yaw), 0],
                [0, 0, 1],
            ]
        )

        # Combined rotation matrix
        R = R_yaw @ R_pitch @ R_roll

        # Apply rotation and translation to each dipole position
        transformed_positions = np.zeros_like(neutral_dipole_positions)
        for i in range(neutral_dipole_positions.shape[0]):  # For each magnet
            for j in range(neutral_dipole_positions.shape[1]):  # For each dipole
                transformed_positions[i, j] = R @ neutral_dipole_positions[i, j] + np.array([x, y, z])

        return transformed_positions

    def _get_dipole_orientations(self, RX_roll: float, RY_pitch: float, RZ_yaw: float) -> np.ndarray:
        """Get the orientations of the dipoles after applying rotations.

        Args:
            RX_roll (float): Rotation around the X-axis in radians.
            RY_pitch (float): Rotation around the Y-axis in radians.
            RZ_yaw (float): Rotation around the Z-axis in radians.

        Returns:
            np.ndarray: A 3D array where each slice corresponds to a magnet's dipole orientations after rotation.
        """
        # Create rotation matrices for roll, pitch, and yaw
        R_roll = np.array(
            [
                [1, 0, 0],
                [0, np.cos(RX_roll), -np.sin(RX_roll)],
                [0, np.sin(RX_roll), np.cos(RX_roll)],
            ]
        )

        R_pitch = np.array(
            [
                [np.cos(RY_pitch), 0, np.sin(RY_pitch)],
                [0, 1, 0],
                [-np.sin(RY_pitch), 0, np.cos(RY_pitch)],
            ]
        )

        R_yaw = np.array(
            [
                [np.cos(RZ_yaw), -np.sin(RZ_yaw), 0],
                [np.sin(RZ_yaw), np.cos(RZ_yaw), 0],
                [0, 0, 1],
            ]
        )

        # Combined rotation matrix
        R = R_yaw @ R_pitch @ R_roll

        # Assuming dipoles are initially aligned along the Z-axis in the neutral position
        initial_orientation = np.array([0, 0, 1])  # Unit vector along Z-axis

        # Apply rotation to get new orientations
        dipole_orientations = np.zeros((3, self._n_dipoles, 3))
        for i in range(3):  # For each magnet
            for j in range(self._n_dipoles):  # For each dipole
                dipole_orientations[i, j] = R @ initial_orientation

        return dipole_orientations

    @staticmethod
    def _dipole_field(r_vecs: np.ndarray, m_vec: np.ndarray) -> np.ndarray:
        """Calculate the magnetic field at one or more points due to a single dipole.

        Args:
            r_vecs (np.ndarray): Position vectors from the dipole to the points of interest.
                Shape (n_sensors, 3) or (3,) for a single point.
            m_vec (np.ndarray): Magnetic moment vector of the dipole (3,).

        Returns:
            np.ndarray: Magnetic field vectors at each point, shape (n_sensors, 3).
        """
        MU0_OVER_4PI = 1e-7  # T·m/A  (= µ0 / 4π)
        r_vecs_m = np.atleast_2d(r_vecs) * 1e-3  # mm → m, shape (n_sensors, 3)
        r_magnitudes = np.linalg.norm(r_vecs_m, axis=1)  # (n_sensors,)
        zero_mask = r_magnitudes == 0
        r_magnitudes_safe = np.where(zero_mask, 1.0, r_magnitudes)
        r_unitvecs = r_vecs_m / r_magnitudes_safe[:, None]  # (n_sensors, 3)
        mdotr = r_unitvecs @ m_vec  # (n_sensors,)
        B_T = (
            MU0_OVER_4PI * (3 * mdotr[:, None] * r_unitvecs - m_vec[None, :]) / r_magnitudes_safe[:, None] ** 3
        )  # (n_sensors, 3), Tesla
        B_T[zero_mask] = 0.0  # field undefined at dipole location
        return B_T * 1e3  # T → mT

    @staticmethod
    def _superposition_of_dipoles(
        sensor_positions: np.ndarray,
        dipole_positions: np.ndarray,
        dipole_orientations: np.ndarray,
        magnetic_moments: np.ndarray,
        magnets: tuple[bool, bool, bool],
    ) -> np.ndarray:
        """Calculate the superposition of magnetic fields from multiple dipoles at sensor positions.

        Args:
            sensor_positions (np.ndarray): Positions of the sensors (n_sensors x 3).
            dipole_positions (np.ndarray): Positions of the dipoles (n_magnets x n_dipoles x 3).
            dipole_orientations (np.ndarray): Orientations of the dipoles (n_magnets x n_dipoles x 3).
            magnetic_moments (np.ndarray): Magnetic moments of the dipoles (n_magnets x n_dipoles).
            magnets (tuple[bool, bool, bool]): Which magnets are active.

        Returns:
            np.ndarray: The resulting magnetic field readings at each sensor position (n_sensors x 3).
        """
        n_sensors = sensor_positions.shape[0]
        B_total = np.zeros((n_sensors, 3))  # Initialize total magnetic field at each sensor

        for i_magnet in range(3):  # For each magnet
            if not magnets[i_magnet]:
                continue  # Skip inactive magnets
            for j_dipole in range(dipole_positions.shape[1]):  # For each dipole
                r_vecs = sensor_positions - dipole_positions[i_magnet, j_dipole]  # (n_sensors, 3)
                m_vec = (
                    magnetic_moments[i_magnet, j_dipole] * dipole_orientations[i_magnet, j_dipole]
                )  # Magnetic moment vector (3,)
                B_field = DipoleModel._dipole_field(r_vecs=r_vecs, m_vec=m_vec)  # (n_sensors, 3)
                B_total += B_field

        return B_total

    def get_expected_readings(
        self,
        x: float,
        y: float,
        z: float,
        RX_roll: float,
        RY_pitch: float,
        RZ_yaw: float,
    ) -> np.ndarray:
        # Return
        # - x,y,z readings for each sensor (superposition of all dipoles)
        # - R should be in degrees, but we will convert to radians for calculations
        # - The readings are in the fixed coordinate system of the sensors

        # Convert degrees to radians
        RX_roll_rad = np.radians(RX_roll)
        RY_pitch_rad = np.radians(RY_pitch)
        RZ_yaw_rad = np.radians(RZ_yaw)

        # Get sensor positions
        sensor_positions = self._get_sensor_positions()
        neutral_dipole_positions = self._get_dipole_positions_neutral()
        true_dipole_positions = DipoleModel._apply_6dof_dipole_positions(
            neutral_dipole_positions=neutral_dipole_positions,
            x=x,
            y=y,
            z=z,
            RX_roll=RX_roll_rad,
            RY_pitch=RY_pitch_rad,
            RZ_yaw=RZ_yaw_rad,
        )
        true_dipole_orientations = self._get_dipole_orientations(
            RX_roll=RX_roll_rad,
            RY_pitch=RY_pitch_rad,
            RZ_yaw=RZ_yaw_rad,
        )
        combined_magnetic_fields = DipoleModel._superposition_of_dipoles(
            sensor_positions=sensor_positions,
            dipole_positions=true_dipole_positions,
            dipole_orientations=true_dipole_orientations,
            magnetic_moments=np.array(
                [
                    [self._magnetic_moment_1 / self._n_dipoles] * self._n_dipoles,
                    [self._magnetic_moment_2 / self._n_dipoles] * self._n_dipoles,
                    [self._magnetic_moment_3 / self._n_dipoles] * self._n_dipoles,
                ]
            ),
            magnets=self._magnets,
        )

        return combined_magnetic_fields
