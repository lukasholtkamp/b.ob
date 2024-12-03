import numpy as np
import casadi as ca
from .transform import *
import json

def f(segments, s):
    result = ca.MX.zeros(
        3
    )  # Initialize a CasADi variable to store the selected (x, y) result

    for segment in segments:
        # Use ca.logic_and to check if s falls within the current segment's time bounds
        condition = ca.logic_and(s >= segment.start_time, s <= segment.end_time)

        # Use ca.if_else to choose the correct segment
        result = ca.if_else(
            condition, segment.f(s), result  # Use this segment if the condition is true
        )  # Keep the current result otherwise

    # Handle the case where s exceeds the last segment's end_time
    last_segment = segments[-1]
    result = ca.if_else(
        s > last_segment.end_time,
        last_segment.f(last_segment.end_time),  # Return the last segment's endpoint
        result,
    )

    return result

def load_path_segments(filename="path_segments_1.json"):
    """Load a list of PathSegment objects from a JSON file."""
    with open(filename, "r") as f:
        segments_data = json.load(f)
    path_segments = [PathSegment.from_dict(segment) for segment in segments_data]
    print(f"Path segments loaded from {filename}")
    return path_segments

# Define the PathSegment class
class PathSegment:
    def __init__(self, start_point=None, end_point=None, segment_type=None):
        self.start_point = (
            np.array(start_point) if start_point is not None else None
        )  # Starting point [x, y]
        self.end_point = (
            np.array(end_point) if end_point is not None else None
        )  # End point [x, y]
        self.start_time = None  # Start time
        self.end_time = None
        self.x_coefficients = None
        self.y_coefficients = None
        self.eta = None
        self.transformation_matrix = None  # 3x3 transformation matrix
        self.inverse_transformation_matrix = None  # Calculate inverse matrix
        self.segment_type = segment_type  # Segment type ('line' or 'parabola')
        self.distance = None
        self.rotation = None

    def to_dict(self):
        """Convert the PathSegment object to a dictionary."""
        return {
            "start_point": self.start_point.tolist(),
            "end_point": self.end_point.tolist(),
            "segment_type": self.segment_type,
            "start_time": self.start_time,
            "end_time": self.end_time,
            "x_coefficients": self.x_coefficients,
            "y_coefficients": self.y_coefficients,
            "eta": self.eta,
            "rotation": self.rotation,
            "distance": self.distance,
            "transformation_matrix": (
                self.transformation_matrix.tolist()
                if self.transformation_matrix is not None
                else None
            ),
            "inverse_transformation_matrix": (
                self.inverse_transformation_matrix.tolist()
                if self.inverse_transformation_matrix is not None
                else None
            ),
        }

    @staticmethod
    def from_dict(data):
        """Create a PathSegment object from a dictionary."""
        segment = PathSegment(
            start_point=data["start_point"],
            end_point=data["end_point"],
            segment_type=data["segment_type"]
        )
        segment.start_time = data["start_time"]
        segment.end_time = data["end_time"]
        segment.x_coefficients = data["x_coefficients"]
        segment.y_coefficients = data["y_coefficients"]
        segment.eta = data["eta"]
        segment.rotation = data["rotation"]
        segment.distance = data["distance"]
        segment.transformation_matrix = np.array(data["transformation_matrix"]) if data["transformation_matrix"] is not None else None
        segment.inverse_transformation_matrix = np.array(data["inverse_transformation_matrix"]) if data["inverse_transformation_matrix"] is not None else None
        return segment
    
    def straight_line_tf(self, s0, v_max, eps):
        """
        Returns a 3x3 transformation matrix that rotates a line segment defined by p1 and p2 to be
        parallel to the x-axis and translates it so the endpoints are equidistant from the y-axis.

        :param p1: The first endpoint of the line (x1, y1).
        :param p2: The second endpoint of the line (x2, y2).
        :return: A 3x3 transformation matrix.
        """
        # Calculate the angle theta between the line and the x-axis

        p1 = self.start_point
        p2 = self.end_point

        # After rotation, we want to translate the line such that its midpoint is on the y-axis
        self.distance = np.linalg.norm([p2[0] - p1[0], p2[1] - p1[1]])

        self.start_time = s0
        self.end_time = (self.distance / g(v_max, 0)) + s0

        ax = 0

        if (self.end_time - s0) == 0:
            bx = 10**6
        else:
            bx = (p2[0] - p1[0]) / (self.end_time - s0)

        cx = p1[0] - s0 * (bx)

        self.x_coefficients = [ax, bx, cx]

        ay = 0

        if (self.end_time - s0) == 0:
            by = 10**6
        else:
            by = (p2[1] - p1[1]) / (self.end_time - s0)

        cy = p1[1] - s0 * (by)

        self.y_coefficients = [ay, by, cy]

        self.rotation = np.arctan2(by, bx)

        self.eta = (ay / bx**2) * np.cos(self.rotation)

        self.set_transformation_matrix(
            transformation_matrix(
                self.rotation, (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
            )
        )

    def parab_tf(
        self,
        s0,
        theta,
        v_max,
        A_prime,
        B_prime,
        C_prime,
        D_prime,
        E_prime,
        F_prime,
        eps,
    ):

        if abs(C_prime) <= 0.1:
            vertex_x, vertex_y = calculate_vertex(A_prime, D_prime, E_prime, F_prime)
            self.eta = -A_prime / E_prime

        else:
            self.eta = C_prime / D_prime
            vertex_y, vertex_x = calculate_vertex(C_prime, E_prime, D_prime, F_prime)

        rvertx = np.cos(theta) * vertex_x - np.sin(theta) * vertex_y
        rverty = np.sin(theta) * vertex_x + np.cos(theta) * vertex_y

        if abs(C_prime) > 0.1:
            theta += np.pi / 2

        theta *= -1

        a = self.eta

        if theta <= 0:
            r = g(v_max, self.eta)
        else:
            r = -g(v_max, self.eta)

        shift = 0

        h = rvertx
        k = rverty

        ax = a * r**2 * np.sin(theta)
        bx = r * np.cos(theta) - 2 * a * shift * r * np.sin(theta)
        cx = a * shift**2 * np.sin(theta) + h - np.cos(theta) * shift

        ay = a * r**2 * np.cos(theta)
        by = -2 * a * shift * r * np.cos(theta) - r * np.sin(theta)
        cy = a * shift**2 * np.cos(theta) + k + shift * np.sin(theta)

        self.rotation = np.arctan2(by, bx)

        self.eta = (ay / bx**2) * np.cos(self.rotation)

        self.set_transformation_matrix(transformation_matrix(self.rotation, cx, cy))

        p1 = self.start_point
        p2 = self.end_point

        a1 = 1
        b1 = -(1 / np.tan(self.rotation))

        c1 = -(1 / np.tan(self.rotation)) * p1[0] - p1[1]
        c2 = -(1 / np.tan(self.rotation)) * h - k
        c3 = -(1 / np.tan(self.rotation)) * p2[0] - p2[1]

        d = np.abs(c1 - c3) / np.sqrt(a1**2 + b1**2)

        s1 = (
            -(
                (1 / np.sin(theta))
                * np.sqrt(
                    -8 * a * h * np.sin(theta)
                    + 8 * a * p1[0] * np.sin(theta)
                    + np.cos(2 * theta)
                    + 1
                )
            )
            / np.sqrt(2)
            + 2 * a * r * s0
            + (1 / np.tan(theta))
        ) / (2 * a)
        s2 = (
            (
                (1 / np.sin(theta))
                * np.sqrt(
                    -8 * a * h * np.sin(theta)
                    + 8 * a * p1[0] * np.sin(theta)
                    + np.cos(2 * theta)
                    + 1
                )
            )
            / np.sqrt(2)
            + 2 * a * r * s0
            + (1 / np.tan(theta))
        ) / (2 * a)

        s3 = -(
            (1 / np.cos(theta))
            * (
                np.sqrt(
                    -8 * a * k * np.cos(theta)
                    + 8 * a * p1[1] * np.cos(theta)
                    - np.cos(2 * theta)
                    + 1
                )
                / np.sqrt(2)
                - 2 * a * r * s0 * np.cos(theta)
                + np.sin(theta)
            )
        ) / (2 * a)
        s4 = (
            (
                (1 / np.cos(theta))
                * np.sqrt(
                    -8 * a * k * np.cos(theta)
                    + 8 * a * p1[1] * np.cos(theta)
                    - np.cos(2 * theta)
                    + 1
                )
            )
            / np.sqrt(2)
            + 2 * a * r * s0
            - np.tan(theta)
        ) / (2 * a)

        if (np.abs(s1 - s3) < 0.0001) or (np.abs(s1 - s4) < 0.0001):
            shift = s1
        else:
            shift = s2

        self.start_time = s0
        self.end_time = (d / g(v_max, self.eta)) + s0

        ax = a * r**2 * np.sin(theta)
        bx = r * np.cos(theta) - 2 * a * shift * r * np.sin(theta)
        cx = a * shift**2 * np.sin(theta) + h - np.cos(theta) * shift

        ay = a * r**2 * np.cos(theta)
        by = -2 * a * shift * r * np.cos(theta) - r * np.sin(theta)
        cy = a * shift**2 * np.cos(theta) + k + shift * np.sin(theta)

        check_x = np.abs(
            ax * self.end_time**2 + bx * self.end_time + cx - self.end_point[0]
        )
        check_y = np.abs(
            ay * self.end_time**2 + by * self.end_time + cy - self.end_point[1]
        )

        if check_x > 0.1 or check_y > 0.1:

            r *= -1

            shift = 0

            h = rvertx
            k = rverty

            ax = a * r**2 * np.sin(theta)
            bx = r * np.cos(theta) - 2 * a * shift * r * np.sin(theta)
            cx = a * shift**2 * np.sin(theta) + h - np.cos(theta) * shift

            ay = a * r**2 * np.cos(theta)
            by = -2 * a * shift * r * np.cos(theta) - r * np.sin(theta)
            cy = a * shift**2 * np.cos(theta) + k + shift * np.sin(theta)

            self.rotation = np.arctan2(by, bx)

            self.eta = (ay / bx**2) * np.cos(self.rotation)

            self.set_transformation_matrix(transformation_matrix(self.rotation, cx, cy))

            p1 = self.start_point
            p2 = self.end_point

            a1 = 1
            b1 = -(1 / np.tan(self.rotation))

            c1 = -(1 / np.tan(self.rotation)) * p1[0] - p1[1]
            c2 = -(1 / np.tan(self.rotation)) * h - k
            c3 = -(1 / np.tan(self.rotation)) * p2[0] - p2[1]

            d = np.abs(c1 - c3) / np.sqrt(a1**2 + b1**2)

            s1 = (
                -(
                    (1 / np.sin(theta))
                    * np.sqrt(
                        -8 * a * h * np.sin(theta)
                        + 8 * a * p1[0] * np.sin(theta)
                        + np.cos(2 * theta)
                        + 1
                    )
                )
                / np.sqrt(2)
                + 2 * a * r * s0
                + (1 / np.tan(theta))
            ) / (2 * a)
            s2 = (
                (
                    (1 / np.sin(theta))
                    * np.sqrt(
                        -8 * a * h * np.sin(theta)
                        + 8 * a * p1[0] * np.sin(theta)
                        + np.cos(2 * theta)
                        + 1
                    )
                )
                / np.sqrt(2)
                + 2 * a * r * s0
                + (1 / np.tan(theta))
            ) / (2 * a)

            s3 = -(
                (1 / np.cos(theta))
                * (
                    np.sqrt(
                        -8 * a * k * np.cos(theta)
                        + 8 * a * p1[1] * np.cos(theta)
                        - np.cos(2 * theta)
                        + 1
                    )
                    / np.sqrt(2)
                    - 2 * a * r * s0 * np.cos(theta)
                    + np.sin(theta)
                )
            ) / (2 * a)
            s4 = (
                (
                    (1 / np.cos(theta))
                    * np.sqrt(
                        -8 * a * k * np.cos(theta)
                        + 8 * a * p1[1] * np.cos(theta)
                        - np.cos(2 * theta)
                        + 1
                    )
                )
                / np.sqrt(2)
                + 2 * a * r * s0
                - np.tan(theta)
            ) / (2 * a)

            if (np.abs(s1 - s3) < 0.0001) or (np.abs(s1 - s4) < 0.0001):
                shift = s1
            else:
                shift = s2

            self.start_time = s0
            self.end_time = (d / g(v_max, self.eta)) + s0

            ax = a * r**2 * np.sin(theta)
            bx = r * np.cos(theta) - 2 * a * shift * r * np.sin(theta)
            cx = a * shift**2 * np.sin(theta) + h - np.cos(theta) * shift

            ay = a * r**2 * np.cos(theta)
            by = -2 * a * shift * r * np.cos(theta) - r * np.sin(theta)
            cy = a * shift**2 * np.cos(theta) + k + shift * np.sin(theta)

        self.x_coefficients = [ax, bx, cx]
        self.y_coefficients = [ay, by, cy]

    # Function to set the transformation matrix later
    def set_transformation_matrix(self, matrix):
        self.transformation_matrix = matrix
        self.inverse_transformation_matrix = np.linalg.inv(matrix)

    # Function to set start and end points
    def set_points(self, start_point, end_point):
        self.start_point = np.array(start_point)
        self.end_point = np.array(end_point)

    # Function to set time bounds
    def set_times(self, start_time, v_max):
        start_point = self.transformed_startpoint()
        end_point = self.transformed_endpoint()

        self.start_time = start_time

    # Function to get transformed start point
    def transformed_startpoint(self):
        if self.transformation_matrix is None:
            raise ValueError("Transformation matrix has not been set.")
        point = self.inv_transform_p(self.start_point)
        return (float(point[0]), float(point[1]))

    # Function to get transformed end point
    def transformed_endpoint(self):
        if self.transformation_matrix is None:
            raise ValueError("Transformation matrix has not been set.")
        point = self.inv_transform_p(self.end_point)
        return (float(point[0]), float(point[1]))

    # Function to transform a point using the transformation matrix
    def transform_point(self, point, matrix):
        point_homogeneous = ca.vertcat(
            point[0], point[1], 1
        )  # Use CasADi vertcat for symbolic points
        transformed_point = ca.mtimes(matrix, point_homogeneous)
        return transformed_point[:2]  # Return (x, y) from homogeneous coordinates

    def transform_p(self, point):
        point_homogeneous = np.array(
            [point[0], point[1], 1]
        )  # Use CasADi vertcat for symbolic points
        transformed_position = self.transformation_matrix @ point_homogeneous
        if self.segment_type == "line":
            return transformed_position[:2].round(decimals=2)
        else:
            return transformed_position[:2]

    def inv_transform_p(self, point):
        point_homogeneous = np.array(
            [point[0], point[1], 1]
        )  # Use CasADi vertcat for symbolic points
        transformed_position = self.inverse_transformation_matrix @ point_homogeneous
        if self.segment_type == "line":
            return transformed_position[:2].round(decimals=2)
        else:
            return transformed_position[:2]

    # Function f(s) to calculate (x, y, 0) based on the parameter s
    def f(self, s):
        if self.start_time is None or self.end_time is None:
            raise ValueError("Start time and end time must be set.")

        if self.segment_type == "line":
            x = (
                self.x_coefficients[0] * s**2
                + self.x_coefficients[1] * s
                + self.x_coefficients[2]
            )
            y = (
                self.y_coefficients[0] * s**2
                + self.y_coefficients[1] * s
                + self.y_coefficients[2]
            )
            return ca.vertcat(x, y, 0)

        elif self.segment_type == "parabola":
            x = (
                self.x_coefficients[0] * s**2
                + self.x_coefficients[1] * s
                + self.x_coefficients[2]
            )
            y = (
                self.y_coefficients[0] * s**2
                + self.y_coefficients[1] * s
                + self.y_coefficients[2]
            )
            return ca.vertcat(x, y, 0)

        else:
            raise NotImplementedError("Segment type not supported.")
