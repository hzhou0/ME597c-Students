# Import relevant libraries.
import json
import math

M_PI = 3.1415926535


class Logger:
    def __init__(self, filename, headers=None):
        if headers is None:
            headers = ["e", "e_dot", "e_int", "stamp"]
        self.filename = filename

        with open(self.filename, "w") as file:
            header_str = ""

            for header in headers:
                header_str += header
                header_str += ", "

            header_str += "\n"

            file.write(header_str)

    def log_values(self, values_list: list):
        with open(self.filename, "a") as file:
            # Write each value in the list as a CSV with a preceeding comma and a new line at the end.
            file.write(",".join(json.dumps(v) for v in values_list) + "\n")

    def save_log(self):
        pass


class FileReader:
    def __init__(self, filename):
        self.filename = filename

    def read_file(self):
        read_headers = False

        table: list[list[any]] = []
        headers: list[str] = []
        with open(self.filename, "r") as file:
            # Skip the header line

            if not read_headers:
                for line in file:
                    values = line.strip().split(",")

                    for val in values:
                        if val == "":
                            break
                        headers.append(val.strip())

                    read_headers = True
                    break

            next(file)

            # Read each line and extract values
            for line in file:
                if line[0]=='[':
                    end=line.find(']')
                    values=[line[:end+1]]+line[end+2:].strip().split(",")
                else:
                    values = line.strip().split(",")

                row = []

                for val in values:
                    if val == "":
                        break
                    row.append(json.loads(val.strip()))

                table.append(row)

        return headers, table


# Function for converting from Quaternion to Euler Angles.
def euler_from_quaternion(quat: list[float]):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat

    # Looking at the first two elements of the first row of the Euler Angle's matrix,
    # cos and sin of alpha (yaw angle) are multiplied by a common factor of cos(beta).
    # From the Quaternion to Rotation matrix, the following can be found.
    cosa_cosb = (w * w) + (x * x) - (y * y) - (z * z)
    sina_cosb = 2 * ((w * z) + (x * y))

    # Unpack the yaw by taking the inverse tangent and return it.
    yaw = math.atan2(sina_cosb, cosa_cosb)
    return yaw
