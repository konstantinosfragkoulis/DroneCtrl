
from enum import Enum
import cv2 as cv
from nptyping import NDArray
from typing import *

"""
The code below is licensed under the MIT License.


Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Contains helper functions to support common operations.
"""
class ColorBGR(Enum):
    """
    Common colors defined in the blue-green-red (BGR) format, with each channel
    ranging from 0 to 255 inclusive.
    """

    blue = (255, 0, 0)
    light_blue = (255, 255, 0)
    green = (0, 255, 0)
    dark_green = (0, 127, 0)
    yellow = (0, 255, 255)
    orange = (0, 127, 255)
    red = (0, 0, 255)
    pink = (255, 0, 255)
    purple = (255, 0, 127)
    black = (0, 0, 0)
    dark_gray = (63, 63, 63)
    gray = (127, 127, 127)
    light_gray = (191, 191, 191)
    white = (255, 255, 255)
    brown = (0, 63, 127)

def clamp(value: float, min: float, max: float) -> float:
    """
    Clamps a value between a minimum and maximum value.

    Args:
        value: The input to clamp.
        min: The minimum allowed value.
        max: The maximum allowed value.

    Returns:
        The value saturated between min and max.

    Example::

        # a will be set to 3
        a = rc_utils.clamp(3, 0, 10)

        # b will be set to 0
        b = rc_utils.remap_range(-2, 0, 10)

        # c will be set to 10
        c = rc_utils.remap_range(11, 0, 10)
    """
    return min if value < min else max if value > max else value

def remap_range(
    val: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
    saturate: bool = False,
) -> float:
    """
    Remaps a value from one range to another range.

    Args:
        val: A number form the old range to be rescaled.
        old_min: The inclusive 'lower' bound of the old range.
        old_max: The inclusive 'upper' bound of the old range.
        new_min: The inclusive 'lower' bound of the new range.
        new_max: The inclusive 'upper' bound of the new range.
        saturate: If true, the new_min and new_max limits are enforced.

    Note:
        min need not be less than max; flipping the direction will cause the sign of
        the mapping to flip.  val does not have to be between old_min and old_max.

    Example::

        # a will be set to 25
        a = rc_utils.remap_range(5, 0, 10, 0, 50)

        # b will be set to 975
        b = rc_utils.remap_range(5, 0, 20, 1000, 900)

        # c will be set to 30
        c = rc_utils.remap_range(2, 0, 1, -10, 10)

        # d will be set to 10
        d = rc_utils.remap_range(2, 0, 1, -10, 10, True)
    """
    old_span: float = old_max - old_min
    new_span: float = new_max - new_min
    new_val: float = new_min + new_span * (float(val - old_min) / float(old_span))

    # If saturate is true, enforce the new_min and new_max limits
    if saturate:
        if new_min < new_max:
            return clamp(new_val, new_min, new_max)
        else:
            return clamp(new_val, new_max, new_min)

    return new_val

def find_contours(
    color_image: Any,
    hsv_lower: Tuple[int, int, int],
    hsv_upper: Tuple[int, int, int],
) -> List[NDArray]:
    """
    Finds all contours of the specified color range in the provided image.

    Args:
        color_image: The color image in which to find contours,
            with pixels represented in the bgr (blue-green-red) format.
        hsv_lower: The lower bound for the hue, saturation, and value of colors
            to contour.
        hsv_upper: The upper bound for the hue, saturation, and value of the colors
            to contour.

    Returns:
        A list of contours around the specified color ranges found in color_image.

    Note:
        Each channel in hsv_lower and hsv_upper ranges from 0 to 255.

    Example::

        # Define the lower and upper hsv ranges for the color blue
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)

        # Extract contours around all blue portions of the current image
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
    """
    assert (
        0 <= hsv_lower[0] <= 179 and 0 <= hsv_upper[0] <= 179
    ), f"The hue of hsv_lower ({hsv_lower}) and hsv_upper ({hsv_upper}) must be in the range 0 to 179 inclusive."

    assert (
        0 <= hsv_lower[1] <= 255 and 0 <= hsv_upper[1] <= 255
    ), f"The saturation of hsv_lower ({hsv_lower}) and hsv_upper ({hsv_upper}) must be in the range 0 to 255 inclusive."

    assert (
        0 <= hsv_lower[0] <= 255 and 0 <= hsv_upper[0] <= 255
    ), f"The value of hsv_lower ({hsv_lower}) and hsv_upper ({hsv_upper}) must be in the range 0 to 255 inclusive."

    assert (
        hsv_lower[1] <= hsv_upper[1]
    ), f"The saturation channel of hsv_lower ({hsv_lower}) must be less than that of hsv_upper ({hsv_upper})."

    assert (
        hsv_lower[2] <= hsv_upper[2]
    ), f"The value channel of hsv_lower ({hsv_lower}) must be less than that of of hsv_upper ({hsv_upper})."

    # Convert the image from a blue-green-red pixel representation to a
    # hue-saturation-value representation
    hsv_image = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)

    # Create a mask containing the pixels in the image with hsv values between
    # hsv_lower and hsv_upper.
    mask: NDArray
    if hsv_lower[0] <= hsv_upper[0]:
        mask = cv.inRange(hsv_image, hsv_lower, hsv_upper)

    # If the color range passes the 255-0 boundary, we must create two masks
    # and merge them
    else:
        mask1 = cv.inRange(hsv_image, hsv_lower, (255, hsv_upper[1], hsv_upper[2]))
        mask2 = cv.inRange(hsv_image, (0, hsv_lower[1], hsv_lower[2]), hsv_upper)
        mask = cv.bitwise_or(mask1, mask2)

    # Find and return a list of all contours of this mask
    return cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[0]

def get_largest_contour(
    contours: List[NDArray], min_area: int = 30
) -> Optional[NDArray]:
    """
    Finds the largest contour with size greater than min_area.

    Args:
        contours: A list of contours found in an image.
        min_area: The smallest contour to consider (in number of pixels)

    Returns:
        The largest contour from the list, or None if no contour was larger
        than min_area.

    Example::

        # Extract the blue contours
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )

        # Find the largest contour
        largest_contour = rc_utils.get_largest_contour(contours)
    """
    # Check that the list contains at least one contour
    if len(contours) == 0:
        return None

    # Find and return the largest contour if it is larger than min_area
    greatest_contour = max(contours, key=cv.contourArea)
    if cv.contourArea(greatest_contour) < min_area:
        return None

    return greatest_contour

def draw_contour(
    color_image: Any,
    contour: NDArray,
    color: Tuple[int, int, int] = ColorBGR.green.value,
) -> None:
    """
    Draws a contour on the provided image.

    Args:
        color_image: The color image on which to draw the contour.
        contour: The contour to draw on the image.
        color: The color to draw the contour, specified as
            blue-green-red channels each ranging from 0 to 255 inclusive.

    Example::

        image = rc.camera.get_color_image()

        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(image, BLUE_HSV_MIN, BLUE_HSV_MAX)
        largest_contour = rc_utils.get_largest_contour(contours)

        # Draw this contour onto image
        if (largest_contour is not None):
            draw_contour(image, largest_contour)
    """
    for channel in color:
        assert (
            0 <= channel <= 255
        ), f"Each channel in color ({color}) must be in the range 0 to 255 inclusive."

    cv.drawContours(color_image, [contour], 0, color, 3)

def draw_circle(
    color_image: Any,
    center: Tuple[int, int],
    color: Tuple[int, int, int] = ColorBGR.yellow.value,
    radius: int = 6,
) -> None:
    """
    Draws a circle on the provided image.

    Args:
        color_image: The color image on which to draw the contour.
        center: The pixel (row, column) of the center of the image.
        color: The color to draw the circle, specified as
            blue-green-red channels each ranging from 0 to 255 inclusive.
        radius: The radius of the circle in pixels.

    Example::

        image = rc.camera.get_color_image()

        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(image, BLUE_HSV_MIN, BLUE_HSV_MAX)
        largest_contour = rc_utils.get_largest_contour(contours)

        # Draw a dot at the center of this contour in red
        if (largest_contour is not None):
            center = get_contour_center(contour)
            draw_circle(image, center, rc_utils.ColorBGR.red.value)
    """
    for channel in color:
        assert (
            0 <= channel <= 255
        ), f"Each channel in color ({color}) must be in the range 0 to 255 inclusive."

    assert (
        0 <= center[0] < color_image.shape[0]
    ), f"center[0] ({center[0]}) must be a pixel row index in color_image."
    assert (
        0 <= center[1] < color_image.shape[1]
    ), f"center[1] ({center[1]}) must be a pixel column index in color_image."
    assert radius > 0, f"radius ({radius}) must be a positive integer."

    # cv.circle expects the center in (column, row) format
    cv.circle(color_image, (center[1], center[0]), radius, color, -1)

def get_contour_center(contour: NDArray) -> Optional[Tuple[int, int]]:
    """
    Finds the center of a contour from an image.

    Args:
        contour: The contour of which to find the center.

    Returns:
        The (row, column) of the pixel at the center of the contour, or None if the
        contour is empty.

    Example::

        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
        largest_contour = rc_utils.get_largest_contour(contours)

        # Find the center of this contour if it exists
        if (largest_contour is not None):
            center = rc_utils.get_contour_center(largest_contour)
    """
    M = cv.moments(contour)

    # Check that the contour is not empty
    # (M["m00"] is the number of pixels in the contour)
    if M["m00"] <= 0:
        return None

    # Compute and return the center of mass of the contour
    center_row = round(M["m01"] / M["m00"])
    center_column = round(M["m10"] / M["m00"])
    return (center_row, center_column)

def get_contour_area(contour: NDArray) -> float:
    """
    Finds the area of a contour from an image.

    Args:
        contour: The contour of which to measure the area.

    Returns:
        The number of pixels contained within the contour

    Example::

        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
        largest_contour = rc_utils.get_largest_contour(contours)

        # Find the area of this contour (will evaluate to 0 if no contour was found)
        area = rc_utils.get_contour_area(contour)
    """
    return cv.contourArea(contour)
"""
The code above is licensed under the MIT License.
"""

def findContour(image, *colors):
    contours = []
    for color in colors:
        contour = get_largest_contour(find_contours(image, color[0], color[1]))
        if contour is not None:
            contours.append(contour)
            draw_contour(image, contour, color[1])
            center = get_contour_center(contour)
            draw_circle(image, center)
    if len(contours) > 0:
        largest_contour = max(contours, key=get_contour_area)
        largest_center = get_contour_center(largest_contour)
        return largest_contour, largest_center
    else:
        return None, None
