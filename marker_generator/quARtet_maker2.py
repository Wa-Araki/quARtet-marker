import adsk.core, adsk.fusion, adsk.cam
import traceback, math
import struct
import zlib
import json
import os

# =============================================================================
# Utility Functions
# =============================================================================
def get_script_directory():
    """
    Get the directory path where this script file is located.
    
    Returns:
        str: Absolute path of the script directory.
    """
    return os.path.dirname(os.path.abspath(__file__))

def get_file_path():
    """
    Open a file dialog and return the selected file path.
    
    Returns:
        str or None: The selected file path or None if no file was selected.
    """
    app = adsk.core.Application.get()
    ui = app.userInterface

    file_dialog = ui.createFileDialog()
    file_dialog.title = "Select a file"
    file_dialog.filter = "All Files (*.*)"
    file_dialog.initialDirectory = get_script_directory()
    file_dialog.isMultiSelectEnabled = False
    dialog_result = file_dialog.showOpen()

    if dialog_result == adsk.core.DialogResults.DialogOK:
        return file_dialog.filename
    else:
        return None

def create_value_input(value):
    """
    Create a ValueInput from a numeric value or a unit string.
    
    Parameters:
        value (int, float, str): The value for the input. If numeric, it's treated as a real number.
                                 If string, it should be a valid units string (e.g. "10 mm").
    
    Returns:
        adsk.core.ValueInput: Created ValueInput object.
    """
    if isinstance(value, (int, float)):
        return adsk.core.ValueInput.createByReal(value)
    elif isinstance(value, str):
        return adsk.core.ValueInput.createByString(value)
    else:
        raise ValueError("Invalid value type: must be a number or a string with units")

# =============================================================================
# Feature Creation Functions
# =============================================================================
def extrude(component, input_entity, distance, taper_angle=0, taper_angle2=None,
            operation_type='Join', direction='OneSide'):
    """
    Create an extrusion feature with the specified parameters.
    
    Parameters:
        component (adsk.fusion.Component): The component where the extrusion will be created.
        input_entity (adsk.core.Base): The profile or face to extrude.
        distance (float or tuple): Distance to extrude. If 'OneSide' or 'Symmetric', 
                                   provide a single value. If 'TwoSides', provide a tuple.
        taper_angle (float or str): The taper angle for the extrusion.
        taper_angle2 (float or str, optional): The second taper angle if using two-sided extrusion.
        operation_type (str): The type of feature operation ('Join', 'Cut', 'Intersect', 'NewBody', 'NewComponent').
        direction (str): The direction type for extrusion ('OneSide', 'TwoSides', 'Symmetric').
    
    Returns:
        adsk.fusion.ExtrudeFeature: The created extrusion feature.
    """
    operations = {
        'Join': adsk.fusion.FeatureOperations.JoinFeatureOperation,
        'Cut': adsk.fusion.FeatureOperations.CutFeatureOperation,
        'Intersect': adsk.fusion.FeatureOperations.IntersectFeatureOperation,
        'NewBody': adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        'NewComponent': adsk.fusion.FeatureOperations.NewComponentFeatureOperation,
    }

    extrudes = component.features.extrudeFeatures
    extrude_input = extrudes.createInput(input_entity, operations[operation_type])
    taper = create_value_input(taper_angle)

    if direction == 'OneSide':
        extrude_distance = create_value_input(distance)
        extrude_input.setDistanceExtent(False, extrude_distance)
        extrude_input.taperAngle = taper

    elif direction == 'TwoSides':
        distance1, distance2 = distance
        extrude_distance1 = create_value_input(distance1)
        extrude_distance2 = create_value_input(distance2)
        taper_angle2_value = taper if taper_angle2 is None else create_value_input(taper_angle2)
        extrude_input.setTwoSidesExtent(
            adsk.fusion.DistanceExtentDefinition.create(extrude_distance1),
            adsk.fusion.DistanceExtentDefinition.create(extrude_distance2),
            taper,
            taper_angle2_value
        )

    elif direction == 'Symmetric':
        extrude_distance = create_value_input(distance)
        extrude_input.setSymmetricExtent(extrude_distance, False)
        extrude_input.taperAngle = taper
    else:
        raise ValueError("Invalid direction: choose 'OneSide', 'TwoSides', or 'Symmetric'")

    return extrudes.add(extrude_input)

def create_rectangular_pattern(component, target_bodies, direction_one, quantity_one, distance_one,
                               pattern_type='extent', direction_two=None, quantity_two=None, distance_two=None):
    """
    Create a rectangular pattern for the given bodies.

    Parameters:
        component (adsk.fusion.Component): The component to add the pattern to.
        target_bodies (list or adsk.core.ObjectCollection): Bodies to pattern.
        direction_one (adsk.fusion.SketchLine): The first direction line.
        quantity_one (int): Number of instances in the first direction.
        distance_one (float): The spacing or extent distance in the first direction.
        pattern_type (str): 'extent' or 'spacing' for pattern distance type.
        direction_two (adsk.fusion.SketchLine, optional): The second direction line.
        quantity_two (int, optional): Number of instances in the second direction.
        distance_two (float, optional): The spacing/extent distance in the second direction.

    Returns:
        adsk.fusion.RectangularPatternFeature: The rectangular pattern feature.
    """
    entities = adsk.core.ObjectCollection.create()
    if isinstance(target_bodies, list):
        for body in target_bodies:
            entities.add(body)
    elif isinstance(target_bodies, adsk.core.ObjectCollection):
        entities = target_bodies
    else:
        entities.add(target_bodies)

    quantity_one_val = adsk.core.ValueInput.createByReal(quantity_one)
    distance_one_val = adsk.core.ValueInput.createByReal(distance_one)

    if pattern_type == 'extent':
        pattern_distance_type = adsk.fusion.PatternDistanceType.ExtentPatternDistanceType
    elif pattern_type == 'spacing':
        pattern_distance_type = adsk.fusion.PatternDistanceType.SpacingPatternDistanceType
    else:
        raise ValueError("Invalid pattern_type: choose 'extent' or 'spacing'")

    rectangular_patterns = component.features.rectangularPatternFeatures
    rectangular_pattern_input = rectangular_patterns.createInput(
        entities,
        direction_one,
        quantity_one_val,
        distance_one_val,
        pattern_distance_type
    )

    if direction_two and quantity_two and distance_two:
        quantity_two_val = adsk.core.ValueInput.createByReal(quantity_two)
        distance_two_val = adsk.core.ValueInput.createByReal(distance_two)
        rectangular_pattern_input.setDirectionTwo(direction_two, quantity_two_val, distance_two_val)

    return rectangular_patterns.add(rectangular_pattern_input)

def create_circular_pattern(component, target_bodies, axis, is_symmetric, quantity, angle):
    """
    Create a circular pattern of the given bodies around an axis.
    
    Parameters:
        component (adsk.fusion.Component): The component to add the pattern to.
        target_bodies (list or adsk.core.ObjectCollection): Bodies to pattern.
        axis (adsk.fusion.ConstructionAxis): The axis to pattern around.
        is_symmetric (bool): Whether the pattern is symmetric.
        quantity (int): The number of pattern instances.
        angle (float): The angle (in degrees) for the pattern.
    
    Returns:
        adsk.fusion.CircularPatternFeature: The circular pattern feature.
    """
    entities = adsk.core.ObjectCollection.create()
    if isinstance(target_bodies, list):
        for body in target_bodies:
            entities.add(body)
    elif isinstance(target_bodies, adsk.core.ObjectCollection):
        entities = target_bodies
    else:
        entities.add(target_bodies)

    circular_patterns = component.features.circularPatternFeatures
    circular_pattern_input = circular_patterns.createInput(entities, axis)
    circular_pattern_input.quantity = adsk.core.ValueInput.createByReal(quantity)
    circular_pattern_input.totalAngle = adsk.core.ValueInput.createByString(f"{angle} deg")
    circular_pattern_input.isSymmetic = is_symmetric

    return circular_patterns.add(circular_pattern_input)

def combine_bodies(component, target_body, tool_bodies, operation_type='Join', is_keep_tool_bodies=False):
    """
    Combine multiple bodies using the specified operation (Join, Cut, Intersect).
    
    Parameters:
        component (adsk.fusion.Component): The component where the combine feature is created.
        target_body (adsk.fusion.BRepBody): The target body.
        tool_bodies (adsk.core.ObjectCollection): The tool bodies.
        operation_type (str): The combine operation ('Join', 'Cut', 'Intersect').
        is_keep_tool_bodies (bool): Whether to keep the tool bodies after combine.
    
    Returns:
        adsk.fusion.CombineFeature: The resulting combine feature.
    """
    combine_features = component.features.combineFeatures
    combine_input = combine_features.createInput(target_body, tool_bodies)
    if operation_type == 'Join':
        combine_input.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    elif operation_type == 'Cut':
        combine_input.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    elif operation_type == 'Intersect':
        combine_input.operation = adsk.fusion.FeatureOperations.IntersectFeatureOperation

    combine_input.isKeepToolBodies = is_keep_tool_bodies
    return combine_features.add(combine_input)

def make_AR(component, bodies, AR_No, AR_pattern):
    """
    Creates a combined AR marker body by joining or hiding bodies according to the AR pattern.
    
    Parameters:
        component (adsk.fusion.Component): The root component.
        bodies (list of adsk.fusion.BRepBody): The bodies making up the pattern.
        AR_No (int): The AR tag index (0 to 3).
        AR_pattern (list of bool): Pattern of dots (True means hide that dot, False means keep).
    
    Returns:
        (adsk.fusion.CombineFeature, adsk.core.ObjectCollection): The combined AR body and the hidden bodies.
    """
    hidden_bodies = adsk.core.ObjectCollection.create()
    combined_bodies = []
    for i, v in enumerate(AR_pattern):
        current_body = bodies[i*4+AR_No]
        if v:
            current_body.isVisible = False
            hidden_bodies.add(current_body)
        else:
            combined_bodies.append(current_body)

    target_body = combined_bodies[0]
    tool_bodies = adsk.core.ObjectCollection.create()
    for body in combined_bodies[1:]:
        tool_bodies.add(body)

    return combine_bodies(component, target_body, tool_bodies, operation_type="Join", is_keep_tool_bodies=False), hidden_bodies

# =============================================================================
# Image Processing Functions
# =============================================================================
def read_png(file_path):
    """
    Read a PNG file and return pixel data as a 2D boolean array where True/False 
    indicates pixel intensity above or below a threshold.
    
    Parameters:
        file_path (str): Path to the PNG file.
    
    Returns:
        list of list of bool: Pixel data.
    """
    def paeth_predictor(a, b, c):
        p = a + b - c
        pa = abs(p - a)
        pb = abs(p - b)
        pc = abs(p - c)
        if pa <= pb and pa <= pc:
            return a
        elif pb <= pc:
            return b
        else:
            return c

    def apply_filter(filter_type, scanline, prev_scanline, bpp):
        result = bytearray(scanline)
        if filter_type == 0:  # None
            return result
        elif filter_type == 1:  # Sub
            for i in range(bpp, len(scanline)):
                result[i] = (result[i] + result[i - bpp]) % 256
        elif filter_type == 2:  # Up
            if prev_scanline is not None:
                for i in range(len(scanline)):
                    result[i] = (result[i] + prev_scanline[i]) % 256
        elif filter_type == 3:  # Average
            for i in range(len(scanline)):
                left = result[i - bpp] if i >= bpp else 0
                up = prev_scanline[i] if prev_scanline is not None else 0
                result[i] = (result[i] + (left + up) // 2) % 256
        elif filter_type == 4:  # Paeth
            for i in range(len(scanline)):
                left = result[i - bpp] if i >= bpp else 0
                up = prev_scanline[i] if prev_scanline is not None else 0
                up_left = prev_scanline[i - bpp] if (prev_scanline is not None and i >= bpp) else 0
                result[i] = (result[i] + paeth_predictor(left, up, up_left)) % 256
        else:
            raise ValueError(f"Unsupported filter type: {filter_type}")
        return result

    with open(file_path, 'rb') as f:
        data = f.read()

    assert data[:8] == b'\x89PNG\r\n\x1a\n', "Not a PNG file"

    chunks = []
    idx = 8
    while idx < len(data):
        length = struct.unpack('>I', data[idx:idx+4])[0]
        chunk_type = data[idx+4:idx+8]
        chunk_data = data[idx+8:idx+8+length]
        chunks.append((chunk_type, chunk_data))
        idx += 12 + length

    ihdr = [chunk for chunk in chunks if chunk[0] == b'IHDR'][0][1]
    width, height = struct.unpack('>II', ihdr[:8])
    bit_depth, color_type, compression, filter_method, interlace = struct.unpack('>BBBBB', ihdr[8:])

    idat_data = b''.join(chunk[1] for chunk in chunks if chunk[0] == b'IDAT')
    decompressed_data = zlib.decompress(idat_data)

    if color_type == 2:
        bytes_per_pixel = 3
    elif color_type == 6:
        bytes_per_pixel = 4
    else:
        raise ValueError("Unsupported color type")

    stride = width * bytes_per_pixel + 1

    image_data = []
    prev_scanline = None
    for y in range(height):
        filter_type = decompressed_data[y * stride]
        scanline = decompressed_data[y * stride + 1:(y + 1) * stride]
        filtered_scanline = apply_filter(filter_type, scanline, prev_scanline, bytes_per_pixel)
        image_data.append(filtered_scanline)
        prev_scanline = filtered_scanline

    pixel_data = []
    for row in image_data:
        pixel_row = []
        for x in range(width):
            if color_type == 2:
                r, g, b = row[x*3:x*3+3]
            elif color_type == 6:
                r, g, b, _ = row[x*4:x*4+4]
            avg = (r + g + b) / 3
            pixel_row.append(avg > 128)
        pixel_data.append(pixel_row)

    return pixel_data

def rotate90_pixel_data(pixel_data, k=1):
    """
    Rotate pixel data by 90 degrees k times.
    
    Parameters:
        pixel_data (list of list of bool): The pixel data.
        k (int): Number of 90-degree rotations.
    
    Returns:
        list of list of bool: Rotated pixel data.
    """
    for _ in range(k):
        rotated = []
        for col in range(len(pixel_data[0])):
            rotated_row = [row[-1 - col] for row in pixel_data]
            rotated.append(rotated_row)
        pixel_data = rotated
    return pixel_data

def trim_pixel_data(pixel_data, k=1):
    """
    Trim k rows/columns from each border of the pixel_data.
    
    Parameters:
        pixel_data (list of list of bool): The pixel data.
        k (int): Number of rows/columns to trim from each edge.
    
    Returns:
        list of list of bool: Trimmed pixel data.
    """
    trimmed = []
    for r, row in enumerate(pixel_data):
        if r < k or r >= (len(pixel_data)-k):
            continue
        trimmed.append(row[k:-k])
    return trimmed

def ravel_pixel_data(pixel_data):
    """
    Flatten the 2D pixel data into a 1D list.
    
    Parameters:
        pixel_data (list of list of bool): The pixel data.
    
    Returns:
        list of bool: Flattened pixel data.
    """
    return [val for row in pixel_data for val in row]

# =============================================================================
# Mode-specific drawing functions
# =============================================================================

# Diagonal mode (original mode 1) functions
def create_base_square(sketches, xy_plane, base_size):
    """
    Create a square sketch centered at the origin in the XY plane.
    
    Parameters:
        sketches (adsk.fusion.Sketches): The sketches collection.
        xy_plane (adsk.fusion.ConstructionPlane): The XY plane.
        base_size (float): The size of the square.
    
    Returns:
        adsk.fusion.Sketch: The created sketch.
    """
    base_sketch = sketches.add(xy_plane)
    base_sketch.sketchCurves.sketchLines.addCenterPointRectangle(
        adsk.core.Point3D.create(0, 0, 0),
        adsk.core.Point3D.create(base_size / 2, base_size / 2, 0)
    )
    return base_sketch

def extrude_square(root_comp, base_sketch, extrude_up, extrude_down, tilt_degree):
    """
    Extrude the base square sketch into a tapered body.
    
    Parameters:
        root_comp (adsk.fusion.Component): The root component.
        base_sketch (adsk.fusion.Sketch): The sketch of the square.
        extrude_up (float): The distance to extrude upwards.
        extrude_down (float): The distance to extrude downwards.
        tilt_degree (float): The tilt angle.
    
    Returns:
        adsk.fusion.ExtrudeFeature: The extrusion feature of the square.
    """
    profiles = base_sketch.profiles
    base_profile = profiles.item(0)

    return extrude(
        root_comp,
        base_profile,
        (extrude_up, extrude_down),
        taper_angle=f"{tilt_degree-90} deg",
        taper_angle2=f"{90-tilt_degree} deg",
        operation_type='NewBody',
        direction='TwoSides'
    )

def draw_square_on_inclined_face_diagonal(sketches, inclined_face, base_size, tag_size_cm, dot_num, tilt_degree):
    """
    Draw a smaller square on the inclined face of the extruded body for diagonal mode.
    
    Parameters:
        sketches (adsk.fusion.Sketches): The sketches collection.
        body (adsk.fusion.BRepBody): The extruded body.
        base_size (float): Base size of the main square.
        tag_size_cm (float): Tag size in cm.
        dot_num (int): Number of dots per side in the tag.
        tilt_degree (float): The tilt angle.
    
    Returns:
        adsk.fusion.Sketch: The inclined sketch with the square.
    """
    inclined_sketch = sketches.add(inclined_face)
    inclined_square_size = tag_size_cm / dot_num
    lines = inclined_sketch.sketchCurves.sketchLines

    midpoint1 = adsk.core.Point3D.create(0, -base_size * math.cos(math.radians(tilt_degree)) / 2, 0)
    midpoint2 = adsk.core.Point3D.create(
        inclined_square_size / (2**0.5),
        -base_size * math.cos(math.radians(tilt_degree)) / 2 + inclined_square_size / (2**0.5),
        0
    )
    midpoint3 = adsk.core.Point3D.create(
        0,
        -base_size * math.cos(math.radians(tilt_degree)) / 2 + inclined_square_size * (2**0.5),
        0
    )
    midpoint4 = adsk.core.Point3D.create(
        -inclined_square_size / (2**0.5),
        -base_size * math.cos(math.radians(tilt_degree)) / 2 + inclined_square_size / (2**0.5),
        0
    )

    lines.addByTwoPoints(midpoint1, midpoint2)
    lines.addByTwoPoints(midpoint2, midpoint3)
    lines.addByTwoPoints(midpoint3, midpoint4)
    lines.addByTwoPoints(midpoint4, midpoint1)

    return inclined_sketch

def draw_whole_square_on_inclined_face_diagonal(sketches, inclined_face, base_size, tag_size_cm, tilt_degree):
    """
    Draw a whole square on the inclined face of the extruded body for diagonal mode.
    
    Parameters:
        sketches (adsk.fusion.Sketches): The sketches collection.
        body (adsk.fusion.BRepBody): The extruded body.
        base_size (float): Base size of the main square.
        tag_size_cm (float): Tag size in cm.
        tilt_degree (float): The tilt angle.
    
    Returns:
        adsk.fusion.Sketch: The inclined sketch with the square.
    """
    inclined_sketch = sketches.add(inclined_face)
    inclined_square_size = tag_size_cm
    lines = inclined_sketch.sketchCurves.sketchLines

    midpoint1 = adsk.core.Point3D.create(0, -base_size * math.cos(math.radians(tilt_degree)) / 2, 0)
    midpoint2 = adsk.core.Point3D.create(
        inclined_square_size / (2**0.5),
        -base_size * math.cos(math.radians(tilt_degree)) / 2 + inclined_square_size / (2**0.5),
        0
    )
    midpoint3 = adsk.core.Point3D.create(
        0,
        -base_size * math.cos(math.radians(tilt_degree)) / 2 + inclined_square_size * (2**0.5),
        0
    )
    midpoint4 = adsk.core.Point3D.create(
        -inclined_square_size / (2**0.5),
        -base_size * math.cos(math.radians(tilt_degree)) / 2 + inclined_square_size / (2**0.5),
        0
    )

    lines.addByTwoPoints(midpoint1, midpoint2)
    lines.addByTwoPoints(midpoint2, midpoint3)
    lines.addByTwoPoints(midpoint3, midpoint4)
    lines.addByTwoPoints(midpoint4, midpoint1)

    return inclined_sketch

def draw_square_on_bottom_face_diagonal(sketches, body, base_size, bottom_height_cm, tilt_degree):
    """
    Draw a square on the bottom face of the extruded body.
    
    Parameters:
        sketches (adsk.fusion.Sketches): The sketches collection.
        body (adsk.fusion.BRepBody): The extruded body.
        base_size (float): Base size of the main square.
        bottom_height_cm (float): The bottom height in cm.
        tilt_degree (float): The tilt angle.
    
    Returns:
        adsk.fusion.Sketch: The bottom sketch with the square.
    """
    min_z = float("inf")
    bottom_face = None

    for face in body.faces:
        bounding_box = face.boundingBox
        z = (bounding_box.minPoint.z + bounding_box.maxPoint.z) / 2
        if z < min_z:
            min_z = z
            bottom_face = face

    bottom_sketch = sketches.add(bottom_face)
    lines = bottom_sketch.sketchCurves.sketchLines
    bottom_size = base_size + bottom_height_cm * 2 / math.tan(math.radians(tilt_degree))

    midpoint1 = adsk.core.Point3D.create(bottom_size / 2, 0, 0)
    midpoint2 = adsk.core.Point3D.create(0, bottom_size / 2, 0)
    midpoint3 = adsk.core.Point3D.create(-bottom_size / 2, 0, 0)
    midpoint4 = adsk.core.Point3D.create(0, -bottom_size / 2, 0)

    lines.addByTwoPoints(midpoint1, midpoint2)
    lines.addByTwoPoints(midpoint2, midpoint3)
    lines.addByTwoPoints(midpoint3, midpoint4)
    lines.addByTwoPoints(midpoint4, midpoint1)

    return bottom_sketch

# Sides mode (original mode 2) functions
def create_trapezoid_sketch(component, plane, base_length, tilt_angle, bottom_thickness):
    """
    Create a trapezoid sketch for sides mode.
    
    Parameters:
        component (adsk.fusion.Component): The component to draw in.
        plane (adsk.fusion.ConstructionPlane): The plane to draw on.
        base_length (float): The length of the base.
        tilt_angle (float): The tilt angle in degrees.
        bottom_thickness (float): The thickness of the bottom.
        
    Returns:
        adsk.fusion.Sketch: The created sketch.
    """
    sketch = component.sketches.add(plane)
    angle_radians = math.radians(tilt_angle)
    height = base_length * math.sin(angle_radians)
    lines = sketch.sketchCurves.sketchLines

    p1 = adsk.core.Point3D.create(0, 0, -bottom_thickness)
    p2 = adsk.core.Point3D.create(0, 0, height)
    p3 = adsk.core.Point3D.create(0, base_length * (1 - math.cos(angle_radians)), height)
    p4 = adsk.core.Point3D.create(0, base_length, 0)
    p5 = adsk.core.Point3D.create(0, base_length, -bottom_thickness)

    lines.addByTwoPoints(p1, p2)
    lines.addByTwoPoints(p2, p3)
    lines.addByTwoPoints(p3, p4)
    lines.addByTwoPoints(p4, p5)
    lines.addByTwoPoints(p5, p1)
    
    return sketch

def draw_square_on_inclined_face_sides(sketches, inclined_face, base_size, margin, dot_count, tilt_angle):
    """
    Draw a small square on the inclined face that serves as the AR tag area for sides mode.
    
    Parameters:
        sketches (adsk.fusion.Sketches): The sketches collection.
        inclined_face (adsk.fusion.BRepFace): The inclined face to draw on.
        base_size (float): The base size of the square.
        margin (float): Margin from the edge.
        dot_count (int): Number of dots per side in the tag.
        tilt_angle (float): The tilt angle in degrees.
        
    Returns:
        adsk.fusion.Sketch: The created sketch.
    """
    inclined_sketch = sketches.add(inclined_face)
    lines = inclined_sketch.sketchCurves.sketchLines
    dot_size = (base_size - margin * 2) / dot_count

    # Calculate starting point based on the reversed flag
    angle_rad = math.radians(tilt_angle)

    x0 = -base_size * math.cos(angle_rad) + margin
    y0 = base_size - margin
    z0 = 0

    pt1 = adsk.core.Point3D.create(x0, y0, z0)
    pt2 = adsk.core.Point3D.create(x0 + dot_size, y0, z0)
    pt3 = adsk.core.Point3D.create(x0 + dot_size, y0 - dot_size, z0)
    pt4 = adsk.core.Point3D.create(x0, y0 - dot_size, z0)

    lines.addByTwoPoints(pt1, pt2)
    lines.addByTwoPoints(pt2, pt3)
    lines.addByTwoPoints(pt3, pt4)
    lines.addByTwoPoints(pt4, pt1)

    return inclined_sketch

def draw_whole_square_on_inclined_face_sides(sketches, inclined_face, base_size, margin, tilt_angle):
    """
    Draw a whole square on the inclined face for sides mode.
    
    Parameters:
        sketches (adsk.fusion.Sketches): The sketches collection.
        inclined_face (adsk.fusion.BRepFace): The inclined face to draw on.
        base_size (float): The base size of the square.
        margin (float): Margin from the edge.
        tilt_angle (float): The tilt angle in degrees.
        
    Returns:
        adsk.fusion.Sketch: The created sketch.
    """
    inclined_sketch = sketches.add(inclined_face)
    lines = inclined_sketch.sketchCurves.sketchLines
    square_size = (base_size - margin * 2)

    # Calculate starting point based on the reversed flag
    angle_rad = math.radians(tilt_angle)

    x0 = -base_size * math.cos(angle_rad) + margin
    y0 = base_size - margin
    z0 = 0

    pt1 = adsk.core.Point3D.create(x0, y0, z0)
    pt2 = adsk.core.Point3D.create(x0 + square_size, y0, z0)
    pt3 = adsk.core.Point3D.create(x0 + square_size, y0 - square_size, z0)
    pt4 = adsk.core.Point3D.create(x0, y0 - square_size, z0)

    lines.addByTwoPoints(pt1, pt2)
    lines.addByTwoPoints(pt2, pt3)
    lines.addByTwoPoints(pt3, pt4)
    lines.addByTwoPoints(pt4, pt1)

    return inclined_sketch


def rotate_around_line_by_points(component, objects_to_rotate, x, y):
    """
    
    Parameters:
        component (adsk.fusion.Component): The component to operate on.
        objects_to_rotate (list or adsk.core.ObjectCollection): The objects to rotate.
        x, y (float)

    Returns:
        adsk.fusion.MoveFeature: The created move feature.
    """
    # Convert to an ObjectCollection
    entities = adsk.core.ObjectCollection.create()
    if isinstance(objects_to_rotate, list):
        for obj in objects_to_rotate:
            entities.add(obj)
    elif isinstance(objects_to_rotate, adsk.core.ObjectCollection):
        entities = objects_to_rotate
    else:
        entities.add(objects_to_rotate)
    
    transform1 = adsk.core.Matrix3D.create()
    transform1.translation = adsk.core.Vector3D.create(-x, -y, 0)
    move_input1 = component.features.moveFeatures.createInput(objects_to_rotate,transform1)
    move_feat1 = component.features.moveFeatures.add(move_input1)
    
    transform2 = adsk.core.Matrix3D.create()
    transform2.setToRotation(math.pi, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0,0,0))
    move_input2 = component.features.moveFeatures.createInput(objects_to_rotate,transform2)
    move_feat2 = component.features.moveFeatures.add(move_input2)

    transform1 = adsk.core.Matrix3D.create()
    transform1.translation = adsk.core.Vector3D.create(x, y, 0)
    move_input1 = component.features.moveFeatures.createInput(objects_to_rotate,transform1)
    move_feat1 = component.features.moveFeatures.add(move_input1)
    return True


# =============================================================================
# Main Pattern Creation Function
# =============================================================================
def create_pattern(
    AR0_file_path,
    AR1_file_path,
    AR2_file_path,
    AR3_file_path,
    whole_size_mm,
    tilt_degree,
    tilt_mode,
    bottom_height_mm,
    tag_size_mm,
    tag_thickness_mm,
    tag_inner_taper_degree,
    image_margin_frame_dots_num,
):
    """
    Create the overall AR marker pattern in Fusion 360.

    Parameters:
        AR0_file_path (str): Path to the AR0 PNG file.
        AR1_file_path (str): Path to the AR1 PNG file.
        AR2_file_path (str): Path to the AR2 PNG file.
        AR3_file_path (str): Path to the AR3 PNG file.
        whole_size_mm (float): Whole square size in millimeters.
        tilt_degree (float): Tilt angle in degrees.
        tilt_mode (str): Tilt rotation mode {"diagonal", "pitch", "pitch_r", "roll", "roll_r"}.
        bottom_height_mm (float): Bottom thickness in millimeters.
        tag_size_mm (float): Tag size in millimeters.
        tag_thickness_mm (float): Tag thickness in millimeters.
        tag_inner_taper_degree (float): Tag inner taper angle in degrees.
        image_margin_frame_dots_num (int): Number of dots to trim from image frame.
    """
    # Convert mm inputs to cm
    whole_size_cm = whole_size_mm / 10.0
    bottom_height_cm = bottom_height_mm / 10.0
    tag_size_cm = tag_size_mm / 10.0
    tag_thickness_cm = tag_thickness_mm / 10.0
    
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        root_comp = design.rootComponent

        # Load and trim AR images
        AR0 = trim_pixel_data(read_png(AR0_file_path), k=image_margin_frame_dots_num)
        AR1 = trim_pixel_data(read_png(AR1_file_path), k=image_margin_frame_dots_num)
        AR2 = trim_pixel_data(read_png(AR2_file_path), k=image_margin_frame_dots_num)
        AR3 = trim_pixel_data(read_png(AR3_file_path), k=image_margin_frame_dots_num)
        tag_dot_num = len(AR0)
        
        sketches = root_comp.sketches
        xy_plane = root_comp.xYConstructionPlane
        
        is_reversed = False
        if tilt_mode == "diagonal":
            # ----- DIAGONAL MODE IMPLEMENTATION -----
            base_size = whole_size_cm * math.sqrt(2)
            
            # Create base square and extrude
            base_sketch = create_base_square(sketches, xy_plane, base_size)
            extrude_up = whole_size_cm * (2**0.5) / 2 * math.tan(math.radians(tilt_degree))
            extrude_down = bottom_height_cm
            extrude_square_feat = extrude_square(root_comp, base_sketch, extrude_up, extrude_down, tilt_degree)
            
            # Draw tag area on inclined face
            inclined_sketch = draw_square_on_inclined_face_diagonal(
                sketches, extrude_square_feat.bodies.item(0).faces.item(3), base_size, 
                tag_size_cm, tag_dot_num, tilt_degree
            )
            
            inclined_whole_sketch = draw_whole_square_on_inclined_face_diagonal(
                sketches, extrude_square_feat.bodies.item(0).faces.item(3), base_size, 
                tag_size_cm, tilt_degree
            )
            
            # Draw square on bottom face
            bottom_sketch = draw_square_on_bottom_face_diagonal(
                sketches, extrude_square_feat.bodies.item(0), 
                base_size, bottom_height_cm, tilt_degree
            )
            
            # Intersect bottom profile
            bottom_profile = bottom_sketch.profiles.item(3)
            bottom_extrude_dist = -extrude_up - extrude_down
            intersect_body = extrude(
                root_comp, bottom_profile, bottom_extrude_dist, taper_angle=0,
                operation_type='Intersect', direction='OneSide'
            )

        elif tilt_mode in ("pitch", "pitch_r", "roll", "roll_r"):
                # Determine if we're using the "pitch" or "roll" mode with reversed flag
            if tilt_mode == "pitch_r":
                tilt_mode = "pitch"
                is_reversed = True
            if tilt_mode == "roll_r":
                tilt_mode = "roll"
                is_reversed = True

            # ----- SIDES MODE IMPLEMENTATION -----
            base_length_cm = whole_size_cm - tag_size_cm
            margin_cm = (base_length_cm - tag_size_cm) / 2
            
            # Create trapezoid sketch and extrude
            trapezoid_sketch = create_trapezoid_sketch(
                root_comp, xy_plane, base_length_cm, tilt_degree, bottom_height_cm
            )
            trapezoid_profile = trapezoid_sketch.profiles[0]
            trapezoid_extrude = extrude(root_comp, trapezoid_profile, (1 if tilt_mode=="pitch" else -1) * base_length_cm)
            trapezoid_body = trapezoid_extrude.bodies.item(0)
            
            # Find the inclined face
            inclined_face = None
            for face in trapezoid_body.faces:
                if face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                    normal = face.geometry.normal
                    z_axis = adsk.core.Vector3D.create(0, 0, 1)
                    angle_deg = math.degrees(normal.angleTo(z_axis))
                    if 0 < angle_deg < 90:
                        inclined_face = face
                        break
            
            if inclined_face is None:
                raise RuntimeError("No suitable inclined face found.")
                
            # Draw the tag area on the inclined face
            inclined_sketch = draw_square_on_inclined_face_sides(
                sketches, inclined_face, base_length_cm, margin_cm, tag_dot_num, tilt_degree
            )
            
            inclined_whole_sketch = draw_whole_square_on_inclined_face_sides(
                sketches, inclined_face, base_length_cm, margin_cm, tilt_degree
            )
        
        # ----- COMMON CODE FOR BOTH MODES -----
        # Extract inclined profile for the tag dot
        inclined_profile = None
        for profile in inclined_sketch.profiles:
            if profile.profileLoops.count == 1:
                inclined_profile = profile
                break
                
        if inclined_profile is None:
            raise RuntimeError("No valid profile found on inclined sketch.")
        
        # Extrude tag dot
        tag_dot_feat = extrude(
            root_comp,
            inclined_profile,
            -tag_thickness_cm,
            taper_angle=f"{abs(tag_inner_taper_degree)} deg",
            operation_type='NewBody',
            direction='OneSide'
        )
        
        # Extrude whole tag area
        whole_profile = None
        for profile in inclined_whole_sketch.profiles:
            if tilt_mode == "diagonal" and profile.profileLoops.count == 1:
                whole_profile = profile
                break
            elif tilt_mode in ("pitch","roll"):
                whole_profile = inclined_whole_sketch.profiles[1]
                break
                
        if whole_profile is None:
            raise RuntimeError("No valid whole profile found on inclined whole sketch.")
            
        tag_whole_feat = extrude(
            root_comp,
            whole_profile,
            -tag_thickness_cm,
            taper_angle=f"{tag_inner_taper_degree} deg",
            operation_type='NewBody',
            direction='OneSide'
        )
        
        # Create a rectangular pattern of dots
        rec_pattern = create_rectangular_pattern(
            component=root_comp,
            target_bodies=tag_dot_feat.bodies.item(0),
            pattern_type='spacing',
            direction_one=inclined_sketch.sketchCurves.sketchLines.item(4),
            quantity_one=tag_dot_num,
            distance_one=-tag_size_cm / tag_dot_num,
            direction_two=inclined_sketch.sketchCurves.sketchLines.item(5),
            quantity_two=tag_dot_num,
            distance_two=(-1 if tilt_mode=="diagonal" else 1) * tag_size_cm / tag_dot_num
        )
        
        # Reorder rectangular pattern bodies
        reordered_rec_pattern = [rec_pattern.bodies[i-1] for i in range(rec_pattern.bodies.count)]
        
        # Create circular pattern of the entire tag pattern
        circular_pattern = create_circular_pattern(
            component=root_comp,
            target_bodies=reordered_rec_pattern,
            axis=root_comp.zConstructionAxis,
            is_symmetric=False,
            quantity=4,
            angle=360
        )
        
        circular_whole_pattern = create_circular_pattern(
            component=root_comp,
            target_bodies=tag_whole_feat,
            axis=root_comp.zConstructionAxis,
            is_symmetric=False,
            quantity=4,
            angle=360
        )
        
        # Reorder circular pattern bodies
        reordered_circular_pattern = []
        for i in range(circular_pattern.bodies.count):
            if i % 4 == 0:
                reordered_circular_pattern.append(circular_pattern.bodies[3*(tag_dot_num**2)+i//4])
            else:
                reordered_circular_pattern.append(circular_pattern.bodies[(i//4)*3+(i%4)-1])
                
        # Reorder circular whole pattern bodies
        reordered_circular_whole_pattern = [circular_whole_pattern.bodies[i-1] for i in range(circular_whole_pattern.bodies.count)]
        
        # Create AR tags by joining or hiding dots
        AR0_feature, AR0_hidden = make_AR(
            root_comp,
            reordered_circular_pattern,
            0,
            ravel_pixel_data(rotate90_pixel_data(AR0, k=3))
        )
        AR1_feature, AR1_hidden = make_AR(
            root_comp,
            reordered_circular_pattern,
            1,
            ravel_pixel_data(rotate90_pixel_data(AR1, k=3))
        )
        AR2_feature, AR2_hidden = make_AR(
            root_comp,
            reordered_circular_pattern,
            2,
            ravel_pixel_data(rotate90_pixel_data(AR2, k=3))
        )
        AR3_feature, AR3_hidden = make_AR(
            root_comp,
            reordered_circular_pattern,
            3,
            ravel_pixel_data(rotate90_pixel_data(AR3, k=3))
        )
        
        # Create main body based on tilt mode
        if tilt_mode == "diagonal":
            main_body = intersect_body.bodies.item(0)
        elif tilt_mode in ("pitch", "roll"):
            # Create a circular pattern of the trapezoid body
            circular_pattern_trapezoid = create_circular_pattern(
                component=root_comp,
                target_bodies=trapezoid_body,
                axis=root_comp.zConstructionAxis,
                is_symmetric=False,
                quantity=4,
                angle=360
            )        
        
        # Adjust bodies based on the taper angle
        if tag_inner_taper_degree >= 0:
            for f, h, w in zip(
                [AR0_feature, AR1_feature, AR2_feature, AR3_feature],
                [AR0_hidden, AR1_hidden, AR2_hidden, AR3_hidden],
                reordered_circular_whole_pattern
            ):            
                rest = combine_bodies(root_comp, w, h, operation_type="Join", is_keep_tool_bodies=False)
                root_comp.features.removeFeatures.add(rest.bodies[0])
        else:
            # tag_inner_taper_degree < 0
            for f, h, w in zip(
                [AR0_feature, AR1_feature, AR2_feature, AR3_feature],
                [AR0_hidden, AR1_hidden, AR2_hidden, AR3_hidden],
                reordered_circular_whole_pattern
            ):            
                rest = combine_bodies(root_comp, w, h, operation_type="Cut", is_keep_tool_bodies=False)
                root_comp.features.removeFeatures.add(f.bodies[0])

        marker_bodies = adsk.core.ObjectCollection.create()

        for i in range(1,5):
            marker_bodies.add(root_comp.bRepBodies.item(i))
        if tilt_mode in ("pitch", "roll"):
            # Final cut operation for tag bodies
        
            if is_reversed:
                for i, feat, x, y in zip(
                    (0,1,2,3),
                    marker_bodies,
                    (base_length_cm/2,-base_length_cm/2,-base_length_cm/2,base_length_cm/2),
                    (base_length_cm/2,base_length_cm/2,-base_length_cm/2,-base_length_cm/2)
                ):
                    
                    feat_temp = adsk.core.ObjectCollection.create()
                    feat_temp.add(feat)
                    feat_temp.add(circular_pattern_trapezoid.bodies.item((i-1)%4))
                    rotate_around_line_by_points(root_comp, feat_temp, x, y)
                
            # Combine the circular pattern bodies into a single body
            tool_bodies = adsk.core.ObjectCollection.create()
            for i in range(circular_pattern_trapezoid.bodies.count):
                if i == 0:
                    target_body = circular_pattern_trapezoid.bodies.item(i)
                else:
                    tool_bodies.add(circular_pattern_trapezoid.bodies.item(i))
            combined_feature = combine_bodies(
                root_comp, target_body, tool_bodies,
                operation_type='Join', is_keep_tool_bodies=False
            )
            main_body = combined_feature.bodies.item(0)

        # Use main_body as the target for the cut operation
        combine_bodies(root_comp, main_body, marker_bodies, operation_type="Cut", is_keep_tool_bodies=True)
        
        ui.messageBox('Pattern created successfully')
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def run(context):
    """
    Entry point for the script. Loads configuration from a JSON file and runs create_pattern.
    """
    app = adsk.core.Application.get()
    ui = app.userInterface
    file_path = get_file_path()

    if file_path:
        try:
            with open(file_path, "r") as file:
                config_data = json.load(file)

            create_pattern(
                AR0_file_path=config_data["AR0_file_path"],
                AR1_file_path=config_data["AR1_file_path"],
                AR2_file_path=config_data["AR2_file_path"],
                AR3_file_path=config_data["AR3_file_path"],
                whole_size_mm=config_data["whole_size_mm"],
                tilt_degree=config_data["tilt_degree"],
                tilt_mode=config_data["tilt_mode"],
                bottom_height_mm=config_data["bottom_height_mm"],
                tag_size_mm=config_data["tag_size_mm"],
                tag_thickness_mm=config_data["tag_thickness_mm"],
                tag_inner_taper_degree=config_data["tag_inner_taper_degree"],
                image_margin_frame_dots_num=config_data["image_margin_frame_dots_num"],
            )
        except Exception:
            ui.messageBox('Config Loading Error:\n{}'.format(traceback.format_exc()))