import pathlib
import stretch_body.hello_utils as hu


def get_latest_urdf():
    """
    Fetches the latest calibrated URDF from the calibration directory.

    Returns
    -------
    str: Absolute filepath to the latest calibrated URDF.
    """
    try:
        fleet_dir = hu.get_fleet_directory()
    except:
        raise FileNotFoundError("Stretch data directory doesn't exist") from None
    calibration_dir = pathlib.Path(fleet_dir) / "exported_urdf"
    if not calibration_dir.is_dir():
        raise FileNotFoundError("URDF calibration directory doesn't exist")
    urdf_path = calibration_dir / "stretch.urdf"
    if not urdf_path.is_file():
        raise FileNotFoundError("URDF doesn't exist")
    urdf_filename = str(urdf_path.absolute())
    return urdf_filename


def generate_urdf():
    pass


if __name__ == "__main__":
    print(get_latest_urdf())
