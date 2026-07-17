from pathlib import Path
import re
import pandas as pd


def load_calibration_data(folder: str | Path) -> pd.DataFrame:
    """
    Load calibration data from a folder containing CSV files.

    Args:
        folder (str | Path): The path to the folder containing the CSV files.

    Returns:
        pd.DataFrame: The concatenated calibration data from all CSV files in the folder.
    """
    folder_path = Path(folder)
    if not folder_path.is_dir():
        raise ValueError(f"The provided path '{folder}' is not a valid directory.")

    # List all CSV files in the folder
    csv_files = list(folder_path.glob("*.csv"))
    if not csv_files:
        raise ValueError(f"No CSV files found in the directory '{folder}'.")

    # Load and concatenate all CSV files into a single DataFrame
    data_frames = []
    for csv_file in csv_files:
        # Infer mode/orientation from filename
        re_pattern = r"(?P<datetime>\d{8}_\d{6})_(?P<magnets>(?:ALL|M\d))_(?:(?P<mode>(?:NO_MAGNETS|NEUTRAL_POSITION))|(?P<DoF>R?[XYZ](?P<value>-?\d+(?:\.\d+)?)(?P<unit>(?:deg|mm))))\.csv"
        re_match = re.fullmatch(re_pattern, csv_file.name)
        # print(f"Processing file: {csv_file.name}")
        # print(f"Regex match: {re_match.groups() if re_match else 'No match'}")

        if not re_match:
            print(f"Warning: Filename '{csv_file.name}' does not match the expected pattern. Skipping this file.")
            continue

        # Extract information from the regex match
        if re_match.group("mode") == "NO_MAGNETS":
            mode = "NO_MAGNETS"
            magnet_1 = False
            magnet_2 = False
            magnet_3 = False
            X = None
            Y = None
            Z = None
            RX = None
            RY = None
            RZ = None
        else:
            mode = "DoF"
            if re_match.group("magnets") == "ALL":
                magnet_1 = True
                magnet_2 = True
                magnet_3 = True
            elif re_match.group("magnets") == "M1":
                magnet_1 = True
                magnet_2 = False
                magnet_3 = False
            elif re_match.group("magnets") == "M2":
                magnet_1 = False
                magnet_2 = True
                magnet_3 = False
            elif re_match.group("magnets") == "M3":
                magnet_1 = False
                magnet_2 = False
                magnet_3 = True
            else:
                print(
                    f"Warning: Unrecognized magnets group '{re_match.group('magnets')}' in filename '{csv_file.name}'. Skipping this file."
                )
                continue

            if re_match.group("mode") == "NEUTRAL_POSITION":
                X = 0.0
                Y = 0.0
                Z = 0.0
                RX = 0.0
                RY = 0.0
                RZ = 0.0
            else:
                # Extract DoF values
                dof = re_match.group("DoF")
                value = float(re_match.group("value"))
                unit = re_match.group("unit")

                # Initialize all DoF values to 0
                X = Y = Z = RX = RY = RZ = 0

                if dof.startswith("X"):
                    X = value
                elif dof.startswith("Y"):
                    Y = value
                elif dof.startswith("Z"):
                    Z = value
                elif dof.startswith("RX"):
                    RX = value
                elif dof.startswith("RY"):
                    RY = value
                elif dof.startswith("RZ"):
                    RZ = value
                else:
                    print(f"Warning: Unrecognized DoF '{dof}' in filename '{csv_file.name}'. Skipping this file.")
                    continue

        # Construct metadata dictionary
        metadata = {
            "mode": mode,
            "magnet_1": magnet_1,
            "magnet_2": magnet_2,
            "magnet_3": magnet_3,
            "X": X,
            "Y": Y,
            "Z": Z,
            "RX": RX,
            "RY": RY,
            "RZ": RZ,
        }

        print(f"Loading file: {csv_file.name} with metadata: {metadata}")

        # Load the CSV file into a DataFrame
        df = pd.read_csv(csv_file)

        # Append metadata columns to the DataFrame
        for key, value in metadata.items():
            df[key] = value

        # Append filename column to the DataFrame
        df["filename"] = csv_file.name

        # Append the DataFrame to the list
        data_frames.append(df)

    concatenated_data = pd.concat(data_frames, ignore_index=True)

    # Convert X, Y, Z, RX, RY, RZ columns to float
    for col in ["X", "Y", "Z", "RX", "RY", "RZ"]:
        concatenated_data[col] = pd.to_numeric(concatenated_data[col], errors="coerce")

    # Sort by timestamp_iso
    if "timestamp_iso" in concatenated_data.columns:
        concatenated_data.sort_values(by="timestamp_iso", inplace=True, ignore_index=True)
    else:
        print("Warning: 'timestamp_iso' column not found in the data. Skipping sorting.")
    return concatenated_data


if __name__ == "__main__":
    # Example usage
    folder_path = Path(__file__).resolve().parent / "../../data/calibration_data"
    calibration_data = load_calibration_data(folder_path)
    print(calibration_data.head())
    print(calibration_data.info())
