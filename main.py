#!/usr/bin/env python3
import yaml
from tqdm import tqdm
from coloradar_plus_processing_tools import DatasetToKitti

def parse_config(config_path):
    """_Parser for ColoradarPlus-to-kitti Configuration file._

    Args:
        config_path (_str_): _Path to the YAML configuration file._

    Returns:
       config_values (_dict_): _Python dict containing values taken from YAML configuration file._
    """

    def load_config(path):
        with open(path, "r") as file:
            return yaml.safe_load(file)
        
    config = load_config(config_path)

    # Configuration as a dict
    crp_config_dict = {
        "main_bag_directory_path": config.get("main_bag_directory_path", []),
        "main_kitti_directory_path": config.get("main_kitti_directory_path", []),
        "generate_sequence_run_stats": (config.get("generate_sequence_run_stats", True)),
        "display_rosbag_data": (config.get("display_rosbag_data", True)),
        "sequences": (
            config["sequences"]
        ),
    }
    return crp_config_dict

def main():
    # Convert values from YAML config to dict for easier handling
    crp_config_dict = parse_config("configs/processing_config_doncey.yaml")

    # Initialize bag_parser object
    dataset2kitti = DatasetToKitti(crp_config_dict)

    # Convert specified runs in sequences to KITTI-style
    dataset2kitti.convert()

if __name__ == "__main__":
    main()
