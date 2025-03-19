from pyproj import CRS, Transformer
from rclpy.logging import get_logger

from .utils import (
    check_crc,
    check_checksum,
    msg_seperate,
    determine_utm_zone_and_hemisphere,
)
import numpy as np


class Decoder:
    def __init__(self) -> None:
        # From PVTSLN message
        self.bestpos_hgt = 0
        self.bestpos_lat = 0
        self.bestpos_lon = 0
        self.bestpos_hgtstd = 0
        self.bestpos_latstd = 0
        self.bestpos_lonstd = 0
        # From KSXT message
        self.vel_east = 0
        self.vel_north = 0
        self.vel_up = 0
        # From GPHPR message
        self.heading = 0
        self.pitch = 0
        self.roll = 0
        # From BESTNAV message
        self.vel_horstd = 0
        self.vel_verstd = 0
        # 转换为UTM
        self.utm_x = 0
        self.utm_y = 0
        # std
        self.vel_east_std = 0
        self.vel_north_std = 0
        self.vel_hor_cov = 0
        self.vel_up_std = 0

        # flags
        self.GNHPR_update = False
        self.BESTNAV_update = False
        self.PVTSLN_update = False

        # raw msgs
        self.GNGGA_msg = ""

        # add logger
        self.logger = get_logger('gnss_decoder')

    def __PVTSLN_solver(self, msg: str):
        parts = msg_seperate(msg)
        self.bestpos_hgt = float(parts[3 + 7])
        self.bestpos_lat = float(parts[4 + 7])
        self.bestpos_lon = float(parts[5 + 7])
        self.bestpos_hgtstd = float(parts[6 + 7])
        self.bestpos_latstd = float(parts[7 + 7])
        self.bestpos_lonstd = float(parts[8 + 7])
        self.PVTSLN_update = True

        self.logger.info("parse PVTSLN msg successfully")

    def __KSXT_solver(self, msg: str):
        parts = msg_seperate(msg)
        self.vel_east = float(parts[18 - 1])
        self.vel_north = float(parts[19 - 1])
        self.vel_up = float(parts[20 - 1])
        
        self.logger.info("parse KSXT msg successfully")

    def __GNHPR_solver(self, msg: str):
        parts = msg_seperate(msg)
        self.heading = float(parts[3 - 1])
        self.pitch = float(parts[4 - 1])
        self.roll = float(parts[5 - 1])
        self.GNHPR_update = True
        
        self.logger.info("parse GNHPR msg successfully")

    def __BESTNAV_solver(self, msg: str):
        parts = msg_seperate(msg)
        self.vel_horstd = float(parts[-1][:3])
        self.vel_verstd = float(parts[-2][4:])
        self.BESTNAV_update = True
        
        self.logger.info("parse BESTNAV msg successfully")

    def __GNGGA_solver(self, msg: str):
        self.GNGGA_msg = msg
        
        self.logger.info("parse GNGGA msg successfully")

    def __utm_trans(self):
        if self.PVTSLN_update:
            wgs84_crs = CRS("epsg:4326")
            zone_number, isnorth = determine_utm_zone_and_hemisphere(
                self.bestpos_lat, self.bestpos_lon
            )
            utm_crs_str = (
                f"epsg:326{zone_number}" if isnorth else f"epsg:327{zone_number}"
            )
            utm_crs = CRS(utm_crs_str)
            transformer = Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)
            self.utm_x, self.utm_y = transformer.transform(
                self.bestpos_lon, self.bestpos_lat
            )
            
            self.logger.info("WGS84 -> UTM successfully")

    def __std_trans(self):
        if self.GNHPR_update and self.BESTNAV_update:
            heading_rad = np.deg2rad(self.heading)
            cos_h = np.cos(heading_rad)
            sin_h = np.sin(heading_rad)
            vel_cov_xy = self.vel_horstd**2
            self.vel_east_std = np.sqrt(vel_cov_xy * cos_h**2)
            self.vel_hor_cov = np.sqrt(vel_cov_xy * cos_h * sin_h)
            self.vel_north_std = np.sqrt(vel_cov_xy * sin_h**2)
            self.vel_up_std = self.vel_verstd
            
            self.logger.info("calculate std successfully")

    def __parse(self, msg: str):
        try:
            if msg.startswith("#PVTSLNA") and check_crc(msg):
                self.__PVTSLN_solver(msg)
            elif msg.startswith("$GNHPR") and check_checksum(msg):
                self.__GNHPR_solver(msg)
            elif msg.startswith("$KSXT") and check_checksum(msg):
                self.__KSXT_solver(msg)
            elif msg.startswith("#BESTNAVA") and check_crc(msg):
                self.__BESTNAV_solver(msg)
            elif msg.startswith("$GNGGA") and check_checksum(msg):
                self.__GNGGA_solver(msg)
        except Exception as e:
            self.logger.warn(f"Illegal msg: {msg}, error: {e}")
        finally:
            pass

    def decode(self, msg: str):
        self.__parse(msg)
        self.__utm_trans()
        self.__std_trans()
