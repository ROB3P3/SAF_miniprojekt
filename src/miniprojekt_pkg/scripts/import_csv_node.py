#!/usr/bin/env python3 
import rclpy
from miniprojekt_pkg.importing_csv import CSVImportNode

def main(args=None):
    rclpy.init(args=args)
    node = CSVImportNode()
    #køre noden indtil den bliver stoppet af ctrl+c eller andet, hvorefter rclpy.shutdown() kaldes
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()