#!/usr/bin/env python3 
import rclpy
import csv
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Int64

class CSVImportNode(Node):
    def __init__(self):
        # Initialize node, subscriptions, and publisher
        super().__init__("csv_import_node")
        self.serverSubscriber_ = self.create_subscription(Int64MultiArray, "/tcp_server", self.onServerMsg, 10)
        self.timePublisher_ = self.create_publisher(Int64, "/palette_time", 10)
        self.get_logger().info("CSV Import Node initialized.")

        self.timeArray = []
        self.timeSend = Int64()

        # Import data from csv file into a 2D array
        with open('./src/miniprojekt_pkg/miniprojekt_pkg/procssing_times_table.csv') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=';')
            for count, row in enumerate(csv_reader):
                if count != 0:
                    self.timeArray.append(row[1:])
    
    def onServerMsg(self, msg):
        """"Runs when receiving a message from the TCP Server Node"""
        self.get_logger().info("CSV Import Node received message from server.")
        self.get_logger().info("Sending message to palette time topic.")
        # Get the processing time based on pallete ID and PLC ID
        # if the pallete ID or PLC ID are higher than 16 just return 0
        if msg.data[0] <= 16 and msg.data[1] <= 16:
            self.timeSend.data = int(self.timeArray[msg.data[0]- 1][msg.data[1] - 1])
        else:
            self.timeSend.data = 0

        self.get_logger().info("Sending wait time of: " + str(self.timeSend.data) + " ms.")
        # Creating a timer that continously publishes the processing time 
        self.publishTimer = self.create_timer(1, self.sendTimeMessage)
        # Destroy the timer when the publisher's subscription count is above 1 as that indicates that the message was received.
        if self.timePublisher_.get_subscription_count() >= 1:
            self.get_logger().info("Message sent to palette time topic.")
            self.publishTimer.cancel()
    
    def sendTimeMessage(self):
        """Callback function for publisher timer."""
        self.timePublisher_.publish(self.timeSend)