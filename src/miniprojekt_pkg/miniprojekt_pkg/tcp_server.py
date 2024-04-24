#!/usr/bin/env python3 
import rclpy
import socket
import xml.sax
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Int64
from miniprojekt_pkg.xml_content_handler import MyContentHandler

class TCPServerNode(Node):
    def __init__(self):
        # Initialize node and publisher
        super().__init__("tcp_server_node")
        self.serverPublisher_ = self.create_publisher(Int64MultiArray, "/tcp_server", 10)
        self.get_logger().info("TCP Server Node initialized.")

        self.xmlContentList = []
        self.waitTime = None

        # Creating the TCP server
        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.port = 20001
        self.hostIP = "172.20.10.3"
        self.serverSocket.bind((self.hostIP, self.port))

        self.tcpServerListen()

    def tcpServerListen(self):
        # Gets the server ready to listen to connections
        self.serverSocket.listen()
        while True:
            # Establish a connection
            self.get_logger().info("Waiting for a connection...")
            client_socket, addr = self.serverSocket.accept()

            self.get_logger().info("Got a connection from %s" % str(addr))

            # Receive the message
            messageReceive = client_socket.recv(1024)
            self.get_logger().info("The client said: %s" % str(messageReceive))

            # Parsing the XML String received from the PLC
            self.xmlContentList = []
            contentHandler = MyContentHandler(self.xmlContentList)
            xml.sax.parseString(messageReceive, contentHandler)

            # Converting the parsed information to an int[] array which contains pallete ID and PLC ID
            publishArray = Int64MultiArray()
            publishArray.data = [int(self.xmlContentList[0][1]), int(self.xmlContentList[2][1].split("_")[1])]
            # Publishes parsed information to publisher.
            self.serverPublisher_.publish(publishArray)

            # Initializing node for receiving processing time
            timeNode = TimeRecipientNode()
            
            # While this node has not yet received the processing time, continously spin the time recipient node
            while self.waitTime == None:
                self.get_logger().info("Spinning time recipient node.")
                # If the time recipient node has received the processing time, set this node's proccessing time equal to that node's processing time
                if timeNode.waitTime != None:
                    self.waitTime = timeNode.waitTime
                rclpy.spin_once(timeNode)
            
            # Destroy the node after receiving the processing time to not have 2 same named subscribers running when a new client connects
            timeNode.destroy_node()

            # Return the processing time to the client
            messageSend = self.waitTime
            client_socket.send(messageSend.to_bytes(2, byteorder='little'))
            self.get_logger().info("Sent: %s" % str(messageSend.to_bytes(2, byteorder='little')))

            # Close the connection
            client_socket.close()

class TimeRecipientNode(Node):
    def __init__(self):
        # Initialize node and subscription to CSV Import node
        super().__init__("time_recipient_node")
        self.timeSubscriber_ = self.create_subscription(Int64, "/palette_time", self.onTimeMsg, 10)
        self.waitTime = None

    def onTimeMsg(self, msg):
        """Callback function for subscription to '/palette_time' topic"""
        self.get_logger().info("Time Recipient Node received message from palette time topic.")
        self.get_logger().info("Received message: " + str(msg.data))
        self.waitTime = msg.data