from talker_msgs.msg import talker_ic_pub
from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point , PoseStamped , PointStamped
from math import cos, sin 
class lidar(Node):

	def __init__(self):
		super().__init__('lidar_güvenlik')
		self.sub = self.create_subscriber(talker_ic_pub , "/ic_veri_dolasim/v3", self.sub) 
		self.pub = self.create_publisher(talker_ic_pub,"ic_veri_dolasim/v4", queue_size=1) 
		self.pub2 = self.create_publisher( Float64, "sinir", queue_size=1) 
		self.last = 0
		self.first = False
		self.geri = False
		self.laser = Laser.scan()
	
	def sub(self, data):
		self.range = data.ranges
		for i in range: 
			if i <= 5 :                 
				laser = self.laser.ranges 	
				shortest_laser = 10000
				point = point()
				for i in range(len(laser)):
					if laser[i] < shortest_laser:
						angle= self.laser.angle_min +  i*self.laser.angle_increment
						x = laser[i]*cos(angle)
						if x > -0.2:
							shortest_laser = laser[i]
							point.x = x
							point.y = shortest_laser*sin(angle)

							print( angle, point.x, point.y)
				 				
def main(args=None):
	rclpy.init(args=args)
	node = lidar()
	rclpy.spin(node)
	rclpy.shutdown()    

if __name__ == '__main__':
	main()
   
   	

