import bagpy
from bagpy import bagreader

reader = bagreader('data_driving_1.bag')

data_imu = reader.message_by_topic('/imu')
data_gps = reader.message_by_topic('/gps')

