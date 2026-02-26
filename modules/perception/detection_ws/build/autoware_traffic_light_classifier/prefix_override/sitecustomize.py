import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/modules/detection/detection_ws/install/autoware_traffic_light_classifier'
