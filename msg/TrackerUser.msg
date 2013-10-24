# TrackerUser: Contains a snapshot of a single users tracking data. an ID, a vector of frames corresponding to each joint, a vector of confidences corresponding to each joint, and a vector of geometry_msgs/Transform messages corresponding to each joint. 
Header header
uint8 uid
string tracker_id
string[] frames
float64[] confs
geometry_msgs/Transform[] transforms
geometry_msgs/Vector3[] projective
geometry_msgs/Vector3 center_of_mass
