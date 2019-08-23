def header_to_string(msg):
    header = "seq,frame_id,secs,nsecs"

    seq = msg.seq
    frame_id = msg.frame_id
    secs = msg.stamp.secs
    nsecs = msg.stamp.nsecs
    timestamp = rospy.Time(secs, nsecs)

    ts = str(timestamp.to_nsec())
    secs = str(ts[0:10])
    nsecs = str(ts[10:19])

    data = ",".join([str(seq), str(frame_id), secs, nsecs])

    return [header, data]
