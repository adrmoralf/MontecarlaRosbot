std_msgs/Header header            # stamp = ahora - scan_duration/2 ; frame_id = "base_link"
float32 scan_duration             # segundos: 0.0 sim ideal, ~2.0 sim realista y real
WifiMeasurement[] measurements    # solo APs visibles en este scan; ausente == ignorado
