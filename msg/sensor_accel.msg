uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 error_count

float32 x		# acceleration in the NED X board axis in m/s^2
float32 y		# acceleration in the NED Y board axis in m/s^2
float32 z		# acceleration in the NED Z board axis in m/s^2

float32 temperature	# temperature in degrees celsius

float32 scaling		# scaling from raw to m/s^2
int16 x_raw
int16 y_raw
int16 z_raw
