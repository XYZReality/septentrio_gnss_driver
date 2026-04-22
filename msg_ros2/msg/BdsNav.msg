# BeiDou broadcast ephemeris (SBF block BDSNav, ID 4081)
# All angular parameters are in semi-circles (multiply by pi to get radians).
# SQRT_A is in m^(1/2); A = sqrt_a * sqrt_a gives the semi-major axis in metres.
# t_oe and t_oc are in BeiDou System Time (lags GPS time by 14 s).
# This is the ROS 2 version of the message.

# ROS message header
std_msgs/Header header

septentrio_gnss_driver/BlockHeader block_header

uint8   prn           # BeiDou satellite PRN number
uint16  wn            # BeiDou week number (0-8191)
uint8   ura           # SV accuracy (URA index, 4-bit)
uint8   sat_h1        # Autonomous satellite health flag (1-bit)
uint8   iodc          # Age of Data, Clock (5-bit)
uint8   iode          # Age of Data, Ephemeris (5-bit)

float32 t_gd1         # Equipment group delay B1I (s)
float32 t_gd2         # Equipment group delay B2I (s)
uint32  t_oc          # Clock data reference time (s, BDT)
float32 a_f2          # SV clock aging (s/s^2)
float32 a_f1          # SV clock drift (s/s)
float32 a_f0          # SV clock bias (s)

float32 c_rs          # Sine harmonic correction to orbit radius (m)
float32 del_n         # Mean motion difference (semi-circle/s)
float64 m_0           # Mean anomaly at reference time (semi-circle)
float32 c_uc          # Cosine harmonic correction to argument of latitude (rad)
float64 e             # Eccentricity
float32 c_us          # Sine harmonic correction to argument of latitude (rad)
float64 sqrt_a        # Square root of semi-major axis (m^(1/2))
uint32  t_oe          # Ephemeris reference time (s, BDT)
float32 c_ic          # Cosine harmonic correction to inclination angle (rad)
float64 omega_0       # Longitude of ascending node at weekly epoch (semi-circle)
float32 c_is          # Sine harmonic correction to inclination angle (rad)
float64 i_0           # Inclination angle at reference time (semi-circle)
float32 c_rc          # Cosine harmonic correction to orbit radius (m)
float64 omega         # Argument of perigee (semi-circle)
float32 omegadot      # Rate of right ascension (semi-circle/s)
float32 idot          # Rate of inclination angle (semi-circle/s)

uint16  wnt_oc        # BeiDou week number for t_oc (mod 8192)
uint16  wnt_oe        # BeiDou week number for t_oe (mod 8192)
