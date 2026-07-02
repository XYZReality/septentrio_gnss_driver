# Galileo broadcast ephemeris (SBF block GALNav, ID 4002)
# Orbital angular parameters (m_0, i_0, omega, omega_0, del_n, omegadot, idot)
# are in semi-circles; multiply by pi to get radians. The harmonic correction
# terms c_uc/c_us/c_ic/c_is are already in radians and c_rc/c_rs are in metres.
# SQRT_A is in m^(1/2); A = sqrt_a * sqrt_a gives the semi-major axis in metres.
# Times t_oe and t_oc are seconds-of-week (GPS time frame).
# source: 2 = I/NAV (L1, E5b), 16 = F/NAV (L1, E5a)
# This is the ROS 2 version of the message.

# ROS message header
std_msgs/Header header

septentrio_gnss_driver/BlockHeader block_header

uint8   svid          # Galileo satellite ID
uint8   source        # Navigation message source: 2=I/NAV, 16=F/NAV

float64 sqrt_a        # Square root of semi-major axis (m^(1/2))
float64 m_0           # Mean anomaly at reference time (semi-circle)
float64 e             # Eccentricity
float64 i_0           # Inclination angle at reference time (semi-circle)
float64 omega         # Argument of perigee (semi-circle)
float64 omega_0       # Longitude of ascending node at weekly epoch (semi-circle)
float32 omegadot      # Rate of right ascension (semi-circle/s)
float32 idot          # Rate of inclination angle (semi-circle/s)
float32 del_n         # Mean motion difference (semi-circle/s)
float32 c_uc          # Cosine harmonic correction to argument of latitude (rad)
float32 c_us          # Sine harmonic correction to argument of latitude (rad)
float32 c_rc          # Cosine harmonic correction to orbit radius (m)
float32 c_rs          # Sine harmonic correction to orbit radius (m)
float32 c_ic          # Cosine harmonic correction to inclination angle (rad)
float32 c_is          # Sine harmonic correction to inclination angle (rad)

uint32  t_oe          # Ephemeris reference time (s, GPS time)
uint32  t_oc          # Clock data reference time (s, source-dependent)
float32 a_f2          # SV clock aging (s/s^2)
float32 a_f1          # SV clock drift (s/s)
float64 a_f0          # SV clock bias (s)

uint16  wnt_oe        # GPS week number for t_oe (mod 4096)
uint16  wnt_oc        # GPS week number for t_oc (mod 4096)
uint16  iodnav        # Issue of Data, Navigation (10-bit)
uint16  health_ossol  # Signal health / DVS bits (L1, E5b, E5a)
uint8   sisa_l1e5a    # Signal-in-space accuracy index (L1/E5a)
uint8   sisa_l1e5b    # Signal-in-space accuracy index (L1/E5b)
float32 bgd_l1e5a     # Broadcast Group Delay L1/E5a (s)
float32 bgd_l1e5b     # Broadcast Group Delay L1/E5b (s)
