# ExcludeSatellites — command message for the gnss_satellite_filter node
#
# Publish this message to the topic  sgps/exclude_satellites  to update
# which satellites the mosaic-X5 includes in its PVT solution.
#
# Satellite ID format follows Septentrio convention:
#   GPS      G01 .. G32
#   GALILEO  E01 .. E36
#   BEIDOU   C01 .. C63
#   GLONASS  R01 .. R24
#
# Examples:
#   include_ids = ["all"]        — re-enable every satellite
#   exclude_ids = ["G03", "E07"] — remove G03 and E07 from PVT
#
# The driver translates these to one or two  ssu  commands:
#   ssu, all<CR>     (reset to all)  followed by
#   ssu, -G03-E07<CR>               (remove the listed ids)
#
# Leave include_ids empty to skip the reset-to-all step.
# This is the ROS 2 version of the message.

std_msgs/Header header

# Satellites to add/restore (use "all" to re-enable everything first)
string[] include_ids

# Satellites to remove from the PVT solution
string[] exclude_ids
