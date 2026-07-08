# Register the main component. As the project grows we'll add more source
# files here (and pull in extra IDF components via REQUIRES).
#
# The control UI is no longer embedded in the firmware - it runs as a separate
# web app on the operator's PC (see ../../webapp) and talks to the device over
# WebSocket. The board only serves a tiny landing page plus the /ws, /api and
# /ota endpoints.
idf_component_register(
    SRCS
        "app_main.c"
        "telemetry.c"
        "led.c"
        "motors.c"
        "encoders.c"
        "imu.c"
        "wifi_ap.c"
        "web_server.c"
        "control.c"
        "wheel_pi.c"
        "balance_pid.c"
        "estimator.c"
    INCLUDE_DIRS "."
)
