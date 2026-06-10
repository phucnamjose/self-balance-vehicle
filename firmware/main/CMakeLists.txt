# Register the main component. As the project grows we'll add more source
# files here (and pull in extra IDF components via REQUIRES).
idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
)
