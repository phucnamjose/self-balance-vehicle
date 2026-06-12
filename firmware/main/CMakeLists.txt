# Register the main component. As the project grows we'll add more source
# files here (and pull in extra IDF components via REQUIRES).
#
# EMBED_TXTFILES bakes the web page into the firmware binary at build time and
# null-terminates it, so we can serve it as a string. It is exposed via the
# linker symbols _binary_index_html_start / _binary_index_html_end.
idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "www/index.html"
)
