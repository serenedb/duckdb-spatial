# GeographicLib geodesic routines (C)

`geodesic.c` / `geodesic.h`, the C transcription of GeographicLib's geodesic
classes by Charles Karney, under the MIT/X11 License. Taken from PROJ 9.3.1
(`src/geodesic.[ch]`), which vendors the same files.

The extension used to reach these routines through PROJ. PROJ was dropped
because `ST_Transform` needs its `proj.db` coordinate-system database and a
SQLite build to read it; the geodesic solver behind the `*_Spheroid` functions
needs neither, so it is vendored directly instead.

Local change: the two `PROJ_*` preprocessor branches in `geodesic.h` (the MSVC
dllexport switch and the `proj_symbol_rename.h` include) were removed.

See `LICENSE` for the MIT/X11 notice carried in these files, and the
provenance note recording that they came from PROJ 9.3.1.
