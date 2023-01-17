gl_dep = dependency('opengl')
cpp = meson.get_compiler('cpp')
winmm_dep = cpp.find_library('winmm')

freeglut_lib = static_library('freeglut', [
	'freeglut_callbacks.c',
	'freeglut_callbacks.c',
	'freeglut_cursor.c',
	'freeglut_display.c',
	'freeglut_ext.c',
	'freeglut_font_data.c',
	'freeglut_font.c',
	'freeglut_gamemode.c',
	'freeglut_geometry.c',
	'freeglut_glutfont_definitions.c',
	'freeglut_init.c',
	'freeglut_input_devices.c',
	'freeglut_internal.h',
	'freeglut_joystick.c',
	'freeglut_main.c',
	'freeglut_menu.c',
	'freeglut_misc.c',
	'freeglut_overlay.c',
	'freeglut_spaceball.c',
	'freeglut_state.c',
	'freeglut_stroke_mono_roman.c',
	'freeglut_stroke_roman.c',
	'freeglut_structure.c',
	'freeglut_teapot.c',
	'freeglut_videoresize.c',
	'freeglut_window.c',
])
freeglut_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: freeglut_lib,
    dependencies: [gl_dep, winmm_dep],
)
