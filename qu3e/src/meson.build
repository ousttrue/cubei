qu3e_lib = static_library('qu3e', [
	'broadphase/q3BroadPhase.cpp',
	'broadphase/q3DynamicAABBTree.cpp',
	'collision/q3Box.cpp',
	'collision/q3Collide.cpp',
	'common/q3Geometry.cpp',
	'common/q3Memory.cpp',
	'dynamics/q3Body.cpp',
	'dynamics/q3Contact.cpp',
	'dynamics/q3ContactManager.cpp',
	'dynamics/q3ContactSolver.cpp',
	'dynamics/q3Island.cpp',
	'math/q3Mat3.cpp',
	'math/q3Quaternion.cpp',
	'math/q3Vec3.cpp',
	'scene/q3Scene.cpp',
])
qu3e_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: qu3e_lib,
)
